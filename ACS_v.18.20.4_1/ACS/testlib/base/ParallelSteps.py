# Copyright (c) 2015 Intel Corporation

"""
Description: This module facilitates the usage of steps in a multiprocess setup.

#######
Usage:#
#######
psteps = ParallelSteps()

step_id = psteps.add_step(some_module.mystep, arg1 = value, arg2 = value)

NOTE: You will need the step id to reference it in the future!

Control methods:
psteps.get_step_info(step_id)      - returns step information in the following format:  {'name': <step_name>, 'status': running/finished, 'exception': <exception_type>, "stdout": None, "stderr": None};
psteps.terminate_step(step_id)     - terminates step;
psteps.step_is_running(step_id)    - returns True if step is still running;
psteps.wait_for_step(step_id, timeout=10)      - waits for the step to execute for timeout seconds. If timeout is None, it waits untill the step finishes.
                                                Accepts a terminate=True/False parameter that terminates the step after the timeout is reached.
psteps.interpret_step(step_id)     - Interprets the step and prints its output, verditc and errors. If the step threw exceptions, it raises them.
                                    Accepts the same extra parameters as the wai_for_step() method. If timeout is given, the step will be terminated after the specified time.

########################
Implementation details:#
########################
When instantiated, if use_control_process=True is used, the class launches a control process that is used to monitor the main process and launch/monitor the paralel steps.
The main reason for using a control process is that we detach it from the main process thus letting it exit when something goes wrong.
The control process is in charge of the following:
- monitor the main process. If either fail, the other one is closed, gracefully.
- accept requests from the main process and execute them, like adding a parallel steps and sending information about their status to the main process.
- monitor paralel steps and kill them when necessary.

NOTE: When using the control process, the output of the steps is shown only when the interpret_step method is called for them.

NOTE: The class can function without the control process and can be turned off by instantiating it with use_control_process=False. Doing this will result in the following:
- When the main process dies, due to an exception(like a failed step on the main process), the process will wait(blocked) for all parallel steps to finish before exiting.
(because the main process is dead we no longer have control over the paralel steps processes)
- The output of the paralel steps is printed directly to the main process stdout, in real time(this can cause confusion because the output is not coherent)

Known issues:
- This won't work with steps called from the same module!
- Not all adb_step tmp files are deleted!

If you find any bugs, please mail me at: corneliu.stoicescu@intel.com .

"""

import sys
import time
import os
import psutil
import signal
from multiprocessing import Process, Manager
from uuid import uuid4
from cStringIO import StringIO
from testlib.external.daemon import createDaemon
from testlib.base.abstract import abstract_utils


class ParallelSteps():

    def __init__(self, use_control_process=True):
        self.processes = []
        self.step_info_template = {'name':None, 'status':None, 'exception':None, "stdout": None, "stderr": None}
        self.steps_info = []

        if use_control_process:
            self._is_control_process = False

            self._request_template = {"id": None, "call_method": None, "args": (), "kwargs": {}}
            self._requests_list = []
            self._request_result_template = {"id": None, "result": None}
            self._requests_result_list = []

            # Using SIGUSR1 handle the event where the control process dies.
            signal.signal(signal.SIGUSR1, self._handle_signal_control_process_died)

            self._start_control_process()
        else:
            self._is_control_process = True

    def _handle_signal_control_process_died(self, signum, stack):
        print 'FATAL: Control process died. Exiting!'
        sys.exit(1)

    def _start_control_process(self, *args, **kwargs):
        manager = Manager()
        requests_list = manager.list()
        requests_result_list = manager.list()
        main_process_pid = os.getpid()

        p = Process(target=self._control_process, args=(main_process_pid, requests_list, requests_result_list,))
        p.start()

        self._requests_list = requests_list
        self._requests_result_list = requests_result_list

    def _control_process(self, main_process_pid, requests_list, requests_result_list):
        try:
            self._is_control_process = True
            self._requests_list = requests_list
            self._requests_result_list = requests_result_list
            self._main_process = psutil.Process(main_process_pid)
            # Detaching the control process from the main thread completely, as a daemon
            createDaemon()
#            # Need to add valid fd for stdout and stderr
#            sys.stdout = open('/dev/null', 'w')
#            sys.stderr = open('/dev/null', 'w')
            time.sleep(1)
            while self._main_process.is_running():
                requests_list = self._requests_list
                for request in requests_list:
                    call_method = request["call_method"]
                    args = request["args"]
                    kwargs = request["kwargs"]

                    result = self._control_process_call_method(call_method, *args, **kwargs)
                    request_result = self._request_result_template
                    request_result["result"] = result
                    request_result["id"] = request["id"]
                    self._requests_result_list.append(request_result)
                    self._requests_list.remove(request)
                time.sleep(0.2)
            else:
                self._control_process_kill_running_steps()
        except:
            os.kill(main_process_pid, signal.SIGUSR1)

    def _control_process_call_method(self, call_method, *args, **kwargs):
        methodToCall = getattr(self, call_method)
        try:
            result = methodToCall(*args, **kwargs)
            if result != None:
                return result
            else:
                return result
        except:
            return None

    def _control_process_kill_running_steps(self):
        for id in range(len(self.processes)):
            self.terminate_step(id)

    def _send_call_to_control_process(self, method, *args, **kwargs):
        request = self._request_template
        request["call_method"] = method
        request["args"] = args
        request["kwargs"] = kwargs
        request_id = self._send_control_process_request(request)
        found_result = False
        while not found_result:
            for result in self._requests_result_list:
                if result["id"] == request_id:
                    found_result = True
                    return result["result"]
                    break

    def _send_control_process_request(self, request):
        #TODO: check request format complies with template?
        request_id = uuid4()
        request["id"] = request_id
        self._requests_list.append(request)
        return request_id

    # Main method that will go in the new process. Launches the step and manages the step_info dictionary that is used to communicate with the main process.
    # Also passes the *args and **kwargs to the step on initialisation.
    def _start_step(self, step_module, step_name, step_info, parallel_iterative=False, stop_on_exception=True, *args, **kwargs):
        self.target_module = abstract_utils.import_module(step_module)
        step = abstract_utils.get_obj(self.target_module, step_name)
        sys.stdout = stepstdout = StringIO()
#        sys.stderr = stepstderr = StringIO()
        step_info['name'] = step.__name__
        step_info['status'] = "running"
        step_info['exception'] = str()

        runcount = (-1 if parallel_iterative else 1)

        while runcount != 0:
            try:
                step(*args, **kwargs)()
            except Exception, e:
                sys.stdout.write(str(e) + "\n")
                step_info['exception'] = str(e)
                if stop_on_exception:
                    break
            finally:
                step_info["stdout"] = str(stepstdout.getvalue())
#                step_info["stderr"] = str(stepstderr.getvalue())
            runcount += -1

        step_info['status'] = "finished"

    # Add a step to a new process. Also passes the **kwargs to the step on initialisation.
    def add_step(self, step=None, step_module=None, step_name=None, **kwargs):
        # Because the multiprocess Manager cannot share instantiated classes (like abstract steps being instantiated by the @abstractStep decorator),
        # we breake them down into strings(module and class name) and reconstitute them in the new process
        if step:
            if step.__module__ == "__main__" and step_module==None and  step_name==None:
                step_module = sys.modules[__name__].__file__.split(".")[0]
                step_name = step.__name__
            elif step.__module__ != "__main__" and step_module==None and  step_name==None:
                step_module = step(**kwargs).__module__.split(".")[-1]
                step_name = step(**kwargs).__class__.__name__
        if not self._is_control_process:
            this_func_name = sys._getframe().f_code.co_name
            result = self._send_call_to_control_process(this_func_name, step=None, step_module=step_module, step_name=step_name, **kwargs)
            return result

        manager = Manager()
        step_info_dict = manager.dict()

        for key, value in self.step_info_template.iteritems():
            step_info_dict[key] = value
        p = Process(target=self._start_step, args=(step_module, step_name, step_info_dict),kwargs=kwargs)

        # The id of the step will be used to identify it in the process, comms queue and step info lists.
        id = len(self.processes)

        self.processes.append(p)
        self.steps_info.append(step_info_dict)

        if len(self.processes) != len(self.steps_info):
            raise Exception("Process and step info lists differ in length. They are nor reliable.")

        # Start the new process
        p.start()

        # Return the step id. This will be used for future access to the step's process and information.
        return id

    def get_step_info(self, id, **kwargs):
        if not self._is_control_process:
            this_func_name = sys._getframe().f_code.co_name
            result = self._send_call_to_control_process(this_func_name, id, **kwargs)
            return result

        return self.steps_info[id]

    def terminate_step(self, id, timeout=1, **kwargs):
        if not self._is_control_process:
            this_func_name = sys._getframe().f_code.co_name
            return self._send_call_to_control_process(this_func_name, id, timeout=timeout, **kwargs)

        self.processes[id].terminate()

    def step_is_running(self, id, **kwargs):
        if not self._is_control_process:
            this_func_name = sys._getframe().f_code.co_name
            return self._send_call_to_control_process(this_func_name, id, **kwargs)

        step_info = self.get_step_info(id)
        p = self.processes[id]
        if step_info['status'] == 'running' and p.is_alive():
            return True
        else:
            return False

    def wait_for_step(self, id, timeout=None, terminate=True, **kwargs):
        if not self._is_control_process:
            this_func_name = sys._getframe().f_code.co_name
            return self._send_call_to_control_process(this_func_name, id, timeout=timeout, terminate=terminate, **kwargs)

        p = self.processes[id]
        if p.is_alive():
            p.join(timeout)
        if p.is_alive() and terminate:
            p.terminate()

    def interpret_step(self, id, timeout=None, **kwargs):
        self.wait_for_step(id, timeout=timeout)
        step_info =  self.get_step_info(id)
        if not self._is_control_process:
            if step_info["status"]=="finished" and not step_info["exception"]:
                resolution = "[PASSED]"
            elif step_info["status"]=="running" and not step_info["exception"]:
                resolution = "[RUNNING with NO EXCEPTIONS raised]"
            elif step_info["status"]=="running" and step_info["exception"]:
                resolution = "[RUNNING with EXCEPTIONS raised]"
            elif step_info["status"]=="finished" and step_info["exception"]:
                resolution = "[FAILED]"
            else:
                resolution = "[UNKNOWN]"
            print "-----------------------------------------------------"
            print "Analizing parallel step " + str(step_info["name"]) + " : "  + resolution
            print "Step output: "
            print self.merge_lines(str(step_info["stdout"]))
            print "END"
            print "-----------------------------------------------------"
        if step_info['exception']:
            raise Exception(step_info['exception'])

    # Merges identical lines and gives them a counter on the right side.
    def merge_lines(self, string):
        str_list = string.split("\n")
        new_string = str()
        i = 0
        while i < (len(str_list)-1):
            counter = 1
            while str_list[i] == str_list[i+1]:
                counter += 1
                i += 1
            else:
                if counter == 1:
                    new_string += str_list[i] + "\n"
                else:
                    new_string += str_list[i] + " (" + str(counter) + ")\n"
            i += 1
        else:
            if i == (len(str_list)-1):
                new_string += str_list[i]
        return new_string
