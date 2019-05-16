"""
:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related
to the source code ("Material") are owned by Intel Corporation or its
suppliers or licensors. Title to the Material remains with Intel Corporation
or its suppliers and licensors. The Material contains trade secrets and
proprietary and confidential information of Intel or its suppliers and
licensors.

The Material is protected by worldwide copyright and trade secret laws and
treaty provisions. No part of the Material may be used, copied, reproduced,
modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual
property right is granted to or conferred upon you by disclosure or delivery
of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

:organization: INTEL MCG PSI
:since: 2011/09/08
:author: asebbane
"""

import time

from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT
from UtilitiesFWK.Utilities import Global


class UiAction(object):

    """
    The base class for I{actions} implementation.

    Subclasses should override the C{_do}, C{_redo} and C{_undo} methods.
    The helper methods C{can_undo} and C{can_redo} may also be overridden
    when needed.
    """

    __TIMEOUT_NO_FAILURE = "NO_FAILURE"
    SLEEP_BEFORE_EXECUTION = True

    def __init__(self, monkey, default_timeout):
        self._can_undo = False
        self._can_redo = False
        self.__result = None
        self.__listeners = []
        self._parameters = None
        self.__monkey = monkey
        self.__triggered_logs = None
        self.__trig_logs_condition = None
        self.__log_message = None
        self.__parameters_processed = False
        self.__iterations = 1
        self.__stop_iter_on_first_success = False

        # split timeout if it is a str
        if default_timeout is not None and isinstance(default_timeout, str):
            (timeout, fail_on_timeout) = self.split_timeout(default_timeout)
            self.__timeout = float(timeout)
            self.__fail_on_timeout = fail_on_timeout
        else:
            self.__timeout = float(default_timeout)
            self.__fail_on_timeout = True

        self._default_timeout = self.__timeout

    def get_name(self):
        """
        Returns the action's name
        """
        return self.__class__.__name__

    def get_monkey(self):
        """
        Returns the monkey instance.
        """
        return self.__monkey

    def split_timeout(self, timeout_params):
        """
        Split timeout_params into timeout + fail_on_timeout

        :param timeout_params: timeout str as defined in xml
        :type timeout_params: String
        :rtype: Tuple
        :return: a tuple containing timeout as String and
                 fail_on_timeout as Boolean
        """
        fail_on_timeout = True
        timeout = timeout_params

        params = timeout_params.split(';')
        if len(params) > 0:
            timeout = params[0]
        if len(params) > 1:
            str_fot = params[1]
            if str_fot == UiAction.__TIMEOUT_NO_FAILURE:
                fail_on_timeout = False

        return (timeout, fail_on_timeout)

    def get_fail_on_timeout(self):
        """
        Returns action's behavior when timeout is reached

        :rtype: Boolean
        :return: True if action has to fail on timeout, False otherwise
        """
        return self.__fail_on_timeout

    def set_fail_on_timeout(self, fail):
        """
        Set the action's behavior on timeout.

        :param fail: True if action has to fail on timeout, False otherwise
        :type fail: Boolean
        """
        self.__fail_on_timeout = fail

    def get_timeout(self):
        """
        Returns action's timeout.

        :rtype: object
        :return: timeout in second as an integer or a str
        """
        return self.__timeout

    def set_timeout(self, timeout):
        """
        Set the action's timeout.

        :param timeout: Timeout in seconds
        :type timeout: float
        """
        self.__timeout = float(timeout)

    def process_parameters(self):
        """
        Processes the parameters by affecting this object's
        attributes to their corresponding value in the parameter list.
        This method is intended for overriding.
        This method delegate its job to the C{_do_process_parameters}
        method.
        """
        self._do_process_parameters()
        self.__parameters_processed = True

    def parameters_processed(self):
        """
        Returns a C{bool} indicating whether the parameters have been
        processed or not.
        :rtype: bool
        :return:
            - C{True} if the parameters have been processed
            - C{False} otherwise
        """
        return self.__parameters_processed

    def _do_process_parameters(self):
        """
        Actually processes the parameters by affecting this object's
        attributes to their corresponding value in the parameter list.
        This method is intended for overriding.
        """
        pass

    def get_stop_iter_on_first_success(self):
        """
        Return true if we have to stop the iteration at the first success.
        Else false.
        :rtype: bool
        :return: True if we have to stop the iteration on first success
        """
        return self.__stop_iter_on_first_success

    def set_stop_iter_on_first_success(self, value):
        """
        Set true if we have to stop the iteration at the first success.
        Else false.
        :type value: bool
        :param iteration: true if we have to stop iteration on first success
        """
        self.__stop_iter_on_first_success = value

    def get_iteration_count(self):
        """
        Returns the number of times that this command
        should be executed. The action of repeating the
        command is not of this object's responsibility.
        :rtype: int
        :return: number of times that this command should be executed
        """
        return self.__iterations

    def set_iteration_count(self, iterations):
        """
        Sets the number of times that this command
        should be executed to the given value.
        The action of repeating the
        command is not of this object's responsibility.
        :type iteration: int
        :param iteration: number of times that this command should be executed
        """
        self.__iterations = iterations

    def get_trigger_log(self):
        """
        Returns a boolean indicating whether this action
        shall trigger LogCat reader or not.
        :rtype: bool
        :return:
            - The log tag that will trigger the log cat reader if
                a
            - C{None} if nothing has to be triggered.
        """
        return self.__triggered_logs

    def set_trigger_log(self, shall_trigger_log):
        """
        Sets the I{shall trigger log} indicator of this object
        to the given value.
        The C{get_trigger_log} parameter corresponds to the tag
        that will trigger the log cat reader.
        :type get_trigger_log: str
        :param get_trigger_log: the new value for I{shall trigger log}
        """
        if shall_trigger_log is not None:
            self.__triggered_logs = []
            if "_-_" in shall_trigger_log:
                # back compatible w/ old AND syntax
                for log in shall_trigger_log.split("_-_"):
                    self.__triggered_logs.append(log.strip())
                self.__trig_logs_condition = "AND"
            elif "_AND_" in shall_trigger_log:
                for log in shall_trigger_log.split("_AND_"):
                    self.__triggered_logs.append(log.strip())
                self.__trig_logs_condition = "AND"
            elif "_OR_" in shall_trigger_log:
                for log in shall_trigger_log.split("_OR_"):
                    self.__triggered_logs.append(log.strip())
                self.__trig_logs_condition = "OR"
            else:
                self.__triggered_logs.append(shall_trigger_log)

    def get_trig_logs_condition(self):
        return self.__trig_logs_condition

    def set_log_message(self, message):
        self.__log_message = message

    def get_log_message(self):
        return self.__log_message

    def set_parameters(self, parameters):
        """
        Sets this action's parameters to the given list.
        It is the class responsibility to the subclasses
        to describes the expected content of the parameters.

        :type parameters: list
        :param parameters: the parameter list
        """
        self._parameters = parameters

    def get_parameters(self):
        """
        Returns the list of all parameters value
        """
        if not self.parameters_processed():
            self.process_parameters()
        return self._parameters

    def _set_result(self, result):
        """
        Sets this action's result to the given value
        :type result: object
        :param result: this action's result
        """
        self.__result = result

    def do(self):
        """
        Runs this action and notify listeners.
        """

        begin_time = time.time()
        result = self._do()
        LOGGER_TEST_SCRIPT.debug("%s execution time: %.2f", self.__class__.__name__, (time.time() - begin_time))
        self.notify()
        self._set_result(result)
        return result

    def _do(self):
        """
        Actually runs this action.
        This method is intended for overriding.
        """
        self._set_result(Global.SUCCESS)
        return Global.SUCCESS

    def undo(self):
        """
        Undoes operations that have been performed in a previous
        call to the C{do} method.
        """
        self._undo()
        self.notify()

    def _undo(self):
        """
        Actually undoes this action.
        This method is intended for overriding.
        """
        pass

    def redo(self):
        """
        Re-runs operations that have been performed in a previous
        call to the C{do} method.
        """
        result = self._redo()
        self.notify()
        return result

    def _redo(self):
        """
        Actually re-runs operations.
        This method is intended for overriding.
        """
        return Global.SUCCESS

    def can_redo(self):
        """
        Returns a C{bool} indicating whether this action can
        be re-done or not.
        :rtype: bool
        :return:
            - C{True} if this action can be re-done
            - C{False} otherwise.
        """
        return self._can_redo

    def can_undo(self):
        """
        Returns a C{bool} indicating whether this action can
        be undone or not.
        :rtype: bool
        :return:
            - C{True} if this action can be undone
            - C{False} otherwise.
        """
        return self._can_undo

    def get_result(self):
        """
        Returns the result of this action's last execution.
        :rtype: object
        :return: the result
        """
        return self.__result

    def notify(self):
        """
        Notifies this object's listener for some changes.
        """
        for listener in self.__listeners:
            listener.result_changed(self)

    def add_listener(self, listener):
        """
        Adds the given C{listener} to this action.
        :type listener: ActionResultListener
        :param listener: the listener instance that will be notified
        about this action's last result.
        """
        if listener is not None:
            self.__listeners.append(listener)

    def remove_listener(self, listener):
        """
        Removes the given listener from this action's
        listener list.
        :type listener: ActionResultListener
        :param listener: the listener instance to remove.
        """
        try:
            self.__listeners.remove(listener)
        except ValueError:
            # No such value, ignore the error
            pass

