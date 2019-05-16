from Queue import Empty
import json
import yaml
import re
import os
import time
from threading import Thread, Timer, Lock
from Core.PathManager import Paths
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from UtilitiesFWK.Patterns import Cancel
from UtilitiesFWK.Utilities import Global, run_local_command
from Core.Report.SecondaryTestReport import SecondaryTestReport
from acs_test_scripts.Utilities.PnPUtilities import get_math_function
from ErrorHandling.DeviceException import DeviceException
from Core.Report.Live.LiveReporting import LiveReporting

class RunPnpDroidbot(DeviceTestStepBase):

    def _send_iteration(self, it_json):
        if it_json:
            res_json = json.loads(it_json)

            output_json = []

            try:
                scores = res_json["scores"]

                for el in scores:
                    output_json.append({"name": el["name"],
                                        "value": el["value"],
                                        "unit": el["desc"]})

                LiveReporting.instance().send_start_tc_info(tc_name=self._iteration_name, iteration=True)

                LiveReporting.instance().update_running_tc_info(test_info={"metrics": output_json},
                                                                iteration=True)

                LiveReporting.instance().send_stop_tc_info(verdict=res_json["verdict"],
                                                           execution_nb=1,
                                                           success_counter=1,
                                                           max_attempt=1,
                                                           acceptance_nb=1,
                                                           tc_comments="",
                                                           iteration=True)

            except KeyError:
                self._logger.info("Result not find. Maybe scenario mode is used.")



    def hack_report(self, device, result_name, computed_result, unit_score, run_count, expected_success, success_count):
        """
        Hack to add test step comment in AWR database for now
        Test steps result are not yet available in AWR database (and not planned...)
        """
        external_tc_report = SecondaryTestReport(device.get_report_tree().get_report_path())

        run_msg = "Run: {0} - Success: {1}/{2} - ".format(run_count, success_count, expected_success)
        if computed_result is None:
            verdict = external_tc_report.verdict.FAIL
            message = "No score found"
        else:
            message = "Score: {0} ({1})".format(computed_result, unit_score)
            if success_count < expected_success:
                verdict = external_tc_report.verdict.FAIL
            else:
                verdict = external_tc_report.verdict.PASS

        self.__json.append({"name": result_name,
                            "value": computed_result,
                            "unit": unit_score})

        LiveReporting.instance().update_running_tc_info(test_info={"metrics": self.__json})

        external_tc_report.add_result(result_name,
                                      verdict, run_msg + message,
                                      self._testcase_name,
                                      self._conf.tc_order)

        return (verdict == external_tc_report.verdict.PASS), run_msg


    def get_result_value_and_desc(self, droidbot_result, score_name):
        """
        example of droidbot json output:
        {"test": "com.intel.droidbot.perf.BenchmarkPI#test",
        "run": 1,
        "scores": [{"name": "score", "lib": true, "value": 306, "desc": "ms"},
                   {"name": "duration", "lib": true, "value": 5, "desc": "Seconds"}]}
        This method will extract a score type from "scores" key dict
        :type score_name: str
        :param score_name: value will be extracted according to this score name
        """
        score_value = None
        desc_value = None

        run_count = droidbot_result.get("numRun")
        expected_run = droidbot_result.get("numExpectedRuns")

        scores = droidbot_result.get("scores")
        if scores:
            for score in scores:
                if score.get("name", "") == score_name:
                    score_value = score.get("value")
                    desc_value = score.get("desc")
        return score_value, desc_value, run_count, expected_run


    def computeScores(self, context, device, testcase_name, json_result):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        success = False

        # get operation type (applied on multiple scores)
        operation = context.get_info("droidbotOperation")
        if not operation:
            operation = "MEDIAN"

        # get scores description path
        droidbot_scores_path = os.path.join(os.path.abspath(Paths.EXECUTION_CONFIG), "SI", "pnp",
                                            "perf", "TC", "script", "droidbotScores.yaml")

        # compute result
        unit_score = ""
        total_run = 0
        expected_run = 0

        functor = get_math_function(operation)

        # load json results
        if json_result:
            data = json.loads(json_result)
            if data:
                # get score list from yaml file
                with open(droidbot_scores_path, "r") as f:
                    all_scores = yaml.load(f.read())
                try:
                    tc_scores = all_scores['TestCases'][testcase_name]
                except KeyError:
                    tc_scores = None
                    success = True

                if tc_scores:
                    for score_name in tc_scores:
                        result_name = tc_scores[score_name]
                        raw_results = []
                        # retrieve score for each iteration result
                        for res in data:
                            score, unit_score, total_run, expected_run = self.get_result_value_and_desc(res, score_name)
                            if score:
                                raw_results.append(score)
                        if raw_results:
                            computed_result = str(functor(raw_results))
                            self.ts_verdict_msg = "Computed {0} for {3} : {1}({2})".format(operation.lower(),
                                                                                           computed_result,
                                                                                           unit_score, score_name)
                            self._logger.info(self.ts_verdict_msg)
                            score_as_str = ";".join(str(x) for x in raw_results)
                            self.ts_verdict_msg = "Scores in {0} : {1}".format(unit_score, score_as_str)
                            self._logger.info(self.ts_verdict_msg)

                            # in order to store data in AWR database
                            self.hack_report(device, result_name, computed_result, unit_score,
                                             total_run, expected_run, len(raw_results))
                        else:
                            self._logger.warning("Cannot find {0} in droidbot results: skipping".format(score_name))
                    success = True
                else:
                    self._logger.warning("Cannot find {0} in {1}".format(testcase_name, droidbot_scores_path))
                    self.ts_verdict_msg = "No result"
            else:
                self._logger.warning("Data can't be loaded from droidbot results")
                self.ts_verdict_msg = "No result"
        else:
            self._logger.warning("Result from Droidbot doest not exist, cannot compute values")
            self.ts_verdict_msg = "No result"

        return success


    def droidbot_watchdog(self, cancelable, min_timeout, logtag):
        timer = Timer(min_timeout, cancelable.cancel)
        timer.start()

        while not cancelable.is_canceled:
            messages = self._device.get_device_logger().get_message_triggered_status(logtag)
            if messages:
                timer.cancel()
                self.__mutex.acquire()
                droidbot_timer = messages[-1].split(' ')[-1]
                if not droidbot_timer:
                    droidbot_timer = "0"
                self.__droidbot_timer = min_timeout + int(droidbot_timer)/1000
                self.__mutex.release()
                self._device.get_device_logger().reset_trigger_message(logtag)
                timer = Timer(self.__droidbot_timer, cancelable.cancel)
                timer.start()
                self._logger.info("Droidbot timer set to {0} seconds".format(self.__droidbot_timer))

            time.sleep(1)

    def _dequeue(self, q):
        """
        Dequeue the stdout generated by command execution
        """
        data_list = []
        qsize = q.qsize
        while qsize > 0:
            try:
                data = unicode(q.get_nowait())
                data = data.strip('\n')
                data_list.append(data)
                qsize = q.qsize
            except Empty:
                break
            except UnicodeDecodeError:
                pass
        return data_list

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """

        DeviceTestStepBase.run(self, context)

        self.__json = []
        self.__mutex = Lock()
        # ------------------------- get params ----------------------------

        droidbot_jar_path = context.get_info("JAR_FILE_PATH_DROIDBOT")
        pnptests_jar_path = context.get_info("JAR_FILE_PATH_PNP_TESTS")

        droidbot_extra_args = self._pars.extra_opts
        self._droidbot_test_name = self._pars.test_name
        self.__droidbot_timer = self._pars.timeout

        self._iteration_name = self._droidbot_test_name.split('.')[-1]

        # get test case name, based on its path
        testcase_name = self._testcase_name.replace("\\", "/").split("/")[-1]

        # add each iteration in external report
        self._logger.info("RunPnpPerf: Starting")

        cmd = "adb shell uiautomator runtest {0} {1} -s -c {2}".format(droidbot_jar_path, pnptests_jar_path,
                                                                       self._droidbot_test_name)

        if droidbot_extra_args:
            cmd += " " + str(droidbot_extra_args)
        # ------------------------- run droidbot ----------------------------

        self._logger.info("Droidbot cmd: {}".format(cmd))

        # Start watchdog for droidbot
        cancelable = Cancel()
        min_timeout = 60
        logtag = "DROIDBOT_TIMER"
        self._device.get_device_logger().add_trigger_message(logtag)
        thread = Thread(target=self.droidbot_watchdog, args=(cancelable, min_timeout, logtag))
        thread.start()

        # run droidbot command, using cancel object from watchdog
        regex_dict_result = "INSTRUMENTATION_STATUS: result=(?P<dict_result>{.*})"
        regex_dict_report = "INSTRUMENTATION_STATUS: report=(?P<dict_report>{.*})"
        _proc, q = run_local_command(cmd)
        output = ""
        self.__droidbot_timer *= 60
        verdict = Global.FAILURE
        while self.__droidbot_timer > 0 and _proc is not None and _proc.poll() is None:
            # pylint: disable=C0103
            # Agree to keep t0 & t1 variable names
            t0 = time.time()
            time.sleep(0.2)
            t1 = time.time()
            self.__mutex.acquire()
            self.__droidbot_timer -= (t1 - t0)
            self.__mutex.release()
            stdout_data_list = self._dequeue(q)
            for el in stdout_data_list:
                output += el + "\n"
                self._logger.debug("uiautomator_output : %s" % el)
                if "report" in el:
                    matches_res = re.compile(regex_dict_report).search(el)
                    if matches_res:
                        content = matches_res.group('dict_report')
                        #FIXME: Need to convert verdict into Global.Status
                        verdict = Global.SUCCESS #if content["verdict"] == "PASS" else
                elif "result" in el:
                    matches_res = re.compile(regex_dict_result).search(el)
                    content = None
                    if matches_res:
                        content = matches_res.group('dict_result')
                    self._send_iteration(content)

        # droidbot command is done : call cancel() to stop watchdog
        cancelable.cancel()

        if verdict != Global.SUCCESS:
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "RunPnpPerf: run_cmd failed")

        # ------------------------- format output ----------------------------

        # json dict is printed in logs with all test output data

        # example :  OK (1 test)
        # this log will indicate that all tests have been successfully executed
        regex_result_ok = re.compile("OK \(\d+ test\)")
        results = []

        success = False
        for line in output.split('\n'):
            # retrieve test output info
            matches_res = re.compile(regex_dict_result).search(line)
            if matches_res:
                content = matches_res.group('dict_result')
                try:
                    # serialize string as dict
                    json_test_result = json.loads(content)
                    results.append(json_test_result)
                except ValueError as error:
                    self._logger.warning("error during loading {0} : {1}".format(content, error))
            elif regex_result_ok.match(line):
                success = True

        # ------------------------- analyze output ----------------------------
        if results:
            # get artifacts from device
            artifacts_dir = os.path.join(self._device.get_report_tree().get_report_path(), self._testcase_name,
                                         "{0}_artifacts".format(testcase_name))
            if not os.path.exists(artifacts_dir):
                os.makedirs(artifacts_dir)
            for result in results:
                if 'artifacts' in result:
                    for artifact in result['artifacts']:
                        artifact_dir = os.path.join(artifacts_dir, os.path.join(*artifact.split('/')[-3:-1]))
                        if not os.path.exists(artifact_dir):
                            os.makedirs(artifact_dir)
                        adb_command = "adb pull " + artifact + " " + os.path.join(artifact_dir, artifact.split('/')[-1])
                        self._device.run_cmd(adb_command, 30)
            # write results in json file
            # example : _Report/ReportCampaign/PHONE1/test_case_name/results.json
            test_data_dir = os.path.join(self._device.get_report_tree().get_report_path(), self._testcase_name,
                                         "{0}_result".format(testcase_name))
            # create folder if it does not exist
            if not os.path.exists(test_data_dir):
                os.makedirs(test_data_dir)

            # dump json to string
            try:
                json_string = json.dumps(results)
            except Exception as e:
                msg = "Error while dumping json results to string ({0})".format(e)
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, "RunPnpPerf: {0}".format(msg))

            # dump data in file
            score_file_path = os.path.join(test_data_dir, "results.json")
            with open(score_file_path, 'w') as outfile:
                outfile.write(json_string)

            # compute scores and add them to secondary report
            success = success and self.computeScores(context, self._device, testcase_name, json_string)

        else:
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "RunPnpPerf: no results returned, test has failed!")

        if not success:
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "RunPnpPerf: check logs, test has not been executed properly")
        self._logger.info("RunPnpPerf: Done")
