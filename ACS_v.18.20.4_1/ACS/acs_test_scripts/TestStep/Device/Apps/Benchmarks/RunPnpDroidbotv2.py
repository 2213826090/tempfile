import json
import re
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from Core.Report.Live.LiveReporting import LiveReporting
from ErrorHandling.DeviceException import DeviceException

class RunPnpDroidbotv2(DeviceTestStepBase):

    def _send_iteration(self, res_json):
        if res_json:
            try:
                LiveReporting.instance().update_running_tc_info(test_info={"metrics": res_json["result"]["metrics"]},
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

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        iteration_name = self._testcase_name.split("test_pnp_")[-1]

        LiveReporting.instance().send_start_tc_info(tc_name=iteration_name, iteration=True)

        response = self.request()
        self._send_iteration(response["reply"])

        self._logger.info("RunPnpPerf: Done")

    def request(self):
        """ Issues a droidbot request
            Return:
                Response
        """


        # Run test
        command = "adb shell am instrument -w"
        if self._pars.parameters is not None:
            parameters = str(self._pars.parameters).split(";")
            for parameter in parameters:
                command += " -e " + " ".join(parameter.split(":"))
        command += " com.intel.droidbot/.pnp.%s.%s" % (self._pars.test_type.lower(), self._pars.scenario)

        _, output = self._device.run_cmd(command, timeout=600)

        # check for proper behaviour
        if 'INSTRUMENTATION_CODE: ' not in output:
            raise Exception('adb command failed', {'command': command, 'output': output})

        # read result
        response = {}
        for match in re.finditer(r'INSTRUMENTATION_RESULT: (\S+)=(.*)', output, re.MULTILINE):
            key = match.group(1).strip().lower()
            value = match.group(2).strip()
            try:
                response[key] = json.loads(value)
            except ValueError:
                response[key] = value

        # check for request DroidbotError
        if 'INSTRUMENTATION_CODE: -1' not in output:
            self._logger.error(str(response))
            LiveReporting.instance().send_stop_tc_info(verdict="FAILED",
                                                       execution_nb=1,
                                                       success_counter=1,
                                                       max_attempt=1,
                                                       acceptance_nb=1,
                                                       tc_comments=response,
                                                       iteration=True)
            raise DeviceException('request failed', response)

        self._logger.debug(str(response))
        return response