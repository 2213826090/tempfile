from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from Core.Report.Live.LiveReporting import LiveReporting

class CheckCmdOutput(DeviceTestStepBase):

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        verdict = "PASSED"
        comment = ""

        DeviceTestStepBase.run(self, context)
        if self._pars.push_iteration:
            LiveReporting.instance().send_start_tc_info(tc_name=self._pars.check_name, iteration=True)

        _, output = self._device.run_cmd(self._pars.cmd, timeout=600)
        expected = str(self._pars.expected_output)

        if output.lower() != expected.lower():
            verdict = "FAILED"
            comment = "cmd '{0}' returned an unexpected result : '{1}' ; expected : '{2}'".format(
                self._pars.cmd, output, expected)
            self._logger.error(comment)
        else:
            comment = "cmd '{0}' returned '{1}' as expected".format(
                self._pars.cmd, expected)

        self._logger.info("The command returned the expected value, no error")

        if self._pars.push_iteration:
            metrics = [{"name": self._pars.check_name, "value": output}]
            LiveReporting.instance().update_running_tc_info(test_info={"metrics": metrics})
            LiveReporting.instance().send_stop_tc_info(verdict=verdict,
                                                       execution_nb=1,
                                                       success_counter=1,
                                                       max_attempt=1,
                                                       acceptance_nb=1,
                                                       tc_comments=comment,
                                                       iteration=True)
