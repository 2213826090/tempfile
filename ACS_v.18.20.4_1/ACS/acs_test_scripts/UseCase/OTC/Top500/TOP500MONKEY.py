"""

:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

:organization: INTEL OTC ANDROID CORE QA
:summary: Running Monkey test on top 500 Android Applications
:since: 26/05/2014
:author: mcchilax
"""

import os
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException
from Core.Report.SecondaryTestReport import SecondaryTestReport
import zipfile

TMP_LOCATION = "/tmp/Android_applications"


class Top500Applications(UseCaseBase):
    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        UseCaseBase.__init__(self, tc_name, global_config)
        self._timeout = self._tc_parameters.get_param_value(param="TIMEOUT",
                                                            default_value=0.0,
                                                            default_cast_type=float)
        self.__applications_location = self._tc_parameters.get_param_value("APPLICATIONS_ZIP")
        self._time_btw = self._tc_parameters.get_param_value(param="TIME_BETWEEN_EVENTS",
                                                             default_value=200,
                                                             default_cast_type=str)
        self._nr_events = self._tc_parameters.get_param_value(param="NUMBER_OF_EVENTS",
                                                              default_value=1500,
                                                              default_cast_type=str)
        self._pre_reboot_device = False
        self._post_reboot_device = True
        self._post_reboot_nok_device = True
        self._disable_reboot = self._device.get_config("disableTcReboot", False,
                                                       default_cast_type='str_to_bool')
        if not self._disable_reboot:
            self._pre_reboot_device = self._tc_parameters.get_param_value(
                "PRE_REBOOT", default_value="False",
                default_cast_type="str_to_bool")
            # Reboot the board after test execution
            self._post_reboot_device = self._tc_parameters.get_param_value(
                "POST_REBOOT", default_value="True",
                default_cast_type="str_to_bool")
            # Reboot the board after test execution if test verdict is failure
            self._post_reboot_nok_device = self._tc_parameters.get_param_value(
                "POST_REBOOT_NOK", default_value="False",
                default_cast_type="str_to_bool")
        self._app_list = {}
        self._tc_report = SecondaryTestReport(self._device.get_report_tree().get_report_path())
        self._system_api = self._device.get_uecmd("System")
        self._app_api = self._device.get_uecmd("AppMgmt")

    def set_up(self):
        UseCaseBase.set_up(self)
        if not self.__applications_location:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "Applications zip cannot be empty")
        if not os.path.isfile(self.__applications_location):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "Applications zip is not a valid file")

        if not self._time_btw:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "Time Between Events cannot be empty")
        if not self._nr_events:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "Number of Events cannot be empty")
        self._logger.info("")
        self._logger.info("Unzipping Android applications...")
        self.__unzip_applications()
        return Global.SUCCESS, "No errors"

    def run_test(self):
        UseCaseBase.run_test(self)
        unlock_command = "adb shell input keyevent 82"
        result = Global.FAILURE
        output = "Error in executing Monkey tool"
        for application in self._app_list:
            self._logger.info("Installing application: {0}".format(application))
            self._app_api.install_device_app(self._app_list[application])
            wait_time = (int(self._nr_events) * int(self._time_btw) + 10000) / 1000
            self._device.run_cmd(unlock_command, self._wait_btwn_cmd)
            cmd = "adb shell monkey -p " + application + " --throttle " + self._time_btw + \
                  " --ignore-crashes " + "--ignore-timeouts " + "--ignore-security-exceptions " + \
                  " -v " + self._nr_events + " --kill-process-after-error"
            self._logger.info("")
            self._logger.info("Starting monkey for {0} seconds".format(wait_time))
            monkey_status, monkey_output = self._device.run_cmd(cmd, wait_time)
            monkey_output_list = monkey_output.split("\n")
            test_successful = False
            if "// Monkey finished" in monkey_output_list[-1]:
                test_successful = True
                self._tc_report.add_result(application,
                                           self._tc_report.verdict.PASS,
                                           monkey_output_list[0],
                                           self.get_name(),
                                           self.tc_order)
            if not test_successful:
                self._tc_report.add_result(application,
                                           self._tc_report.verdict.FAIL,
                                           monkey_output_list[-1],
                                           self.get_name(),
                                           self.tc_order)
            if test_successful:
                result = Global.SUCCESS
                output = "Monkey finished"
            else:
                result = Global.FAILURE
                output = "Monkey failed on some tests, please see detailed report for result"
            self._app_api.uninstall_device_app(application, timeout=100)
        return result, output

    def tear_down(self):
        UseCaseBase.tear_down(self)
        result, output = Global.SUCCESS, ""
        if not self._disable_reboot:
            rebooted = self._device.reboot()
            if rebooted:
                result = Global.SUCCESS
                output = "Board rebooted successfully."
            else:
                result = Global.FAILURE
                output = "An error occurred when rebooting the board."
        return result, output

    def finalize(self):
        result = Global.SUCCESS
        output = ""
        self._logger.info("Removing applications...")
        import shutil
        try:
            shutil.rmtree(TMP_LOCATION)
        except OSError, e:
            result = Global.FAILURE
            output = e
        return result, output

    def __unzip_applications(self):
        with zipfile.ZipFile(self.__applications_location) as zf:
            zf.extractall(TMP_LOCATION)
        for f in os.listdir(TMP_LOCATION):
            if f.endswith(".apk"):
                package_name = f.split("_")[0]
                self._app_list[package_name] = TMP_LOCATION + os.sep + f
