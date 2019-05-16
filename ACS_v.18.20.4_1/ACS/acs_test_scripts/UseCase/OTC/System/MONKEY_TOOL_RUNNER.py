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
:summary: Use Case base for connecting to an AP
:since: 09/05/2014
:author: mcchilax
"""

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException

class MonkeyStressRunner(UseCaseBase):
    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        UseCaseBase.__init__(self, tc_name, global_config)
        self._timeout = self._tc_parameters.get_param_value(param="TIMEOUT",
                                                            default_value=0.0,
                                                            default_cast_type=float)
        self._pakage_name = self._tc_parameters.get_param_value("PACKAGE_NAME")
        self._time_btw = self._tc_parameters.get_param_value(param="TIME_BETWEEN_EVENTS",
                                                             default_value=300,
                                                             default_cast_type=str)
        self._nr_events = self._tc_parameters.get_param_value(param="NUMBER_OF_EVENTS",
                                                              default_value=10000,
                                                              default_cast_type=str)
        self._ignore_crashes = self._tc_parameters.get_param_value(param="IGNORE_CRASHES",
                                                              default_value=False,
                                                              default_cast_type="str_to_bool")
        self._ignore_timeouts = self._tc_parameters.get_param_value(param="IGNORE_TIMEOUTS",
                                                              default_value=False,
                                                              default_cast_type="str_to_bool")
        self._ignore_security_exceptions = self._tc_parameters.get_param_value(param="IGNORE_SECURITY_EXCEPTIONS",
                                                              default_value=False,
                                                              default_cast_type="str_to_bool")
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

    def set_up(self):
        UseCaseBase.set_up(self)
        if not self._time_btw:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "Time Between Events cannot be empty")
        if not self._nr_events:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "Number of Events cannot be empty")
        return Global.SUCCESS, "No errors"

    def run_test(self):
        UseCaseBase.run_test(self)
        unlock_command = "adb shell input keyevent 82"
        self._device.run_cmd(unlock_command, self._wait_btwn_cmd)
        result = Global.FAILURE
        output = "Error in executing Monkey tool"
        cmd = "adb shell monkey"
        if self._pakage_name:
            cmd += " -p "+ self._pakage_name
        cmd += " --throttle " + \
              self._time_btw + " -v " + self._nr_events + " --kill-process-after-error"
        if self._ignore_crashes == True:
            cmd += " --ignore-crashes "
        if self._ignore_timeouts == True:
            cmd += " --ignore-timeouts "
        if self._ignore_security_exceptions == True:
            cmd += " --ignore-security-exceptions "
        monkey_status, monkey_output = self._device.run_cmd(cmd, self._timeout)
        monkey_output_list = monkey_output.split("\n")
        for line in monkey_output_list:
            if "// Monkey finished" in line:
                result = Global.SUCCESS
                output = "Monkey finished"
        return result, output + monkey_output_list[0]

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
