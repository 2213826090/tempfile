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
:since: 04/05/2014
:author: mcchilax
"""

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException
from UseCase.OTC.Communications.Wifi.Scripts.ConfigureAP import get_config_cmds
from UseCase.OTC.Communications.Wifi.Scripts.ConfigureAP import split_config
import time


class ConnectToAP(UseCaseBase):
    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        UseCaseBase.__init__(self, tc_name, global_config)
        self._timeout = self._tc_parameters.get_param_value(param="TIMEOUT",
                                                            default_value=0.0,
                                                            default_cast_type=float)
        self._ap_operation_mode = self._tc_parameters.get_param_value(
            "OPERATION_MODE")
        self._ap_security_mode = self._tc_parameters.get_param_value(
            "SECURITY_MODE")
        self._ap_algorithm = self._tc_parameters.get_param_value("ALGORITHM")
        self._wifi_password = self._tc_parameters.get_param_value("WIFI_PASSWORD")
        self._radius_ipaddr = self._tc_parameters.get_param_value("RADIUS_IP", default_value="Not used")
        self._radius_key = self._tc_parameters.get_param_value("RADIUS_KEY", default_value="Not used")
        self._cert = self._tc_parameters.get_param_value("CERT")
        self._am = self._em.get_artifact_manager("ARTIFACT_MANAGER")
        self._test_apk = self._tc_parameters.get_param_value("TEST_APK",
                                                             default_value="INSTRUMENTATION/ANDROID/ApiTests.apk")
        self._process_name = self._tc_parameters.get_param_value("PROCESS_NAME")
        self._am_extra = self._tc_parameters.get_param_value("AM_EXTRA")
        self.wifi_ap_name = self._device.get_config("WiFi_Support_Ap_Name")
        if self._am_extra:
            if self.wifi_ap_name and "ddwrt" in self._am_extra:
                self._am_extra = self._am_extra.replace("ddwrt", self.wifi_ap_name.strip(), 1)

        # get the access point from bench config
        try:
            self._configurable_ap = self._em.get_computer("AccessPoint")
        except:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                         "Invalid parameter for AccessPoint in Bench Config. "
                                         "Please update the AP Equipment name with: "
                                         "name='AccessPoint' and add the type of the AP "
                                         "'ddwrt_atheros' or 'ddwrt_broadcom'")
        try:
            self.ap_type = self._global_conf.benchConfig.get_parameters("AccessPoint").get_dict()["type"]["value"]
        except:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                    "'type' parameter missing for 'AccessPoint. Please update the AP Equipment with: "
                                    "parameter 'type' with 'value' 'ddwrt_atheros' or 'ddwrt_broadcom'")

        self._pre_reboot_device = False
        self._post_reboot_device = False
        self._post_reboot_nok_device = False
        if self._device.get_config("DisableTcReboot", False,
                                   default_cast_type='str_to_bool'):
            self._pre_reboot_device = self._tc_parameters.get_param_value(
                "PRE_REBOOT", default_value="False",
                default_cast_type="str_to_bool")
            # Reboot the board after test execution
            self._post_reboot_device = self._tc_parameters.get_param_value(
                "POST_REBOOT", default_value="False",
                default_cast_type="str_to_bool")
            # Reboot the board after test execution if test verdict is failure
            self._post_reboot_nok_device = self._tc_parameters.get_param_value(
                "POST_REBOOT_NOK", default_value="False",
                default_cast_type="str_to_bool")

    def set_up(self):
        verify_parameters = [self._ap_operation_mode, self._ap_security_mode,
                             self._timeout, self._am_extra,
                             self._ap_algorithm, self._process_name,
                             self._test_apk]
        for parameter in verify_parameters:
            if not parameter:
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                         "Invalid parameter in test config")
        if not self._am:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "Artifact Repository is not defined in Bench Config")
        if not self._configurable_ap:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "Configurable AP is not defined in Bench Config")
        if self._wifi_password and self._ap_security_mode == "open":
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "Password is set for a unsecured network, please verify the test case")
        local_artifact = self._am.get_artifact(artifact_name=self._test_apk,
                                               transfer_timeout=3600)
        self._device.run_cmd("adb install -r {0}".format(local_artifact), 600)
        if self._cert is not None:
            local_cert = self._am.get_artifact(self._cert)
            self._device.run_cmd("adb push {0} /sdcard/client.p12".format(local_cert), 600)
        self._configurable_ap.init()

        cmd_list = get_config_cmds(self._ap_operation_mode,
                                   self._ap_security_mode,
                                   self._ap_algorithm,
                                   self._wifi_password,
                                   radius_ip=self._radius_ipaddr,
                                   radius_key=self._radius_key,
                                   ap_type=self.ap_type)

        for cmd in split_config(cmd_list):
            self._configurable_ap.run_cmd(cmd)

        try:
            if self.ap_type == "ddwrt_broadcom":
                self._configurable_ap.run_cmd("reboot")
            elif self.ap_type == "ddwrt_atheros":
                self._configurable_ap.run_cmd("rc restart")
        except:
            pass
        time.sleep(30)
        self._configurable_ap.release()
        return Global.SUCCESS, "No errors"

    def run_test(self):
        unlock_command = "adb shell input keyevent 82"
        am_command = "adb shell am instrument {0} -w {1}".format(self._am_extra, self._process_name)
        self._device.run_cmd(unlock_command, self._timeout)
        result, output = self._device.run_cmd(am_command, self._timeout)
        if result == 0 and "OK (1 test)" in output:
            output = ""
        elif result == 0 and ('FAILURES!!!' not in output and "OK (" in output):
            output = ""
        elif output.strip() == "":
            result, output = Global.FAILURE, "Am command has no output"
        else:
            result = Global.FAILURE
        return result, output

    def tear_down(self):
        return Global.SUCCESS, "No errors"

    def finalize(self):
        return Global.SUCCESS, "No errors"
