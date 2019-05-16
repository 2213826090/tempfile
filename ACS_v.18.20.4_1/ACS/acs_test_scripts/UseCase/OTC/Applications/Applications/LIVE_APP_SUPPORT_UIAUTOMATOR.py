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
:summary: Use Case base for running uiautomator for AppSupport tests
:since: 12/05/2014
:author: apalkox
"""

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.Device.UECmd.Imp.Android.Common.Networking.Networking import Networking
import os


class AppSupportRunner(UseCaseBase):

    """
    Android Apps support test implementation
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        UseCaseBase.__init__(self, tc_name, global_config)

        #########################
        # MANDATORY PARAMETERS  #
        #########################
        # Timeout definition for the test we are going to run
        self._timeout = self._tc_parameters.get_param_value(param="TIMEOUT", default_value=0.0,
                                                            default_cast_type=float)
        # Use WIFI definition for the test we are going to run
        self._use_wifi = self._tc_parameters.get_param_value(param="USE_WIFI", default_cast_type="str_to_bool")
        # Path in artifactory to the Jar file necessary for running the test
        self._uiautomator_jar = self._tc_parameters.get_param_value("UIAUTOMATOR_JAR")
        # Package name(inside Jar file) of the test that we are going to run
        self._package_name = self._tc_parameters.get_param_value("PACKAGE_NAME")
        # Class name(inside Jar file) of the test that we are going to run
        self._class_name = self._tc_parameters.get_param_value("CLASS_NAME")
        # Name(inside Jar file) of the test that we are going to run
        self._test_name = self._tc_parameters.get_param_value("TEST_NAME")
        # Use(or not) Google account during test
        self._use_account = self._tc_parameters.get_param_value(param="USE_ACCOUNT", default_cast_type="str_to_bool")

        #################
        # Optional ones #
        #################
        # Account username and password parameters, mandatory only if USE ACCOUNT is True
        self._account_username = self._tc_parameters.get_param_value("ACCOUNT_USERNAME")
        self._account_password = self._tc_parameters.get_param_value("ACCOUNT_PASSWORD")

        # Reboot the board before executing the test
        self._pre_reboot_device = False
        self._post_reboot_device = False
        self._post_reboot_nok_device = False
        if not self._device.get_config("disableTcReboot", False, default_cast_type='str_to_bool'):
            self._pre_reboot_device = self._tc_parameters.get_param_value("PRE_REBOOT", default_value="False",
                                                                          default_cast_type="str_to_bool")
            # Reboot the board after test execution
            self._post_reboot_device = self._tc_parameters.get_param_value("POST_REBOOT", default_value="False",
                                                                           default_cast_type="str_to_bool")
            # Reboot the board after test execution if test verdict is failure
            self._post_reboot_nok_device = self._tc_parameters.get_param_value("POST_REBOOT_NOK", default_value="False",
                                                                               default_cast_type="str_to_bool")
        # Retrieve artifact manager
        self._artifact_manager = self._em.get_artifact_manager("ARTIFACT_MANAGER")

    def _push_jar(self):
        """
        Push jar file on the device
        """
        # Retrieve jar file on the local host
        local_artifact = self._artifact_manager.get_artifact(artifact_name=self._uiautomator_jar, transfer_timeout=3600)
        # Push jar file on /data/local/tmp
        if os.path.isfile(local_artifact):
            result, output = self._device.push(local_artifact, "/data/local/tmp", 600)
            if result != Global.SUCCESS:
                return result, output
        else:
            return Global.BLOCKED, "Cannot find {0}".format(local_artifact)

        return Global.SUCCESS, "No errors"

    def _connect_to_wifi(self):
        """
        Connects DUT to wifi only if USE_WIFI is True
        """
        # Get parameters dictionary for wifi router equipment
        if self._em.get_global_config().benchConfig.has_parameter("NO_SECURITY_WIFI_ROUTER"):
            bench_dic = self._em.get_global_config().benchConfig.get_parameters("NO_SECURITY_WIFI_ROUTER")
        elif self._em.get_global_config().benchConfig.has_parameter("WEP_WIFI_ROUTER"):
            bench_dic = self._em.get_global_config().benchConfig.get_parameters("WEP_WIFI_ROUTER")
        elif self._em.get_global_config().benchConfig.has_parameter("WPA2_WIFI_ROUTER"):
            bench_dic = self._em.get_global_config().benchConfig.get_parameters("WPA2_WIFI_ROUTER")
        elif self._em.get_global_config().benchConfig.has_parameter("WPA_WIFI_ROUTER"):
            bench_dic = self._em.get_global_config().benchConfig.get_parameters("WPA_WIFI_ROUTER")
        else:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                      "Not any wifi router found in Bench Config")
        # Get SSID for Wifi from benchconfig
        ssid = bench_dic.get_param_value("SSID")
        # Get passphrase for Wifi from benchconfig
        passphrase = bench_dic.get_param_value("passphrase")
        # Get security for Wifi from benchconfig
        wifi_security = bench_dic.get_param_value("WIFI_SECURITY")
        # Validate SSID parameter
        if not ssid:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                      "SSID for wifi router is not defined in Bench Config")
        # Validate passphrase parameter
        if not passphrase:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                      "Passphrase for wifi router is not defined in Bench Config")
        # Validate security parameter
        if not wifi_security:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                      "Security for wifi router is not defined in Bench Config")
        # Append '' to SSID if it contains white spaces
        if " " in ssid:
            ssid = "'"+ssid+"'"
        # Append '' to passphrase if it contains white spaces
        if " " in passphrase:
            passphrase = "'"+passphrase+"'"
        # Open Wifi on the DUT and remove current configuration
        wifi_network = Networking(device=self._device)
        wifi_network.set_wifi_power("on")
        wifi_network.wifi_remove_config('all')
        # Set the configuration of Wifi connection on the DUT
        wifi_network.set_wificonfiguration(ssid, passphrase, wifi_security)
        # Connect DUT to the Wifi
        wifi_network.wifi_connect(ssid)

        return Global.SUCCESS, "No errors"

    def set_up(self):
        """
        UC Setup, do your test configuration here
        """
        UseCaseBase.set_up(self)
        result, output = Global.SUCCESS, "No errors"
        # Validate timeout parameter
        if not type(self._timeout) == float:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                      "Invalid parameter TIMEOUT")
        # Validate USE_WIFI parameter
        if not (self._use_wifi == True or self._use_wifi == False):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                      "Invalid parameter USE_WIFI. Possible values: True or False")
        # Validate jar path parameter and initialize jar name parameter
        if not self._uiautomator_jar:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                      "UIautomator jar path is not defined in Test Case")
        elif " " in self._uiautomator_jar:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                      "UIAUTOMATOR_JAR: {0}".format(self._uiautomator_jar))
        else:
            self._jar_name = self._uiautomator_jar.split("/")[-1]
        # Validate package name parameter
        if not self._package_name:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                      "Package name is not defined in Test Case")
        elif " " in self._package_name:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                      "PACKAGE_NAME: {0}".format(self._package_name))
        # Validate class name parameter
        if not self._class_name:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                      "Class name is not defined in Test Case")
        elif " " in self._class_name:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                      "CLASS_NAME: {0}".format(self._class_name))
        # Validate test name parameter
        if not self._test_name:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                      "Test name is not defined in Test Case")
        elif " " in self._test_name:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                      "TEST_NAME: {0}".format(self._test_name))
        # Validate USE_ACCOUNT parameter
        if not (self._use_account == True or self._use_account == False):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                      "Invalid parameter USE_ACCOUNT. Possible values: True or False"\
                                      .format(self._use_account))
        # Validate username and password, only if USE_ACCOUNT is True
        if self._use_account == True:
            if not self._account_username:
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                      "Account username is not defined in Test Case")
            elif " " in self._account_username:
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                        "ACCOUNT_USERNAME: {0}".format(self._account_username))
            if not self._account_password:
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                      "Account password is not defined in Test Case")
            elif " " in self._account_password:
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                        "ACCOUNT_PASSWORD: {0}".format(self._account_password))
        # Validate artifact manager parameter from bench config
        if not self._artifact_manager:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                      "Artifact Repository is not defined in Bench Config")
        # Reboot the DUT
        if self._pre_reboot_device:
            if not self._device.reboot():
                result, output = Global.BLOCKED, "Cannot reboot the device"
        # Push the jar file on the DUT
        if result == Global.SUCCESS:
            result, output = self._push_jar()
        # Connect DUT to Wifi
        if result == Global.SUCCESS and self._use_wifi == True:
            result, output = self._connect_to_wifi()
        elif result == Global.SUCCESS and self._use_wifi == False:
            wifi_network = Networking(device=self._device)
            wifi_network.set_wifi_power("off")

        return result, output

    def run_test(self):
        """
        UC Execution, execute your test here
        """
        UseCaseBase.run_test(self)
        # Send keyevent for unlock the DUT
        unlock_command = "adb shell input keyevent 82"
        self._device.run_cmd(unlock_command, self._timeout)
        # Compute the command for starting Uiautomator test
        if self._use_account == True:
            start_command = \
                "adb shell uiautomator runtest {0} -c {1}.{2}#{3} -e account {4} -e password {5} -e addAccount {6} -s"\
                 .format(self._jar_name, \
                         self._package_name, \
                         self._class_name, \
                         self._test_name, \
                         self._account_username, \
                         self._account_password, \
                         self._use_account)
        else:
            start_command = \
                "adb shell uiautomator runtest {0} -c {1}.{2}#{3} -e addAccount {4} -s"\
                 .format(self._jar_name, \
                         self._package_name, \
                         self._class_name, \
                         self._test_name, \
                         self._use_account)

        # Execute the command
        self._logger.info("Executing command: {0}".format(start_command))
        result, output = self._device.run_cmd(start_command, self._timeout)
        # Interpret the results
        if result == 0 and "OK (1 test)" in output:
            output = "No errors"
        elif result == 0 and ('FAILURES!!!' not in output and "OK (" in output):
            output = "No errors"
        elif output.strip() == "":
            result, output = Global.FAILURE, "Start UIautomator command has no output"
        else:
            result = Global.FAILURE
        # Log an error if test fails
        if result == Global.FAILURE:
            self._logger.error(output)

        return result, output

    def tear_down(self):
        """
        UC TearDown, do your test cleaning here
        """
        UseCaseBase.tear_down(self)
        result, output = Global.SUCCESS, "No errors"
        # Disconnect the device from the Wifi and remove configuration
        if self._use_wifi == True:
            wifi_network = Networking(device=self._device)
            wifi_network.wifi_remove_config('all')
            wifi_network.set_wifi_power("off")
        # Remove jar file
        self._logger.info("Removing file: /data/local/tmp/{0}".format(self._jar_name))
        self._device.run_cmd("adb shell rm /data/local/tmp/{0}".format(self._jar_name), 100)
        # Reboot the device
        if self._post_reboot_device or (not self.test_ok and self._post_reboot_nok_device):
            if not self._device.reboot():
                result, output = Global.FAILURE, "Cannot reboot the device"

        return result, output
