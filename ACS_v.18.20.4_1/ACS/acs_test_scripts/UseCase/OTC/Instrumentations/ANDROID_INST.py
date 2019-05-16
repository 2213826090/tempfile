# pylint: disable=invalid-name
"""

:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
The source code contained or described herein and all documents related
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

:since: 2014-10-02
:author: sfusilie
"""
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global, get_timestamp
import os
import re


class AndroidInstrumentation(UseCaseBase):
    """
    ABT Android instrumentation test implementation
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        UseCaseBase.__init__(self, tc_name, global_config)

    def initialize(self):
        """
        Process the **<Initialize>** section of the XML file and execute defined test steps.
        """
        UseCaseBase.initialize(self)
        result, output = Global.SUCCESS, ""

        # Test data that will be installed on the device
        self._test_data = {}
        self._rm_list = {}
        self._rm_list['REMOVE_BINARY'] = {}
        #########################
        # MANDATORY PARAMETERS  #
        #########################
        # Timeout definition for the test we are going to run
        # AKA "timeout" in ATF
        self._timeout = self._tc_parameters.get_param_value(param="TIMEOUT", default_value=0.0,
                                                            default_cast_type=float)
        # Process name of the application to execute
        # AKA "test_script" in ATF
        self._process_name = self._tc_parameters.get_param_value("PROCESS_NAME")

        #################
        # Optional ones #
        #################
        # AM parameters to start the test (within the process)
        # AKA "am_extra" in ATF
        self._am_extra = self._tc_parameters.get_param_value("AM_EXTRA", default_value="")
        self.wifi_ap_name = self._device.get_config("WiFi_Connection_Ap_Name")
        self.wifi_ap_passwd = self._device.get_config("WiFi_Connection_Passwd")
        self.wifi_connection_test_page = self._device.get_config("WiFi_Connection_TestPage")
        if self._am_extra:
            if self.wifi_ap_name and "Android Core QA" in self._am_extra:
                self._am_extra = self._am_extra.replace("Android Core QA", self.wifi_ap_name.strip(), 1)
            if self.wifi_ap_passwd and "AndroidQA" in self._am_extra:
                self._am_extra = self._am_extra.replace("AndroidQA", self.wifi_ap_passwd.strip(), 1)
            if self.wifi_connection_test_page and "www.google.ro" in self._am_extra:
                self._am_extra = self._am_extra.replace("www.google.ro", self.wifi_connection_test_page.strip(), 1)
        # APKs to be installed, may be split by ";"
        # AKA "apks" in ATF
        self._apks = self._tc_parameters.get_param_value("APKS")

        # TODO: put those features in the fwk
        # Reboot the board before executing the test
        # AKA "PRE_REBOOT" in ATF
        self._pre_reboot_device = False
        self._post_reboot_device = False
        self._post_reboot_nok_device = False
        self._disable_tc_reboot = self._device.get_config("disableTcReboot", False, default_cast_type='str_to_bool')
        if not self._disable_tc_reboot:
            self._pre_reboot_device = self._tc_parameters.get_param_value("PRE_REBOOT", default_value="False",
                                                                          default_cast_type="str_to_bool")
            # Reboot the board after test execution
            # AKA "POST_REBOOT" in ATF
            self._post_reboot_device = self._tc_parameters.get_param_value("POST_REBOOT", default_value="False",
                                                                           default_cast_type="str_to_bool")
            # Reboot the board after test execution if test verdict is failure
            # AKA "POST_REBOOT_NOK" in ATF
            self._post_reboot_nok_device = self._tc_parameters.get_param_value("POST_REBOOT_NOK", default_value="False",
                                                                               default_cast_type="str_to_bool")
        # Test file to be deployed on the target, may be split by ";"
        # AKA "media_file" in ATF
        self._test_files = self._tc_parameters.get_param_value("TEST_FILES")
        # Destination of test file on the device, may be split by ";" and must contain
        # as much as destination than test files
        # AKA "adb_destination" in ATF
        self._test_files_dest = self._tc_parameters.get_param_value("TEST_FILES_DEST")
        # Set some properties on the device, if needed, may be split by ";"
        # AKA "set_prop" in ATF
        self._set_props = self._tc_parameters.get_param_value("SET_PROPS")
        # Retrieve data from the device after test, and put it in the report
        # AKA "retrieve_artefacts" in ATF
        self._retrieve_artifact = self._tc_parameters.get_param_value("RETRIEVE_ARTIFACT")
        # Result of run test method
        self._run_test_result = Global.SUCCESS

        # TODO: Handle these parameters in that TC or in the fwk to install specific test bin/apk of that test
        # AKA "binaries" in ATF
        self._binary_files = self._tc_parameters.get_param_value("BINARY")
        # Remove at the setup time the BINARY file available on the device
        # and restore back at the end of the test
        # AKA "rm_pkg" in ATF
        self._remove_binary_files = self._tc_parameters.get_param_value("REMOVE_BINARY")
        self._pre_factory_reset_device = self._tc_parameters.get_param_value("PRE_TEST_FACTORY_RESET",
                                                                             default_value="False",
                                                                             default_cast_type="str_to_bool")
        # Retrieve artifact manager
        self._artifact_manager = self._em.get_artifact_manager("ARTIFACT_MANAGER")

        return result, output

    def _rm_binary(self):
        '''
        This function will check inside ACS test cases XML to see if there
        are files(REMOVE_BINARY field) to be removed from DUT
        '''
        if self._remove_binary_files:
            for element in self._remove_binary_files.strip().split(';'):
                status, status_msg = self._device.pull(element, "/tmp/", 600)
                local_path = os.path.join('/tmp/', os.path.basename(element))
                self._rm_list['REMOVE_BINARY'][local_path] = element
                self._device.set_filesystem_rw()
                cmmd = "adb shell rm -rf %s" % element
                status, status_msg = self._device.run_cmd(cmmd, 20)
                if element.startswith('/system/priv-app/') and element.endswith('.apk') \
                        and self._disable_tc_reboot:
                    self._device.reboot()
                else:
                    status, status_msg = self._device.run_cmd("adb shell sync", 5)
        return Global.SUCCESS, ""

    def _uninstall_files(self):
        # TODO: uninstall APKs

        # Uninstall test files
        for k, v in self._test_data:
            file_path_on_device = "{0}{1}".format(v, os.path.split(k)[1])
            self._device.run_cmd("adb shell rm {0}".format(file_path_on_device), 100)
        return Global.SUCCESS, ""

    def _install_files(self):
        # Install apks
        if self._apks:
            self._apks = [x.strip() for x in self._apks.split(";")]

            for apk in self._apks:
                # Download APKs
                local_artifact = self._artifact_manager.get_artifact(artifact_name=apk, transfer_timeout=3600)
                self._device.run_cmd("adb install -r {0}".format(local_artifact), 600)

        # Install test files
        if self._test_files:
            self._test_files = [x.strip() for x in self._test_files.split(";")]
            if self._test_files_dest:
                self._test_files_dest = [x.strip() for x in self._test_files_dest.split(";")]
                self._test_data = [(el, self._test_files_dest[idx] if (idx + 1) <= len(self._test_files_dest)
                else self._test_files_dest[-1]) for idx, el in enumerate(self._test_files)]
            else:
                self._test_data = [(x, "/mnt/sdcard/") for x in self._test_files]

            for k, remotepath in self._test_data:
                local_artifact = self._artifact_manager.get_artifact(artifact_name=k, transfer_timeout=3600)
                if os.path.isfile(local_artifact):
                    remotepath = self._device.get_device_os_path().join(remotepath, os.path.basename(local_artifact))
                    result, output = self._device.push(local_artifact, remotepath, 600)
                    if result != Global.SUCCESS:
                        return result, output
                else:
                    return Global.FAILURE, "Cannot find {0} in {1}".format(k)

        return Global.SUCCESS, ""

    def _set_properties(self):
        if self._set_props:
            for prop in [x.strip() for x in self._set_props.split(";")]:
                key, value = prop.split("=", 1)
                self._device.set_property_value(key.strip(), value.strip())
        return Global.SUCCESS, ""

    def _retrieve_artifacts(self):
        # TODO: if there is only an extension specified (e.g. png) ATF will
        # retrieve all the png files from the / mnt / sdcard / directory

        if self._retrieve_artifact:
            for artifact in [x.strip() for x in self._retrieve_artifact.split(";")]:
                try:
                    # pull artifact to reporting folder
                    artifact_report_name = '{0}_{1}_{2}'.format(get_timestamp(), self.get_name().split(os.sep)[-1],
                                                                artifact.split("/")[-1])
                    artifacts_folder = "FAILURE_ARTIFACTS"
                    pull_dest_path = os.path.join(self.reporting_folder_path, artifacts_folder, artifact_report_name)
                    self._device.pull(artifact, pull_dest_path, 600)
                    # remove artifact from device
                    remove_cmd = "adb shell rm {0}".format(artifact)
                    self._device.run_cmd(remove_cmd, 100)
                except:
                    # log only a message to not change the test result
                    self._logger.warning("Exception was encountered when retrieving artifact {0}".format(artifact))

    def set_up(self):
        """
        UC Setup, do your test configuration here
        """
        UseCaseBase.set_up(self)
        result, output = Global.SUCCESS, ""

        if not self._process_name:
            result, output = Global.FAILURE, "Please specify PROCESS_NAME parameter"

        if not self._am_extra:
            result, output = Global.FAILURE, "Please specify AM_EXTRA parameter"

        if not self._timeout:
            result, output = Global.FAILURE, "Please specify TIMEOUT parameter"

        if result == Global.SUCCESS:
            result, output = self._rm_binary()

        if result == Global.SUCCESS:
            result, output = self._install_files()

        if result == Global.SUCCESS:
            result, output = self._set_properties()

        if result == Global.SUCCESS and self._pre_reboot_device:
            if not self._device.reboot():
                result, output = Global.FAILURE, "Cannot reboot the device"

        return result, output

    def run_test(self):
        """
        UC Execution, execute your test here
        """
        UseCaseBase.run_test(self)

        unlock_command = "adb shell input keyevent 82"
        self._device.run_cmd(unlock_command, self._timeout)

        if "uiautomator " in self._process_name:
            # Starting ui automator test, update the cmd line
            # TODO: see if we shall not create another UC
            am_command = "adb shell {0} {1}".format(self._process_name, self._am_extra)
        else:
            # Std instrumentation testing
            am_command = "adb shell am instrument {0} -w {1}".format(self._am_extra, self._process_name)

        # Execute the command
        if re.search("\\\u\d{4}", am_command):
            am_command = am_command.decode('unicode-escape')
        result, output = self._device.run_cmd(am_command, self._timeout)

        if result == 0 and "OK (1 test)" in output:
            output = ""
        elif result == 0 and ('FAILURES!!!' not in output and "OK (" in output):
            output = ""
        elif output.strip() == "":
            result, output = Global.FAILURE, "Am command has no output"
        else:
            result = Global.FAILURE

        # save the result (for retrieving artifacts only on failure)
        self._run_test_result = result

        return result, output

    def tear_down(self):
        """
        UC TearDown, do your test cleaning here
        """
        UseCaseBase.tear_down(self)
        result, output = Global.SUCCESS, ""

        # retrieve artifacts on failure
        if self._run_test_result != Global.SUCCESS:
            self._retrieve_artifacts()
        # uninstal file
        self._uninstall_files()

        # put back binaries
        for element in self._rm_list['REMOVE_BINARY']:
            # commd = 'adb push {0} {1}'.format(element, self._rm_list['REMOVE_BINARY'][element])
            self._device.push(element, self._rm_list['REMOVE_BINARY'][element], 600)

        # reboot DUT in case conditions are met
        if self._post_reboot_device or (not self.test_ok and self._post_reboot_nok_device):
            if not self._device.reboot():
                result, output = Global.FAILURE, "Cannot reboot the device"

        return result, output
