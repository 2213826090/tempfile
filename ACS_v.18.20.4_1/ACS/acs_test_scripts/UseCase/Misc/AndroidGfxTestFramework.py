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
:summary: Enable running GFX test framework in ACS
:since: 25/06/2013
:author: kturban
"""
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from Core.Report.SecondaryTestReport import SecondaryTestReport
from UtilitiesFWK.Utilities import Global, internal_shell_exec
import os
import sys
import re
import csv
import shutil
import zipfile


class AndroidGFXTestFramework(UseCaseBase):

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        UseCaseBase.__init__(self, tc_name, global_config)

        # get networking api in order to switch off WIFI
        self._networking_api = self._device.get_uecmd("Networking")
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")
        self._ui_api = self._device.get_uecmd("Ui")
        self._ui_api.set_global_config(global_config)
        self._ui_api.init()
        # Get TC Parameters
        # GFX test framework test path
        self._gfx_fwk_test = self._tc_parameters.get_param_value("GFX_FWK_TEST")
        # extra params to pass as cmd parameters
        self._cmd_params = self._tc_parameters.get_param_value(param="TEST_CMD_LINES", default_value="")
        # execution timeout
        self._timeout = self._tc_parameters.get_param_value(param="CMD_TIMEOUT", default_cast_type=int)
        self.__tc_report = SecondaryTestReport(self._device.get_report_tree().get_report_path())
        self.acs_gfx_results = ""

    def set_up(self):
        """
        Setup the test
        HOST:
        - retrieve gfx test framework
        DEVICE:
        - setup properties to run tests properly
        """
        UseCaseBase.set_up(self)

        if self._gfx_fwk_test is None:
            self._device.get_logger().error("GFX_FWK_TOOL_PATH is a directory where GFX test framework is stored")
            return Global.FAILURE, "You need to specify a GFX_FWK_PATH value."

        if self._timeout is None:
            return Global.FAILURE, "You need to specify a TIMEOUT value."

        serial_number = self._device.get_serial_number()
        if serial_number:
            if self._cmd_params:
                self._cmd_params += " "
            self._cmd_params += "--device %s" % (serial_number,)

        self.acs_gfx_results = os.path.join(self._device.get_report_tree().get_report_path(), "gfx_results.zip")
        # Not possible to configure the GFX output dir
        self.original_results_gfx_location = os.path.join(os.path.expanduser("~"), "test_output")
        if os.path.isdir(self.original_results_gfx_location):
            shutil.rmtree(self.original_results_gfx_location)

        verdict, computed_path = self._get_file_path(self._gfx_fwk_test)
        if verdict != Global.SUCCESS:
            return Global.FAILURE, "%s is not an existing gfx python test" % (self._gfx_fwk_test,)
        else:
            # use the computed value
            self._gfx_fwk_test = computed_path

        # setup the board if has not been done previously
        verdict, output = self.__setup_board_for_gfx()
        if verdict != Global.SUCCESS:
            return Global.FAILURE, "Setup the board to execute GFX tests has failed"

        return Global.SUCCESS, "No errors"

    def _disable_wizard(self):
        """ Disable phone wizard on the device

        :rtype: str
        :return: msg output.
        """
        verdict, msg = self._ui_api.run_operation_set("phone_unlock")
        if verdict == Global.SUCCESS:
            self._ui_api.run_operation_set("go_home")
        else:
            return verdict, msg

        verdict, msg = self._ui_api.run_operation_set("unlock_wizard")
        if verdict == Global.SUCCESS:
            self._ui_api.run_operation_set("go_home")
        else:
            return verdict, msg

        return Global.SUCCESS, "Wizard deactivated !"

    def _disable_lock_screen(self):
        """ Remove the device lock screen

        :rtype: str
        :return: msg output.
        """
        verdict, msg = self._ui_api.run_operation_set("phone_unlock")
        if verdict == Global.SUCCESS:
            self._ui_api.run_operation_set("go_home")
        else:
            return verdict, msg

        verdict, msg = self._ui_api.run_operation_set("deactivate_lockscreen")
        if verdict == Global.SUCCESS:
            self._ui_api.run_operation_set("go_home")
        else:
            return verdict, msg
        return Global.SUCCESS, "Lock screen removed !"

    def zip_results(self):
        """ Zip all GFX output results directory """
        if not os.path.isfile(self.acs_gfx_results):
            zip = zipfile.ZipFile(self.acs_gfx_results, 'w', zipfile.ZIP_DEFLATED)
        else:
            zip = zipfile.ZipFile(self.acs_gfx_results, 'a', zipfile.ZIP_DEFLATED)

        for root, dirs, files in os.walk(self.original_results_gfx_location):
            for file in files:
                zip.write(os.path.join(root, file))
        zip.close()

    def __setup_board_for_gfx(self):
        """
        Setup the board to run properly GFX tests
        - remove 1st use android wizard
        - remove lock screen
        - set stay on while plugged in to True
        - disable screen off timeout
        - remove connectivities which are able to push results on public web access

        :rtype: str
        :return: msg output.
        """
        verdict = Global.SUCCESS
        msg = "Set up the board for GFX : OK"
        # No previous setup done on the board
        if self._device.get_property_value('GFX_SETUP_STATUS') in (None, ""):
            # set CTS necessary properties

            # disable user wizard
            verdict, msg = self._disable_wizard()
            if not verdict == Global.SUCCESS:
                return verdict, msg
            else:
                self._device.get_logger().info(msg)

            # remove lock screen
            verdict, msg = self._disable_lock_screen()
            if not verdict == Global.SUCCESS:
                return verdict, msg
            else:
                self._device.get_logger().info(msg)

            # Stay on if plugged
            self._phonesystem_api.set_stay_on_while_plugged_in(3)

            # Remove screen off timeout
            self._phonesystem_api.set_screen_timeout(3600)

            # switch OFF wireless technologies which are able to push on the web (some
            # benchmark upload results to public results ...)
            list_of_wireless = self._networking_api.get_available_technologies()
            if "cellular" in list_of_wireless:
                self._networking_api.clean_all_data_connections()
            if "wifi" in list_of_wireless:
                self._networking_api.set_wifi_power("off")

            # remove security about applications
            self._phonesystem_api.set_verify_application(False)

            msg = "All settings have been applied"
            self._device.get_logger().info(msg)

            self._device.run_cmd("adb shell setprop GFX_SETUP_STATUS 1", 1)
        else:
            self._device.get_logger().info("A previous GFX setup has already been done")
        return verdict, msg

    def get_csv_result_from_logoutput(self, log_output):
        """
        Extract the path of gfx csv file results from its output

        :param file_path: gfx log output
        :type file_path: str

        :rtype: str or None
        :return: file path or None if not found.
        """
        gfx_result_path = None
        for line in iter(log_output.splitlines()):
            # try to locate gfx csv results
            regex = "Result file name is: (?P<result_location>.*)"
            matches_test = re.compile(regex).search(line)
            if matches_test is not None:
                gfx_result_path = os.path.abspath(matches_test.group("result_location"))
                break
        return gfx_result_path

    def extract_results(self, log_output):
        """
        compute results from gfx csv file test results
        - add each test result to a secondary report

        :param file_path: gfx log output
        :type file_path: str

        :rtype: str
        :return: output message
        """
        test_executed = 0
        result = Global.FAILURE
        output = ""
        gfx_result_path = self.get_csv_result_from_logoutput(log_output)
        if gfx_result_path is None:
            self._device.get_logger().error("There is no specified csv file from GFX fwk !")
            output = "GFX result not detected in the log, No result"
            result = Global.FAILURE
        elif not os.path.isfile(gfx_result_path):
            self._device.get_logger().error(
                "Specified GFX result file %s does not exist, cannot compute properly results !" % (gfx_result_path,))
            output = "GFX result is missing, No result"
            result = Global.FAILURE
        else:
            with open(gfx_result_path, "r") as gfx_report:
                csv_content = csv.reader(gfx_report, delimiter=';', quotechar='|')
                result = Global.SUCCESS
                for row in csv_content:
                    if len(row) >= 2:
                        if "PASSED" in row[1]:
                            test_executed += 1
                            self.__tc_report.add_result(
                                row[0], self.__tc_report.verdict.PASS, "Test PASS", self.get_name(), self.tc_order)
                        elif "BLOCKED DEPENDENCY" in row[1]:
                            test_executed += 1
                            self.__tc_report.add_result(
                                row[0], self.__tc_report.verdict.BLOCKED, "Test BLOCKED", self.get_name(), self.tc_order)
                        elif "NA" in row[1]:
                            # TEST SHOULD NOT BE RUN FOR THIS PLATFORM, so pass to another one
                            continue
                        else:
                            result = Global.FAILURE
                            output = "Some tests are FAIL"
                            test_executed += 1
                            self.__tc_report.add_result(row[0], self.__tc_report.verdict.FAIL, "Test status is %s, see log output for more details" % row[
                                                        1], self.get_name(), self.tc_order)
                    else:
                        result = Global.FAILURE
                        output = "CSV seems corrupted, there are less than 2 rows!"

                if result != Global.FAILURE:
                    output = "All tests are PASS"
                if not test_executed:
                    result = Global.FAILURE
                    output = "No tests have been executed"
        return result, output

    def run_test(self):
        """
        Execute the gfx tests
        """
        UseCaseBase.run_test(self)
        crt_dir = os.getcwd()
        test_dir = os.path.dirname(self._gfx_fwk_test)
        test_file = os.path.basename(self._gfx_fwk_test)

        result = Global.FAILURE

        # Execute GFX test framework
        os.chdir(test_dir)
        cmd = "%s %s %s" % (sys.executable, test_file, self._cmd_params)
        # GFX framework always ends with a success return code
        result, output = internal_shell_exec(cmd, self._timeout)
        os.chdir(crt_dir)

        # so, parse the output to compute a results ...
        result, output = self.extract_results(output)

        self._device.get_logger().info("Move GFX results into ACS result, as zipfile %s" %
                                       (os.path.basename(self.acs_gfx_results)))

        if not os.path.isdir(self.original_results_gfx_location):
            self._device.get_logger().warning(
                "%s directory does not exist, cannot copy GFX results" % (self.original_results_gfx_location,))
        else:
            # zip results
            self.zip_results()

        return result, output
