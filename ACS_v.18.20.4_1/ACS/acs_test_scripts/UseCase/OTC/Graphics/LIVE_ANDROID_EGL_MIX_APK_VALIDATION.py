"""

:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

:organization: INTEL OTC ANDROID QA
:description: UseCase to run the GFX conformance on Intel Android devices using
test binaries for Khronos and Oglconform, for those tests that require a combination
of them both
:since: 5/22/14
:author: mmaracix
"""

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.GFXUtilities.OGLConform import OGLConform
from acs_test_scripts.Utilities.GFXUtilities.Khronos2 import Khronos2 as KhronosApk
from acs_test_scripts.Utilities.GFXUtilities.GFXFunctions import *
from Core.Report.SecondaryTestReport import SecondaryTestReport
import os
import re
from acs_test_scripts.Utilities.GFXUtilities.KhronosBin import KhronosBin

import Core.PathManager as PathManager

class LiveEGLMixAPKValidation(UseCaseBase):
    """
    GFX Conformance mix validation(Khronos and OGLConform) Test class
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        UseCaseBase.__init__(self, tc_name, global_config)

        self._oglconform = OGLConform(self._device)

        self._report_pathname = self._device.get_report_tree().get_report_path()
        self._output_filename = "%s/%s.csv" % (self._report_pathname, os.path.basename(self.get_name()))
        self._output_filename_khronos = "%s/khronos_%s.csv" % (self._report_pathname, os.path.basename(self.get_name()))

        # get TestCase parameters
        self._feature_name = str(self._tc_parameters.get_param_value("FEATURE_NAME", ""))
        self._diag_parameter = str(self._tc_parameters.get_param_value("DIAG_PARAMETER", "")).split(";")
        self._test_parameter = str(self._tc_parameters.get_param_value("TEST_PARAMETER", "")).replace("\n", "").split(
            ";")
        self._khronos_parameter = str(self._tc_parameters.get_param_value("KHRONOS_PARAMETER", "")).split(
            ";")
        self._remove_parameter = str(self._tc_parameters.get_param_value("REMOVE_PARAMS", "")).replace("\n", "").split(
            ";")

        self._device_logdir = self._tc_parameters.get_param_value("DEVICE_LOGDIR")

        self._cert_mode = self._tc_parameters.get_param_value("CERT_MODE")

        self._run_type = 'DRY'

        self.__tc_report = SecondaryTestReport(self._device.get_report_tree().get_report_path())
        self._adb_remove = "adb shell rm -rf /data/app/"
        self._proc_name = "org.khronos.gl_cts"

        self._report_path = self._device.get_report_tree().get_report_path()

        self._khronos_object = KhronosApk(self._device, self._device_logdir, self._report_path, None)

        self.original_results_gfx_location = os.path.join(os.path.expanduser("~"), "test_output")

        self.acs_gfx_results = os.path.join(self._device.get_report_tree().get_report_path(), "gfx_results.zip")

        self._helper_scripts_path = 'OTC/TC/ACS/Graphics/Graphics/scripts'
        self._scripts_path = PathManager.absjoin(KhronosApk.TEST_SUITES_PATH,
                                                       self._helper_scripts_path)
        self._activity_name = self._tc_parameters.get_param_value("ACTIVITY_NAME")
        self._default_logging_dir = "/sdcard/"
        name_list = self.get_name().split("/")
        tcs_name = name_list[len(name_list)-1]
        self._logs_folder_name = "gles_%s_logs" % tcs_name
        self._pull_logs_path = PathManager.absjoin(self._report_path, self._logs_folder_name)

        self._output_filename = "%s/%s.csv" % (self._report_pathname, os.path.basename(self.get_name()))
        self._output_filename_khronos = "%s/khronos_%s.csv" % (self._report_pathname, os.path.basename(self.get_name()))

        self._khronos = KhronosBin(self._device, self._report_pathname, self._logs_folder_name)
        # self._khronos_object = KhronosApk(self._device, self._device_logdir, self._report_path, self._activity_name)
        self._khronos_object = KhronosApk(self._device, self._device_logdir, self._pull_logs_path, self._activity_name)

    def set_up(self):
        """
        Initialize the test
        """

        UseCaseBase.set_up(self)
        adb_chmod = "adb shell chmod -R 777 /data/app/"
        adb_clean = "adb shell rm -rf %s/*" % "/sdcard/gfx-mockup"
        clean_res, clean_output = self._device.run_cmd(adb_clean, 20)
        chmod_res, chmod_output = self._device.run_cmd(adb_chmod, 20)
        adb_makedir = "adb shell mkdir %s%s" % (self._default_logging_dir, self._device_logdir)
        makedir_res, makedir_msg = self._device.run_cmd(adb_makedir, 20)

        if clean_res == Global.FAILURE:
            result, output = Global.FAILURE, clean_output
        elif chmod_res == Global.FAILURE:
            result, output = Global.FAILURE, chmod_output

        else:
            result, output = Global.SUCCESS, "No errors were encountered in the setup part of the test"
        return result, output

    def replace_multiple(self, dict, text):
        # Create a regular expression  from the dictionary keys
        regex = re.compile("(%s)" % "|".join(map(re.escape, dict.keys())))

        # For each match, look-up corresponding value in dictionary
        return regex.sub(lambda mo: dict[mo.string[mo.start():mo.end()]], text)

    def execute_tests(self):
        result = Global.SUCCESS
        result_file = open_file_with_backup(self._output_filename, 'w')
        if self._cert_mode == 'True':
            self._oglconform.execute_conformance(result_file)
        else:
            if self._feature_name:
                if self._diag_parameter:
                    for diag_option in self._diag_parameter:
                        if '-diag' in diag_option:
                            output = self._oglconform.treat_diag(diag_option, self._feature_name)
                            write_report(self._feature_name, output, result_file)
            if self._test_parameter:
                for testcase in self._test_parameter:
                    testcase_string = str(testcase).split(" ")
                    repl = {"-es1 ": "", "-libGL ": "", "-test ": "", "                -es2 ": "", "-es2": "",
                            "                -es1 ": ""}
                    if len(testcase_string) >= 3:
                        test_report_name = self.replace_multiple(repl, testcase)
                        if '-test' in testcase:
                            output = self._oglconform.treat_testcase(testcase)
                            write_report(test_report_name, output, result_file)
            if self._khronos_parameter:
                if self._activity_name:
                    for test in self._khronos_parameter:
                        run_result, run_output = self._khronos_object.full_run(self._proc_name, self._cert_mode, test, self._run_type, timeout=4500)
                else:
                    for test in self._khronos_parameter:
                        run_result, run_output = self._khronos.run_one('conform', test)

                        if run_result == Global.SUCCESS:
                            pull_result, pull_output = self._khronos.pull_results_gles1(self._report_pathname, 100)
                        else:
                            return run_result, run_output
                        if pull_result == Global.SUCCESS:
                            result_csv = self._khronos.parse_all(self._output_filename_khronos)
                            result, output = self._khronos_object.extract_results(self.get_name(), self.tc_order, self.__tc_report,
                                                                          result_csv)
                        else:
                            return pull_result, pull_output
                    result_file.close()
            else:
                result, output = Global.FAILURE, "Could not run Khronos Conformance tests required"

        return result, self._output_filename

    def run_test(self):
        """
        Run the test per se
        """
        UseCaseBase.run_test(self)

        result, result_filename = self.execute_tests()
        result, output = self._khronos_object.extract_results(self.get_name(), self.tc_order, self.__tc_report,
                                                              result_filename)
        return result, output

    def remove_param(self):
        """
        This is a substitute function for removing all the test binaries that were pushed onto the device.
        Disclaimer: This is a placeholder method.
        """
        result = Global.FAILURE
        output = "Did not yet remove the parameters"
        if self._remove_parameter:
            for param in self._remove_parameter:
                if len(param) > 0:
                    adb_cmd = self._adb_remove + param
                    self._device.run_cmd(adb_cmd, 300)
                    result = Global.SUCCESS
                    output = "All files have been removed"
        else:
            result = Global.SUCCESS
            output = "Nothing to clean up after the tests"
        return result, output


    def tear_down(self):
        """
        Cleaning up the files after the test has successfully ran
        """
       # result, output = self.remove_param()
        # return result, output
        return Global.SUCCESS, "Tear down was reached successfully"
