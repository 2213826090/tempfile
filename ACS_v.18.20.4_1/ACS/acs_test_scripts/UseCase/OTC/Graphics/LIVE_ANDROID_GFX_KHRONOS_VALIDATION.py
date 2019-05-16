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

:since: 5/23/14
:author: mmaracix
"""

import sys
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.Utilities.GFXUtilities.GFXFunctions import *
from Core.Report.SecondaryTestReport import SecondaryTestReport
import os
import sys
import re
import csv
import shutil
import zipfile



class LiveGFXKhronosValidation(UseCaseBase):
    """
    Logcat Parser Test class
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        UseCaseBase.__init__(self, tc_name, global_config)

        #get TC parameters
        self._result_filename = str(self._tc_parameters.get_param_value("RESULT_FILENAME", ""))
        self._manifest_filename = str(self._tc_parameters.get_param_value("MANIFEST_FILENAME", "")).split(";")
        self._remove_parameter = str(self._tc_parameters.get_param_value("REMOVE_PARAMS", "")).replace("\n", "").split(";")
        self.__tc_report = SecondaryTestReport(self._device.get_report_tree().get_report_path())
        self.acs_gfx_results = ""
        self._adb_remove = "adb shell rm -rf /data/app/"

    def set_up(self):
        """
        Initialize the test
        """

        UseCaseBase.set_up(self)
        serial_number = self._device.get_serial_number()

        self.acs_gfx_results = os.path.join(self._device.get_report_tree().get_report_path(), "gfx_results.zip")
        self.original_results_gfx_location = os.path.join(os.path.expanduser("~"), "test_output")
        if os.path.isdir(self.original_results_gfx_location):
            shutil.rmtree(self.original_results_gfx_location)
        return Global.SUCCESS, "No errors"


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


    def extract_results(self, result_filename):
        """
        compute results from gfx csv file test results
        - add each test result to a secondary report

        :param result_filename: gfx log output and it's location
        :type file_path: str

        :rtype: str
        :return: output message
        """
        test_executed = 0
        result = Global.FAILURE
        output = ""
        if result_filename is None:
            self._device.get_logger().error("There is no specified csv file from GFX fwk !")
            output = "GFX result not detected in the log, No result"
            result = Global.FAILURE
        elif not os.path.isfile(result_filename):
            self._device.get_logger().error("Specified GFX result file %s does not exist, cannot compute properly results !" % (result_filename,))
            output = "GFX result is missing, No result"
            result = Global.FAILURE
        else:
            with open(result_filename, "r") as gfx_report:
                csv_content = csv.reader(gfx_report, delimiter=';', quotechar='|')
                result = Global.SUCCESS
                for row in csv_content:
                    if len(row) >= 2:
                        if "PASSED" in row[1]:
                            test_executed += 1
                            self.__tc_report.add_result(row[0], self.__tc_report.verdict.PASS, "Test PASS", self.get_name(), self.tc_order)
                        elif "BLOCKED DEPENDENCY" in row[1]:
                            test_executed += 1
                            self.__tc_report.add_result(row[0], self.__tc_report.verdict.BLOCKED, "Test BLOCKED", self.get_name(), self.tc_order)
                        elif "NA" in row[1]:
                            # TEST SHOULD NOT BE RUN FOR THIS PLATFORM, so pass to another one
                            continue
                        else:
                            result = Global.FAILURE
                            output = "Some tests are FAIL"
                            test_executed += 1
                            self.__tc_report.add_result(row[0], self.__tc_report.verdict.FAIL, "Test status is %s, see log output for more details" % row[1], self.get_name(), self.tc_order)
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
        UseCaseBase.run_test(self)
        #manifest_instance =
        for manifest_instance in self._manifest_filename:
            result_file = execute_test_group(manifest_instance, self._result_filename)

        result, output = self.extract_results(result_file)
        self._device.get_logger().info("Move GFX results into ACS result, as zipfile %s" % (os.path.basename(self.acs_gfx_results)))

        if not os.path.isdir(self.original_results_gfx_location):
            self._device.get_logger().warning("%s directory does not exist, cannot copy GFX results" % (self.original_results_gfx_location,))
        else:
            # zip results
            self.zip_results()

        return result, output

    def remove_param(self):
        """
        This is a substitute function for removing all the test binaries that were pushed onto the device.
        Disclaimer: This is a placeholder method.
        """
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
        result, output = self.remove_param()
        return result, output
