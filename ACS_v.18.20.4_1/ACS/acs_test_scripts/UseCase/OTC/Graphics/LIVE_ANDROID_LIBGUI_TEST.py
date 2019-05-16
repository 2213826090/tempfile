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
:author: mmaraci
"""

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.GFXUtilities.libguiTest import libguiTest
from acs_test_scripts.Utilities.GFXUtilities.Khronos2 import Khronos2 as KhronosApk
from acs_test_scripts.Utilities.GFXUtilities.GFXFunctions import *
from Core.Report.SecondaryTestReport import SecondaryTestReport
import os
import re
from acs_test_scripts.Utilities.GFXUtilities.KhronosBin import KhronosBin

import Core.PathManager as PathManager

class LiveLibguiTest(UseCaseBase):
    """
    GFX Conformance mix validation(Khronos and OGLConform) Test class
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        UseCaseBase.__init__(self, tc_name, global_config)

        self._libgui_object = libguiTest()

        self._report_pathname = self._device.get_report_tree().get_report_path()
        self._output_filename = "%s/%s.csv" % (self._report_pathname, os.path.basename(self.get_name()))

        # get TestCase parameters

        ###########################################
        self._libgui_test = str(self._tc_parameters.get_param_value("LIBGUI_TEST")).replace("\n", "").replace("\t", "").split(";")

        self.__tc_report = SecondaryTestReport(self._device.get_report_tree().get_report_path())
        self._adb_remove = "adb shell rm -rf /data/app/"


    def set_up(self):
        """
        Initialize the test
        """

        UseCaseBase.set_up(self)
        adb_chmod = "adb shell chmod -R 777 /data/app/"
        result, output = self._device.run_cmd(adb_chmod, 20)
        # result, output = Global.SUCCESS, "Set up"
        return result, output

    def replace_multiple(self, dict, text):
        # Create a regular expression  from the dictionary keys
        regex = re.compile("(%s)" % "|".join(map(re.escape, dict.keys())))

        # For each match, look-up corresponding value in dictionary
        return regex.sub(lambda mo: dict[mo.string[mo.start():mo.end()]], text)

    def execute_tests(self):
        result = Global.BLOCKED
        result_file = open_file_with_backup(self._output_filename, 'w')
        if self._libgui_test:
            for test in self._libgui_test:
                test_result = self._libgui_object.execute_single_run(test, result_file)
                if test_result == "PASSED":
                    result = Global.SUCCESS
                else:
                    result = Global.FAILURE
        result_file.close()

        return result, self._output_filename

    def run_test(self):
        """
        Run the test per se
        """
        result, output = self.execute_tests()
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
        # result, output = self.remove_param()
        # return result, output
        return Global.SUCCESS, "Tear down was reached successfully"
