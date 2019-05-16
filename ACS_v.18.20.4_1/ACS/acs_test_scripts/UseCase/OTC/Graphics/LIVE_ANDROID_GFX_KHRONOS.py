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

from acs_test_scripts.Utilities.GFXUtilities.KhronosBin import KhronosBin
from acs_test_scripts.Utilities.GFXUtilities.Khronos2 import Khronos2 as KhronosApk


class LiveGFXKhronos(UseCaseBase):
    """
    Logcat Parser Test class
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        UseCaseBase.__init__(self, tc_name, global_config)

        #get TC parameters
        self._remove_parameter = str(self._tc_parameters.get_param_value("REMOVE_PARAMS", "")).replace("\n", "").split(";")
        self._report_pathname = self._device.get_report_tree().get_report_path()
        self.__tc_report = SecondaryTestReport(self._report_pathname)
        self.acs_gfx_results = ""
        self._adb_remove = "adb shell rm -rf /data/app/"
        name_list = self.get_name().split("/")
        tcs_name = name_list[len(name_list)-1]
        self._logs_folder_name = "gles_%s_logs" % tcs_name
        self._result_filename = "%s/%s.csv" % (self._report_pathname, os.path.basename(self.get_name()))

        self._khronos_object = KhronosApk(self._device, None, self._report_pathname, None)

        self._khronos = KhronosBin(self._device, self._report_pathname, self._logs_folder_name)

    def set_up(self):
        """
        Initialize the test
        """
        UseCaseBase.set_up(self)
        adb_chmod = "adb shell chmod -R 777 /data/app/"
        chmod_res, chmod_output = self._device.run_cmd(adb_chmod, 20)

        return Global.SUCCESS, "No errors"


    def run_test(self):
        UseCaseBase.run_test(self)
        run_result, run_output = self._khronos.run_conformance()
        if run_result == Global.SUCCESS:
            pull_result, pull_output = self._khronos.pull_results_gles1(self._report_pathname, 100)
        else:
            return run_result, run_output
        if pull_result == Global.SUCCESS:
            result_csv = self._khronos.parse_all(self._result_filename)
            result, output = self._khronos_object.extract_results(self.get_name(), self.tc_order, self.__tc_report,
                                                              result_csv)
        else:
            return pull_result, pull_output
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
