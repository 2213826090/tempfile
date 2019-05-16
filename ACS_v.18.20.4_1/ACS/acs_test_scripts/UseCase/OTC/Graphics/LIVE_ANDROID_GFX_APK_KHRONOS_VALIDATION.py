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

:since: 8/27/2014
:author: mmaracix
"""

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.GFXUtilities.GFXFunctions import *
from Core.Report.SecondaryTestReport import SecondaryTestReport
import os
from Core.PathManager import Folders
from Device.DeviceLogger.LogCatLogger.LogCatReaderThread import LogCatReaderThread
from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT
import threading

import Core.PathManager as PathManager
from acs_test_scripts.Utilities.GFXUtilities.Khronos2 import Khronos2 as KhronosApk


class LiveGFXAPKKhronosValidation(UseCaseBase):
    """
    LiveGFXAPKKhronosValidation Test class
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        UseCaseBase.__init__(self, tc_name, global_config)

        self._logger = LOGGER_TEST_SCRIPT
        logcat_cmd_line = self._device.get_config("logcatCmdLine", "adb shell logcat -v threadtime")
        self._logcat_extract = os.path.join(Folders.REPORTS, "parserlogcat.log")

        self._logcat_analyzer = LogCatReaderThread(self._device, logger=self._logger, enable_writer=False,
                                                   logcat_cmd_line=logcat_cmd_line)
        self._proc_name = "org.khronos.gl_cts"

        #parameter that says how you want to run the tests: dry or full
        #dry: run once, if it fails, log the result, close execution
        #full: if it fails, re-run from the last entry, log the results as you go
        self._run_type = self._tc_parameters.get_param_value("RUN_TYPE")

        #this parameter says if the run is in certification mode or not
        self._cert_mode = self._tc_parameters.get_param_value("CERT_MODE")

        #give the location on the device for the logs
        self._device_logdir = self._tc_parameters.get_param_value("DEVICE_LOGDIR")

        #initializing the secondary test results
        #this is also where the intermediate and so on results must be pulled
        self._report_path = self._device.get_report_tree().get_report_path()
        self.__tc_report = SecondaryTestReport(self._report_path)

        #the sdcard is the default logging directory for the ES APK
        self._default_logging_dir = "/sdcard/"
        self._helper_scripts = 'OTC/TC/ACS/Graphics/Graphics/scripts'
        #this is also where the group and test lists must be pulled
        self._helper_scripts_gfx = PathManager.absjoin(KhronosApk.TEST_SUITES_PATH, self._helper_scripts)

        self._activity_name = self._tc_parameters.get_param_value("ACTIVITY_NAME")

        self._group_list_filename = self._tc_parameters.get_param_value("GROUPS_FILENAME")
        if self._tc_parameters.get_param_value("SINGLE_TEST_PARAM"):
            self._single_test_parameter = str(self._tc_parameters.get_param_value("SINGLE_TEST_PARAM")).split(";")
        else:
            self._single_test_parameter = None

        name_list = self.get_name().split("/")
        tcs_name = name_list[len(name_list)-1]
        self._logs_folder_name = "gles_%s_logs" % tcs_name
        self._pull_logs_path = PathManager.absjoin(self._report_path, self._logs_folder_name)

        if self._group_list_filename:
            self._group_list = PathManager.absjoin(self._helper_scripts_gfx, self._group_list_filename)
        else:
            self._group_list = self._helper_scripts_gfx
        # self._khronos_object = KhronosApk(self._device, self._device_logdir, self._report_path, self._activity_name)
        self._khronos_object = KhronosApk(self._device, self._device_logdir, self._pull_logs_path, self._activity_name)

        self._output_filename_khronos = "%s/khronos_%s.csv" % (self._report_path, os.path.basename(self.get_name()))

    def set_up(self):
        """
        Initialize the test
        """

        UseCaseBase.set_up(self)
        adb_clean = "adb shell rm -rf %s" % "/sdcard/gfx-mockup"
        clean_res, clean_output = self._device.run_cmd(adb_clean, 20)
        adb_clean2 = "adb shell rm -rf %s" % "/sdcard/mock"
        clean_res, clean_output = self._device.run_cmd(adb_clean2, 20)
        adb_makedir = "adb shell mkdir %s%s" % (self._default_logging_dir, self._device_logdir)
        makedir_res, makedir_msg = self._device.run_cmd(adb_makedir, 20)
        return Global.SUCCESS, "No errors"


    def run_test(self):
        """
        """
        UseCaseBase.run_test(self)
        result, output = Global.BLOCKED, "Execution not really started..."
        if self._single_test_parameter is not None:
            for test in self._single_test_parameter:
                run_result, run_output = self._khronos_object.full_run(self._proc_name, self._cert_mode, test, self._run_type, timeout=4500)
        elif self._single_test_parameter is None:
            run_result, run_output = self._khronos_object.full_run(self._proc_name, self._cert_mode, self._group_list, self._run_type, timeout=4500)
        if run_result == Global.SUCCESS:
            # result, output = self._khronos_object.pull_and_extract(self.get_name(), self.tc_order, self.__tc_report)
            # result, output = self._khronos_object.pull_and_extract_with_lists(self.get_name(), self.tc_order, self.__tc_report)
            # result, output = self._khronos_object.pull_and_extract_with_lists(self.get_name(), self.tc_order, self.__tc_report, self._output_filename_khronos)
            result, output = self._khronos_object.pull_and_extract_no_secondary(self._output_filename_khronos)
        else:
            return run_result, run_output
        return result, output


    def tear_down(self):
        """
        Cleaning up the files after the test has successfully ran
        """
        result, output = Global.SUCCESS, "Finished execution"
        return result, output
