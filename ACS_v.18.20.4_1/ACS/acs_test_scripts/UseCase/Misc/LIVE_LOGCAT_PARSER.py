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

:since: 2014-24-04
:author: mcchilax
"""

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException
from Core.PathManager import Folders
import time

import os

class LogcatParser(UseCaseBase):

    """
    Logcat Parser Test class
    """
    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        UseCaseBase.__init__(self, tc_name, global_config)

        # Messages that the parser looks for
        self._message = self._tc_parameters.get_param_value("MESSAGE")
        if self._message in (None,""):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "MESSAGE must not be an empty string")

        # Time to wait for logcat dump
        self._time = self._tc_parameters.get_param_value(param="TIME",
                                                            default_value=20.0,
                                                            default_cast_type=float)

        self._pre_reboot_device = False
        self._post_reboot_device = False
        self._post_reboot_nok_device = False
        if self._device.get_config("DisableTcReboot", False, default_cast_type='str_to_bool'):
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

    def set_up(self):
        """
        Initialize the test
        """

        UseCaseBase.set_up(self)
        self._logcat_extract = os.path.join(Folders.REPORTS, "parserlogcat.log")
        return Global.SUCCESS, ""

    def run_test(self):

        UseCaseBase.run_test(self)
        adb_command = "adb logcat -d > %s" %(self._logcat_extract)
        time.sleep(self._time)
        os.system(adb_command)
        time.sleep(self._wait_btwn_cmd)
        result, output = self.analyzeLog(self._logcat_extract)
        return result, output

    def tear_down(self):
        time.sleep(self._wait_btwn_cmd)
        os.remove(self._logcat_extract)
        return Global.SUCCESS, ""

    def analyzeLog(self, logcat_file):
        """
        This function will analize DUT logcat against some input expressions
        so deciding a PASS/FAILED status based on expressions presence
        """
        output = "SUCCESS"
        logcat = open(logcat_file, 'r')
        file_list = logcat.readlines()
        content = "".join(file_list)
        logcat.close()
        if not content.strip():
            raise Exception("logcat was not saved")
        searchParams = self._message.strip('"').split(',')
        bStatus = False
        for line in file_list:
            if bStatus:
                break
            else:
                for var in searchParams:
                    term = var.strip(' ')
                    if 'SIGSEGV' in term and term in line:
                        result = Global.FAILURE
                        output ="Segmentation fault error appeared in LOGCAT\n"
                        output +="on line= %s\n Full result:\n" %line
                        index = file_list.index(line)
                        output += str(file_list[(index - 10):(index + 20)])
                        bStatus = True
                        break
                    if term in line:
                        result = Global.FAILURE
                        output = "Logcat TAG= %s looked for was found " %term
                        output +="on line= %s\n Full result:\n" %line
                        index = file_list.index(line)
                        output += str(file_list[(index - 10):(index + 20)])
                        bStatus = True
                        break
        if not bStatus:
            result = Global.SUCCESS
        if 'Segmentation fault' in output:
            output = "Logcat TAG= %s looked for was found" %term
            output +="on line= %s\n Full result:\n" %line
            index = file_list.index(line)
            output += str(file_list[(index - 10):(index + 20)])
        return result, output
