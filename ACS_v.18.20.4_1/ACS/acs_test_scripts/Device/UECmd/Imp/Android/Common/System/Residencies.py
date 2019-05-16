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
:summary: This file implements the System UEcmd for Android device
:author: pbluniex
"""
import re
from acs_test_scripts.Device.UECmd.Imp.Android.Common.Base import Base
from acs_test_scripts.Device.UECmd.Interface.System.IResidencies import IResidencies


class Residencies(Base, IResidencies):
    """
    Residencies reading/cleaning UECmd
    """

    def __init__(self, device):
        """
        Constructor
        """
        Base.__init__(self, device)
        IResidencies.__init__(self, device)

        self._residencies = None
        self.sleep_mode_apis = self._device.get_uecmd("SleepMode")

    def clear(self, delay_time_s=0):
        """
        Clear residencies
        """
        self.sleep_mode_apis.clear_residency_counters(delay_time_s)

    def device_has_residencies(self):
        """
        To know if the device has a residency file

        :rtype: bool
        :return: true if the device has a residency file, else false
        """
        return self.sleep_mode_apis.device_has_residencies()

    def get_value(self, name, mode):
        """
        Return the residency column value named "name" for the given mode

        :type name: str
        :param name: Name of the requested residency column

        :type mode: str
        :param mode: Requested mode

        :rtype: float
        :return: The value of the column "name" for the given mode
        """
        if not self.device_has_residencies():
            return 0.0

        for residency in self.sleep_mode_apis.get():
            if residency.get("mode") == str(mode).lower():
                return float(residency.get(name))

    def get(self):
        """
        Return the residencies table

        :rtype: dict
        :return: the residencies table
        """
        if self.device_has_residencies():
            return self.sleep_mode_apis.get()
        else:
            return []

    def convert_criteria_list(self, target_criteria):
        conversion_dict = {
            "GT": ">",
            "GE": ">=",
            "LT": "<",
            "LE": "<=",
            "EQ": "==",
            "NE": "!="
        }
        converted_target_criteria = []
        for criteria in target_criteria:
            if not criteria.upper() in conversion_dict.keys():
                return None
            else:
                converted_target_criteria.append(conversion_dict[criteria.upper()])

        return converted_target_criteria

    def parse_socwatch_nc_dstates_info(self, result_file_fullpath, target_block):
        """
        Parse D state info in the SOCWatch csv file
        """
        states_title = ["D0i0", "D0i1", "D0i2", "D0i3"]
        socwatch_result = open(result_file_fullpath)
        for line in socwatch_result:
            if target_block in line:
                states_info = re.findall(r"\d*\.\d+|\d+", line)
                return states_title, states_info[1:]
        return None

    def parse_socwatch_sstates_info(self, result_file_fullpath):
        """
        Parse S state info in the SOCWatch csv file
        """
        states_title = ["S0", "S0i1", "S0i2", "S0i3", "S0iR", "S3"]
        socwatch_result = open(result_file_fullpath)
        for line in socwatch_result:
            check_line = re.findall(r"%", line)
            if len(check_line) == len(states_title):
                states_info = re.findall(r"\d*\.\d+|\d+", line)
                return states_title, states_info
        return None

    def disable_s3(self):
        """
        Blocks s3 sleep state
        """
        cmd = "adb shell echo 0 > /sys/power/wake_lock"
        self._exec(cmd, force_execution=True)
