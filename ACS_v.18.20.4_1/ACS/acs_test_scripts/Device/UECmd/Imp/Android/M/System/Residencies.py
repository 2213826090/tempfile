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

:organization: TMT TTD AN
:summary: This file implements the System UEcmd for Android device
:author: ahkhowaj
:since: 11/15/2015
"""
import re
from acs_test_scripts.Device.UECmd.Imp.Android.Common.Base import Base
from acs_test_scripts.Device.UECmd.Interface.System.IResidencies import IResidencies
from acs_test_scripts.Device.UECmd.Imp.Android.Common.System.Residencies import Residencies as ResidenciesCommon


class Residencies(ResidenciesCommon, IResidencies):
    """
    Residencies reading/cleaning UECmd
    """

    def parse_socwatch_nc_dstates_info(self, result_file_fullpath, target_block):
        """
        Parse D state info in the SOCWatch 2.0 csv file for Android-M
        """
        self._logger.info("Using parser for SOCWatch v2.0")
        states_title = ["D0i0", "D0i1", "D0i2", "D0i3"]
        states_info = []
        socwatch_result = open(result_file_fullpath)
        for line in socwatch_result:
            if target_block in line:
                idx = line.replace(' ', '').split(',').index(target_block)
                #read next four lines
                states_info = [re.findall(r"\d*\.\d+|\d+", next(socwatch_result).split(',')[idx])[0] for x in xrange(4)]
                self._logger.info("states_info: %s" %states_info)
                return states_title, states_info

        return None

    def parse_socwatch_sstates_info(self, result_file_fullpath):
        """
        Parse S state info in the SOCWatch 2.0 csv file for Android-M
        """
        self._logger.info("Using parser for SOCWatch v2.0")
        states_title = ["S0i0", "S0iR", "S0i1", "S0i2", "S0i3", "S3"]
        states_info = []
        socwatch_result = open(result_file_fullpath)
        S0ix_State = "S-State,  SYSTEM"
        ACPI_SState = "ACPI State,  SYSTEM"
        S0ix_State_done = False
        ACPI_SState_done = False
        for line in socwatch_result:
            if S0ix_State_done and ACPI_SState_done:
                self._logger.info("states_info: %s" %states_info)
                return states_title, states_info
            if S0ix_State in line:
                #read next five lines
                states_info = [re.findall(r"\d*\.\d+|\d+", next(socwatch_result).split(',')[1])[0] for x in xrange(5)]
                S0ix_State_done = True
            if ACPI_SState in line:
                #read next one line
                states_info.append(re.findall(r"\d*\.\d+|\d+", next(socwatch_result).split(',')[1])[0])
                ACPI_SState_done = True
        return None
