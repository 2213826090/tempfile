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
:summary: This file implements the LAB NFC CARD EMULATION FELICA UC
:since: 04/06/2013
:author: lpastor

"""

from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.LocalConnectivity.LAB_NFC_BASE import LabNfcBase
from ErrorHandling.AcsConfigException import AcsConfigException


class LabNfcCardEmulFelica(LabNfcBase):

    """
    Lab NFC Felica card emulation.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabNfcBase.__init__(self, tc_name, global_config)

        self._pcd_antenna_x = "0"
        self._pcd_antenna_y = "0"
        self._pcd_antenna_up = "0"
        self._pcd_antenna_down = "0"

        # Create NFC software
        self._mp_manager = self._em.get_nfc_tool("NFC_TOOL4")

        # Get Secure element to communicate with from test case xml file
        self._secure_element = \
            str(self._tc_parameters.get_param_value("SECURE_ELEMENT"))

        # Read script name from test case xml file
        self._script = str(self._tc_parameters.get_param_value("SCRIPT"))

        self._nfc_robot_param = global_config.benchConfig.get_parameters("NFC_ROBOT1")

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LabNfcBase.set_up(self)

        self._pcd_antenna_x = self._nfc_robot_param.get_param_value("PcdAntennaX")
        self._pcd_antenna_y = self._nfc_robot_param.get_param_value("PcdAntennaY")
        self._pcd_antenna_up = self._nfc_robot_param.get_param_value("PcdAntennaUp")
        self._pcd_antenna_down = self._nfc_robot_param.get_param_value("PcdAntennaDown")

        # Check NFC tool in use is the NFC emulator
        if self._mp_manager.get_model() != "MP_MANAGER":
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "MP Manager must be used!")

        if self._secure_element not in ["EMBEDDED", "UICC"]:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Unknown Secure element")

        if self._secure_element == "EMBEDDED":
            self._logger.info("Try to select embedded secure element")
            self._nfc_api.select_secure_element("com.nxp.smart_mx.ID")
        else:
            self._logger.info("Try to select UICC secure element")
            self._nfc_api.select_secure_element("com.nxp.uicc.ID")

        self._robot_positioning(self._pcd_antenna_x, self._pcd_antenna_y, "null", "null")

        # Put DUT on antenna
        self._robot_positioning("null", "null", self._pcd_antenna_down, "null")

        self._mp_manager.init()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        LabNfcBase.run_test(self)

        self._mp_manager.run_script(self._script)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        LabNfcBase.tear_down(self)

        # Remove DUT from antenna
        self._robot_positioning("null", "null", self._pcd_antenna_up, "null")

        self._mp_manager.release()

        return Global.SUCCESS, "No errors"
