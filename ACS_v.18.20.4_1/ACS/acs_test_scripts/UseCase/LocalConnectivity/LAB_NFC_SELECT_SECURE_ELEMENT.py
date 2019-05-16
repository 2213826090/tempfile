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
:summary: This file implements the LIVE NFC SELECT SECURE ELEMENT UC
:since: 29/06/2012
:author: lpastor

"""

import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.LocalConnectivity.LAB_NFC_BASE import LabNfcBase
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class LabNfcSelectSecureElement(LabNfcBase):

    """
    Live NFC select SE test.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabNfcBase.__init__(self, tc_name, global_config)

        self._nfc_robot_param = global_config.benchConfig.get_parameters("NFC_ROBOT1")

        # Read SE_SELECTION_SEQUENCE from test case xml file
        self._se_selection_sequence = \
            str(self._tc_parameters.get_param_value("SE_SELECTION_SEQUENCE"))

        self._gpshell = self._em.get_nfc_tool("NFC_TOOL1")

        self._reader_x = "0"
        self._reader_y = "0"
        self._reader_up = "0"
        self._reader_down = "0"

    def set_up(self):
        """
        Initialize the test
        """
        LabNfcBase.set_up(self)

        self._reader_x = self._nfc_robot_param.get_param_value("ExternalReaderX")
        self._reader_y = self._nfc_robot_param.get_param_value("ExternalReaderY")
        self._reader_up = self._nfc_robot_param.get_param_value("ExternalReaderUp")
        self._reader_down = self._nfc_robot_param.get_param_value("ExternalReaderDown")

        # Place DUT in front of reader
        self._robot_positioning(self._reader_x, self._reader_y, "null", "null")

        return Global.SUCCESS, "No errors"

    def run_test(self):
        """
        Execute the test
        """
        LabNfcBase.run_test(self)

        self._logger.info("Select following secure element :"
                          + self._se_selection_sequence)
        time.sleep(self._wait_btwn_cmd)

        seqlist = self._se_selection_sequence.strip().split(";")
        for switch in seqlist:
            if switch.upper() == "EMBEDDED":
                self._logger.info("Try to select embedded secure element")
                self._nfc_api.select_secure_element("com.nxp.smart_mx.ID")
                time.sleep(self._wait_btwn_cmd)
                self._check_connection_with_reader()
            elif switch.upper() == "UICC":
                self._logger.info("Try to select SIM card secure element")
                self._nfc_api.select_secure_element("com.nxp.uicc.ID")
                time.sleep(self._wait_btwn_cmd)
                self._check_connection_with_reader()
            elif switch.upper() in ("DISABLED", "NONE"):
                self._logger.info("Disabled Smartcard")
                self._nfc_api.select_secure_element("DISABLED")
                time.sleep(self._wait_btwn_cmd)
            else:
                msg = switch.upper() + "is trying to be set as secure element. It is not a possible value!" \
                    + "You must use DISABLED, UICC or EMBEDDED in test case xml file"
                self._logger.error(msg)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        return Global.SUCCESS, "No errors"

    def _check_connection_with_reader(self):
        """
        Check the connection with the NFC reader
        """
        # Put DUT on reader
        self._robot_positioning("null", "null", self._reader_down, "null")

        # Check external reader detects the device
        if not self._gpshell.check_reader_connection():
            msg = "ERROR : reader cannot detect/connect DUT"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        self._logger.info("Reader detects DUT: secure element is correctly selected")

        # Put DUT away from the reader
        self._robot_positioning("null", "null", self._reader_up, "null")

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        LabNfcBase.tear_down(self)

        # Put DUT away from the reader
        self._robot_positioning("null", "null", self._reader_up, "null")

        return Global.SUCCESS, "No errors"
