"""
:copyright: (c)Copyright 2012, Intel Corporation All Rights Reserved.
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
:summary: Card emulation detection over a real reader
:since: 6/05/2013
:author: smaurelx
"""

from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.LocalConnectivity.LAB_NFC_BASE import LabNfcBase
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class LabNfcCardEmulDetection(LabNfcBase):

    """
    Lab NFC Adapter on/off field on test
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabNfcBase.__init__(self, tc_name, global_config)

        self._reader_x = "0"
        self._reader_y = "0"
        self._reader_up = "0"
        self._reader_down = "0"

        self._nfc_robot_param = global_config.benchConfig.get_parameters("NFC_ROBOT1")

        # Get Secure element to communicate with from test case xml file
        self._secure_element = self._tc_parameters.get_param_value("SECURE_ELEMENT", "").upper()

        self._gpshell = self._em.get_nfc_tool("NFC_TOOL1")

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LabNfcBase.set_up(self)

        self._reader_x = self._nfc_robot_param.get_param_value("ExternalReaderX")
        self._reader_y = self._nfc_robot_param.get_param_value("ExternalReaderY")
        self._reader_up = self._nfc_robot_param.get_param_value("ExternalReaderUp")
        self._reader_down = self._nfc_robot_param.get_param_value("ExternalReaderDown")

        if self._secure_element not in ["EMBEDDED", "UICC"]:
            msg = "Unknown Secure element"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        if self._secure_element == "EMBEDDED":
            self._logger.info("Try to select embedded secure element")
            self._nfc_api.select_secure_element("com.nxp.smart_mx.ID")
        else:
            self._logger.info("Try to select UICC secure element")
            self._nfc_api.select_secure_element("com.nxp.uicc.ID")

        # Place DUT in front of reader
        self._robot_positioning(self._reader_x, self._reader_y, "null", "null")

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        LabNfcBase.run_test(self)

        # Put DUT away from the reader
        self._robot_positioning("null", "null", self._reader_up, "null")

        # Enable NFC
        self._nfc_api.nfc_enable()

        # Put DUT on reader
        self._robot_positioning("null", "null", self._reader_down, "null")

        # Check external reader detects the device
        if not self._gpshell.check_reader_connection():
            msg = "Error: DUT with NFC ON over the reader -"\
                + " NFC on but reader cannot detect/connect it"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        self._logger.info("DUT with NFC ON over the reader - Detected: Ok")

        # Put DUT away from the reader
        self._robot_positioning("null", "null", self._reader_up, "null")

        # Disable NFC
        self._nfc_api.nfc_disable()

        # Put DUT on reader
        self._robot_positioning("null", "null", self._reader_down, "null")

        # Check external reader cannot detect the device
        if self._gpshell.check_reader_connection():
            msg = "Error: DUT with NFC OFF over the reader -"\
                + " NFC on but reader cannot detect/connect it"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        self._logger.info("DUT with NFC OFF over the reader - Not Detected: Ok")

        # Put DUT away from the reader
        self._robot_positioning("null", "null", self._reader_up, "null")

        # Switch off the phone
        self._device.switch_off()

        # Put DUT on reader
        self._robot_positioning("null", "null", self._reader_down, "null")

        # Check external reader cannot detect the device
        if self._gpshell.check_reader_connection():
            msg = "Error: DUT with Power OFF over the reader -" \
                + " NFC on but reader cannot detect/connect it"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        self._logger.info("DUT with Power OFF over the reader - Not Detected: Ok")

        # Put DUT away from the reader
        self._robot_positioning("null", "null", self._reader_up, "null")

        # Switch on the phone
        error_code, error_msg = self._device.switch_on()
        if error_code != Global.SUCCESS:
            msg = "Error: DUT switch On [" + error_code + "] " + error_msg
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # Put DUT on reader
        self._robot_positioning("null", "null", self._reader_down, "null")

        # Check external reader cannot detect the device
        if self._gpshell.check_reader_connection():
            msg = "Error: DUT with Power ON over the reader -" \
                + " NFC on but reader cannot detect/connect it"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        self._logger.info("DUT with Power ON (and NFC OFF) over the reader -"
                          + " Not Detected: Ok")

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        LabNfcBase.tear_down(self)

        # Put DUT away from the reader
        self._robot_positioning("null", "null", self._reader_up, "null")

        return Global.SUCCESS, "No errors"
