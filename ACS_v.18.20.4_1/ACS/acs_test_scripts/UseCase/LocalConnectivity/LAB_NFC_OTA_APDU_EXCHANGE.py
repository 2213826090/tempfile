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
:summary: This file implements the LAB NFC OTA APDU EXCHANGE UC
:since: 20/10/2012
:author: lpastor
"""

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.LocalConnectivity.LAB_NFC_BASE import LabNfcBase
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class LabNfcOtaApduExchange(LabNfcBase):

    """
    Lab NFC select SE test
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabNfcBase.__init__(self, tc_name, global_config)

        # Get the NFC tool equipment
        self._nfc_tool = self._em.get_nfc_tool("NFC_TOOL3")

        # Get DUT state from test case xml file
        self._dut_state = \
            str(self._tc_parameters.get_param_value("DUT_STATE"))

        # Get Secure element to communicate with from test case xml file
        self._secure_element = \
            str(self._tc_parameters.get_param_value("SECURE_ELEMENT"))

        # Get protocol type to use from test case xml file
        self._protocol_type = \
            str(self._tc_parameters.get_param_value("PROTOCOL_TYPE"))

        # Get APDU case to use from test case xml file
        self._apdu_case = \
            str(self._tc_parameters.get_param_value("APDU_CASE"))

        # Get bitrate to use from test case xml file
        self._bitrate = \
            str(self._tc_parameters.get_param_value("BITRATE"))

        # Get data length to use from test case xml file
        self._data_length = \
            str(self._tc_parameters.get_param_value("DATA_LENGTH"))

        # Get the number of commands to send to stress the system
        self._loop = \
            str(self._tc_parameters.get_param_value("LOOP"))

        self._phone_system_api = self._device.get_uecmd("PhoneSystem")

        self._nfc_robot_param = global_config.benchConfig.get_parameters("NFC_ROBOT1")

        self._reader_x = "0"
        self._reader_y = "0"
        self._reader_up = "0"
        self._reader_down = "0"

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LabNfcBase.set_up(self)

        # check parameter validity
        if self._dut_state not in ["UNLOCKED", "SCREEN_OFF", "PHONE_OFF"]:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Unknown DUT's state")

        if self._secure_element not in ["EMBEDDED", "UICC"]:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Unknown Secure element")

        if self._protocol_type not in ["A", "a", "B", "b"]:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Unknown Protocol type")

        if self._apdu_case not in ["1", "2", "3", "4", "ALL"]:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Unknown APDU case")

        if self._bitrate not in ["106", "212", "424"]:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Invalid bitrate")

        if not self._data_length.isdigit() or int(self._data_length) < 1 or int(self._data_length) > 256:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Invalid data length")
        self._data_length = int(self._data_length)

        if not self._loop.isdigit():
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Invalid loop parameter")
        self._loop = int(self._loop)

        if self._secure_element == "EMBEDDED":
            self._logger.info("Try to select embedded secure element")
            self._nfc_api.select_secure_element("com.nxp.smart_mx.ID")
        else:
            self._logger.info("Try to select UICC secure element")
            self._nfc_api.select_secure_element("com.nxp.uicc.ID")

        # put DUT in expected state : unlocked, screen off, phone off
        if self._dut_state == "SCREEN_OFF":
            self._logger.info("Switch off the screen")
            if self._phone_system_api.get_screen_status():
                self._io_card.press_power_button(0.1)
            if self._phone_system_api.get_screen_status():
                msg = "Screen should be switched off"
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        elif self._dut_state == "PHONE_OFF":
            self._logger.info("Switch off the phone")
            self._device.hard_shutdown(False)

        # Check NFC tool in use and get target coordinates accordingly
        if self._nfc_tool.get_model() == "GPSHELL":
            self._reader_x = self._nfc_robot_param.get_param_value("ExternalReaderX")
            self._reader_y = self._nfc_robot_param.get_param_value("ExternalReaderY")
            self._reader_up = self._nfc_robot_param.get_param_value("ExternalReaderUp")
            self._reader_down = self._nfc_robot_param.get_param_value("ExternalReaderDown")
        elif self._nfc_tool.get_model() == "NFC_EMULATOR":
            self._reader_x = self._nfc_robot_param.get_param_value("PcdAntennaX")
            self._reader_y = self._nfc_robot_param.get_param_value("PcdAntennaY")
            self._reader_up = self._nfc_robot_param.get_param_value("PcdAntennaUp")
            self._reader_down = self._nfc_robot_param.get_param_value("PcdAntennaDown")
        else:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Invalid NFC tool")

        # place DUT in front of reader
        self._robot_positioning(self._reader_x, self._reader_y, "null", "null")

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        LabNfcBase.run_test(self)

        # Put DUT on reader
        self._robot_positioning("null", "null", self._reader_down, "null")

        # Run program that sends APDU
        result = self._nfc_tool.exchange_apdu(self._protocol_type,
                                              self._bitrate,
                                              self._data_length,
                                              self._apdu_case,
                                              self._loop)

        # Remove DUT from reader
        self._robot_positioning("null", "null", self._reader_up, "null")

        if not result:
            raise DeviceException(DeviceException.OPERATION_FAILED, "Error during apdu exchange test")

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        UseCaseBase.tear_down(self)

        # In case of failure, restore robot initial position
        self._robot_positioning("null", "null", self._reader_up, "null")

        # Put DUT in initial state
        if self._dut_state == "SCREEN_OFF":
            self._logger.info("Switch on the screen")
            self._io_card.press_power_button(0.1)

        elif self._dut_state == "PHONE_OFF":
            self._logger.info("Switch on the phone")
            self._device.switch_on()

        LabNfcBase.tear_down(self)

        return Global.SUCCESS, "No errors"
