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
:summary: This file implements the LAB NFC EXCHANGE CONTACT UC
:since: 11/03/2013
:author: lpastor

"""

import threading
import time
import Queue
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.LocalConnectivity.LAB_NFC_BASE import LabNfcBase
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class LabNfcEmulExchangeContact(LabNfcBase):

    """
    Lab NFC exchange contact.
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

        # Get PhoneSystem UECmdLayer
        self._phone_system_api = self._device.get_uecmd("PhoneSystem")
        # Get ContentOperate UECmdLayer
        self._contacts_api = self._device.get_uecmd("Contacts")
        self._screen_resolution = ""

        # Create NFC tool
        self._nfc_tool = self._em.get_nfc_tool("NFC_TOOL2")

        # Read P2P role from test case xml file
        self._role = \
            str(self._tc_parameters.get_param_value("ROLE"))

        # Read P2P mode from test case xml file
        self._mode = \
            str(self._tc_parameters.get_param_value("MODE"))

        # Read P2P bitrate from test case xml file
        self._bitrate = \
            str(self._tc_parameters.get_param_value("BITRATE"))

        # Read P2P direction from test case xml file
        self._direction = \
            str(self._tc_parameters.get_param_value("DIRECTION"))

        self._nfc_robot_param = global_config.benchConfig.get_parameters("NFC_ROBOT1")

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LabNfcBase.set_up(self)

        # Check NFC tool in use is the NFC emulator
        if self._nfc_tool.get_model() != "NFC_EMULATOR":
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "NFC emulator must be used!")

        self._role = self._role.upper()
        self._mode = self._mode.upper()
        self._direction = self._direction.upper()

        # check parameter validity
        if self._role not in ["INITIATOR", "TARGET"]:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Unknown P2P role")

        if self._mode not in ["ACTIVE", "PASSIVE"]:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Unknown P2P mode")

        if self._bitrate not in ["106", "212", "424"]:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Invalid bitrate")

        if self._direction not in ["SEND", "RECEIVE"]:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Unknown P2P communication direction")

        if self._mode == "PASSIVE" and self._role == "INITIATOR":
            self._antenna_x = int(self._nfc_robot_param.get_param_value("PiccAntennaX"))
            self._antenna_y = int(self._nfc_robot_param.get_param_value("PiccAntennaY"))
            self._antenna_up = int(self._nfc_robot_param.get_param_value("PiccAntennaUp"))
            self._antenna_down = int(self._nfc_robot_param.get_param_value("PiccAntennaDown"))
        else:
            self._antenna_x = int(self._nfc_robot_param.get_param_value("PcdAntennaX"))
            self._antenna_y = int(self._nfc_robot_param.get_param_value("PcdAntennaY"))
            self._antenna_up = int(self._nfc_robot_param.get_param_value("PcdAntennaUp"))
            self._antenna_down = int(self._nfc_robot_param.get_param_value("PcdAntennaDown"))

        self._robot_positioning(self._antenna_x, self._antenna_y, "null", "null")

        # set P2P configuration
        self._nfc_api.set_nfc_p2p_configuration(self._mode, self._role, self._bitrate)
        self._screen_resolution = self._phone_system_api.get_screen_resolution()

        if self._direction == "RECEIVE":
            self._contacts_api.contact_all_delete()
        else:
            self._contacts_api.contact_insert(self._nfc_tool.CONTACT_EXPECTED_BY_EMULATOR)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        LabNfcBase.run_test(self)

        if self._direction == "RECEIVE":
            self.__nfc_p2p_receive_contact()
        else:
            self.__nfc_p2p_send_contact()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        UseCaseBase.tear_down(self)

        # restore default P2P configuration
        self._nfc_api.set_default_nfc_p2p_configuration()

        LabNfcBase.tear_down(self)

        # In case of failure, restore robot initial position
        self._robot_positioning("null", "null", self._antenna_up, "null")

        if self._direction == "SEND":
            # delete contact
            self._contacts_api.contact_delete(self._nfc_tool.CONTACT_EXPECTED_BY_EMULATOR)
            # check contact is removed
            contact_count = self._contacts_api.check_contact_in_list(self._nfc_tool.CONTACT_EXPECTED_BY_EMULATOR)

            if contact_count is not "0":
                self._logger.error("Contact has not been removed!")
                raise DeviceException(DeviceException.OPERATION_FAILED, "Error deleting contact")

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def __nfc_p2p_receive_contact(self):
        """
        Run "receive contact" scenario
        """
        if self._mode == "ACTIVE" or self._role == "INITIATOR":
            robot_on_thread = threading.Thread(target=self._robot_positioning,
                                               args=('null', 'null', self._antenna_down, 'null'))

            robot_on_thread.start()
            time.sleep(3)

            # Run program to receive contact
            self._nfc_tool.exchange_contact(self._mode, self._role, self._bitrate, "RECEIVE")
            robot_on_thread.join()

            # Remove DUT from antenna
            self._robot_positioning("null", "null", self._antenna_up, "null")

        else:
            # Put DUT on antenna
            self._robot_positioning("null", "null", self._antenna_down, "null")

            # Run program to receive contact
            self._nfc_tool.exchange_contact(self._mode, self._role, self._bitrate, "RECEIVE")

            # Remove DUT from antenna
            self._robot_positioning("null", "null", self._antenna_up, "null")

        # check contact is received
        contact_count = self._contacts_api.check_contact_in_list(self._nfc_tool.CONTACT_SENT_BY_EMULATOR)

        if contact_count is not "1":
            raise DeviceException(DeviceException.OPERATION_FAILED, "Error receiving contact")

        # delete contact
        self._contacts_api.contact_delete(self._nfc_tool.CONTACT_SENT_BY_EMULATOR)
        # check contact is removed
        contact_count = self._contacts_api.check_contact_in_list(self._nfc_tool.CONTACT_SENT_BY_EMULATOR)

        if contact_count is not "0":
            self._logger.error("Contact has not been removed!")
            raise DeviceException(DeviceException.OPERATION_FAILED, "Error deleting contact")

#------------------------------------------------------------------------------

    def __nfc_p2p_send_contact(self):
        """
        Run "send contact" scenario
        """
        result_queue = Queue.Queue()
        emulation_thread = threading.Thread(target=self._nfc_tool.exchange_contact,
                                            args=(self._mode, self._role, self._bitrate, 'SEND', result_queue))

        self._contacts_api.contact_display(self._nfc_tool.CONTACT_EXPECTED_BY_EMULATOR)
        time.sleep(1)

        if self._mode == "ACTIVE" or self._role == "INITIATOR":
            robot_on_thread = threading.Thread(target=self._robot_positioning,
                                               args=('null', 'null', self._antenna_down, 'null'))

            robot_on_thread.start()
            time.sleep(3)

            # Run program to receive contact
            emulation_thread.start()
            self._nfc_api.nfc_touch_to_beam(self._screen_resolution)

            robot_on_thread.join()

            # Remove DUT from antenna
            self._robot_positioning("null", "null", self._antenna_up, "null")

        else:
            # Put DUT on antenna
            self._robot_positioning("null", "null", self._antenna_down, "null")

            # Run program to send contact
            emulation_thread.start()
            self._nfc_api.nfc_touch_to_beam(self._screen_resolution)

            # Remove DUT from antenna
            self._robot_positioning("null", "null", self._antenna_up, "null")

        emulation_thread.join()
        result = result_queue.get()

        if not result:
            self._logger.error("Contact not correctly received by emulator")
            raise DeviceException(DeviceException.OPERATION_FAILED, "Error during P2P contact exchange test")
