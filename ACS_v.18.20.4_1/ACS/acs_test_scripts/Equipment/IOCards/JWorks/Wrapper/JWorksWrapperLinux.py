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
:summary: wrapper for J-Works relay card
:since: 11/12/2013
:author: vdechefd
"""

from acs_test_scripts.Equipment.IOCards.JWorks.Wrapper.JWorksWrapper import \
    JWorksWrapper, VENDOR_ID, RELAY_STATE_ON, RELAY_STATE_OFF
import libusb1
import usb1

# request type for linux driver
REQUEST_TYPE = libusb1.LIBUSB_TYPE_VENDOR | libusb1.LIBUSB_RECIPIENT_ENDPOINT


class JWorksWrapperLinux(JWorksWrapper):
    """
    Class that implements wrapper that will drive J-Works relay cards on linux
    """

    def __init__(self, product_cmds, logger):
        """
        Constructor
        :type product_cmds: dict
        :param product_cmds: dictionary containing the commands to send through the linux driver
        :type logger: Logger
        :param logger: the logger of the relay card instance
        """
        JWorksWrapper.__init__(self, logger)
        self.__cmds = product_cmds

        # get a handle on the USB device
        context = usb1.USBContext()
        self.__deviceHandle = context.openByVendorIDAndProductID(VENDOR_ID, self.__cmds["PRODUCT_ID"])

        # retrieve serial number from the device
        self._serial_number = self.__deviceHandle.getASCIIStringDescriptor(self.__cmds["SERIAL_NUM_INDEX"])

        # get number of relay on this device
        result = self.__deviceHandle.controlRead(REQUEST_TYPE, self.__cmds["REQUEST_NUM_RELAY"], 0, 0, 8)
        self._number_of_relay = ord(result[0])

    def is_relay_on(self, relay_num):
        """
        Check if the given relay's state is ON
        :type relay_num: int
        :param relay_num: the number of the relay to check (starting at 1)
        :rtype: bool
        :return: True if relay is ON, False otherwise
        """
        # get all relays state as a bit field
        relays_state = self._get_all_relays_state()
        # get a mask for the relays_state, depending on the relay number
        relay_index = relay_num - 1
        relay_bit_value = 1 << relay_index
        # combine the mask and the bit field to obtain relay status
        is_relay_on = (relays_state & relay_bit_value) != 0
        return is_relay_on

    def _get_all_relays_state(self):
        """
        :rtype: int
        :return: the relays state as a bit field
        """
        # get all relays state from the relay card
        state_table = self.__deviceHandle.controlRead(REQUEST_TYPE, self.__cmds["REQUEST_RELAY_STATE"], 1, 0, 8)
        # the first char of state_table contains the relays state as a bit field
        relays_state = ord(state_table[0])
        return relays_state

    def _set_relay_state(self, relay_num, state):
        """
        Set the given relay's state
        :type relay_num: int
        :param relay_num: the number of the relay to activate (starting at 1)
        :type state: int
        :param state: the state to set (can be RELAY_STATE_OFF or RELAY_STATE_ON)
        :rtype: bool
        :return: True if relay is set to "state", False otherwise
        """
        # define the request to send through USB, depending the desired state
        request = (self.__cmds["REQUEST_SET_RELAY"] if state == RELAY_STATE_ON else self.__cmds["REQUEST_CLEAR_RELAY"])
        # create the bit field containing "1" for the relay to set
        relay_index = relay_num - 1
        relay_bit_value = 1 << relay_index
        # send the request to the relay card
        self.__deviceHandle.controlRead(REQUEST_TYPE, request, relay_bit_value, 0, 8)
        # check that operation succeeded
        is_relay_on = self.is_relay_on(relay_num)
        is_relay_set = (state == RELAY_STATE_ON and is_relay_on) or (state == RELAY_STATE_OFF and not is_relay_on)
        return is_relay_set

    def _set_all_relays_state(self, state):
        """
        Set all relays state at once
        :type state: int
        :param state: the state to set (can be RELAY_STATE_OFF or RELAY_STATE_ON)
        :rtype: bool
        :return: True if all relays are set to "state", False otherwise
        """
        # we want to address all the relays at once, so we create a field of bit, all at 1
        relaysToSetBitValue = (2 << self.number_of_relay) - 1
        # define request type (relay ON or OFF) depending desired state
        request = (self.__cmds["REQUEST_SET_RELAY"] if state == RELAY_STATE_ON else self.__cmds["REQUEST_CLEAR_RELAY"])
        # send the request to the card
        self.__deviceHandle.controlRead(REQUEST_TYPE, request, relaysToSetBitValue, 0, 8)
        # check that operation succeeded. IE: if state is ON every relay bit should be 1, else should be 0
        state_table = self._get_all_relays_state()
        is_relay_set = ((state == RELAY_STATE_ON and state_table == relaysToSetBitValue)
                      or (state == RELAY_STATE_OFF and state_table == 0))
        return is_relay_set
