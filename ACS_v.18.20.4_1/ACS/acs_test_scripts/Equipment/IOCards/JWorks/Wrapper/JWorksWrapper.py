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

from ErrorHandling.TestEquipmentException import TestEquipmentException
import time

# constants for J-Works relay cards
VENDOR_ID = 0x07c3
RELAY_STATE_ON = 1
RELAY_STATE_OFF = 0


class JWorksWrapper(object):
    """
    Abstract Class that implements wrapper that will drive J-Works relay cards
    """

    def __init__(self, logger):
        """
        Constructor
        :type logger: Logger
        :param logger: the logger of the relay card instance
        """
        self._logger = logger
        self._serial_number = "---"
        self._number_of_relay = 0

    @property
    def serial_number(self):
        """
        :rtype: int
        :return: the relay card serial number
        """
        return self._serial_number

    @property
    def number_of_relay(self):
        """
        :rtype: int
        :return: the number of relays available on this card
        """
        return self._number_of_relay

    def is_relay_on(self, relay_num):
        """
        Check if the given relay's state is ON
        :type relay_num: int
        :param relay_num: the number of the relay to check (starting at 1)
        :rtype: bool
        :return: True if relay is ON, False otherwise
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_relay_on(self, relay_num):
        """
        Set the given relay's state to ON
        :type relay_num: int
        :param relay_num: the number of the relay to activate (starting at 1)
        :rtype: bool
        :return: True if relay is ON, False otherwise
        """
        return self._set_relay_state(relay_num, RELAY_STATE_ON)

    def set_relay_off(self, relay_num):
        """
        Set the given relay's state to OFF
        :type relay_num: int
        :param relay_num: the number of the relay to activate (starting at 1)
        :rtype: bool
        :return: True if relay is OFF, False otherwise
        """
        return self._set_relay_state(relay_num, RELAY_STATE_OFF)

    def press_relay(self, relay_num, duration):
        """
        Press relay during time duration, like a button.
        :type relay_num: int
        :param relay_num: the number of the relay to activate (starting at 1)
        :type duration: float
        :param duration: time during which the button is pressed
        :rtype: bool
        :return: True if relay was successfully pressed then released, False otherwise
        """
        is_pressed = False
        # try to set relay to ON
        if not self.set_relay_on(relay_num):
            self._logger.error("Unable to press button")
        else:
            # wait for "press" duration
            time.sleep(duration)
            is_pressed = True
        # release the button
        if not self.set_relay_off(relay_num):
            self._logger.error("Unable to release pressed button")
            is_pressed = False

        return is_pressed

    def clear_all_relays(self):
        """
        Set all relays to OFF at once
        :rtype: bool
        :return: True if all relays are set to OFF, False otherwise
        """
        return self._set_all_relays_state(RELAY_STATE_OFF)

    def set_wiring_table(self, wiring_table):
        """
        Set all relays state at once, using values in wiring table
        :type wiringTable: int
        :param wiringTable: a bit field, containing bit 1 for relays to set ON, and 0 for relays to set OFF
        :raise TestEquipmentException: if relays cannot be set according to wiring table
        """
        self._logger.debug("Setting wiring table")
        # loop over all available relays, and set them one by one
        for relay_index in range(0, self.number_of_relay):
            # get a mask to extract state from wiring table (1 for ON, 0 for OFF)
            mask = 1 << relay_index
            state = RELAY_STATE_ON if (wiring_table & mask) != 0 else RELAY_STATE_OFF
            # relay number starts at 1
            relay_number = relay_index + 1
            if not self._set_relay_state(relay_number, state):
                raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, "Unable to set wiring table")

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def _set_all_relays_state(self, state):
        """
        Set all relays state at once
        :type state: int
        :param state: the state to set (can be RELAY_STATE_OFF or RELAY_STATE_ON)
        :rtype: bool
        :return: True if all relays are set to "state", False otherwise
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)
