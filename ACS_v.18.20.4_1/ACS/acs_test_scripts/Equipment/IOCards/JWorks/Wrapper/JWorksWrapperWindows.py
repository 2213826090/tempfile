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

from ctypes import wintypes
import ctypes

from acs_test_scripts.Equipment.IOCards.JWorks.Wrapper.JWorksWrapper import \
    JWorksWrapper, RELAY_STATE_ON, RELAY_STATE_OFF


class JWorksWrapperWindows(JWorksWrapper):
    """
    Class that implements wrapper that will drive J-Works relay cards on Windows
    """

    def __init__(self, dllLoader, logger):
        """
        Constructor
        :type dllLoader: DllLoader
        :param dllLoader: the object that will load JWorks driver dll
        :type logger: Logger
        :param logger: the logger of the relay card instance
        """
        JWorksWrapper.__init__(self, logger)

        # load the driver dll
        dllLoader.load_driver()
        self.__dll = dllLoader.get_dll()

        # ---- get serial number ----
        # get method from dll
        funcProto = ctypes.WINFUNCTYPE(ctypes.c_int, ctypes.c_byte, wintypes.LPSTR, ctypes.c_int)
        jwRelaySerialNumber = funcProto(("jwRelaySerialNumber", self.__dll))
        # call the method
        moduleNum = ctypes.c_byte(1)
        strReturn = wintypes.LPSTR("--------")
        sizeString = ctypes.c_int(len(strReturn.value))
        jwRelaySerialNumber(moduleNum, strReturn, sizeString)
        self._serial_number = strReturn.value

        # ---- get number of relay ----
        # get method from dll
        funcProto = ctypes.WINFUNCTYPE(ctypes.c_short)
        jwRelayNumberOfModules = funcProto(("jwRelayNumberOfModules", self.__dll))
        # call the method
        self._number_of_relay = jwRelayNumberOfModules()

    def is_relay_on(self, relay_num):
        """
        Check if the given relay's state is ON
        :type relay_num: int
        :param relay_num: the number of the relay to check (starting at 1)
        :rtype: bool
        :return: True if relay is ON, False otherwise
        """
        # get method from dll
        funcProto = ctypes.WINFUNCTYPE(ctypes.c_bool, wintypes.LPSTR, ctypes.c_byte)
        jwRelayIsRelayOn = funcProto(("jwRelayIsRelayOn", self.__dll))
        # call the method
        strSerial = wintypes.LPSTR(self._serial_number)
        bRelayNum = ctypes.c_byte(relay_num)
        is_relay_on = jwRelayIsRelayOn(strSerial, bRelayNum)
        return is_relay_on

    def _get_all_relays_state(self):
        """
        :rtype: int
        :return: the relays state as a bit field
        """
        # get method from dll
        funcProto = ctypes.WINFUNCTYPE(ctypes.c_uint, wintypes.LPSTR)
        jwRelayBitPattern = funcProto(("jwRelayBitPattern", self.__dll))
        # call the method
        strSerial = wintypes.LPSTR(self._serial_number)
        relays_state = jwRelayBitPattern(strSerial)
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
        # get method from dll
        funcProto = ctypes.WINFUNCTYPE(ctypes.c_void_p, wintypes.LPSTR, ctypes.c_byte, ctypes.c_byte)
        jwRelaySetRelayState = funcProto(("jwRelaySetRelayState", self.__dll))
        # call the method
        strSerial = wintypes.LPSTR(self._serial_number)
        bRelayNum = ctypes.c_byte(relay_num)
        bState = ctypes.c_byte(state)
        jwRelaySetRelayState(strSerial, bRelayNum, bState)
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
        # we want to address all the relays at once, so we create a field of bit
        # all set to 0 for state OFF, and all set to 1 (depending nb of relays) if ON
        relaysToSetBitValue = 0 if state == RELAY_STATE_OFF else ((2 << self.number_of_relay) - 1)

        # get method from dll
        funcProto = ctypes.WINFUNCTYPE(ctypes.c_void_p, wintypes.LPSTR, ctypes.c_uint)
        jwRelaySetBitPattern = funcProto(("jwRelaySetBitPattern", self.__dll))
        # call the method
        strSerial = wintypes.LPSTR(self._serial_number)
        bRelaysToSetBitValue = ctypes.c_uint(relaysToSetBitValue)
        jwRelaySetBitPattern(strSerial, bRelaysToSetBitValue)
        # check that operation succeeded. IE: if state is ON every relay bit should be 1, else should be 0
        state_table = self._get_all_relays_state()
        is_relay_set = ((state == RELAY_STATE_ON and state_table == relaysToSetBitValue)
                      or (state == RELAY_STATE_OFF and state_table == 0))
        return is_relay_set
