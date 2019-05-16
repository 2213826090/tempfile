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

:organization: INTEL PCCG CI
:summary: This file implements the self USB to serial key strokes
:since: 12/05/2013
:author: Sivarama Krishna Vellanki
"""

from acs_test_scripts.Equipment.IEquipment import EquipmentBase
from acs_test_scripts.Equipment.KeyboardEmulator.Interface.IUSBKM import IUSBKM
from ErrorHandling.TestEquipmentException import TestEquipmentException
from ErrorHandling.AcsConfigException import AcsConfigException
import serial
import time


class USBKM232(EquipmentBase, IUSBKM):
    """
        This class implements means to send keystrokes to a device
        on a serial link
    """
    _MAX_CONNECTION_RETRIES = 3
    _MAX_RSP_RETRIES = 10
    _CLEAR = '\x38'
    _DEFAULT_TIMEOUT = 0.5
    _DEFAULT_TIMESLEEP = 1
    _SERIAL_WRITE_TIMESLEEP = 0.05
    _SERIAL_SPEED = 9600
    # QWERTY keyboard keys
    _KEYS = {
        # row 1
        '`': 1,
        '1': 2,
        '2': 3,
        '3': 4,
        '4': 5,
        '5': 6,
        '6': 7,
        '7': 8,
        '8': 9,
        '9': 10,
        '0': 11,
        '-': 12,
        '=': 13,
        '<undef1>': 14,
        '<backspace>': 15,
        '<tab>': 16,
        'q': 17,
        'w': 18,
        'e': 19,
        'r': 20,
        't': 21,
        'y': 22,
        'u': 23,
        'i': 24,
        'o': 25,
        'p': 26,
        '[': 27,
        ']': 28,
        '\\': 29,
        # row 2
        '<capslock>': 30,
        'a': 31,
        's': 32,
        'd': 33,
        'f': 34,
        'g': 35,
        'h': 36,
        'j': 37,
        'k': 38,
        'l': 39,
        ';': 40,
        '\'': 41,
        '<undef2>': 42,
        '<enter>': 43,
        # row 3
        '<lshift>': 44,
        '<undef3>': 45,
        'z': 46,
        'x': 47,
        'c': 48,
        'v': 49,
        'b': 50,
        'n': 51,
        'm': 52,
        ',': 53,
        '.': 54,
        '/': 55,
        '[clear]': 56,
        '<rshift>': 57,
        # row 4
        '<lctrl>': 58,
        '<undef5>': 59,
        '<lalt>': 60,
        ' ': 61,
        '<ralt>': 62,
        '<undef6>': 63,
        '<rctrl>': 64,
        '<undef7>': 65,
        '<mouse_left>': 66,
        '<mouse_right>': 67,
        '<mouse_up>': 68,
        '<mouse_down>': 69,
        '<lwin>': 70,
        '<rwin>': 71,
        '<win apl>': 72,
        '<mouse_lbtn_press>': 73,
        '<mouse_rbtn_press>': 74,
        '<insert>': 75,
        '<delete>': 76,
        '<mouse_mbtn_press>': 77,
        '<undef16>': 78,
        '<larrow>': 79,
        '<home>': 80,
        '<end>': 81,
        '<undef23>': 82,
        '<uparrow>': 83,
        '<downarrow>': 84,
        '<pgup>': 85,
        '<pgdown>': 86,
        '<mouse_scr_up>': 87,
        '<mouse_scr_down>': 88,
        '<rarrow>': 89,
        # numpad
        '<numlock>': 90,
        '<num7>': 91,
        '<num4>': 92,
        '<num1>': 93,
        '<undef27>': 94,
        '<num/>': 95,
        '<num8>': 96,
        '<num5>': 97,
        '<num2>': 98,
        '<num0>': 99,
        '<num*>': 100,
        '<num9>': 101,
        '<num6>': 102,
        '<num3>': 103,
        '<num.>': 104,
        '<num->': 105,
        '<num+>': 106,
        '<numenter>': 107,
        '<undef28>': 108,
        '<mouse_slow>': 109,
        # row 0
        '<esc>': 110,
        '<mouse_fast>': 111,
        '<f1>': 112,
        '<f2>': 113,
        '<f3>': 114,
        '<f4>': 115,
        '<f5>': 116,
        '<f6>': 117,
        '<f7>': 118,
        '<f8>': 119,
        '<f9>': 120,
        '<f10>': 121,
        '<f11>': 122,
        '<f12>': 123,
        '<prtscr>': 124,
        '<scrllk>': 125,
        '<pause/brk>': 126,
    }

    def __init__(self, name, model, eqt_params, bench_params):
        """
        This is the constructor and will initiate serial connection

        Raises:
          AcsConfigException: if no communication port parameter defined in benchconfig file
        """
        EquipmentBase.__init__(self, name, model, eqt_params)
        IUSBKM.__init__(self)
        self.__bench_params = bench_params
        if (self.__bench_params.has_parameter(IUSBKM.COM_PORT_ID_PARAM) and
                    self.__bench_params.get_param_value(IUSBKM.COM_PORT_ID_PARAM) != ""):
            self.__com_port = str(self.__bench_params.get_param_value(IUSBKM.COM_PORT_ID_PARAM))
        else:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "No %s parameter defined in your benchconfig for equipment %s"
                                     % (self.COM_PORT_ID_PARAM,name))
        self.__serial_device = None

    def init(self):
        """
        Initializes the equipment. The equipment is ready to use

        Raises:
          TestEquipmentException: if no connection can be made with the device
        """
        if not self.__serial_device:
            tries = self._MAX_CONNECTION_RETRIES
            while tries > 0:
                try:
                    self.__serial_device = serial.Serial(self.__com_port, self._SERIAL_SPEED,
                                                         serial.EIGHTBITS,
                                                         serial.PARITY_NONE,
                                                         serial.STOPBITS_ONE,
                                                         None)
                    self.__serial_device.setTimeout(self._DEFAULT_TIMEOUT)
                    self.__serial_device.setWriteTimeout(self._DEFAULT_TIMEOUT)
                except serial.SerialException as e:
                    if tries == 1:
                        msg = "No connection to the device could be established: %s" % str(e)
                        raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, msg)
                    tries -= 1
                    time.sleep(self._DEFAULT_TIMESLEEP)
                else:
                    tries = 0

    def __press_key(self, pressed_key):
        """Send char corresponding to key pressed .
        :type pressed_key: str
        :param pressed_key: key to press
        :rtype: object
        :return: Proper encoding to send to the uart side of the usbkm to create the
          desired key press. None if entry key is invalid.
        """
        key = self._KEYS.get('%s' % pressed_key)
        if key:
            res = chr(key)
        else:
            res = None
        return res

    def __release_key(self, released_key):
        """Release key by returning corresponding signal*

        :type released_key: str
        :param released_key: key to release
        :rtype: object
        :return: Proper encoding to send to the uart side of the usbkm to create the
          desired key release. None if entry key is invalid
        """
        key = self._KEYS.get(released_key)
        if key:
            res = '%c' % (key | 0x80)
        else:
            res = None
        return res

    def __response(self, char):
        """
        Check response after sending character to self.
        The response is the one's complement of the value sent. This method
        blocks until proper response is received.
        :type char: str
        :param char: original character sent

        Raises:
          TestEquipmentException: if response was incorrect or timed out
        """
        count = 0
        response = self.__serial_device.read(1)
        self.get_logger().debug("re-read response = " + str(response))
        while (len(response) != 1 or ord(char) != (~ord(response) & 0xff)) \
                and count < self._MAX_RSP_RETRIES:
            response = self.__serial_device.read(1)
            self.get_logger().debug("re-read response = " + str(response))
            time.sleep(self._DEFAULT_TIMESLEEP)
            count += 1
        if count == self._MAX_RSP_RETRIES:
            # in case no response receive try release previous key
            msg = "Failed to get correct response from target device"
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, msg)
        self.get_logger().debug("Target device: response [-] = \\0%03o 0x%02x"
                                % (ord(response), ord(response)))

    def __send_commands(self, cmds_list, check=False, clear=True):
        """
        Write list of commands to self.

        :type cmds_list: list
        :param cmds_list: list of encoded commands to send to the UART side of the
            self.
        :type check: bool
        :param check: determines whether response from self should be
            checked.
        :type clear: bool
        :param clear: determines whether keystroke clear should be sent at end of the sequence
        """
        for i, write_ch in enumerate(cmds_list):
            self.__serial_device.write(write_ch)
            if check:
                self.__response(write_ch)
            time.sleep(self._SERIAL_WRITE_TIMESLEEP)
        if clear:
            self.__serial_device.write(self._CLEAR)
            if check:
                self.__response(self._CLEAR)

    def write_keys(self, keys):
        """
        Write keys to usbkm using a set of predefined keys in
            USBKM232._KEYS

        :type keys: list or str
        :param keys: alphanumeric str or list of commands among _KEYS table to be sent
        """
        commands = []
        if self._KEYS.has_key(keys):
            char = self.__press_key(keys)
            release_char = self.__release_key(keys)
            if char and release_char:
                commands.append(char)
                commands.append(release_char)
            else:
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Input string contains invalid char: "
                                                                                   "%s " % keys)
            self.__send_commands(commands)
        else:
            for key in keys:
                char = self.__press_key(key.lower())
                release_char = self.__release_key(key.lower())
                if char and release_char:
                    commands.append(char)
                    commands.append(release_char)
                else:
                    raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Input string contains invalid char: "
                                                                                   "%s " % keys)
            self.__send_commands(commands)

    def release(self):
        """
        Release usbkm device
        """
        if self.__serial_device:
            self.__serial_device.close()
            self.__serial_device = None
