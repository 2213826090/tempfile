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
:summary: This file implements the System UEcmd for Android device
:since: 19/06/2013
:author: pbluniex
"""
from acs_test_scripts.Device.UECmd.Imp.Android.Common.Base import Base
from acs_test_scripts.Device.UECmd.Interface.System.IKeyEvent import IKeyEvent
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException


class KeyEvent(Base, IKeyEvent):

    """
    :summary: System UEcommands operations for Android platforms
    using an C{Intent} based communication to the I{DUT}.
    """

    def __init__(self, device):
        """
        Constructor.

        :type device: IDevice
        :param device: The DUT
        """
        Base.__init__(self, device)
        IKeyEvent.__init__(self, device)

        # List of keycodes used in ACS
        self.__keycode = None
        self.__key_requested = None
        self.__keycodes = {"HOME": 3,
                           "BACK": 4,
                           "DPAD_UP": 19,
                           "DPAD_DOWN": 20,
                           "DPAD_LEFT": 21,
                           "DPAD_RIGHT": 22,
                           "DPAD_CENTER": 23,
                           "POWER_BUTTON": 26,
                           "TAB": 61,
                           "ENTER": 66,
                           "MENU": 82,
                           "MEDIA_PLAY_PAUSE": 85,
                           "BUTTON_MODE": 110,
                           "ESCAPE": 111,
                           "MOVE_HOME": 122,
                           "MOVE_END": 123}

    def __keyevent(self):
        """
        Generate the event

        :type keycode: integer
        :param keycode: The keycode to press
        """
        if self.__keycode is not None:
            self._exec("adb shell input keyevent %d" % self.__keycode, 10)
        else:
            raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED,
                                  "There is no key event named '%s'" % self.__key_requested)

    def __getattr__(self, attr):
        """
        Dynamic function that send key event on the DUT

        :type attr: str
        :param attr: name of the keycode
        """
        self.__key_requested = attr
        self.__keycode = self.__keycodes.get(attr.upper())
        return self.__keyevent

    def scenario(self, keyevents, delay=0.2):
        """
        Execute many keyevent in one command

        :type keyevents: list
        :param keyevents: list of keyevent
        """

        command = ""
        for keyevent in keyevents:
            keycode = self.__keycodes.get(keyevent.upper(), None)
            if keycode is None:
                self._logger.warning("Keyevent::scenario : Ignore keycode %s" % keycode)
                continue

            if not command:
                command = "input keyevent %s" % keycode
            else:
                command += " && sleep %f && input keyevent %s" % (delay, keycode)

        self._exec("adb shell %s && echo Ok || echo NOk" % command)

    def tap_on_screen(self, loc_x, loc_y):
        """
        Tap on screen
        :type loc_x: int
        :param loc_x: coordinate on the x axis
        :type loc_y: int
        :param loc_y: coordinate on the x axis
        :rtype: None
        :return: None
        """
        log = "Tap on screen on %s %s" % (str(loc_x), str(loc_y))
        self._logger.info(log)

        # Check coordinate values
        if (loc_x, loc_y) <= (0, 0):
            msg = "Invalid parameters for coordinates"
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        cmd = "adb shell input tap %s %s" % (str(loc_x), str(loc_y))
        self._exec(cmd)

    def swipe(self, local_x, local_y, time_swipe, local_swipe_x=0, local_swipe_y=0):
        """
        Do a swipe on screen.
        :type local_x: int
        :param local_x: x coordinate in pixel
        :type local_y: int
        :param local_y: y coordinate in pixel
        :type time_swipe: int
        :param time_swipe: number of milliseconds to perform the swipe
        :type local_swipe_x: int
        :param local_swipe_x: coordinate in pixel of target x
        :type local_swipe_y: int
        :param local_swipe_y: coordinate in pixel of target y
        :rtype: None
        :return: None
        """
        self._logger.info("Swipe on screen")
        # If any target is specified, swipe will be
        # from (local_x, local_y) to (local_x + 300, local_y)
        if local_swipe_x == 0:
            local_swipe_x = local_x + 300
        if local_swipe_y == 0:
            local_swipe_y = local_y

        self._exec("adb shell input touchscreen swipe %s %s %s %s %s" % (local_x, local_y, local_swipe_x, local_swipe_y,
                                                                         time_swipe))
