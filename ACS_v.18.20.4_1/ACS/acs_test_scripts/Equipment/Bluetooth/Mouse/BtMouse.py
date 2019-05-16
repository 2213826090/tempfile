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
:summary: Nokia BH214 implementation
:since: 20/02/2014
:author: fbongiax
"""

from acs_test_scripts.Equipment.Bluetooth.IOCardBased import IOCardBased


class BtMouse(IOCardBased):

    """
    Class that implements BlueTooth Mouse equipment
    """

    SLEEP_BEFORE_PAIR_SECS = 5

    def __init__(self, name, model, eqt_params, bench_params, factory=None):
        """
        Constructor
        :type name: str
        :param name: the bench configuration name of the equipment
        :type model: str
        :param model: the model of the equipment
        :type eqt_params: dict
        :param eqt_params: the dictionary containing equipment parameters
        :type bench_params: dict
        :param bench_params: the dictionary containing equipment bench parameters
        """
        IOCardBased.__init__(self, name, model, eqt_params, bench_params, factory)

        self._powerbutton = None
        self._pairbutton = None
        self._left_button = None
        self._right_button = None
        self._middle_button = None
        self._keypresstimer = None
        self._pairtimer = None

    def init(self):
        """
        Initializes the equipment. The equipment is ready to use
        """
        IOCardBased.init(self)

        self.get_logger().info("BT mouse initialization")

        self._powerbutton = self._set_button_line("powerButton")
        self._pairbutton = self._set_button_line("pairButton")
        self._left_button = self._set_button_line("leftButton")
        self._right_button = self._set_button_line("rightButton")
        self._middle_button = self._set_button_line("middleButton")
        self._keypresstimer = self._set_button_timer("keyPressTimer")
        self._pairtimer = self._set_button_timer("pairTimer")

    def switch_on(self):
        """
        Switches the mouse on
        """
        self._raise_error_if_any_is_none([self._powerbutton], "powerButton not configured; No action taken!")
        self._iocard.enable_line(self._powerbutton)

    def switch_off(self):
        """
        Switch the mouse off
        """
        self._raise_error_if_any_is_none([self._powerbutton], "powerButton not configured; No action taken!")
        self._iocard.disable_line(self._powerbutton)

    def set_discoverable(self):
        """
        Set the mouse in pairable mode
        """
        self._raise_error_if_any_is_none([self._powerbutton, self._pairbutton, self._pairtimer],
                                         "powerButton/pairButton/pairTimer not configured; No action taken!")
        # Switch off/on the mouse, to make sure we start from a known state
        self.switch_off()
        self.switch_on()
        self._wait_for_secs(self.SLEEP_BEFORE_PAIR_SECS)
        self._press_relay(self._pairbutton, self._pairtimer)

    def left_click(self):
        """
        Clicks the left button on the mouse
        """
        self._raise_error_if_any_is_none([self._left_button, self._keypresstimer],
                                         "leftButton/keyPressTimer not configured; No action taken!")
        self._press_relay(self._left_button, self._keypresstimer)

    def right_click(self):
        """
        Clicks the right button on the mouse
        """
        self._raise_error_if_any_is_none([self._right_button, self._keypresstimer],
                                         "rightButton/keyPressTimer not configured; No action taken!")
        self._press_relay(self._right_button, self._keypresstimer)

    def middle_click(self):
        """
        Clicks the middle button on the mouse
        """
        self._raise_error_if_any_is_none([self._middle_button, self._keypresstimer],
                                         "middleButton/keyPressTimer not configured; No action taken!")
        self._press_relay(self._middle_button, self._keypresstimer)
