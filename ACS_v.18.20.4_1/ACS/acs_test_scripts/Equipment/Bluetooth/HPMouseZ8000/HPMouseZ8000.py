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

:organization: INTEL OTC Android
:summary: This class implements by differentiation the HP Mouse Z8000 Bluetooth
    Low Energy equipment
:since: 07/20/15
:author: mmaraci
"""

from acs_test_scripts.Equipment.Bluetooth.Mouse.BtMouse import BtMouse
from acs_test_scripts.Equipment.Bluetooth.IOCardBased import IOCardBased


class HPMouseZ8000(BtMouse, IOCardBased):
    """
    This class implements by differentiation the HP Mouse Z8000 Bluetooth
    Low Energy equipment
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
        self._single_button = None
        self._keypresstimer = None
        self._pairtimer = None

    def init(self):
        """
        Initializes the equipment. The equipment is ready to use
        """
        IOCardBased.init(self)

        self.get_logger().info("HPMouseZ8000 mouse initialization")

        self._powerbutton = self._set_button_line("powerButton")
        self._pairbutton = self._set_button_line("pairButton")
        self._single_button = self._set_button_line("middleButton")
        self._keypresstimer = self._set_button_timer("keyPressTimer")
        self._pairtimer = self._set_button_timer("pairTimer")

    def perform_click(self):
        """
        Clicks the only button on the mouse
        """
        self._raise_error_if_any_is_none([self._single_button, self._keypresstimer],
                                         "singleButton/keyPressTimer not configured; No action taken!")
        self._press_relay(self._single_button, self._keypresstimer)