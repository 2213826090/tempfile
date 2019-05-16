"""
@copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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
:summary: This file implements a Test Step to set power of bluetooth headset
:since 15/07/2013
:author: fbongiax
"""

import time
from UtilitiesFWK.Utilities import split_and_strip, TestConst
from acs_test_scripts.TestStep.Equipment.Bluetooth.HeadSetBase import HeadSetBase


class HeadSetPower(HeadSetBase):
    """
    Implements a test step to set power on a HeadSet.
    The expected power state is given to the text step as string.

    Valid Attributes:
        STATE: power state. Valid value are: on/off/pairable/reset_off
            ON: press on/off button on headset without any check of previous
            power state
            OFF: press on/off button on headset without any check of previous
            power state
            PAIRABLE: long press to configure headset discoverable at
            power on, without any check of previous power state.
            RESET_OFF: long press to configure headset discoverable at
            power on, use DUT to scan and find headset, then switch off headset
            STATE's value can be a sequence of states blank separated
    """

    # # Constants
    STR_PAR_STATE = "STATE"
    STR_PAIRABLE = "pairable"
    STR_RESET_OFF = "reset_off"
    STR_SEPARATOR = ","

    STR_PHONE1 = "PHONE1"
    STR_MSG_SET = "Set Headset power state to %s"
    INT_PAUSE_BEFORE_END = 1

    STATES = [TestConst.STR_ON, TestConst.STR_OFF, STR_PAIRABLE, STR_RESET_OFF]

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        HeadSetBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

        dut = self._factory.create_device(self.STR_PHONE1)
        self._dut_bt_api = dut.get_uecmd("LocalConnectivity")

    def run(self, context):
        """
        Execute the test step
        @see BtHeadSetBase
        """

        HeadSetBase.run(self, context)

        assert self._pars.state, "STATE parameter should have been checked by the framework and be valid at this point"
        for state in split_and_strip(self._pars.state.lower(), self.STR_SEPARATOR):
            self._do_set_power(state)

    def _do_set_power(self, state):
        """
        Set the state of the head set

        @type state: string
        @param state: state to switch the head set to
        """

        assert state in self.STATES, "State must be a valid one (%s is not!)" % state

        if state == self.STR_RESET_OFF:
            self._reset_off()
        elif state == self.STR_PAIRABLE:
            self._logger.info("Set Headset power state to ON and Pairable")
            self._api.set_discoverable()
        elif state == TestConst.STR_ON:
            state_b = True
            self._logger.info(self.STR_MSG_SET % TestConst.STR_ON)
            self._api.set_power(state_b)
        else:
            assert state == TestConst.STR_OFF, "This is supposed to be the last available option, so it should be verified"
            state_b = False
            self._logger.info(self.STR_MSG_SET % TestConst.STR_OFF)
            self._api.set_power(state_b)

        time.sleep(self.INT_PAUSE_BEFORE_END)

    def _reset_off(self):
        self._logger.info("Clear Headset paired device and turn it OFF "
            "using DUT bluetooth scan")
        bt_was_off = self._turn_dut_bt_on_if_needed()
        # Set HS in pairing mode
        self._api.set_discoverable()
        # DUT scan to discover remote headset
        if self._dut_bt_api.bt_find_device(self._api.get_bdaddress()):
            # Turn headset OFF
            self._logger.info(self.STR_MSG_SET % self.STR_RESET_OFF)
            self._api.set_power(False)
        else:
            self._api.set_discoverable()
            time.sleep(1)
            # then turn it off
            self._logger.info(self.STR_MSG_SET % self.STR_RESET_OFF)
            self._api.set_power(False)  # Headset is now OFF, turn on and pairable again
        # If Bluetooth was OFF before that step, turn it off again
        if bt_was_off:
            self._dut_bt_api.set_bt_power("off")

    def _turn_dut_bt_on_if_needed(self):
        bt_was_off = False
        if (self._dut_bt_api.get_bt_power_status_eot() == 'STATE_OFF'):
            bt_was_off = True
            self._dut_bt_api.set_bt_power("on")
        return bt_was_off

