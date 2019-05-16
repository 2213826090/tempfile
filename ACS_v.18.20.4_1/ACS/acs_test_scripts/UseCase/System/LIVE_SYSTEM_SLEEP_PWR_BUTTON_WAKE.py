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
:summary: This file implements the PWR button wake-up UC
:since: 10/24/2011
:author: vtinelli
"""

import time
from UtilitiesFWK.Utilities import Global
from SYSTEM_SLEEP_BASE import SystemSleepBase


class LiveSystemSleepPowerButtonWake(SystemSleepBase):

    """
    S0i3 Power Button wake-up class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        SystemSleepBase.__init__(self, tc_name, global_config)

        # Get TC parameters
        self._pwr_btn_timer = int(self._tc_parameters.get_param_value("PWRBTN_USB_TIMER"))

    def run_test(self):
        """
        Execute the test
        """

        SystemSleepBase.run_test(self)

        # Get initial wake up count
        wake_matched_before_value = \
            self._phonesystem_api.get_sleep_wakeup_count(self._sleep_mode)

        # It's time to sleep !
        time.sleep(self._wait_btwn_cmd)
        self._phonesystem_api.sleep_mode("on")

        # Unplug usb cable
        if self._io_card is not None:
            self._device.disconnect_board()
            self._io_card.usb_host_pc_connector(False)
            # unplug wall charger only if it is AC_CHGR
            if self._device.get_default_wall_charger() == self._io_card.AC_CHGR:
                self._io_card.wall_charger_connector(False)

        # Wait for entering in sleep mode
        self._logger.info("Wait for %s s to enter in %s" % (str(self._duration), self._sleep_mode))
        time.sleep(self._duration)

        # Wake-up with power button
        self._io_card.press_power_button(0.1)
        time.sleep(self._pwr_btn_timer)

        # Plug usb cable
        if self._io_card is not None:
            # plug wall charger only if it is AC_CHGR
            if self._device.get_default_wall_charger() == self._io_card.AC_CHGR:
                self._io_card.wall_charger_connector(True)
            self._io_card.usb_host_pc_connector(True)
            if not self._device.is_available():
                self._device.connect_board()

        # Get the wake up source
        wakeup_source = self._phonesystem_api.get_wakeup_source()
        if wakeup_source == "PWR_BTN":
            return_code = Global.SUCCESS
        else:
            return_code = Global.FAILURE

        return_msg = "Device wakes up from %s" % wakeup_source

        # Get last wake up count
        wake_matched_after_value = \
            self._phonesystem_api.get_sleep_wakeup_count(self._sleep_mode)
        wake_diff = wake_matched_after_value - wake_matched_before_value

        if wake_diff <= 0:
            return_msg += " but did not enter in %s" % self._sleep_mode
        else:
            return_msg += " - %s wake-up %d" % (self._sleep_mode, wake_diff)

        return return_code, return_msg
