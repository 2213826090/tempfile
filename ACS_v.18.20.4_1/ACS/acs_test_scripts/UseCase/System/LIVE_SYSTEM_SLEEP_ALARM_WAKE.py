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
:summary: This file implements the alarm UC
:since: 07/06/2011
:author: vtinelli
"""


import time
from UtilitiesFWK.Utilities import Global
from SYSTEM_SLEEP_BASE import SystemSleepBase


class LiveSystemSleepAlarmWake(SystemSleepBase):

    """
    S0i3 Alarm class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        SystemSleepBase.__init__(self, tc_name, global_config)

        # Get TC parameters
        self._alarm_timer = int(self._tc_parameters.get_param_value("ALARM_USB_TIMER"))

    def run_test(self):
        """
        Execute the test
        """

        SystemSleepBase.run_test(self)

        wake_matched_before_value = self._phonesystem_api.get_sleep_wakeup_count(self._sleep_mode)

        # It's time to sleep !
        time.sleep(self._wait_btwn_cmd)
        self._phonesystem_api.sleep_mode("on")

        # Set alarm to wake up
        time.sleep(self._wait_btwn_cmd)
        start_time = time.time()
        self._phonesystem_api.sleep_mode_for("on", self._duration)

        # Unplug usb cable
        self._device.disconnect_board()
        self._io_card.usb_host_pc_connector(False)
        # Unplug wall charger only if it is AC_CHGR
        if self._device.get_default_wall_charger() == self._io_card.AC_CHGR:
            # Unplug wall charger
            self._io_card.wall_charger_connector(False)

        # Wait for entering in sleep mode
        current_time = time.time() - start_time
        alarm_time_diff = int(self._duration - current_time)
        self._logger.info("Wait for %d s to enter in %s" % (alarm_time_diff, self._sleep_mode))
        time.sleep(alarm_time_diff)

        # Add delay, to avoid USB wake up instead of alarm
        self._logger.info("Wait for %s s to trigger alarm" % str(self._alarm_timer))
        time.sleep(self._alarm_timer)
        # plug wall charger only if it is AC_CHGR
        if self._device.get_default_wall_charger() == self._io_card.AC_CHGR:
            # Plug wall charger
            self._io_card.wall_charger_connector(True)
        # Plug usb cable
        self._io_card.usb_host_pc_connector(True)
        self._device.connect_board()

        # Get the wake up source
        wakeup_source = self._phonesystem_api.get_wakeup_source()
        if wakeup_source == "RTC":
            return_code = Global.SUCCESS
        else:
            return_code = Global.FAILURE

        return_msg = "Device wakes up from %s" % wakeup_source

        # Retrieve wakeup count in sleep mode
        wake_matched_after_value = self._phonesystem_api.get_sleep_wakeup_count(self._sleep_mode)
        wake_diff = wake_matched_after_value - wake_matched_before_value

        if wake_diff <= 0:
            return_msg += " but did not enter in %s" % self._sleep_mode
        else:
            return_msg += " - %s wake-up %d" % (self._sleep_mode, wake_diff)

        return return_code, return_msg
