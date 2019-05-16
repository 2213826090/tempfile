"""
:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
The source code contained or described herein and all documents related
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
:summary: Toggle screen On/Of when NFC is enabled and check CrashLog
:since: 19/11/2012
:author: smaurelx
"""

import time
from acs_test_scripts.UseCase.LocalConnectivity.LIVE_NFC_BASE import LiveNfcBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.DeviceException import DeviceException


class LiveNfcToggleScreenONOff(LiveNfcBase):

    """
    Lab NFC select SE test
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LiveNfcBase.__init__(self, tc_name, global_config)

        # Retreive test parameters
        self._duration_on = int(self._tc_parameters.get_param_value("ON_DURATION"))
        self._duration_off = int(self._tc_parameters.get_param_value("OFF_DURATION"))

        # Get Tools instances
        self._phone_system_api = self._device.get_uecmd("PhoneSystem")

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LiveNfcBase.set_up(self)

        # Switch phone screen to off if it's on
        self._logger.info("Switch off the screen")
        if self._phone_system_api.get_screen_status():
            self._io_card.press_power_button(0.1)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        LiveNfcBase.run_test(self)

        # Remain the start test time
        start_time = time.time()

        # Power Button to On
        if not self._phone_system_api.get_screen_status():
            self._io_card.press_power_button(0.1)
        time.sleep(self._duration_on)

        # Power Button to Off
        if self._phone_system_api.get_screen_status():
            self._io_card.press_power_button(0.1)
        time.sleep(self._duration_off)

        if self._nfc_api.check_nfc_crash(start_time):
            msg = "NFC crash detected (see log above) or crash info cannot be retrieved"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        return Global.SUCCESS, "No errors"
