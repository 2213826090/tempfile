"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

:organization: UMG PSI Validation
:summary: This file implements the Wifi Direct specific methods.
:since: 2014-10-27
:author: razzix

"""

from acs_test_scripts.Device.UECmd.Imp.Android.KK.Networking.WifiDirect import WifiDirect as WifiDirectKK
from ErrorHandling.DeviceException import DeviceException

class WifiDirect(WifiDirectKK):

    """
    Implementation of the Wifi Direct specific methods.
    """

    def __init__(self, device):
        """
        Constructor
        """
        WifiDirectKK.__init__(self, device)

    def open_wifi_direct_menu_settings(self):
        """
        Display the Wifi direct settings page
        """
        # Go back to HOME to ensure there's no other activity in the front.
        self._device.get_uecmd("PhoneSystem").home_page_menu()

        # Open the Wifi settings
        self._exec("adb shell am start -n com.android.settings/com.android.settings.SubSettings -e :settings:show_fragment com.android.settings.wifi.p2p.WifiP2pSettings")

        # Register the ACS listeners to get the P2P updates from android.
        method = "wifiDirectRegisterListeners"
        self._internal_exec_v2(self._wifidirect_module, method, is_system=True)

    def set_wifi_direct_frequency(self, wifi_direct_frequency):
        """
        Sets the WiFi direct frequency.
        :type wifi_direct_frequency: int
        :param wifi_direct_frequency: Frequency to force. Set a valid WiFi frequency in MHz.
        """
        msg = "set_wifi_direct_frequency is not available for Android Lollipop"
        self._logger.error(msg)
        raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    def set_wifi_direct_mode(self, wifi_direct_mode):
        """
        Sets the WiFi direct mode (CLI/GO).
        :type wifi_direct_mode: String
        :param wifi_direct_mode: Mode to set. Possible values : GO or CLI.
        """
        msg = "set_wifi_direct_mode is not available for Android Lollipop"
        self._logger.error(msg)
        raise DeviceException(DeviceException.OPERATION_FAILED, msg)
