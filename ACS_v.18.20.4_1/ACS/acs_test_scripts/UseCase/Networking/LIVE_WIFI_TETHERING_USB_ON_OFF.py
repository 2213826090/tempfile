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
:summary: This file implements the LIVE WIFI TETHERING USB ON OFF
:since: 14/10/2013
:author: apairex
"""
from acs_test_scripts.UseCase.Networking.LIVE_WIFI_BASE import LiveWifiBase
from Device.Common.Common import Global
from UtilitiesFWK.Utilities import str_to_bool_ex
from ErrorHandling.DeviceException import DeviceException
import time
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase


class LiveWifiTetheringUsbOnOff(LiveWifiBase):
    """
    Live Wifi Tethering USB ON OFF Test class.
    """
    _TIME_OUT = 15

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LiveWifiBase.__init__(self, tc_name, global_config)

        self._disconnected = False
        self._unplugged = False
        self._test_usb_disconnect = str_to_bool_ex(self._tc_parameters.get_param_value("TEST_USB_DISCONNECT"))

    def set_up(self):
        """
        Initialize the test
        """
        LiveWifiBase.set_up(self)

        if self._test_usb_disconnect is None:
            msg = "invalid TC parameter [TEST_USB_DISCONNECT: %s]. Please use true/false value." \
                    % self._tc_parameters.get_param_value("TEST_USB_DISCONNECT")
            self._logger.error(msg)
            raise DeviceException(DeviceException.INVALID_PARAMETER, msg)

        # Initialize USB tethering status to "disable"
        self._networking_api.stop_usb_tethering(unplug=True)

        # Checks that USB Tethering is disabled
        self._networking_api.check_usb_tethering_state(False)

        return Global.SUCCESS, "No error"

    def run_test(self):
        """
        Execute the test
        """
        LiveWifiBase.run_test(self)

        # Start USB Tethering
        self._networking_api.start_usb_tethering(unplug=True)

        # Control that USB Tethering starts well
        self._networking_api.check_usb_tethering_state(True)

        if self._test_usb_disconnect:
            # Disconnect ADB
            self._device.disconnect_board()
            self._disconnected = True
            # Unplug USB
            self._io_card.usb_host_pc_connector(False)
            self._unplugged = True

            # Wait some time USB cable unplugged
            time.sleep(5)

            # plug USB
            self._io_card.usb_host_pc_connector(True)
            self._unplugged = False
            # Reconnect ADB
            self._device.connect_board()
            self._disconnected = False
        else:
            # Disable USB Tethering
            self._networking_api.stop_usb_tethering(unplug=True)

        # Control that USB Tethering stops well
        self._networking_api.check_usb_tethering_state(False)

        return Global.SUCCESS, "No error"

    def tear_down(self):
        """
        End and dispose the test
        """
        # Handle reconnection in case of crash during a disconnected period
        if self._disconnected:
            UseCaseBase.tear_down(self)
            if self._unplugged:
                self._io_card.usb_host_pc_connector(True)
                self._unplugged = False
            self._device.connect_board()
            self._disconnected = False

        LiveWifiBase.tear_down(self)

        # Reset USB tethering status to "disable"
        self._networking_api.stop_usb_tethering(unplug=True)

        return Global.SUCCESS, "No error"
