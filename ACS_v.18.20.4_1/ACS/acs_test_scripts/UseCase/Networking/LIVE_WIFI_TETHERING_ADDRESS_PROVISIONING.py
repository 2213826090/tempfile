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

:summary: This use case test the MAC address of the Hot Spot interface and
        verifies that it is the same that the address of the wifi interface
:author: rneux
:since:15/06/2012
"""
import time

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.DeviceException import DeviceException


class LiveWifiTetheringAddressProvisioning(UseCaseBase):

    """
    Lab wifi hotspot ping.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        UseCaseBase.__init__(self, tc_name, global_config)

        # Get hot spot configuration according HOTSPOT_SSID
        self._hotspot_ssid = \
            str(self._tc_parameters.get_param_value("HOTSPOT_SSID"))

        # Get hot spot configuration according HOTSPOT_SECURITY
        self._hotspot_security = \
            str(self._tc_parameters.get_param_value("HOTSPOT_SECURITY"))

        # Get hot spot configuration according HOTSPOT_PASSWORD
        self._hotspot_passphrase = \
            str(self._tc_parameters.get_param_value("HOTSPOT_PASSWORD"))

        # Get hot spot configuration according WIFI_INTERFACE_NAME
        self._wifi_interface = \
            str(self._dut_config.get("wlanInterface"))

        # Get hot spot configuration according TETHER_EXTERNAL_INTERFACE_NAME
        self._hotspot_ext_interface = \
            str(self._dut_config.get("hotspotExtInterface"))
        # Get hot spot configuration according TETHER_INTERNAL_INTERFACE_NAME
        self._hotspot_int_interface = \
            str(self._dut_config.get("hotspotIntInterface"))

        # Get UECmdLayer
        self._networking_api = self._device.get_uecmd("Networking")

        self._original_wifi_power_status = 0
        self._wifi_mac_addr = None
        self._hotspot_ext_mac_addr = None
        self._hotspot_int_mac_addr = None

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        UseCaseBase.set_up(self)

        self._original_wifi_power_status = \
            self._networking_api.get_wifi_power_status()

        # Turn wifi on
        self._networking_api.set_wifi_power("on")
        time.sleep(self._wait_btwn_cmd)

        # get wifi MAC Address
        self._wifi_mac_addr = self._networking_api. \
            get_interface_mac_addr(self._wifi_interface)

        # Turn wifi off
        self._networking_api.set_wifi_power("off")
        time.sleep(self._wait_btwn_cmd)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        UseCaseBase.run_test(self)

        # Configure hot spot and turn on
        self._networking_api.set_wifi_hotspot("on", self._hotspot_ssid,
                                              self._hotspot_security, self._hotspot_passphrase)
        time.sleep(self._wait_btwn_cmd)

        # get Wifi Hot Spot MAC address
        self._hotspot_ext_mac_addr = self._networking_api. \
            get_interface_mac_addr(self._hotspot_ext_interface)
        self._hotspot_int_mac_addr = self._networking_api. \
            get_interface_mac_addr(self._hotspot_int_interface)

        # compare the MAC addresses
        # both hotspot internal and external MAC should be equal to wifi MAC address
        if (self._hotspot_int_mac_addr != self._wifi_mac_addr) or \
                (self._hotspot_int_mac_addr != self._hotspot_ext_mac_addr):
            msg = "MAC Addresses %s or %s do not match reference: %s" % (self._hotspot_ext_mac_addr,
                                                                         self._hotspot_int_mac_addr,
                                                                         self._wifi_mac_addr)
            self._logger.error(msg)
            raise DeviceException(DeviceException.PROHIBITIVE_MEASURE, msg)

        self._logger.info("Hot Spot MAC addresses do match the Wifi MAC address")

        # Turn hotspot off
        self._networking_api.set_wifi_hotspot("off")

        return Global.SUCCESS, "no error"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        UseCaseBase.tear_down(self)

        # Turn hotspot off
        self._networking_api.set_wifi_hotspot("off")

        if self._original_wifi_power_status != 0:
            # Turn wifi on
            self._networking_api.set_wifi_power("on")
            time.sleep(self._wait_btwn_cmd)

        return Global.SUCCESS, "No errors"
