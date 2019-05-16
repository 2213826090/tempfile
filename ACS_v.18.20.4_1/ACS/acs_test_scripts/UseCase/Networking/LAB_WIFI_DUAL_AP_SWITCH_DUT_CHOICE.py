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
:summary: This file implements the LAB WIFI DUAL AP SWITCH NEXT TO DUT CHOICE
:since: 19/04/2012 BZ2827
:author: apairex
"""

from acs_test_scripts.UseCase.Networking.LAB_WIFI_DUAL_BASE import LabWifiDualBase
from UtilitiesFWK.Utilities import Global

import time
from ErrorHandling.DeviceException import DeviceException


class LabWifiDualApSwitchDutChoice(LabWifiDualBase):

    """
    Lab Wifi Dual AP Switch Next to DUT Choice Test class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabWifiDualBase.__init__(self, tc_name, global_config)

        # Parameter for ping controls
        self._packetsize = 32
        self._packnb = 4
        self._lossrate = 25

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LabWifiDualBase.set_up(self)

        # Disconnect Wifi from both APs
        self._networking_api.wifi_disconnect_all()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        LabWifiDualBase.run_test(self)

        # Connect DUT to the 1st AP and check connection SSID
        self._networking_api.wifi_connect(self._ssid, True)
        time.sleep(self._wait_btwn_cmd)

        # Waiting for an IP address from DHCP
        self._networking_api.wait_for_wifi_dhcp_connected()

        # retrieve DUT ip address
        dut_ip_address = self._networking_api.get_wifi_ip_address()

        # Checks connection using Ping
        packetloss = self._networking_api.ping(self._wifirouter_ip,
                                               self._packetsize, self._packnb,
                                               source_address=dut_ip_address)
        if packetloss.value > self._lossrate:
            msg = "AP1: Ping command fails (%s% loss)" % str(packetloss.value)
            self._logger.error(msg)
            raise DeviceException(DeviceException.PROHIBITIVE_MEASURE, msg)

        # Connect DUT to the 2nd AP check connection SSID
        self._networking_api.wifi_connect(self._ssid_ap2, True)
        time.sleep(self._wait_btwn_cmd)

        # Waiting for an IP address from DHCP
        self._networking_api.wait_for_wifi_dhcp_connected()

        # retrieve DUT ip address
        dut_ip_address = self._networking_api.get_wifi_ip_address()

        # Checks connection using Ping
        packetloss = self._networking_api.ping(self._wifirouter_ip_ap2,
                                               self._packetsize, self._packnb,
                                               source_address=dut_ip_address)
        if packetloss.value > self._lossrate:
            msg = "AP2: Ping command fails (%s% loss)" % str(packetloss.value)
            self._logger.error(msg)
            raise DeviceException(DeviceException.PROHIBITIVE_MEASURE, msg)

        return Global.SUCCESS, "No errors"
