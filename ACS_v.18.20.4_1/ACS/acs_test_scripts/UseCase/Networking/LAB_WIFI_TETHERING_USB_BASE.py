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
:summary: This file implements the Base for all USB TETHERING UseCases
:since: 18/10/2013
:author: apairex
"""

from acs_test_scripts.UseCase.Networking.LAB_WIFI_BASE import LabWifiBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.TestEquipmentException import TestEquipmentException
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.Device.UECmd.UECmdTypes import Measure


class LabWifiTetheringUsbBase(LabWifiBase):

    """
    Lab Wifi Tethering USB FTP Test class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabWifiBase.__init__(self, tc_name, global_config)

        self._computer = None

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # Configure and connect wifi to the AP
        LabWifiBase.set_up(self)

        # Get local computer to run dhclient
        self._computer = self._em.get_computer("COMPUTER1")

        # Check that USBInterface parameter is filled in BenchConfig
        # This is required to request for an DHCP client renew
        if not self._computer.get_usb_interface():
            msg = "USBInterface parameter is not filled in COMPUTER1"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG, msg)

        # Initialize USB Tethering to disable
        self._networking_api.stop_usb_tethering(unplug=True)

        # Disconnect from Bench network
        self._computer.disconnect_from_bench_network()

        # Test that we cannot establish a connection to the bench network
        self._test_server_unreachable(self._wifi_server_ip_address)

        return Global.SUCCESS, "No error"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        # Need to reconnect board to bench Network, before calling tear_down
        if self._computer:
            self._computer.connect_from_bench_network()

        LabWifiBase.tear_down(self)

        # Ensure USB tethering is OFF
        self._networking_api.stop_usb_tethering(unplug=True)

        return Global.SUCCESS, "No error"

#------------------------------------------------------------------------------

    def _test_server_unreachable(self, host):
        """
        Control that the IP route is not available to the host

        :type host: str
        :param host: server on the bench network to ping
        """
        packet_size = 64
        packet_count = 4

        packet_loss = None
        try:
            packet_loss = self._computer.ping(host, packet_size, packet_count)
        except TestEquipmentException:
            # Network is unreachable
            packet_loss = Measure()
            packet_loss.value = 100

        if packet_loss is None or packet_loss.value < 100:
            msg = "Server is still reachable"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
