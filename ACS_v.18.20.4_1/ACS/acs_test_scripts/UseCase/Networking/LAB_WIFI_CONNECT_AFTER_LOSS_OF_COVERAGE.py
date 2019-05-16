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
:summary: This file implements the LIVE WIFI CONNECT AFTER LOSS OF COVERAGE UC
:since: 12/04/2012 BZ2408
:author: apairex
"""

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from acs_test_scripts.UseCase.Networking.LAB_WIFI_BASE import LabWifiBase
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Device.UECmd.UECmdTypes import Measure

import time
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsBaseException import AcsBaseException


class LabWifiConnectAfterLossOfCoverage(LabWifiBase):

    """
    Lab Wifi Connect After a Loss Of Coverage Test class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabWifiBase.__init__(self, tc_name, global_config)

        # Parameter for ping controls
        self._packetsize = 32
        self._packnb = 4
        self._lossrate = 25
        self._lossrate_disconnected = 99
        # source IP address to use (interface from which the ping will be send)
        self.__dut_ip_adr = None

        # Time to wait for the DUT to reconnect after Wifi is back
        self._wifi_reconnection_time = 60

        # Get UECmdLayer for bluetooth
        self._bt_api = self._device.get_uecmd("LocalConnectivity")

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # pylint: disable=E1101
        LabWifiBase.set_up(self)

        # Retrieve DUT ip address
        self.__dut_ip_adr = self._networking_api.get_wifi_ip_address(self._use_ipv6)

        # Checks connection using Ping
        packetloss = self._networking_api.ping(self._wifirouter_ip,
                                               self._packetsize, self._packnb,
                                               source_address=self.__dut_ip_adr)
        if packetloss.value > self._lossrate:
            msg = "Ping command fails (%s%% loss)" % str(packetloss.value)
            self._logger.error(msg)
            raise DeviceException(DeviceException.PROHIBITIVE_MEASURE, msg)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        LabWifiBase.run_test(self)

        # Disable SSID
        self._ns.init()
        try:
            self._ns.disable_wireless()
        finally:
            self._ns.release()

        # Check Wifi is lost
        try:
            packetloss = self._networking_api.ping(self._wifirouter_ip,
                                                   self._packetsize,
                                                   self._packnb,
                                                   source_address=self.__dut_ip_adr)
        except AcsBaseException as e:
            if "Network is unreachable" in e.get_error_message():
                # Then the Exception has been raised
                # because of a ping connection failure
                packetloss = Measure()
                packetloss.value = 100
                packetloss.units = "%"
            else:
                self._logger.error("Ping command fails: " + e.get_error_message())
                raise e
        if packetloss.value < self._lossrate_disconnected:
            msg = "Wifi connection still established (ping OK)"
            self._logger.error(msg)
            raise DeviceException(DeviceException.PROHIBITIVE_MEASURE, msg)

        # Enable SSID
        self._ns.init()
        try:
            self._ns.enable_wireless()
        finally:
            self._ns.release()
        init_timer = time.time()
        ping_continue = True
        while ping_continue:
            self._logger.debug("Wait " + str(self._wifi_reconnection_time) + "s to reconnect before ping")
            time.sleep(self._wifi_reconnection_time)

            # Check Wifi connection is back
            try:
                packetloss = self._networking_api.ping(self._wifirouter_ip,
                                                   self._packetsize, self._packnb)
            except AcsBaseException as e:
                if "Network is unreachable" in e.get_error_message():
                    packetloss = Measure()
                    packetloss.value = 100
                    packetloss.units = "%"
                else:
                    self._logger.error("Ping command fails: " + e.get_error_message())
                    raise e
            if packetloss.value > self._lossrate:
                msg = "Fail to recover Wifi connection after loss of coverage " \
                      + "(ping NOK: %s%% packet loss)" % str(packetloss.value)
                self._logger.warning(msg)
                if (time.time() - init_timer) > 180:
                    self._logger.error("Reconnection time > 3min")
                    raise DeviceException(DeviceException.PROHIBITIVE_MEASURE, msg)
            else:
                ping_continue = False

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        UseCaseBase.tear_down(self)

        self._ns.init()
        self._ns.enable_wireless()
        self._ns.release()

        LabWifiBase.tear_down(self)

        return Global.SUCCESS, "No errors"
