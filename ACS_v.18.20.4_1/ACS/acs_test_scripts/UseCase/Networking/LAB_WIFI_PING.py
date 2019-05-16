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
:summary: This file implements the Lab WIFI PING UC
:since: 26/08/2011
:author: szhen11
"""
import time
from acs_test_scripts.UseCase.Networking.LAB_WIFI_BASE import LabWifiBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class LabWifiPing(LabWifiBase):

    """
    Lab Wifi Ping test.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabWifiBase.__init__(self, tc_name, global_config)

        # Get TC Parameters
        self._direction = self._tc_parameters.get_param_value("DIRECTION")
        self._packetsize = \
            int(self._tc_parameters.get_param_value("PACKET_SIZE"))
        self._count = int(self._tc_parameters.get_param_value("PACKET_COUNT"))
        self._target_ping_packet_loss_rate = \
            float(self._tc_parameters.get_param_value("TARGET_PACKET_LOSS_RATE"))
        self._waiting_time = \
            int(self._tc_parameters.get_param_value("DELAY_AFTER_PING"))
        self._dut_ip = None
        self._computer = None
        self._all_ip_to_ping = []

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LabWifiBase.set_up(self)

        self._dut_ip = self._networking_api.get_wifi_ip_address(self._use_ipv6)

        if self._direction == "DL":
            self._all_ip_to_ping = [self._dut_ip]
            if not self._use_ipv6:
                # With IPv6,we already have it from LAB_WIFI_BASE
                self._computer = self._em.get_computer("COMPUTER1")
        else:
            if self._use_ipv6:
                self._all_ip_to_ping = self._computer.get_ipv6_addresses()
            else:
                remote_ip = self._wifirouter_ip
                self._all_ip_to_ping = [remote_ip]
        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        LabWifiBase.run_test(self)

        # Put display on and off to avoid scan failure.
        self._phone_system_api.display_on()
        time.sleep(self._wait_btwn_cmd)
        self._phone_system_api.display_off()

        self._logger.info("Ping direction : " + self._direction)

        for ip_to_ping in self._all_ip_to_ping:
            if self._direction == "UL":
                # ping from DUT to Server
                self._logger.info("Ping address " + str(ip_to_ping) +
                                  " with " + str(self._count) + " packets of " +
                                  str(self._packetsize) + " bytes...")

                packet_loss = self._networking_api.ping(ip_to_ping, self._packetsize,
                                                        self._count, source_address=self._dut_ip)

            elif self._direction == "DL":
                # ping from Server to DUT
                self._logger.info("Ping address " + str(ip_to_ping) +
                                  " with " + str(self._count) + " packets of " +
                                  str(self._packetsize) + " bytes...")

                packet_loss = self._computer.ping(ip_to_ping,
                                                  self._packetsize, self._count)

            else:
                msg = "%s is not a valid xfer direction" % self._direction
                self._logger.error(msg)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

            msg = "Measured Packet Loss: %.0f%s (Target: %.0f%s)" \
                % (packet_loss.value, packet_loss.units,
                   self._target_ping_packet_loss_rate, packet_loss.units)
            self._logger.info(msg)

            # Compute verdict depending on % of packet loss
            if not self._key_exchange_should_fail:
                if packet_loss.value > self._target_ping_packet_loss_rate:
                    msg = "Ping packet loss is not acceptable [%s]" \
                        % str(packet_loss.value)
                    self._logger.error(msg)
                    raise DeviceException(DeviceException.OPERATION_FAILED, msg)
            else:
                if ip_to_ping != '0.0.0.0':
                    if packet_loss.value < 100:
                        msg = "Ping packet loss is too low [%s]" \
                            % str(packet_loss.value)
                        self._logger.error(msg)
                        raise DeviceException(DeviceException.OPERATION_FAILED, msg)

            if self._waiting_time > 0:
                self._logger.info("Waiting for %s sec" % str(self._waiting_time))
                time.sleep(self._waiting_time)

        return Global.SUCCESS, "No errors"
