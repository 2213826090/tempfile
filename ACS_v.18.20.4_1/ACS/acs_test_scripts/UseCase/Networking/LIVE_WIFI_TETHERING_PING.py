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
:summary: This file implements LIVE WIFI TETHERING PING UC
:since: 26/06/2012
:author: jpstierlin BZ3235
"""
import time

from LIVE_WIFI_TETHERING_BASE import LiveWifiTetheringBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.DeviceException import DeviceException


class LiveWifiTetheringPing(LiveWifiTetheringBase):

    """
    Live wifi Tethering ping.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LIVE_WIFI_TETHERING_BASE Init function
        LiveWifiTetheringBase.__init__(self, tc_name, global_config)

        # Read the number of pings to do
        self._nb_pings = self._tc_parameters.get_param_value("PACKET_COUNT")

        # Read the data size of a packet
        self._packet_size = self._tc_parameters.get_param_value("PACKET_SIZE")

        # Get target % of received packet for ping
        self._target_ping_packet_loss_rate = \
            float(self._tc_parameters.get_param_value("TARGET_PACKET_LOSS_RATE"))

        self._waiting_time = \
            int(self._tc_parameters.get_param_value("DELAY_AFTER_PING"))

        # Get the IP server to ping
        self._server_ip = self._tc_parameters.\
            get_param_value("SERVER_TO_PING")
        self._server_2_ping = None

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LiveWifiTetheringBase.set_up(self)

        if str(self._server_ip) in ["None", ""] or self._flight_mode:
            # Get the IP of the softAP wifi interface
            self._server_2_ping = self._networking_api.\
                get_interface_ipv4_address(self._hotspot_ext_interface)

            if str(self._server_ip) not in ["None", ""] \
                    and self._flight_mode:
                # Display a warning because the given parameter cannot be used
                self._logger.warning("IP server set as parameter is " +
                                     "ignored, as test is run in flight " +
                                     "mode. Using SoftAP IP instead " +
                                     "(%s)" % self._server_2_ping)
        else:
            self._server_2_ping = self._server_ip

        return Global.SUCCESS, "No error"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        LiveWifiTetheringBase.run_test(self)

        time.sleep(2)

        if self._computer is not None:
            # ping phone1 from remote computer's wifi interface
            packet_loss = self._computer.ping(self._server_2_ping,
                                              self._packet_size, self._nb_pings)
        else:
            # ping phone1 from phone2
            packet_loss = self._networking_api2.ping(self._server_2_ping,
                                                     self._packet_size, self._nb_pings)

        self._error.Code = Global.SUCCESS
        self._error.Msg = "Measured Packet Loss: %.0f%s (Target: %.0f%s)"\
            % (packet_loss.value,
               packet_loss.units,
               self._target_ping_packet_loss_rate,
               packet_loss.units)
        self._logger.info(self._error.Msg)

        # Compute verdict depending on % of packet loss
        if packet_loss.value > self._target_ping_packet_loss_rate:
            msg = "Ping packet loss is not acceptable [%s]" \
                % str(packet_loss.value)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        if self._waiting_time > 0:
            self._logger.info("Waiting for %s sec" % str(self._waiting_time))
            time.sleep(self._waiting_time)

        return self._error.Code, self._error.Msg
