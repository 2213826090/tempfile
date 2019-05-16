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
:summary: This file implements the Live WIFI PING UC
:since: 30/03/2010
:author: cbresoli
"""
import time
import re
import numpy
from acs_test_scripts.UseCase.Networking.LIVE_WIFI_BASE import LiveWifiBase
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.NetworkingUtilities import compute_verdict
from ErrorHandling.AcsConfigException import AcsConfigException


class LiveWifiPing(LiveWifiBase):

    """
    Live Wifi Ping test.
    """
    DEFAULT_TOLERANCE = .20

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LiveWifiBase.__init__(self, tc_name, global_config)

        # Get TC Parameters
        self._packetsize = \
            int(self._tc_parameters.get_param_value("PACKET_SIZE"))
        self._count = int(self._tc_parameters.get_param_value("PACKET_COUNT"))
        self._target_ping_packet_loss_rate = \
            float(self._tc_parameters.get_param_value("TARGET_PACKET_LOSS_RATE"))

        self._connection_time_list = list()
        self._current_iteration_num = 0

        self._expected_connection_time = str(self._tc_parameters.get_param_value("REF_CONNECTION_TIME"))

        self._tolerance = str(self._tc_parameters.get_param_value("TOLERANCE"))

        if re.match(r'\d+\.?\d*', self._tolerance) is not None:
            self._tolerance = float(self._tolerance) / 100
        else:
            self._tolerance = self.DEFAULT_TOLERANCE
            self._logger.warning("Tolerance not found, set to default value")
        self._logger.debug("Tolerance set to %2.2f%%" % (self._tolerance * 100))

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Initialize the test
        """
        LiveWifiBase.set_up(self)

        if str(self._expected_connection_time).upper() in ["NONE", "", "0", "NO"]:
            self._monitor_connection_time = False
        elif str(self._expected_connection_time).isdigit() and int(self._expected_connection_time) > 0:
            self._monitor_connection_time = True
            self._expected_connection_time = int(self._expected_connection_time)
        else:
            msg = "Wrong parameter for REF_CONNECTION_TIME: read value %s" % str(self._expected_connection_time)
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # Update back to back iteration and the wifi connection time list
        if self._monitor_connection_time:
            msg = "DUT connected in %d seconds" % self._connection_time
            self._logger.info(msg)
            self._current_iteration_num += 1

            if self._connection_time != -1:
                self._connection_time_list.append(self._connection_time)
            else:
                self._logger.warning("unable to measure connection time")
        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        LiveWifiBase.run_test(self)
        # monitor wifi connection time if requested
        mean = -1
        if self._monitor_connection_time and self.get_b2b_iteration() == self._current_iteration_num:
            # This is the last iteration of back to back test
            # compute standard deviation, mean and verdict
            mean = float(numpy.mean(self._connection_time_list))
            std_deviation = float(numpy.std(self._connection_time_list))
            compute_verdict(self._expected_connection_time, self._tolerance, mean, std_deviation, self._logger)

        # init values
        self._error.Code = Global.FAILURE
        self._error.Msg = "ping failed"
        if self._wrong_passphrase is not None:
            self._error.Code = Global.SUCCESS
            self._error.Msg = "OK - Connection failed with wrong password, ping not needed!"
        else:
            time.sleep(self._wait_btwn_cmd)
            self._logger.info("Ping address " + str(self._server_ip_address) +
                              " with " + str(self._count) + " packets of " +
                              str(self._packetsize) + " bytes...")

            source_address = self._networking_api.get_wifi_ip_address()
            packet_loss = self._networking_api.\
                ping(self._server_ip_address,
                     self._packetsize,
                     self._count,
                     source_address=source_address)

            # Compute verdict depending on % of packet loss
            if packet_loss.value > self._target_ping_packet_loss_rate:
                self._error.Code = Global.FAILURE
            else:
                self._error.Code = Global.SUCCESS

            self._error.Msg = "Measured Packet Loss: %.0f%s (Target: %.0f%s)."\
                % (packet_loss.value,
                   packet_loss.units,
                   self._target_ping_packet_loss_rate,
                   packet_loss.units)
            if mean > 0:
                self._error.Msg += " Mean connection time is %2.2f sec" % mean

        return self._error.Code, self._error.Msg
