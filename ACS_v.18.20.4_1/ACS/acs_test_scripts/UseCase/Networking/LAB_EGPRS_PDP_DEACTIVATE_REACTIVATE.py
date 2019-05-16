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

:summary: Use Case EGPRS PDP Context Deactivate and reactivate with ping check Mo.
:organization: INTEL MCG PSI
:author: hbianx
:since: 24/09/2012
"""

import time

from LAB_EGPRS_BASE import LabEgprsBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsBaseException import AcsBaseException


class LabEgprsPdpDeactivateReactivate(LabEgprsBase):

    """
    Lab EGPRS PDP Context Deactivate and reactivate with MO ping check
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LAB_EGPRS_BASE Init function
        LabEgprsBase.__init__(self, tc_name, global_config)

        # Set the default number of pings to do
        self._nb_pings = 10

        # Set the default data size of a packet
        self._packet_size = 32

        # Get target % of received packet for ping
        self._target_ping_packet_loss_rate = \
            self._tc_parameters.get_param_value("TARGET_PACKET_LOSS_RATE")

        if isinstance(self._target_ping_packet_loss_rate, (int, float)):
            self._target_ping_packet_loss_rate = \
                float(self._target_ping_packet_loss_rate)
        else:
            # if user doesn't enter the correct value, set the default value 0
            self._target_ping_packet_loss_rate = 0

#------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """

        # Call LAB_EGPRS_BASE Run function
        LabEgprsBase.run_test(self)

        time.sleep(self._wait_btwn_cmd)

        # Ping ftp server to verify pdp context actived.
        # Ping should be successful
        packet_loss = self._networking_api.ping(
            self._server_ip_address,
            self._packet_size,
            self._nb_pings)

        self._error.Msg = "Measured Packet Loss: %.0f%s (Target: %.0f%s)" % (
            packet_loss.value,
            packet_loss.units,
            self._target_ping_packet_loss_rate,
            packet_loss.units)

        # Compute verdict depending on % of packet loss
        # Pdp Context active, ping should be successful
        if packet_loss.value > self._target_ping_packet_loss_rate:
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                   self._error.Msg)
        else:
            self._logger.info("Ping successful:  %s" % self._error.Msg)

        # Deactivate pdp context
        self._networking_api.deactivate_pdp_context(self._ssid)

        # Ping ftp server to verify pdp context deactivated.
        # The test should be failed
        try:
            packet_loss = self._networking_api.ping(
                self._server_ip_address,
                self._packet_size,
                self._nb_pings)
        except AcsBaseException as error:
            self._logger.info(error.get_error_message())
        except:
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "PDP context isn't correctly deactivated,"
                                  " ping packets reach the server.")

        # Re-activate pdp context
        self._networking_api.activate_pdp_context(self._ssid)

        self._ns_data_2g.check_data_connection_state("PDP_ACTIVE",
                                                     self._registration_timeout,
                                                     blocking=False)

        packet_loss = self._networking_api.ping(
            self._server_ip_address,
            self._packet_size,
            self._nb_pings)

        # Compute verdict depending on % of packet loss
        # Pdp Context active, ping should be successful
        if packet_loss.value > self._target_ping_packet_loss_rate:
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                   self._error.Msg)
        else:
            self._logger.info("Ping successful:  %s" % self._error.Msg)

        return Global.SUCCESS, "No errors"
