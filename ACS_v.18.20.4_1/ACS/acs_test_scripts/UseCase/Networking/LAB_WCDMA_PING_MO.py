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
:summary: This file implements LAB WCDMA PING MO UC
:author: ymorel
:since:08/02/2011
"""

import time

from LAB_WCDMA_BASE import LabWcdmaBase
from UtilitiesFWK.Utilities import Global


class LabWcdmaPingMo(LabWcdmaBase):

    """
    Lab WCDMA mobile originated ping.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LAB_WCDMA_BASE Init function
        LabWcdmaBase.__init__(self, tc_name, global_config)

        # Read the number of pings to do
        self._nb_pings = self._tc_parameters.get_param_value("PACKET_COUNT")

        # Read the data size of a packet
        self._packet_size = self._tc_parameters.get_param_value("PACKET_SIZE")

        # Get target % of received packet for ping
        self._target_ping_packet_loss_rate = \
            float(self._tc_parameters.get_param_value("TARGET_PACKET_LOSS_RATE"))

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        LabWcdmaBase.run_test(self)

        # init values
        self._error.Code = Global.FAILURE
        self._error.Msg = "ping failed"

        time.sleep(self._wait_btwn_cmd)

        packet_loss = self._networking_api.\
            ping(self._server_ip_address,
                 self._packet_size,
                 self._nb_pings)

        # Compute verdict depending on % of packet loss
        if packet_loss.value > self._target_ping_packet_loss_rate:
            self._error.Code = Global.FAILURE
        else:
            self._error.Code = Global.SUCCESS

        self._error.Msg = "Measured Packet Loss: %.0f%s (Target: %.0f%s)"\
            % (packet_loss.value,
               packet_loss.units,
               self._target_ping_packet_loss_rate,
               packet_loss.units)

        return self._error.Code, self._error.Msg
