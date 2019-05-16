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
:summary: This file implements the LIVE BT Scan UC
:since: 29/09/2010
:author: skgurusX, vgombert
"""

from LIVE_BT_BASE import LiveBTBase
from UtilitiesFWK.Utilities import Global


class LiveBTL2CAPPing(LiveBTBase):

    """
    Live BT Ping test.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        LiveBTBase.__init__(self, tc_name, global_config)

        # Get TC Parameters
        self._target_ping_packet_loss_rate = \
            float(self._tc_parameters.get_param_value("TARGET_PACKET_LOSS_RATE"))

        self._device_bd_address = \
            self._tc_parameters.get_param_value("REMOTE_DEVICE_BD_ADDRESS")

        self._packetsize = \
            int(self._tc_parameters.get_param_value("PACKET_SIZE"))
        self._count = int(self._tc_parameters.get_param_value("PACKET_COUNT"))

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        LiveBTBase.run_test(self)

        # init values
        self._error.Code = Global.FAILURE
        self._error.Msg = "ping failed"

        self._logger.info("ping Bluetooth address " +
                          str(self._device_bd_address) +
                          " with " + str(self._count) + " packets of " +
                          str(self._packetsize) + " bytes...")

        packet_loss = self._bt_api.\
            bt_l2cap_ping(self._device_bd_address,
                          self._count,
                          self._packetsize)

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
