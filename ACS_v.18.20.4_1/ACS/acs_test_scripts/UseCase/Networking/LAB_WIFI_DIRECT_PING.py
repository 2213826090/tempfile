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
:summary: This file implements the LAB_WIFI_DIRECT_PING UC
:since: 19/06/2013
:author: smaurel
"""

from acs_test_scripts.UseCase.Networking.LAB_WIFI_DIRECT_CONNECT import LabWifiDirectConnect
from UtilitiesFWK.Utilities import Global
from ErrorHandling.DeviceException import DeviceException


class LabWifiDirectPing(LabWifiDirectConnect):
    """
    Lab Wifi Direct ping class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabWifiDirectConnect.__init__(self, tc_name, global_config)

        self._ping_size = self._tc_parameters.get_param_value("PING_SIZE")
        self._ping_count = self._tc_parameters.get_param_value("PING_COUNT")

    def run_test(self):
        """
        Execute the ping test
        """
        LabWifiDirectConnect.run_test(self)

        # Ping device
        packet_loss = self._networking_api.ping(self._device1_ip,
                                                self._ping_size,
                                                self._ping_count)
        if packet_loss.value > 0:
            msg = "Ping Measured Packet Loss: %.0f%s (Target: %.0f%s)" \
                % (packet_loss.value, packet_loss.units,
                   0,  # self._target_ping_packet_loss_rate
                   packet_loss.units)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        return Global.SUCCESS, "No error"
