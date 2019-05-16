"""

@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

@organization: UMG PSI Validation
@summary: This file implements the step to ping a given IP address from a computer and saves the percentage of packet loss

@author: emarchan

"""

from acs_test_scripts.TestStep.Equipment.Computer.ComputerBase import ComputerBase


class ComputerPing(ComputerBase):
    """
    Pings a given IP address from a computer and saves the percentage of packet loss
    """

    def run(self, context):
        """
        Run the test step
        """
        ComputerBase.run(self, context)
        destination_ip = str(self._pars.destination_ip)
        packet_size = int(self._pars.packet_size)
        packet_count = int(self._pars.packet_count)
        interval = float(self._pars.interval)
        flood_mode = bool(self._pars.flood_mode)

        # Where the information will be stored into the context
        save_as = self._pars.save_as

        self._logger.info("Ping the IP address %s ..." % destination_ip)

        packet_loss = self._computer.ping(ip_address=destination_ip,
                                                packet_size=packet_size,
                                                packet_count=packet_count,
                                                interval=interval,
                                                flood_mode=flood_mode)

        # We have the value, let's save it into the context
        context.set_info(save_as, str(packet_loss.value))
        self.ts_verdict_msg = "VERDICT: %s stored as {0}".format(str(packet_loss.value)) % save_as
        self._logger.debug(self.ts_verdict_msg)

