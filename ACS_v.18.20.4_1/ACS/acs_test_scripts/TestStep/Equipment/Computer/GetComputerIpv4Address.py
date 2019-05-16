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
@summary: This file implements the step to get the IPv4 address of the COMPUTER for the test network.

@author: emarchan

"""

from acs_test_scripts.TestStep.Equipment.Computer.ComputerBase import ComputerBase


class GetComputerIpv4Address(ComputerBase):
    """
    Gets the IPv4 address of the COMPUTER for the test network.
    """

    def run(self, context):
        """
        Run the test step
        """
        ComputerBase.run(self, context)

        ip_addr = self._computer.get_host_on_test_network()

        self._logger.info("IPv4 address is %s" % ip_addr)
        context.set_info(self._pars.ip_addr, ip_addr)

        self.ts_verdict_msg = "VERDICT: %s stored as {0}".format(ip_addr) % self._pars.ip_addr
        self._logger.debug(self.ts_verdict_msg)
