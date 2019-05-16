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

:organization: INTEL QCTV
:summary: This file implements Test Step to wake a remote PC using ethernet capability
:since 08/04/2016
:author: pblunie
"""
import time
from acs_test_scripts.TestStep.Equipment.Computer.ComputerBase import ComputerBase
from wakeonlan import wol

class ComputerWakeOnLan(ComputerBase):
    """
    Implements wake on lan test step
    """

    def run(self, context):
        """
        Run the test step
        """
        ComputerBase.run(self, context)

        self._logger.debug("Wake the computer with mac addr '%s'" % self._pars.mac_addr)
        wol.send_magic_packet(self._pars.mac_addr)

        self._logger.debug("Let the computer boot some time")
        time.sleep(self._pars.boot_expected_time)
        timeout = time.time() + self._pars.boot_timeout
        current = time.time()
        while True:
            try:
                computer_connection = self._computer.init()
                return True
            except SSHException as e:
                if current > timeout:
                    raise e
                else:
                    self._logger.info("Boot ongoing since %d on %d seconds" %
                                      (current, self._pars.boot_timeout))
                    time.sleep(5)
                    current = time.time()
