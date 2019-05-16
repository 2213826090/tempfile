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
@summary: This file implements the unit test for ComputerIpv4Address

@author: emarchan

"""
from mock import Mock, MagicMock
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from Core.TestStep.TestStepContext import TestStepContext
from acs_test_scripts.TestStep.Equipment.Computer.GetComputerIpv4Address import GetComputerIpv4Address

DEFAULT_SAVE_AS = "my_ip_addr"
DEFAULT_IP_ADDRESS = "192.168.43.56"

class Test_GetComputerIpv4Address(UTestTestStepBase):

    def setUp(self):
        UTestTestStepBase.setUp(self)
        self._context = TestStepContext()
        self._sut = None

    def test_connect_got_ip_ok(self):
        self._create_sut({"IP_ADDR": DEFAULT_SAVE_AS})

        self._sut.run(self._context)

        self._sut._computer.get_host_on_test_network.assert_called_with()


    def _create_sut(self, args=None):
        self._sut = GetComputerIpv4Address(None, Mock(), args, Mock())

        return self._sut
