# @PydevCodeAnalysisIgnore
# pylint: disable=E0602,W0212,C0103,C0111
"""
@copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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
:summary: Unit test module
:since: 17/07/14
:author: jfranchx
"""

import mock
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from acs_test_scripts.TestStep.Equipment.ConfigurableAP.APWifiACLSetAddr import APWifiACLSetAddr


class APWifiACLSetAddrTest(UTestTestStepBase):
    MAC_ADDRESS_VALID = "AA:BB:CC:DD:EE:FF"
    MAC_ADDRESS_INVALID = "AA:BB:CC:DD:EE"

    def setUp(self):
        UTestTestStepBase.setUp(self)

    def test_add_ok(self):
        sut = self._create_sut({"SET_ADDR": "ADD", "MAC_ADDR": self.MAC_ADDRESS_VALID})
        self._assert_run_succeeded(sut)
        sut._configurable_ap.add_mac_address_to_acl.assert_called_once_with(self.MAC_ADDRESS_VALID)

    def test_remove_ok(self):
        sut = self._create_sut({"SET_ADDR": "REMOVE", "MAC_ADDR": self.MAC_ADDRESS_VALID})
        self._assert_run_succeeded(sut)
        sut._configurable_ap.del_mac_address_from_acl.assert_called_once_with(self.MAC_ADDRESS_VALID)

    def test_invalid_mac_addr(self):
        sut = self._create_sut({"SET_ADDR": "ADD", "MAC_ADDR": self.MAC_ADDRESS_INVALID})
        self._assert_run_throw_config_exception(sut,
                                                "Parameter %s is not a valid mac address" % self.MAC_ADDRESS_INVALID)

    def _create_sut(self, args=None):
        sut = APWifiACLSetAddr(None, mock.Mock(), args, mock.Mock())
        return sut
