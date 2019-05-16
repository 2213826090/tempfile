"""
@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related to the source code ("Material") are owned by
Intel Corporation or its suppliers or licensors. Title to the Material remains with Intel Corporation or its suppliers
and licensors. The Material contains trade secrets and proprietary and confidential information of Intel or its
suppliers and licensors.

The Material is protected by worldwide copyright and trade secret laws and treaty provisions. No part of the Material
may be used, copied, reproduced, modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual property right is granted to or conferred
upon you by disclosure or delivery of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express and approved by Intel in writing.

:organization: INTEL MCG PSI
:summary: unit test
:since 30/10/2014
:author: emarchan
"""
import mock

from acs_test_scripts.test.unit_test.UtTestStep.UtDevice.UtWireless.UtCellular.UtCellularBase import UtCellularBase
from Core.TestStep.TestStepContext import TestStepContext
from acs_test_scripts.TestStep.Device.Wireless.Cellular.SetCoexManagerTraces import SetCoexManagerTraces

class SetCoexManagerTracesTest(UtCellularBase):
    """
    Base for GetSafeChannelsForLteTest test cases
    """

    def setUp(self):
        """
        Set up
        """
        UtCellularBase.setUp(self)
        self._context = TestStepContext()


    def test_enable_coex_mgr_traces_ok(self):
        sut = self._create_sut({"COEX_MANAGER_TRACES_STATE": "on"})
        sut.run(self._context)
        sut._modem_api.set_lte_coex_manager_messages.assert_called_once_with("VERBOSE")

    def test_disable_coex_mgr_traces_ko(self):
        sut = self._create_sut({"COEX_MANAGER_TRACES_STATE": "off"})
        sut.run(self._context)
        sut._modem_api.set_lte_coex_manager_messages.assert_called_once_with('')

    def test_coex_mgr_bad_param_ko(self):
        sut = self._create_sut({"COEX_MANAGER_TRACES_STATE": "Dummy"})

        with self.assertRaisesRegexp(AssertionError, "is invalid at this stage"):
            sut.run(self._context)


    def _create_sut(self, params=None):
        """
        Create the SUT with only test step pars
        """
        sut = SetCoexManagerTraces(None, None, params, mock.Mock())
        return sut

