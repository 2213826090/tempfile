"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

:organization: INTEL MCG
:summary: unit test
:since: 2014-08-05
:author: emarchan

"""
import mock

from unit_test.UtTestStep.UtDevice.UtWireless.UtWifi.UtWifiDirectBase import UtWifiDirectBase
from acs_test_scripts.TestStep.Device.Wireless.WifiDirect.GetWifiDirectRememberedGroups import GetWifiDirectRememberedGroups
from Core.TestStep.TestStepContext import TestStepContext

class GetWifiDirectRememberedGroupsTest(UtWifiDirectBase):
    """
    Gets the Wifi direct remembered groups.
    """

    def setUp(self):
        """
        Set up
        """
        UtWifiDirectBase.setUp(self)
        self._context = TestStepContext()

    def test_get_wifi_direct_remembered_groups_call_ok(self):
        sut = self._create_sut({self.SAVE_AS_PARAM:self.DEFAULT_SAVE_VAR})

        sut.run(self._context)

        sut._api.get_wifi_direct_remembered_groups.assert_called_once_with()

    def test_get_wifi_direct_remembered_groups_save_ok(self):
        sut = self._create_sut({self.SAVE_AS_PARAM:self.DEFAULT_SAVE_VAR})

        str_exp_result = ""
        for cur_value in self.DEFAULT_GROUP_LIST:
            str_exp_result = str_exp_result + cur_value + '|'
        if str_exp_result.endswith('|'):
            str_exp_result = str_exp_result[:-1]

        sut.run(self._context)

        self.assertEqual(str_exp_result, self._context.get_info(self.DEFAULT_SAVE_VAR))


    # pylint: disable=W0212
    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """
        sut = GetWifiDirectRememberedGroups(None, None, test_step_pars, mock.Mock())
        sut._api.get_wifi_direct_remembered_groups.return_value = self.DEFAULT_GROUP_LIST

        return sut
