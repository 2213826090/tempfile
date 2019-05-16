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

from acs_test_scripts.test.unit_test.UtTestStep.UtDevice.UtWireless.UtCellular.test_GetSafeChannelsForLteBase import GetSafeChannelsForLteTestBase
from acs_test_scripts.TestStep.Device.Wireless.Cellular.GetBleSafeChannelsForLte import GetBleSafeChannelsForLte

class GetBleSafeChannelsForLteTest(GetSafeChannelsForLteTestBase):
    """
    GetSafeChannelsForLteTest - BLE
    """

    def setUp(self):
        GetSafeChannelsForLteTestBase.setUp(self)
        self._domain = "BLE"
        self._dest_var = "%s_safe_range" % self._domain

    def test_get_safe_range_ok(self):
        GetSafeChannelsForLteTestBase.get_safe_range_ok(self)

    def test_get_safe_range_not_enabled_ko(self):
        GetSafeChannelsForLteTestBase.get_safe_range_not_enabled_ko(self)

    def test_get_safe_range_empty_ok(self):
        GetSafeChannelsForLteTestBase.get_safe_range_empty_ok(self)

    def _create_sut(self):
        """
        Create the SUT with only test step pars
        """
        sut = GetBleSafeChannelsForLte(None, None, {"BLE_SAFE_CHANNELS": self._dest_var}, mock.Mock())
        sut._get_sysdebug_logs = self._side_effect_stats_root_only
        sut._delay_between_checks = 0
        return sut

