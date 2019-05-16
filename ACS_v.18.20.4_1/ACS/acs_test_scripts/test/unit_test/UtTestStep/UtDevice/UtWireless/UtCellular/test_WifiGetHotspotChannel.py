"""
@copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
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
:since 12/01/2015
:author: emarchan
"""
import mock

from acs_test_scripts.TestStep.Device.Wireless.Wifi.WifiGetHotspotChannel import WifiGetHotspotChannel
from Core.TestStep.TestStepContext import TestStepContext
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase

from lxml import etree

CHANNEL = "6"
class test_WifiGetHotspotChannel(UTestTestStepBase):
    """
    Base for GetSafeChannelsForLteTest test cases
    """

    def setUp(self):
        """
        Set up
        """
        UTestTestStepBase.setUp(self)
        self._context = TestStepContext()
        self._dest_var = 'MY_VAR'

    def test_get_safe_range_ok(self):
        sut = self._create_sut()
        sut._get_sysdebug_logs = self._side_effect_stats_with_range

        sut.run(self._context)
        self.assertEqual(sut.ts_verdict_msg, "VERDICT: Wi-Fi SoftAP channel is %s." % CHANNEL)
        self.assertEqual(CHANNEL, str(self._context.get_info(self._dest_var)))

    def test_get_safe_range_empty_ok(self):
        sut = self._create_sut()

        self._assert_run_throw_device_exception(sut, "VERDICT: Can't find the Wi-Fi SoftAP channel in logs.")

    def _side_effect_stats_with_range(self):
        """
        Simulate the stats output with a safe range
        """
        xmltree = self._side_effect_stats_root_only()
        xmltree.attrib["value"] = CHANNEL

        return xmltree

    def _side_effect_stats_root_only(self):
        """
        Simulate the stats output containing sysdebug root only (safe range not activated at init)
        """
        xmltree = etree.Element("WifiSoftApChannel")

        return xmltree

    def _create_sut(self):
        """
        Create the SUT with only test step pars
        """
        sut = WifiGetHotspotChannel(None, None, {"SAVE_WIFI_HOSTPOT_CHANNEL": self._dest_var}, mock.Mock())
        sut._get_sysdebug_logs = self._side_effect_stats_root_only
        sut._delay_between_checks = 0
        return sut

