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
from lxml import etree

MIN_RANGE = "2400"
MAX_RANGE = "2463"
class GetSafeChannelsForLteTestBase(UtCellularBase):
    """
    Base for GetSafeChannelsForLteTest test cases
    """

    def setUp(self):
        """
        Set up
        """
        UtCellularBase.setUp(self)
        self._context = TestStepContext()


    def get_safe_range_ok(self):
        sut = self._create_sut()
        sut._get_sysdebug_logs = self._side_effect_stats_with_range

        sut.run(self._context)
        self.assertEqual(sut.ts_verdict_msg, "VERDICT: %s safe range is [ %s - %s ]" % (self._domain, MIN_RANGE, MAX_RANGE))
        self.assertEqual(MIN_RANGE, str(self._context.get_info("%s:MIN" % self._dest_var)))
        self.assertEqual(MAX_RANGE, str(self._context.get_info("%s:MAX" % self._dest_var)))

    def get_safe_range_not_enabled_ko(self):
        sut = self._create_sut()
        self._assert_run_throw_device_exception(sut, "Can't find SafeRange in the logs, did you enable it at init?")

    def get_safe_range_empty_ok(self):
        sut = self._create_sut()
        sut._get_sysdebug_logs = self._side_effect_stats_no_range

        self._assert_run_throw_device_exception(sut, "VERDICT: Can't find the %s safe range in logs." % self._domain)

    def _side_effect_stats_with_range(self):
        """
        Simulate the stats output with a safe range
        """
        xmltree = self._side_effect_stats_root_only()
        stattree = etree.Element("SafeRange%s" % self._domain)
        xmlrange = etree.Element("SafeRange")
        xmlrange.attrib["min"] = MIN_RANGE
        xmlrange.attrib["max"] = MAX_RANGE
        stattree.append(xmlrange)
        xmltree.append(stattree)

        return xmltree

    def _side_effect_stats_no_range(self):
        """
        Simulate the stats output, in the case of no message found in the log matching the safe range
        """
        xmltree = self._side_effect_stats_root_only()
        stattree = etree.Element("SafeRange%s" % self._domain)
        xmltree.append(stattree)

        return xmltree

    def _side_effect_stats_root_only(self):
        """
        Simulate the stats output containing sysdebug root only (safe range not activated at init)
        """
        xmltree = etree.Element("SysDebug")

        return xmltree
