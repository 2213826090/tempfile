# @PydevCodeAnalysisIgnore
# pylint: disable=E0602,W0212,C0103,C0111
"""
:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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
:since: 19/01/2015
:author: jfranchx
"""

import mock
from Device.DeviceManager import DeviceManager
from Core.TestStep.TestStepContext import TestStepContext
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from acs_test_scripts.Utilities.ThroughputMeasure import ThroughputMeasure
from acs_test_scripts.TestStep.Utilities.Checks.GetWifiDirectThroughputTargets import GetWifiDirectThroughputTargets


class TestDevice():
    def __init__(self):
        self._device = "BLACKBAY-Android-JB"

    def get_phone_model(self):
        return self._device


class GetWifiDirectThroughputTargetTest(UTestTestStepBase):

    TEST_DEVICE = "PHONE1"
    PARAM_PROTOCOL_TCP = "tcp"
    PARAM_PROTOCOL_UDP = "udp"
    PARAM_DIRECTION_UL = "up"
    PARAM_DIRECTION_DL = "down"
    PARAM_PROTOCOL_BANDWIDTH = "20"
    CTX_DATA_SAVE = "SAVE_WIFI_DIRECT_TARGET"
    CTX_GET_TARGET_VALUE = CTX_DATA_SAVE + ":TARGET_VALUE"
    CTX_GET_TARGET_UNIT = CTX_DATA_SAVE + ":TARGET_UNIT"
    CTX_GET_FAILURE_VALUE = CTX_DATA_SAVE + ":FAILURE_VALUE"
    CTX_GET_FAILURE_UNIT = CTX_DATA_SAVE + ":FAILURE_UNIT"

    BLACKBAY_TARGET_VALUE = "20000.0"
    BLACKBAY_FAILURE_VALUE = "10000.0"
    MEASURE_UNIT = ThroughputMeasure.KBPS_UNIT

    def _return_device(self, device):
        return TestDevice()

    def setUp(self):
        UTestTestStepBase.setUp(self)
        self._context = TestStepContext()
        self._sut = None

    def test_get_wifi_direct_throughput_target_dl_tcp(self):
        self._create_sut({"DEVICE": "PHONE1", "IPERF_PROTOCOL": self.PARAM_PROTOCOL_TCP, "DIRECTION": self.PARAM_DIRECTION_DL, "BANDWIDTH": self.PARAM_PROTOCOL_BANDWIDTH, "TARGET_THROUGHPUT": self.CTX_DATA_SAVE})
        self._assert_run_succeeded(self._sut)
        self.assertEqual(str(self.BLACKBAY_TARGET_VALUE), str(self._context.get_info(self.CTX_GET_TARGET_VALUE)))
        self.assertEqual(str(self.MEASURE_UNIT), str(self._context.get_info(self.CTX_GET_TARGET_UNIT)))
        self.assertEqual(str(self.BLACKBAY_FAILURE_VALUE), str(self._context.get_info(self.CTX_GET_FAILURE_VALUE)))
        self.assertEqual(str(self.MEASURE_UNIT), str(self._context.get_info(self.CTX_GET_FAILURE_UNIT)))

    def test_get_wifi_direct_throughput_target_ul_tcp(self):
        self._create_sut({"DEVICE": "PHONE1", "IPERF_PROTOCOL": self.PARAM_PROTOCOL_TCP, "DIRECTION": self.PARAM_DIRECTION_UL, "BANDWIDTH": self.PARAM_PROTOCOL_BANDWIDTH, "TARGET_THROUGHPUT": self.CTX_DATA_SAVE})
        self._assert_run_succeeded(self._sut)
        self.assertEqual(str(self.BLACKBAY_TARGET_VALUE), str(self._context.get_info(self.CTX_GET_TARGET_VALUE)))
        self.assertEqual(str(self.MEASURE_UNIT), str(self._context.get_info(self.CTX_GET_TARGET_UNIT)))
        self.assertEqual(str(self.BLACKBAY_FAILURE_VALUE), str(self._context.get_info(self.CTX_GET_FAILURE_VALUE)))
        self.assertEqual(str(self.MEASURE_UNIT), str(self._context.get_info(self.CTX_GET_FAILURE_UNIT)))

    def test_get_wifi_direct_throughput_target_dl_udp(self):
        self._create_sut({"DEVICE": "PHONE1", "IPERF_PROTOCOL": self.PARAM_PROTOCOL_UDP, "DIRECTION": self.PARAM_DIRECTION_DL, "BANDWIDTH": self.PARAM_PROTOCOL_BANDWIDTH, "TARGET_THROUGHPUT": self.CTX_DATA_SAVE})
        self._assert_run_succeeded(self._sut)
        self.assertEqual(str(self.BLACKBAY_TARGET_VALUE), str(self._context.get_info(self.CTX_GET_TARGET_VALUE)))
        self.assertEqual(str(self.MEASURE_UNIT), str(self._context.get_info(self.CTX_GET_TARGET_UNIT)))
        self.assertEqual(str(self.BLACKBAY_FAILURE_VALUE), str(self._context.get_info(self.CTX_GET_FAILURE_VALUE)))
        self.assertEqual(str(self.MEASURE_UNIT), str(self._context.get_info(self.CTX_GET_FAILURE_UNIT)))

    def test_get_wifi_direct_throughput_target_ul_udp(self):
        self._create_sut({"DEVICE": "PHONE1", "IPERF_PROTOCOL": self.PARAM_PROTOCOL_UDP, "DIRECTION": self.PARAM_DIRECTION_UL, "BANDWIDTH": self.PARAM_PROTOCOL_BANDWIDTH, "TARGET_THROUGHPUT": self.CTX_DATA_SAVE})
        self._assert_run_succeeded(self._sut)
        self.assertEqual(str(self.BLACKBAY_TARGET_VALUE), str(self._context.get_info(self.CTX_GET_TARGET_VALUE)))
        self.assertEqual(str(self.MEASURE_UNIT), str(self._context.get_info(self.CTX_GET_TARGET_UNIT)))
        self.assertEqual(str(self.BLACKBAY_FAILURE_VALUE), str(self._context.get_info(self.CTX_GET_FAILURE_VALUE)))
        self.assertEqual(str(self.MEASURE_UNIT), str(self._context.get_info(self.CTX_GET_FAILURE_UNIT)))

    def _create_sut(self, test_step_pars=None):
        self._sut = GetWifiDirectThroughputTargets(None, None, test_step_pars, mock.Mock())
        DeviceManager().get_device = self._return_device
        return self._sut
