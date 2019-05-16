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
:summary: unit test for GetExpectedSafeRangesForLte
:since: 2014-12-11
:author: emarchan

"""

import mock
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from acs_test_scripts.TestStep.Device.Wireless.Cellular.GetExpectedSafeRangesForLte import GetExpectedSafeRangesForLte

GOOD_CHANNEL = 2850
WRONG_CHANNEL = 2851
GOOD_TARGET = "Dummy"
DEST_VAR = "report"
class Test_GetExpectedSafeRangesForLte(UTestTestStepBase):
    def setUp(self):
        UTestTestStepBase.setUp(self)
        self._sut = None

    def test_tgts_model_not_specified_ko(self):
        self._create_sut({"LTE_CHANNEL": GOOD_CHANNEL, "EXPECTED_SAFE_RANGES":DEST_VAR})
        self._sut._device.get_phone_model.return_value = "bad"
        self._assert_run_throw_config_exception(self._sut, "\.*DeviceModel \S+ does not exist \.*")

    def test_bad_channel_ko(self):
        self._create_sut({"LTE_CHANNEL": WRONG_CHANNEL, "EXPECTED_SAFE_RANGES":DEST_VAR})
        self._sut._device.get_phone_model.return_value = GOOD_TARGET
        self._assert_run_throw_config_exception(self._sut, "LTE_Expected_Safe_Ranges does not have targets for %s model and .*" % (GOOD_TARGET))

    def test_call_ok(self):
        self._create_sut({"LTE_CHANNEL": GOOD_CHANNEL, "EXPECTED_SAFE_RANGES":DEST_VAR})
        self._sut._device.get_phone_model.return_value = GOOD_TARGET
        self._sut.run(self._context)

        assert(self._context.get_info("report:WIFI_MIN_CHANNEL") >= 0)
        assert(self._context.get_info("report:WIFI_MAX_CHANNEL") >= 0)
        assert(self._context.get_info("report:WIFI_MIN_FREQ") >= 0)
        assert(self._context.get_info("report:WIFI_MAX_FREQ") >= 0)
        assert(self._context.get_info("report:BLE_MIN_CHANNEL") >= 0)
        assert(self._context.get_info("report:BLE_MAX_CHANNEL") >= 0)
        assert(self._context.get_info("report:BLE_MIN_FREQ") >= 0)
        assert(self._context.get_info("report:BLE_MAX_FREQ") >= 0)
        assert(self._context.get_info("report:BT_MIN_CHANNEL") >= 0)
        assert(self._context.get_info("report:BT_MAX_CHANNEL") >= 0)
        assert(self._context.get_info("report:BT_MIN_FREQ") >= 0)
        assert(self._context.get_info("report:BT_MAX_FREQ") >= 0)


    def _create_sut(self, params):
        self._sut = GetExpectedSafeRangesForLte(None, None, params, mock.Mock())
        return self._sut
