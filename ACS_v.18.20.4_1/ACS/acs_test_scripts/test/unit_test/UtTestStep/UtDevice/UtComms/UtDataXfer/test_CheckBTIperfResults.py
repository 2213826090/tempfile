# @PydevCodeAnalysisIgnore
# pylint: disable=E0602,W0212,C0103,C0111
"""
@summary: Unit test module for CHECK_BT_IPERF_RESULTS test step.

@since 24 June 2014
@author: Val Peterson
@organization: INTEL PEG-SVE-DSV

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
otherwise. Any license under such intellectual property rights must be expressed
and approved by Intel in writing.
"""

import mock
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from acs_test_scripts.TestStep.Device.Comms.DataXfer.CheckBTIperfResults import CheckBTIperfResults


class Test(UTestTestStepBase):
    def setUp(self):
        UTestTestStepBase.setUp(self)
        self._sut = None

    def test_tgts_not_met_strict(self):
        self._create_sut("both", 1000.0, 1000.0, True, "PAN")
        self._assert_run_throw_device_exception(self._sut, "CheckBTIperfResults failed")

    def test_tgts_met_strict(self):
        self._create_sut("both", 1000000.0, 1000000.0, True, "PAN")
        self._assert_run_succeeded(self._sut)

    def test_tgts_not_met_relaxed(self):
        self._create_sut("both", 1000.0, 1000.0, False, "PAN")
        self._assert_run_succeeded(self._sut)

    def test_tgts_model_not_specified(self):
        self._create_sut("both", 1000000.0, 1000000.0, True, "PAN")
        self._sut._device.get_phone_model.return_value = "MOFD_V0_64-Bogus"
        self._assert_run_throw_config_exception(self._sut, "\.*DeviceModel \S+ does not exist \.*")

    def test_tgts_not_specified(self):
        self._create_sut("both", 1000000.0, 1000000.0, True, "XYZ")
        self._assert_run_throw_config_exception(self._sut, "BT_Throughput_Targets does not have targets for\.*")

    def _create_sut(self, dir, ul, dl, strict, prot):
        self._sut = CheckBTIperfResults(None, None, {"DIRECTION": dir, "BT_PROTOCOL": prot, "STRICT_TARGET": strict,
                                                      "UL_VALUE": ul, "DL_VALUE": dl, "UL_UNITS": "kbits/sec",
                                                      "DL_UNITS": "kbits/sec"}, mock.Mock())
        self._sut._device.get_phone_model.return_value = "MOFD_V0_64-Android-KK"
        return self._sut
