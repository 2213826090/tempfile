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
:since 01/12/2014
:author: jfranchx
"""
import mock

from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from Core.TestStep.TestStepContext import TestStepContext
from TestStep.Device.Power.BatteryGetCapacity import BatteryGetCapacity


class BatteryGetCapacityTest(UTestTestStepBase):
    """
    BatteryGetCapacity unit test
    """

    def setUp(self):
        """
        Set up
        """
        UTestTestStepBase.setUp(self)
        self._context = TestStepContext()
        self._dest_var = "battery_current_capacity"
        self._capacity_info = {"CAPACITY": (90, 'none')}
        self._battery_info = {"BATTERY": self._capacity_info}

    def test_get_power_on_ok(self):
        sut = self._create_sut()

        self._method.return_value = self._battery_info
        sut.run(self._context)

        value_got = self._context.get_info(self._dest_var)
        self._assert_method_called_and_value_good(sut, self._method, 90, value_got,
                                                  'VERDICT: %s stored as %s' % (self._dest_var, str(90)))

    # pylint: disable=W0212
    def _create_sut(self):
        """
        Create the SUT with only test step pars
        """
        sut = BatteryGetCapacity(None, None, {"SAVE_BATTERY_CAPACITY_AS": self._dest_var}, mock.Mock())
        self._method = sut._em_api.get_msic_registers
        return sut
