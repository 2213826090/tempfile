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
:since 08/09/2014
:author: jfranchx
"""
import mock

from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from Core.TestStep.TestStepContext import TestStepContext
from acs_test_scripts.TestStep.Device.Wireless.NFC.NFCGetPower import NFCGetPower
from UtilitiesFWK.Utilities import TestConst


class NFCGetPowerTest(UTestTestStepBase):
    """
    NFC GetPower test cases
    """

    GET_PWR_ON = "on"
    GET_PWR_OFF = "off"

    def setUp(self):
        """
        Set up
        """
        UTestTestStepBase.setUp(self)
        self._dest_var = "result"

        self._context = TestStepContext()

    def test_get_power_on_ok(self):
        sut = self._create_sut()

        valueExp = self.GET_PWR_ON
        self._method.return_value = self.GET_PWR_ON

        sut.run(self._context)

        ValueGot = self._context.get_info(self._dest_var)

        self._assert_method_called_and_value_good(sut, self._method, valueExp, ValueGot,
                                                  'VERDICT: %s stored as %s' % (self._dest_var, str(valueExp)))

    def test_get_power_off_ok(self):
        sut = self._create_sut()

        valueExp = self.GET_PWR_OFF
        self._method.return_value = self.GET_PWR_OFF

        sut.run(self._context)

        ValueGot = self._context.get_info(self._dest_var)

        self._assert_method_called_and_value_good(sut, self._method, valueExp, ValueGot,
                                                  'VERDICT: %s stored as %s' % (self._dest_var, str(valueExp)))

    def test_get_power_off_ko(self):
        sut = self._create_sut()

        valueExp = self.GET_PWR_OFF
        self._method.return_value = self.GET_PWR_ON

        sut.run(self._context)

        ValueGot = self._context.get_info(self._dest_var)

        self._assert_method_called_and_value_bad(self._method, valueExp, ValueGot)

    def test_get_power_on_ko(self):
        sut = self._create_sut()

        valueExp = self.GET_PWR_ON
        self._method.return_value = self.GET_PWR_OFF

        sut.run(self._context)

        ValueGot = self._context.get_info(self._dest_var)

        self._assert_method_called_and_value_bad(self._method, valueExp, ValueGot)

    # pylint: disable=W0212
    def _create_sut(self):
        """
        Create the SUT with only test step pars
        """
        sut = NFCGetPower(None, None, {"SAVE_AS": self._dest_var}, mock.Mock())
        self._method = sut._api.get_nfc_status
        return sut
