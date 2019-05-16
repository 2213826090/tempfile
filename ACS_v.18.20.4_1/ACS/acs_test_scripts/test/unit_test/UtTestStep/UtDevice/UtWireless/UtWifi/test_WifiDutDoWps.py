"""
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
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

:organization: INTEL MCG PSI
:summary: unit test
:since 28/11/2014
:author: Gangx, Lu
"""
import mock

from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from TestStep.Device.Wireless.Wifi.WifiDutDoWps import WifiDutDoWps
from ErrorHandling.DeviceException import DeviceException


class WifiDutDoWpsTest(UTestTestStepBase):
    """
    WifiDutDoWps test cases
    """

    def setUp(self):
        """
        Set up
        """
        UTestTestStepBase.setUp(self)
        self._wifi_power_status = 1

    def _return_wifi_power_status(self):
        """
        Stub method
        """
        return self._wifi_power_status

    def test_wifi_dut_do_wps_wifi_off_fail(self):
        self._wifi_power_status = 0
        sut = self._create_sut({"SSID":"ACS_TEST", "WPS_METHOD":"WPS_PBC", "WPS_AP_PIN":None})
        with self.assertRaises(DeviceException):
            sut.run(self._context)

    def test_wifi_dut_do_wps_ok(self):
        sut = self._create_sut({"SSID":"ACS_TEST", "WPS_METHOD":"WPS_PBC", "WPS_AP_PIN":None})
        sut.run(self._context)
        sut._api.wifi_setkeyexchange.assert_called_once_with("ACS_TEST", "WPS_PBC", None)

    # pylint: disable=W0212
    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """
        sut = WifiDutDoWps(None, None, test_step_pars, mock.Mock())
        sut._api.get_wifi_power_status = self._return_wifi_power_status
        return sut
