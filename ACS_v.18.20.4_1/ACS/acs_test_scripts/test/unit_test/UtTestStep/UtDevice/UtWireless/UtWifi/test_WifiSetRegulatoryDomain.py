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
:since 07/08/2014
:author: jfranchx
"""
import mock

from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from acs_test_scripts.TestStep.Device.Wireless.Wifi.WifiSetRegulatoryDomain import WifiSetRegulatoryDomain
from Core.TestStep.TestStepContext import TestStepContext


class WifiSetRegulatoryDomainTest(UTestTestStepBase):
    """
    WifiSetRegulatoryDomain test cases
    """

    WIFI_REGULATORY_DOMAIN_FR = "FR"
    WIFI_REGULATORY_DOMAIN_US = "US"
    WIFI_REGULATORY_DOMAIN_JP = "JP"
    WIFI_INTERFACE = "wlan0"
    WIFI_ON = 1
    WIFI_OFF = 0

    def setUp(self):
        """
        Set up
        """
        UTestTestStepBase.setUp(self)
        self._context = TestStepContext()
        self._wifi_power_status = self.WIFI_ON
        self._responsed = []

    def _return_wifi_power_status(self):
        """
        Stub method
        """
        return self._wifi_power_status

    def test_set_regulatory_domain_fr_ok(self):
        sut = self._create_sut({"REGULATORY_DOMAIN": self.WIFI_REGULATORY_DOMAIN_FR, "INTERFACE": self.WIFI_INTERFACE})
        self._simulate_get_regulatorydomain_returning(sut, [self.WIFI_REGULATORY_DOMAIN_FR])
        self._assert_run_returned(sut, self.WIFI_REGULATORY_DOMAIN_FR, self.WIFI_INTERFACE)

    def test_set_regulatory_domain_us_ok(self):
        sut = self._create_sut({"REGULATORY_DOMAIN": self.WIFI_REGULATORY_DOMAIN_US, "INTERFACE": self.WIFI_INTERFACE})
        self._simulate_get_regulatorydomain_returning(sut, [self.WIFI_REGULATORY_DOMAIN_US])
        self._assert_run_returned(sut, self.WIFI_REGULATORY_DOMAIN_US, self.WIFI_INTERFACE)

    def test_set_regulatory_domain_fail(self):
        sut = self._create_sut({"REGULATORY_DOMAIN": self.WIFI_REGULATORY_DOMAIN_US, "INTERFACE": self.WIFI_INTERFACE})
        self._simulate_get_regulatorydomain_returning(sut, [self.WIFI_REGULATORY_DOMAIN_JP])
        self._assert_run_throw_device_exception(sut, "Set WiFi regulatory domain US failure")

    def test_set_regulatory_domain_wifi_off_fail(self):
        self._wifi_power_status = self.WIFI_OFF
        sut = self._create_sut({"REGULATORY_DOMAIN": self.WIFI_REGULATORY_DOMAIN_JP, "INTERFACE": self.WIFI_INTERFACE})
        self._assert_run_throw_device_exception(sut, "Can't set WiFi regulatory domain, WiFi is currently OFF")


    def test_set_regulatory_domain_ok_after_several_tries(self):
        sut = self._create_sut({"REGULATORY_DOMAIN": self.WIFI_REGULATORY_DOMAIN_US, "INTERFACE": self.WIFI_INTERFACE})
        sut._timeout = 5
        self._simulate_get_regulatorydomain_returning(sut,
                                                      [self.WIFI_REGULATORY_DOMAIN_FR, self.WIFI_REGULATORY_DOMAIN_FR,
                                                       self.WIFI_REGULATORY_DOMAIN_US])
        self._assert_run_returned(sut, self.WIFI_REGULATORY_DOMAIN_US, self.WIFI_INTERFACE)

    # ------------------------------------------------------------------------------------------------------------------

    def _side_effect_get_regulatorydomain_response(self):
        return self._responsed.pop(0)

    def _simulate_get_regulatorydomain_returning(self, sut, expected_regulatorydomain):
        self._responsed = expected_regulatorydomain
        sut._api.get_regulatorydomain.side_effect = self._side_effect_get_regulatorydomain_response
        return sut

    def _assert_run_returned(self, sut, regulatory_domain, interface):
        sut.run(None)
        sut._api.set_regulatorydomain.assert_called_once_with(regulatory_domain, interface)

    # pylint: disable=W0212
    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """
        sut = WifiSetRegulatoryDomain(None, None, test_step_pars, mock.Mock())
        sut._api.get_wifi_power_status = self._return_wifi_power_status
        sut._timeout = 0
        return sut

