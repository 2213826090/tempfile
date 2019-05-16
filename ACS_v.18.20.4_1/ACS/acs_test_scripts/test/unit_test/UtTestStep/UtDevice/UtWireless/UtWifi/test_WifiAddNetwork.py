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
:since 16/07/2014
:author: jfranchx
"""
import mock
import unittest

from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from acs_test_scripts.TestStep.Device.Wireless.Wifi.WifiAddNetwork import WifiAddNetwork


class WifiAddNetworkTest(UTestTestStepBase):
    """
    Wifi Add Network test cases
    """

    def setUp(self):
        """
        Set up
        """
        UTestTestStepBase.setUp(self)

    def test_add_network_simple_ok(self):
        sut = self._create_sut({"SSID": "TEST_SSID", "SECURITY": "WPA2-PSK-AES", "PASS_PHRASE": "1234567890123"})
        self._assert_run_succeeded(sut)
        self._method_connect.assert_called_with('TEST_SSID', "1234567890123", "WPA2-PSK-AES", "dhcp", 'None', 'None',
                                                'None', 'None', 'None', "NONE", 'None', 'None', 'None')

    def test_add_network_static_ip_ok(self):
        sut = self._create_sut({"SSID": "TEST_SSID", "SECURITY": "WPA2-PSK-AES", "PASS_PHRASE": "1234567890123",
                                "STATIC_IP": "static", "IP_ADDRESS": "192.168.0.142",
                                "IP_NETMASK": "255.255.255.0", "IP_GATEWAY": "192.168.0.1",
                                "IP_DNS1": "0.0.0.0", "IP_DNS2": "0.0.0.0"})
        self._assert_run_succeeded(sut)
        self._method_connect.assert_called_with('TEST_SSID', "1234567890123", "WPA2-PSK-AES", "static", "192.168.0.142",
                                                "255.255.255.0", "192.168.0.1", "0.0.0.0", "0.0.0.0", "NONE", 'None',
                                                'None', 'None')

    def test_add_network_proxy_ok(self):
        sut = self._create_sut(
            {"SSID": "TEST_SSID", "SECURITY": "WPA2-PSK-AES", "PASS_PHRASE": "1234567890123", "PROXY_CONFIG": "MANUAL",
             "PROXY_ADDRESS": "192.168.0.222", "PROXY_PORT": "8080", "PROXY_BYPASS": "192.168.0.150"})
        self._assert_run_succeeded(sut)
        self._method_connect.assert_called_with('TEST_SSID', "1234567890123", "WPA2-PSK-AES", "dhcp", 'None', 'None',
                                                'None', 'None', 'None', "MANUAL", "192.168.0.222", "8080",
                                                "192.168.0.150")

    def test_add_network_full_ok(self):
        sut = self._create_sut(
            {"SSID": "TEST_SSID", "SECURITY": "WPA2-PSK-AES", "PASS_PHRASE": "1234567890123", "STATIC_IP": "static",
             "IP_ADDRESS": "192.168.0.142", "IP_NETMASK": "255.255.255.0", "IP_GATEWAY": "192.168.0.1",
             "IP_DNS1": "0.0.0.0", "IP_DNS2": "0.0.0.0", "PROXY_CONFIG": "MANUAL", "PROXY_ADDRESS": "192.168.0.222",
             "PROXY_PORT": "8080", "PROXY_BYPASS": "192.168.0.150"})
        self._assert_run_succeeded(sut)
        self._method_connect.assert_called_with('TEST_SSID', "1234567890123", "WPA2-PSK-AES", "static", "192.168.0.142",
                                                "255.255.255.0", "192.168.0.1", "0.0.0.0", "0.0.0.0", "MANUAL",
                                                "192.168.0.222", "8080", "192.168.0.150")


    # pylint: disable=W0212
    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """
        sut = WifiAddNetwork(None, None, test_step_pars, mock.Mock())
        self._method_connect = sut._api.set_wificonfiguration
        return sut


if __name__ == "__main__":
    # import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
