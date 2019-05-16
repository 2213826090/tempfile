# @PydevCodeAnalysisIgnore
# pylint: disable=E0602,W0212,C0103,C0111
"""
@copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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
:since: 17/07/14
:author: jfranchx
"""

import mock
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from acs_test_scripts.TestStep.Equipment.ConfigurableAP.APWifiSetAuthentication import APWifiSetAuthentication


class APWifiSetAuthenticationTest(UTestTestStepBase):
    AUTHENTICATION_OPEN = "OPEN"
    AUTHENTICATION_WPA2_PSK = "WPA2-PSK-AES"
    AUTHENTICATION_WPA2_EAP = "EAP-WPA2"
    PASSPHRASE_OPEN = "None"
    PASSPHRASE_WPA2 = "1234567890123"
    PASSPHRASE_EAP = "None"
    RADIUS_IP = "192.168.0.150"
    RADIUS_PORT = "1815"
    RADIUS_SECRET = "RadiusPass"
    STANDARD = "n2.4G"

    def setUp(self):
        UTestTestStepBase.setUp(self)

    def test_set_authentication_open_ok(self):
        sut = self._create_sut({"AUTHENTICATION_TYPE": self.AUTHENTICATION_OPEN, "PASSPHRASE": self.PASSPHRASE_OPEN,
                                "RADIUS_IP": self.RADIUS_IP, "RADIUS_PORT": self.RADIUS_PORT,
                                "RADIUS_SECRET": self.RADIUS_SECRET, "STANDARD": self.STANDARD})
        self._assert_run_succeeded(sut)
        sut._configurable_ap.set_wifi_authentication.assert_called_once_with(self.AUTHENTICATION_OPEN,
                                                                             self.PASSPHRASE_OPEN, self.RADIUS_IP,
                                                                             self.RADIUS_PORT, self.RADIUS_SECRET,
                                                                             self.STANDARD)

    def test_set_authentication_wpa2_psk_ok(self):
        sut = self._create_sut({"AUTHENTICATION_TYPE": self.AUTHENTICATION_WPA2_PSK, "PASSPHRASE": self.PASSPHRASE_WPA2,
                                "RADIUS_IP": self.RADIUS_IP, "RADIUS_PORT": self.RADIUS_PORT,
                                "RADIUS_SECRET": self.RADIUS_SECRET, "STANDARD": self.STANDARD})
        self._assert_run_succeeded(sut)
        sut._configurable_ap.set_wifi_authentication.assert_called_once_with(self.AUTHENTICATION_WPA2_PSK,
                                                                             self.PASSPHRASE_WPA2, self.RADIUS_IP,
                                                                             self.RADIUS_PORT, self.RADIUS_SECRET,
                                                                             self.STANDARD)

    def test_set_authentication_wpa2_eap_ok(self):
        sut = self._create_sut({"AUTHENTICATION_TYPE": self.AUTHENTICATION_WPA2_EAP, "PASSPHRASE": self.PASSPHRASE_EAP,
                                "RADIUS_IP": self.RADIUS_IP, "RADIUS_PORT": self.RADIUS_PORT,
                                "RADIUS_SECRET": self.RADIUS_SECRET, "STANDARD": self.STANDARD})
        self._assert_run_succeeded(sut)
        sut._configurable_ap.set_wifi_authentication.assert_called_once_with(self.AUTHENTICATION_WPA2_EAP,
                                                                             self.PASSPHRASE_EAP, self.RADIUS_IP,
                                                                             self.RADIUS_PORT, self.RADIUS_SECRET,
                                                                             self.STANDARD)

    def _create_sut(self, args=None):
        sut = APWifiSetAuthentication(None, mock.Mock(), args, mock.Mock())
        return sut
