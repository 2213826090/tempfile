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
:since: 28/11/14
:author: jfranchx
"""

import mock
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from acs_test_scripts.TestStep.Equipment.ConfigurableAP.APWifiSetFullConfig import APWifiSetFullConfig


class APWifiSetFullConfigTest(UTestTestStepBase):
    SSID = "SSID"
    HIDDEN = True
    STANDARD = "n2.4G"
    AUTHENTICATION_TYPE = "OPEN"
    PASSPHRASE_TYPE = "None"
    CHANNEL = "1"
    DTIM = "3"
    BEACON = "250"
    WMM = "on"
    BANDWIDTH = "20"
    MIMO = False
    RADIUS_IP = "192.168.0.150"
    RADIUS_PORT = "1815"
    RADIUS_SECRET = "RadiusPass"

    def setUp(self):
        UTestTestStepBase.setUp(self)

    def test_set_authentication_open_ok(self):
        sut = self._create_sut({"SSID": self.SSID,
                                "HIDDEN": self.HIDDEN,
                                "STANDARD": self.STANDARD,
                                "AUTHENTICATION_TYPE": self.AUTHENTICATION_TYPE,
                                "PASSPHRASE": self.PASSPHRASE_TYPE,
                                "CHANNEL": self.CHANNEL,
                                "DTIM": self.DTIM,
                                "BEACON": self.BEACON,
                                "WMM": self.WMM,
                                "BANDWIDTH": self.BANDWIDTH,
                                "MIMO": self.MIMO,
                                "RADIUS_IP": self.RADIUS_IP,
                                "RADIUS_PORT": self.RADIUS_PORT,
                                "RADIUS_SECRET": self.RADIUS_SECRET})
        self._assert_run_succeeded(sut)
        sut._configurable_ap.set_wifi_config.assert_called_once_with(self.SSID,
                                                                     self.HIDDEN,
                                                                     self.STANDARD,
                                                                     self.AUTHENTICATION_TYPE,
                                                                     self.PASSPHRASE_TYPE,
                                                                     self.CHANNEL,
                                                                     self.DTIM,
                                                                     self.BEACON,
                                                                     self.WMM,
                                                                     self.BANDWIDTH,
                                                                     self.MIMO,
                                                                     self.RADIUS_IP,
                                                                     self.RADIUS_PORT,
                                                                     self.RADIUS_SECRET)

    def _create_sut(self, args=None):
        sut = APWifiSetFullConfig(None, mock.Mock(), args, mock.Mock())
        return sut
