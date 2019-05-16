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
from acs_test_scripts.TestStep.Equipment.ConfigurableAP.APWifiSetChannel import APWifiSetChannel


class APWifiSetChannelTest(UTestTestStepBase):

    def setUp(self):
        UTestTestStepBase.setUp(self)

    def test_set_channel_2_4GHz_ok(self):
        sut = self._create_sut({"CHANNEL": "2", "STANDARD": "n2.4G"})
        self._assert_run_succeeded(sut)
        sut._configurable_ap.set_wifi_channel.assert_called_once_with("n2.4G", "2")

    def test_set_channel_5GHz_ok(self):
        sut = self._create_sut({"CHANNEL": "44", "STANDARD": "n5G"})
        self._assert_run_succeeded(sut)
        sut._configurable_ap.set_wifi_channel.assert_called_once_with("n5G", "44")

    def _create_sut(self, args=None):
        sut = APWifiSetChannel(None, mock.Mock(), args, mock.Mock())
        return sut
