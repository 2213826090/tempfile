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

:organization: INTEL MCG
:summary: unit test
:since 2014-07-23
:author: emarchan
"""
import mock

from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from acs_test_scripts.TestStep.Device.Wireless.WifiDirect.SetWifiDirectDevName import SetWifiDirectDevName

NAME_PARAM = "PEER_DEVICE_NAME"
class WifiDirectSetDevNameTest(UTestTestStepBase):
    """
    Set wifi direct device name
    """

    def setUp(self):
        """
        Set up
        """
        UTestTestStepBase.setUp(self)

    def test_set_name_ok(self):
        dev_name = "blabla"
        sut = self._create_sut({NAME_PARAM: dev_name})
        sut.run(self._context)
        sut._api.set_wifi_direct_dev_name.assert_called_once_with(dev_name)

    # pylint: disable=W0212
    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """
        sut = SetWifiDirectDevName(None, None, test_step_pars, mock.Mock())
        return sut
