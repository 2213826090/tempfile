"""
:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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
:since:08/01/2014
:author: fbongiax
"""
import unittest
import mock

from acs_test_scripts.TestStep.Device.Wireless.BT.BtConnectProfile import BtConnectProfile
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase


class BtConnectProfileTest(UTestTestStepBase):
    """
    GetAddress test cases
    """
    _COMMON_ARGS = {"PROFILE": "A2DP", "BDADDR": "11:22:33:44:55:66"}

    def test_connect_ok(self):
        """
        Test connection succeeds
        """
        # pylint: disable=W0212
        sut = self._create_sut({"CONNECT": True})
        # api returns connection was successful
        sut._api.connect_bt_device.return_value = True

        self._assert_run_succeeded(sut)

    def test_connect_failed(self):
        """
        Test connection fails
        """
        # pylint: disable=W0212
        sut = self._create_sut({"CONNECT": True})
        # api returns connection was unsuccessful
        sut._api.connect_bt_device.return_value = False

        self._assert_run_throw_device_exception(sut, "Unable to connect profile A2DP to 11:22:33:44:55:66")

    def test_disconnect_ok(self):
        """
        Test connection succeeds
        """
        # pylint: disable=W0212
        sut = self._create_sut({"CONNECT": False})
        # api returns connection was successful
        sut._api.disconnect_bt_device.return_value = True

        self._assert_run_succeeded(sut)

    def test_disconnect_failed(self):
        """
        Test connection fails
        """
        # pylint: disable=W0212
        sut = self._create_sut({"CONNECT": False})
        # api returns connection was unsuccessful
        sut._api.disconnect_bt_device.return_value = False

        self._assert_run_throw_device_exception(sut, "Unable to disconnect profile A2DP from 11:22:33:44:55:66")

    # pylint: disable=W0212
    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """
        args = self._COMMON_ARGS if not test_step_pars else dict(self._COMMON_ARGS.items() + test_step_pars.items())
        sut = BtConnectProfile(None, None, args, mock.Mock())
        return sut

if __name__ == "__main__":
    # import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
