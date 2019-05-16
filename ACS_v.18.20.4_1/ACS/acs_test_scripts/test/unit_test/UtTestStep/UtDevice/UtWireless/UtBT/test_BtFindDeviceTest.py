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
:since: 19/05/2014
:author: jfranchx
"""
import unittest2 as unittest
import mock

from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.TestStep.Device.Wireless.BT.BtFindDevice import BtFindDevice
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from Core.TestStep.TestStepContext import TestStepContext


class BtFindDeviceTest(UTestTestStepBase):
    """
    SetPower test cases
    """

    def setUp(self):
        """
        Set up
        """
        UTestTestStepBase.setUp(self)
        self._context = TestStepContext()

    def test_find_device_ok(self):
        """
        Test device found succeed
        """
        sut = self._create_sut({"DEVICE_TO_FIND": "FIND_DEVICE_OK", "MUST_FIND": True})
        self._set_available_device(True)
        self._assert_run_succeeded(sut)

    def test_find_device_fail(self):
        """
        Test device found fail
        """
        sut = self._create_sut({"DEVICE_TO_FIND": "FIND_DEVICE_FAIL", "MUST_FIND": True})
        self._set_available_device(False)
        self._assert_run_throw_device_exception(sut, "Device FIND_DEVICE_FAIL not found")

    def test_not_find_device_ok(self):
        """
        Test not found device succeed
        """
        sut = self._create_sut({"DEVICE_TO_FIND": "NOT_FIND_DEVICE_OK", "MUST_FIND": False})
        self._set_available_device(False)
        self._assert_run_succeeded(sut)

    def test_not_find_device_fail(self):
        """
        Test not found device fail
        """
        sut = self._create_sut({"DEVICE_TO_FIND": "NOT_FIND_DEVICE_FAIL", "MUST_FIND": False})
        self._set_available_device(True)
        self._assert_run_throw_device_exception(sut, "Device NOT_FIND_DEVICE_FAIL found")

    def _set_available_device(self, device_found):
        """
        Set result info returned by paring api
        """
        self._method.return_value = device_found

    # pylint: disable=W0212
    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """

        sut = BtFindDevice(None, None, test_step_pars, mock.Mock())
        self._method = sut._api.bt_find_device

        return sut

if __name__ == "__main__":
    # import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
