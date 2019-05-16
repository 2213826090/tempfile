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
:since: 03/09/2014
:author: jfranchx
"""
import unittest2 as unittest
import mock

from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.TestStep.Device.Wireless.BT.BtRequestScan import BtRequestScan
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from Core.TestStep.TestStepContext import TestStepContext


class BtRequestScanTest(UTestTestStepBase):
    """
    BT request scan test cases
    """

    BT_SCAN_COMPLETE = "LIST_OF_BLUETOOTH_SCANNED_DEVICES"
    BT_SCAN_NULL = None

    def setUp(self):
        """
        Set up
        """
        UTestTestStepBase.setUp(self)
        self._context = TestStepContext()
        self._exception = False
        self._result = None

    def test_request_scan_ok(self):
        """
        Test device found succeed
        """
        sut = self._create_sut({"RAISE_ERROR": True})
        self._result = self.BT_SCAN_COMPLETE
        self._assert_run_succeeded(sut)

    def test_request_scan_null(self):
        """
        Test device found fail
        """
        sut = self._create_sut({"RAISE_ERROR": True})
        self._result = self.BT_SCAN_NULL
        self._assert_run_throw_device_exception(sut, "Scan result is null")

    def test_request_scan_exception(self):
        """
        Test device found fail
        """
        sut = self._create_sut({"RAISE_ERROR": True})
        self._exception = True
        self._assert_run_throw_device_exception(sut, "bt_scan_devices error")

    def test_request_scan_ignore_error_ok(self):
        """
        Test not found device succeed
        """
        sut = self._create_sut({"RAISE_ERROR": False})
        self._result = self.BT_SCAN_COMPLETE
        self._assert_run_succeeded(sut)

    def test_request_scan_ignore_error_null(self):
        """
        Test not found device succeed
        """
        sut = self._create_sut({"RAISE_ERROR": False})
        self._result = self.BT_SCAN_NULL
        self._assert_run_succeeded(sut)

    def test_request_scan_ignore_error_exception(self):
        """
        Test device found fail
        """
        sut = self._create_sut({"RAISE_ERROR": False})
        self._exception = True
        self._assert_run_succeeded(sut)

    def _bt_scan_devices(self):
        """
        Set result info returned by paring api
        """
        if self._exception is True:
            raise DeviceException(DeviceException.OPERATION_FAILED, "bt_scan_devices error")
        else:
            return self._result

    # pylint: disable=W0212
    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """

        sut = BtRequestScan(None, None, test_step_pars, mock.Mock())
        sut._api.bt_scan_devices = self._bt_scan_devices
        return sut

if __name__ == "__main__":
    # import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
