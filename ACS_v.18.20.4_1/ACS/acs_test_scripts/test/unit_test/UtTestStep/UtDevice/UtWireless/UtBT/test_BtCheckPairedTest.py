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
:since:20/12/2013
:author: fbongiax
"""
import unittest
import mock

from acs_test_scripts.TestStep.Device.Wireless.BT.BtCheckPaired import BtCheckPaired
from acs_test_scripts.Device.UECmd.UECmdTypes import BluetoothDevice
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase


class CheckPairedTest(UTestTestStepBase):
    """
    CheckPaired test cases
    """

    def _stub_list_paired_device(self):
        """
        Stub method
        """
        device = BluetoothDevice()
        device.name = "Unit test"
        device.address = "11:22:33:44:55:66"
        return [device]

    def test_device_found(self):
        """
        Test device found
        """
        sut = self._create_sut({"BDADDR": "11:22:33:44:55:66"})
        self._assert_run_succeeded(sut)

    def test_device_not_found(self):
        """
        Test device not found
        """
        sut = self._create_sut({"BDADDR": "00:00:33:44:55:66"})
        self._assert_run_throw_device_exception(sut, "Device 00:00:33:44:55:66 is not paired")

    # pylint: disable=W0212
    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """

        sut = BtCheckPaired(None, None, test_step_pars, mock.Mock())
        sut._api.list_paired_device = self._stub_list_paired_device

        return sut

if __name__ == "__main__":
    # import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
