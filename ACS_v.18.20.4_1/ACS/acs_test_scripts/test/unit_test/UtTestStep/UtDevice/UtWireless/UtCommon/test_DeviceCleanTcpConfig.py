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
:since 04/12/2014
:author: jfranchx
"""
import mock
import unittest

from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from acs_test_scripts.TestStep.Device.Wireless.Common.DeviceCleanTcpConfig import DeviceCleanTcpConfig


class DeviceCleanTcpConfigTest(UTestTestStepBase):
    """
    Device clean TCP config test cases
    """

    def setUp(self):
        """
        Set up
        """
        UTestTestStepBase.setUp(self)
        self._sut = None


    def test_connect_ok(self):
        sut = self._create_sut({"DEVICE": "PHONE1"})
        self._assert_run_succeeded(sut)
        sut._networking_api.clean_tcp_records_device.assert_called_once_with()


    # pylint: disable=W0212
    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """
        sut = DeviceCleanTcpConfig(None, None, test_step_pars, mock.Mock())
        return sut

if __name__ == "__main__":
    # import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
