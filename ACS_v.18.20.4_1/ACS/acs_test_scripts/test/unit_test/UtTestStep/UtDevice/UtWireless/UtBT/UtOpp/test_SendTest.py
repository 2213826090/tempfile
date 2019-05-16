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
:since:30/12/2013
:author: fbongiax
"""
import unittest
import mock

from acs_test_scripts.TestStep.Device.Wireless.BT.Opp.BtOppSend import BtOppSend
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase


class BtOppSendTest(UTestTestStepBase):
    """
    BtOppSend test cases
    """

    FILES_ARGS = {"FILES": "1MB.txt,2MB.txt"}

    # pylint: disable=C0103
    def setUp(self):
        """
        Setup method
        """
        UTestTestStepBase.setUp(self)

        self._check_service_response = []

    def test_run_ok(self):
        """
        Test runs ok
        """
        sut = self._create_sut(self.FILES_ARGS)

        self._assert_run_succeeded(sut)
        sut._api.bt_opp_clean_notification_list.assert_called_with()
        sut._api.bt_opp_send_file.assert_called_with("/sdcard/acs_files/1MB.txt,/sdcard/acs_files/2MB.txt", None)

    def test_run_ok_save_size_as(self):
        """
        Test runs ok save file size
        """
        sut = self._create_sut({"SAVE_FILE_SIZE_AS": "size"})

        self._assert_run_succeeded(sut)

    def test_run_ok_save_info_as(self):
        """
        Test runs ok save file size
        """
        sut = self._create_sut({"SAVE_INFO_AS": "info"})

        self._assert_run_succeeded(sut)

    def test_run_no_file_size(self):
        """
        Test runs can't find the source file
        """
        sut = self._create_sut()
        # device's get_file_size() returns 0
        # pylint: disable=W0212
        sut._phonesystem_api.get_file_size.return_value = 0

        self._assert_run_throw_device_exception(sut, "Filesize is not accessible. File is probably missing on the device")

    # pylint: disable=W0212
    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """
        args = self.FILES_ARGS if not test_step_pars else dict(self.FILES_ARGS.items() + test_step_pars.items())
        sut = BtOppSend(None, None, args, mock.Mock())
        # Force multimedia_path to something known
        sut._device.multimedia_path = "/sdcard/acs_files/"
        # Stub get_file_size returning a known value
        sut._phonesystem_api.get_file_size.return_value = 100

        return sut

if __name__ == "__main__":
    # import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
