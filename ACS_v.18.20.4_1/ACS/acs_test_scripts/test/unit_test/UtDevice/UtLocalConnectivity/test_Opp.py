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
:since:13/01/2014
:author: fbongiax
"""
import mock

from unit_test.UtDevice.UtLocalConnectivity.BaseTestCase import BaseTestCase
from ErrorHandling.DeviceException import DeviceException


# Ignore W0212 has mocking / stubbing is needed for testing purpose (hence, accessing protected members)
# Ignore C0111 has methods names should be self describing.
# Ignore W0212 has some times names longer than 30 chars are needed to well express what the method does
# pylint: disable=C0111,C0103,W0212
class OppTest(BaseTestCase):
    """
    OPP test suite
    """
    def setUp(self):
        """
        Setup
        """
        BaseTestCase.setUp(self)
        self._files = None
        self._expected = {"id": "1", "address": self.BDADDRESS, "filename": "unittest.txt",
                    "filesize": "100", "downloadedsize": "100",
                    "status": "downloaded", "timestamp": "1234", "currenttime": "5678", "direction": "received"}
        self._expected_list = [self._expected, self._expected]

        # Not interested in knowing what we send to exec call
        self._sut._exec = lambda x: None

    def test_bt_opp_init(self):
        self._sut._phone_system.delete = mock.MagicMock()
        self._sut.bt_opp_init("unittest.txt")
        self._sut._phone_system.delete.assert_called_with("/sdcard/bluetooth/unittest.txt", False)

    def test_bt_opp_clean_notification_list(self):
        self._sut.bt_opp_clean_notification_list()
        self._assert_bluetooth_api_was_called_with("emptyOPPNotificationsList")

    def test_bt_opp_check_service_raw(self):
        self._sut._internal_exec_multiple_v2 = lambda x, y: self._expected
        result = self._sut.bt_opp_check_service_raw()

        self.assertEqual(self._expected, result)

    def test_bt_opp_check_service(self):
        self._sut._internal_exec_multiple_v2 = lambda x, y: self._expected_list

        expected = (['1', '1'],
                     ['11:22:33:44:55:66', '11:22:33:44:55:66'],
                     ['unittest.txt', 'unittest.txt'],
                     ['100', '100'],
                     ['100', '100'],
                     ['downloaded', 'downloaded'],
                     ['1234', '1234'],
                     ['5678', '5678'],
                     ['received', 'received'])

        # bt_opp_check_service() transforms the raw data into a tuple of arrays
        result = self._sut.bt_opp_check_service()

        self.assertEqual(expected, result)

    def test_bt_opp_send_file(self):
        self._sut.bt_opp_send_file("unittest.txt", self.BDADDRESS)
        self._assert_bluetooth_api_was_called_with("sendOPPfile", "--es Address 11:22:33:44:55:66 " \
                                                 "--es fileName unittest.txt", timeout=120)

    def test_bt_opp_cancel_send(self):
        self._sut.bt_opp_cancel_send()
        self._assert_bluetooth_api_was_called_with("cancelOPPTransfer")

    def test_bt_opp_get_files_checksum_no_file(self):
        with self.assertRaisesRegexp(DeviceException, "File unittest.txt not found"):
            self._sut.bt_opp_get_files_checksum(None, ["unittest.txt"])

    def test_bt_opp_get_files_checksum_file_not_found(self):
        self._sut._exec = lambda x: "No such file or directory"

        with self.assertRaisesRegexp(DeviceException, "File missing.txt not found"):
            self._sut.bt_opp_get_files_checksum(None, ["missing.txt"])

    def test_bt_opp_get_files_checksum(self):
        self._sut._exec = lambda x: "0423595f89569eb0e8ebbe30e86f3a6d  /sdcard/acs_files/1MB.txt"
        expected = {"1MB.txt": "0423595f89569eb0e8ebbe30e86f3a6d"}

        result = self._sut.bt_opp_get_files_checksum("/sdcard/acs_files", ["1MB.txt"])
        self.assertEqual(expected, result)
