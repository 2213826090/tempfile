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
from ErrorHandling.AcsConfigException import AcsConfigException


# Ignore W0212 has mocking / stubbing is needed for testing purpose (hence, accessing protected members)
# Ignore C0111 has methods names should be self describing.
# Ignore W0212 has some times names longer than 30 chars are needed to well express what the method does
# pylint: disable=C0111,C0103,W0212
class ScanTest(BaseTestCase):
    """
    Scan tests
    """
    def test_scan_ok(self):
        """
        Test scan ok
        """

        self._uecmd_will_return([{"Address": self.BDADDRESS, "Name": "Unittest"}])
        devices = self._sut.bt_scan_devices()

        self.assertEqual(1, len(devices), "Devices count is supposed to be 1")
        self.assertEqual(self.BDADDRESS, devices[0].address)
        self.assertEqual("Unittest", devices[0].name)

    def test_scan_no_devices(self):
        """
        Test no devices found
        """
        self._uecmd_will_return([])

        devices = self._sut.bt_scan_devices()

        self.assertEqual(0, len(devices), "Devices count is supposed to be 0")

    def test_find_device_invalid_add(self):
        """
        Invalid address passed to bt_find_device
        """
        self._sut.bt_scan_devices = lambda: []
        self.assertFalse(self._sut.bt_find_device("XX:XX:XX:XX:XX:XX"), "bt_find_device is supposed to return False")

    def test_find_device_not_found(self):
        """
        Valid Device address is not in the list
        """
        self._sut.bt_scan_devices = lambda: []
        self.assertFalse(self._sut.bt_find_device(self.BDADDRESS), "bt_find_device is supposed to return False")

    def test_find_device_addr_ok(self):
        """
        Valid Device address is not in the list
        """
        self._uecmd_will_return([{"Address": self.BDADDRESS}])
        self.assertTrue(self._sut.bt_find_device(self.BDADDRESS), "bt_find_device is supposed to return True")

    def test_find_device_name_ok(self):
        """
        Valid Device address is not in the list
        """
        self._uecmd_will_return([{"Name": "Unittest"}])
        self.assertTrue(self._sut.bt_find_device("Unittest"), "bt_find_device is supposed to return True")

    def test_get_bt_scan_mode(self):
        """
        Test get_bt_scan_mode
        """
        self._sut._internal_exec_v2 = lambda x, y: {"result": "both"}
        value = self._sut.get_bt_scan_mode()

        self.assertEqual("both", value, "get_bt_scan_mode is supposed to return 'both'")

    def test_browsing_invalid_addr(self):
        """
        Test bt_service_browsing gets invalid address
        """
        with self.assertRaisesRegexp(AcsConfigException, "BD address 'XX:XX:XX' has a bad format!"):
            self._sut.bt_service_browsing("xx:xx:xx")

    def test_browsing_invalid_class(self):
        """
        Test bt_service_browsing gets an invalid bluetooth class
        """
        with self.assertRaisesRegexp(AcsConfigException,
                                     "bt_service_browsing : Parameter class_to_browse fake is not valid"):
            self._sut.bt_service_browsing(self.BDADDRESS, "fake")

    def test_browsing_class_found(self):
        """
        Test bt_service_browsing with an existing class
        """
        self._make_sut_return_a2dp()
        result, device = self._sut.bt_service_browsing(self.BDADDRESS, "A2DP")

        self.assertTrue(result, "result is supposed to be True")
        self.assertEqual(1, len(device.uuids))
        self.assertEqual("0000110D-0000-1000-8000-00805F9B34FB", device.uuids[0])

    def test_browsing_class_not_found(self):
        """
        Test bt_service_browsing with an existing class
        """
        self._make_sut_return_a2dp()
        result, device = self._sut.bt_service_browsing(self.BDADDRESS, "AVRCP-TG")

        self.assertFalse(result, "result is supposed to be False")
        self.assertEqual(1, len(device.uuids))
        self.assertEqual("0000110D-0000-1000-8000-00805F9B34FB", device.uuids[0])

    def test_browsing_class_not_given(self):
        """
        Test bt_service_browsing with an existing class
        """
        self._make_sut_return_a2dp()
        result, device = self._sut.bt_service_browsing(self.BDADDRESS)

        self.assertTrue(result, "result is supposed to be True")
        self.assertEqual(1, len(device.uuids))
        self.assertEqual("0000110D-0000-1000-8000-00805F9B34FB", device.uuids[0])

    def test_browsing_classes_not_found(self):
        """
        Test bt_service_browsing with an existing class
        """
        self._uecmd_will_return([])
        result, device = self._sut.bt_service_browsing(self.BDADDRESS)

        self.assertFalse(result, "result is supposed to be False")
        self.assertEqual(0, len(device.uuids))

    def test_flush_bt_scanned_devices(self):
        """
        """
        self._exec_will_return(None)
        self._sut.flush_bt_scanned_devices()
        self._sut._exec.assert_called_with("adb shell am force-stop com.android.settings")

    def _make_sut_return_a2dp(self):
        """
        Return a2dp service record
        """
        self._uecmd_will_return([{"Address": self.BDADDRESS, "Name": "Unittest",
                                                                "Uuid": "0000110D-0000-1000-8000-00805F9B34FB"}])

    def _uecmd_will_return(self, expected):
        """
        Sets what internal exec will return
        """
        self._sut._internal_exec_multiple_v2 = mock.MagicMock(name="_internal_exec_v2", return_value=expected)
