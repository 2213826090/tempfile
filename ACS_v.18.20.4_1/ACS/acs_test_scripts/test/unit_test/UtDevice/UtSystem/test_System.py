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

from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.Device.UECmd.Imp.Android.Common.System.System import System
from unit_test_fwk.UTestBase import UTestBase


# Ignore W0212 has mocking / stubbing is needed for testing purpose (hence, accessing protected members)
# Ignore C0111 has methods names should be self describing.
# Ignore W0212 has some times names longer than 30 chars are needed to well express what the method does
# pylint: disable=C0111,C0103,W0212
class SystemTest(UTestBase):
    """
    Test suite for System API
    """

    STR_BTN_LEFT = "EV_MSC\tMSC_SCAN\t\t00090001\nEV_KEY\tBTN_LEFT\t\tDOWN\n" \
    "EV_MSC\tMSC_SCAN\t\t00090001\nEV_KEY\tBTN_LEFT\t\tUP\n"

    STR_BTN_RIGHT = "EV_MSC\tMSC_SCAN\t\t00090001\nEV_KEY\tBTN_RIGHT\t\tDOWN\n" \
    "EV_MSC\tMSC_SCAN\t\t00090001\nEV_KEY\tBTN_RIGHT\t\tUP\n"

    STR_ADD_DEVICE = "add device 1: /dev/input/event4\n\tname:\t\"Broadcom Bluetooth HID\""

    STR_MOUSE_NAME = "Broadcom Bluetooth HID"

    def setUp(self):
        UTestBase.setUp(self)
        self._sut = self._create_sut()

    def _create_sut(self):
        return System(mock.MagicMock())

    def test_check_hid_events_invalid_evt(self):
        with self.assertRaisesRegexp(DeviceException, "Invalid event argument \(forward\)"):
            self._sut.check_hid_events("unittest", ["forward"])

    def test_check_hid_events_device_not_found(self):
        self._exec_will_return("")
        with self.assertRaisesRegexp(DeviceException, "Bluetooth device not found \(unittest\)"):
            self._sut.check_hid_events("unittest", ["BTN_LEFT"])

    def test_check_hid_events_event_not_received(self):
        self._exec_seq_will_return([self.STR_ADD_DEVICE, ""])
        self.assertFalse(self._sut.check_hid_events(self.STR_MOUSE_NAME, ["BTN_LEFT"], 0), \
                         "check_hid_events is supposed to return false")

    def test_check_hid_events_ok(self):
        self._exec_seq_will_return([self.STR_ADD_DEVICE, self.STR_BTN_LEFT, self.STR_BTN_RIGHT])

        self.assertTrue(self._sut.check_hid_events(self.STR_MOUSE_NAME, ["BTN_LEFT", "BTN_RIGHT"], 0), \
                        "check_hid_events is supposed to return true")

    def test_check_hid_events_adb_command_syntax(self):
        self._exec_seq_will_return([self.STR_ADD_DEVICE, self.STR_BTN_LEFT])
        self._sut.check_hid_events(self.STR_MOUSE_NAME, ["BTN_LEFT"], 0)

        self._sut._exec.assert_called_with("adb shell getevent -c 8 -l /dev/input/event4", 0)
