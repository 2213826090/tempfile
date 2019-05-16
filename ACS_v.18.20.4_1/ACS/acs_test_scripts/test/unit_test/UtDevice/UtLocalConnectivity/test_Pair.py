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

from acs_test_scripts.Device.UECmd.UECmdTypes import BT_PINVARIANT, BT_BOND_STATE
from acs_test_scripts.Device.UECmd.Imp.Android.Common.LocalConnectivity.LocalConnectivity import LocalConnectivity
from acs_test_scripts.Device.UECmd.Imp.Android.Common.LocalConnectivity.LocalConnectivity import BondState
from acs_test_scripts.Device.UECmd.Imp.Android.Common.LocalConnectivity.LocalConnectivity import BtPinVariant
from acs_test_scripts.Device.UECmd.UECmdTypes import BluetoothDevice
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException


# Ignore W0212 has mocking / stubbing is needed for testing purpose (hence, accessing protected members)
# Ignore C0111 has methods names should be self describing.
# Ignore W0212 has some times names longer than 30 chars are needed to well express what the method does
# pylint: disable=C0111,C0103,W0212
class PairTest(BaseTestCase):
    """
    Scan tests
    """
    STR_BONDED = "bonded"
    STR_BONDING = "bonding"

    def test_bt_check_msg(self):
        self._sut.bt_check_msg("hello world")
        self._assert_bluetooth_api_was_called_with("checkMessage", "--es Message 'hello world' --ei msglength 11")

    def test_bt_receive_msg(self):
        self._sut.bt_receive_msg()
        self._assert_bluetooth_api_was_called_with("waitForMessage")

    def test_bt_send_msg(self):
        self._sut.bt_send_msg(self.BDADDRESS, "hello world")
        self._assert_bluetooth_api_was_called_with("sendMessage", "--es Address %s " \
                                                 "--es Message 'hello world'" % self.BDADDRESS)

    def test_set_bt_pairable_not_implemented(self):
        self._sut.set_bt_pairable(None)
        self._assert_not_implemented_by_warning("set_bt_pairable")

    def test_get_bt_pairable_status(self):
        self._sut.get_bt_pairable_status()
        self._assert_not_implemented_by_warning("get_bt_pairable_status")

    def test_set_bt_discoverable_none(self):
        self._assert_set_bt_discoverable_called_with("none", 0, "--es mode 20 --es timeout 0")

    def test_set_bt_discoverable_noscan(self):
        self._assert_set_bt_discoverable_called_with("noscan", 0, "--es mode 20 --es timeout 0")

    def test_set_bt_discoverable_page(self):
        self._assert_set_bt_discoverable_called_with("page", 120, "--es mode 21 --es timeout 120")

    def test_set_bt_discoverable_connectable(self):
        self._assert_set_bt_discoverable_called_with("connectable", 120, "--es mode 21 --es timeout 120")

    def test_set_bt_discoverable_both(self):
        self._sut._exec = mock.MagicMock(name="_exec")
        self._sut.set_bt_discoverable("both", 120)
        self._sut._internal_exec_v2.assert_any_call(LocalConnectivity._BLUETOOTH_MODULE, "openPopupSetDiscoverableTimeout",
                                                       "--ei timeout 120")
        self._sut._internal_exec_v2.assert_called_with(LocalConnectivity._BLUETOOTH_MODULE, "setScanMode",
                                                       "--es mode 23 --es timeout 120")

    def test_set_bt_discoverable_invalid_timeout(self):
        with self.assertRaisesRegexp(AcsConfigException, "Parameter mode is not valid"):
            self._sut.set_bt_discoverable("both", 10)

    def test_set_bt_discoverable_invalid_inquiry(self):
        with self.assertRaisesRegexp(AcsConfigException, "set_bt_discoverable: Android does not support inquiry mode"):
            self._sut.set_bt_discoverable("inquiry", 0)

    def test_set_bt_discoverable_invalid_mode(self):
        with self.assertRaisesRegexp(AcsConfigException, "Parameter mode is not valid"):
            self._sut.set_bt_discoverable("no_valid_mode", 0)

    def test_get_bt_pairable_timeout_not_implemented(self):
        self._sut.get_bt_pairable_timeout()
        self._assert_not_implemented_by_warning("get_bt_pairable_timeout")

    def test_set_bt_autoconnect_not_implemented(self):
        self._sut.set_bt_autoconnect(None)
        self._assert_not_implemented_by_warning("set_bt_autoconnect")

    def test_get_bt_autoconnect_status_not_implemented(self):
        self._sut.get_bt_autoconnect_status()
        self._assert_not_implemented_by_warning("get_bt_autoconnect_status")

    def test_set_bt_scanning_not_implemented(self):
        self._sut.set_bt_scanning(None)
        self._assert_not_implemented_by_warning("set_bt_scanning")

    def test_pair_to_device_invalid_address(self):
        with self.assertRaisesRegexp(AcsConfigException, "BD address 'INVALID ADDRESS' has a bad format"):
            self._sut.pair_to_device("invalid address")

    def test_pair_to_device_invalid_reply_arg(self):
        with self.assertRaisesRegexp(AcsConfigException, "BD replyval \(4\) has a bad format"):
            self._sut.pair_to_device(self.BDADDRESS, replyval=4)

    def test_pair_to_device_pincode_arg_exceed_16_chars(self):
        with self.assertRaisesRegexp(AcsConfigException, "pincode '0123456789ABCDEFX' is too long"):
            self._sut.pair_to_device(self.BDADDRESS, pincode="0123456789ABCDEFX")

    def test_pair_to_device_passkey_arg_exceed_6_chars(self):
        with self.assertRaisesRegexp(AcsConfigException, "passkey '1234567' is too long"):
            self._sut.pair_to_device(self.BDADDRESS, passkey=1234567)

    def test_pair_to_device_invalid_reconnect_arg(self):
        with self.assertRaisesRegexp(AcsConfigException, "reconnect : Bad parameter value x"):
            self._sut.pair_to_device(self.BDADDRESS, reconnect="x")

    def test_pair_to_device_unpair_first_pin(self):
        device = BluetoothDevice()
        device.name = "unittest"
        device.address = self.BDADDRESS

        self._sut.list_paired_device = mock.MagicMock(return_value=[device])
        self._sut.unpair_bt_device = mock.MagicMock()
        self._internal_exec_v2_will_return({BondState.NAME: self.STR_BONDED, BtPinVariant.NAME: str(BtPinVariant.PIN)})

        self.assertEqual((BT_BOND_STATE.BOND_BONDED, BT_PINVARIANT.PIN), self._sut.pair_to_device(self.BDADDRESS, "on"))
        self._sut.unpair_bt_device.assert_called_with(self.BDADDRESS)

    def test_pair_to_device_no_pinres_returned(self):
        self._internal_exec_v2_will_return({BondState.NAME: self.STR_BONDED})
        self.assertEqual((BT_BOND_STATE.BOND_BONDED, BT_PINVARIANT.ERROR), self._sut.pair_to_device(self.BDADDRESS))

    def test_pair_to_device_passkey(self):
        self._internal_exec_v2_will_return({BondState.NAME: self.STR_BONDED,
                                            BtPinVariant.NAME: str(BtPinVariant.PASSKEY)})
        self.assertEqual((BT_BOND_STATE.BOND_BONDED, BT_PINVARIANT.PASSKEY), self._sut.pair_to_device(self.BDADDRESS))

    def test_pair_to_device_passkey_reply(self):
        self._internal_exec_v2_will_return({BondState.NAME: self.STR_BONDED,
                                            BtPinVariant.NAME: str(BtPinVariant.PASSKEY_CONFIRMATION)})
        self.assertEqual((BT_BOND_STATE.BOND_BONDED, BT_PINVARIANT.REPLY), self._sut.pair_to_device(self.BDADDRESS))

    def test_pair_to_device_passkey_consent(self):
        self._internal_exec_v2_will_return({BondState.NAME: self.STR_BONDED,
                                            BtPinVariant.NAME: str(BtPinVariant.CONSENT)})
        self.assertEqual((BT_BOND_STATE.BOND_BONDED, BT_PINVARIANT.REPLY), self._sut.pair_to_device(self.BDADDRESS))

    def test_pair_to_device_display_passkey(self):
        self._internal_exec_v2_will_return({BondState.NAME: self.STR_BONDED,
                                            BtPinVariant.NAME: str(BtPinVariant.DISPLAY_PASSKEY)})
        self.assertEqual((BT_BOND_STATE.BOND_BONDED, BT_PINVARIANT.NONE), self._sut.pair_to_device(self.BDADDRESS))

    def test_pair_to_device_display_pin(self):
        self._internal_exec_v2_will_return({BondState.NAME: self.STR_BONDED,
                                            BtPinVariant.NAME: str(BtPinVariant.DISPLAY_PIN)})
        self.assertEqual((BT_BOND_STATE.BOND_BONDED, BT_PINVARIANT.NONE), self._sut.pair_to_device(self.BDADDRESS))

    def test_pair_to_device_oob_consent(self):
        self._internal_exec_v2_will_return({BondState.NAME: self.STR_BONDED,
                                            BtPinVariant.NAME: str(BtPinVariant.OOB_CONSENT)})
        self.assertEqual((BT_BOND_STATE.BOND_BONDED, BT_PINVARIANT.NONE), self._sut.pair_to_device(self.BDADDRESS))

    def test_pair_to_device_pin_error(self):
        INVALID_PIN_VARIANT = "7"
        self._internal_exec_v2_will_return({BondState.NAME: self.STR_BONDED, BtPinVariant.NAME: INVALID_PIN_VARIANT})
        self.assertEqual((BT_BOND_STATE.BOND_BONDED, BT_PINVARIANT.ERROR), self._sut.pair_to_device(self.BDADDRESS))

    def test_pair_to_device_failed(self):
        self._internal_exec_v2_will_return({BondState.NAME: "none",
                                            BtPinVariant.NAME: str(BtPinVariant.PIN)})
        self.assertEqual((BT_BOND_STATE.BOND_NONE, BT_PINVARIANT.PIN), self._sut.pair_to_device(self.BDADDRESS))

    def test_pair_to_device_bad_return_value(self):
        self._internal_exec_v2_will_return({BondState.NAME: "invalid_state"})
        with self.assertRaisesRegexp(DeviceException, "pairDevice: Bad return value invalid_state"):
            self._sut.pair_to_device(self.BDADDRESS)

    def test_pair_to_device_no_bond_state_returned(self):
        self._internal_exec_v2_will_return({})
        with self.assertRaisesRegexp(DeviceException, "pairDevice ERROR"):
            self._sut.pair_to_device(self.BDADDRESS)

    def test_pair_to_device_specific_error(self):
        self._internal_exec_v2_will_return({"output": "specific error"})
        with self.assertRaisesRegexp(DeviceException, "pairDevice: Bad return value: specific error"):
            self._sut.pair_to_device(self.BDADDRESS)

    def test_unpair_bt_device(self):
        self._sut.unpair_bt_device(self.BDADDRESS)
        self._assert_bluetooth_api_was_called_with("unpairDevice", "--es Address %s" % self.BDADDRESS)

    def test_wait_for_pairing_invalid_address(self):
        with self.assertRaisesRegexp(AcsConfigException, "BD address 'XX:XX' has a bad format"):
            self._sut.wait_for_pairing("XX:XX")

    def test_wait_for_pairing_bonded(self):
        self._internal_exec_v2_will_return({"output": self.STR_BONDED})
        self.assertEqual(BT_BOND_STATE.BOND_BONDED, self._sut.wait_for_pairing(self.BDADDRESS))

    def test_wait_for_pairing_custom_timeout(self):
        self._internal_exec_v2_will_return({"output": self.STR_BONDED})
        self._sut.wait_for_pairing(self.BDADDRESS, timeout=3)
        self._sut._internal_exec_v2.assert_called_with("acscmd.connectivity.bt.BluetoothModule", "waitPairReq",
                                                       "--es Address %s --ei PairRep 1 " \
                                                       "--es Pincode 0000 --ei Passkey 0 --ei timeout 3" \
                                                       % self.BDADDRESS)

    def test_wait_for_pairing_bonding(self):
        self._internal_exec_v2_will_return({"output": self.STR_BONDING})
        self.assertEqual(BT_BOND_STATE.BOND_BONDING, self._sut.wait_for_pairing(self.BDADDRESS))

    def test_wait_for_pairing_invalid_bond_state(self):
        self._internal_exec_v2_will_return({"output": "invalid"})
        with self.assertRaisesRegexp(DeviceException, "Invalid device state \(waitPairReq ERROR: invalid\)"):
            self._sut.wait_for_pairing(self.BDADDRESS)

    def test_wait_for_pairing_invalid_reply_arg(self):
        with self.assertRaisesRegexp(AcsConfigException, "BD replyval \(4\) has a bad format"):
            self._sut.wait_for_pairing(self.BDADDRESS, replyval=4)

    def test_wait_for_pairing_pincode_arg_exceed_16_chars(self):
        with self.assertRaisesRegexp(AcsConfigException, "pincode '0123456789ABCDEFX' is too long"):
            self._sut.wait_for_pairing(self.BDADDRESS, pincode="0123456789ABCDEFX")

    def test_wait_for_pairing_passkey_arg_exceed_6_chars(self):
        with self.assertRaisesRegexp(AcsConfigException, "passkey '1234567' is too long"):
            self._sut.wait_for_pairing(self.BDADDRESS, passkey=1234567)

    def test_wait_for_pairing_invalid_reconnect_arg(self):
        with self.assertRaisesRegexp(AcsConfigException, "reconnect : Bad parameter value y"):
            self._sut.wait_for_pairing(self.BDADDRESS, reconnect="y")

    def test_wait_for_pairing_invalid_device_state(self):
        self._internal_exec_v2_will_return({})
        with self.assertRaisesRegexp(DeviceException, "Invalid device state \(waitPairReq ERROR\)"):
            self._sut.wait_for_pairing(self.BDADDRESS)

    def test_wait_for_pairing_canceled_fail(self):
        self._sut.listen_to_logcat = mock.MagicMock(return_value=(False, None))
        self.assertFalse(self._sut.wait_for_pairing_canceled(), "wait_for_pairing_canceled is supposed to return False")
        self._sut._logger.warning.assert_called_with_msg("Pairing canceled status not received from logcat")

    def test_wait_for_pairing_canceled_ok(self):
        self._sut.listen_to_logcat = mock.MagicMock(return_value=(True, None))
        self.assertTrue(self._sut.wait_for_pairing_canceled(), "wait_for_pairing_canceled is supposed to return True")

    def test_list_paired_device_none_returned(self):
        self._internal_exec_multiple_v2_will_return([])
        devices = self._sut.list_paired_device()
        self.assertEqual(0, len(devices))

    def test_list_paired_device_one_returned(self):
        self._internal_exec_multiple_v2_will_return([{"Address": self.BDADDRESS, "Name": "Unittest"}])

        devices = self._sut.list_paired_device()
        self.assertEqual(self.BDADDRESS, devices[0].address)
        self.assertEqual("Unittest", devices[0].name)

    def _assert_set_bt_discoverable_called_with(self, mode, timeout, expected_args):
        self._sut._internal_exec_v2 = mock.MagicMock(name="_internal_exec_v2")
        self._sut.set_bt_discoverable(mode, timeout)
        self._assert_bluetooth_api_was_called_with("setScanMode", expected_args)
