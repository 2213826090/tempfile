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
:since:21/12/2013
:author: fbongiax
"""
import unittest
import mock
from unit_test.UtDevice.UtLocalConnectivity.BaseTestCase import BaseTestCase
from acs_test_scripts.Device.UECmd.Imp.Android.Common.LocalConnectivity.LocalConnectivity import LocalConnectivity
from acs_test_scripts.Device.UECmd.Imp.Android.Common.LocalConnectivity.LocalConnectivity import BtState
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException


# Ignore W0212 has mocking / stubbing is needed for testing purpose (hence, accessing protected members)
# Ignore C0111 has methods names should be self describing.
# Ignore W0212 has some times names longer than 30 chars are needed to well express what the method does
# pylint: disable=C0111,C0103,W0212
class ConfigTest(BaseTestCase):
    """
    Test cases testing config bluetooth device's API
    """
    STR_STATE_OFF = "STATE_OFF"
    STR_STATE_ON = "STATE_ON"
    STR_STATE_TURNING_OFF = "STATE_TURNING_OFF"
    STR_STATE_TURNING_ON = "STATE_TURNING_ON"

    def test_get_power_off(self):
        self._uecmd_will_return(BtState.STATE_OFF)
        self.assertEqual(self.STR_STATE_OFF, self._sut.get_bt_power_status())

    def test_get_power_on(self):
        self._uecmd_will_return(BtState.STATE_ON)
        self.assertEqual(self.STR_STATE_ON, self._sut.get_bt_power_status())

    def test_get_power_turning_on(self):
        self._uecmd_will_return(BtState.STATE_TURNING_ON)
        self.assertEqual(self.STR_STATE_TURNING_ON, self._sut.get_bt_power_status())

    def test_get_power_turning_off(self):
        self._uecmd_will_return(BtState.STATE_TURNING_OFF)
        self.assertEqual(self.STR_STATE_TURNING_OFF, self._sut.get_bt_power_status())

    def test_get_power_invalid_state(self):
        self._uecmd_will_return(0)
        with self.assertRaisesRegexp(DeviceException, "BT_STATE : invalid state value 0"):
            self._sut.get_bt_power_status()

    def test_set_power_on_ok(self):
        self.set_initial_bt_state("STATE_OFF")
        self._sut.set_bt_power("on")
        self._assert_set_power_success("--es state 1")

    def test_set_power_1_ok(self):
        self.set_initial_bt_state("STATE_OFF")
        self._sut.set_bt_power("1")
        self._assert_set_power_success("--es state 1")

    def test_set_power_off_ok(self):
        self.set_initial_bt_state("STATE_ON")
        self._sut.set_bt_power("off")
        self._assert_set_power_success("--es state 0")

    def test_set_power_0_ok(self):
        self.set_initial_bt_state("STATE_ON")
        self._sut.set_bt_power("0")
        self._assert_set_power_success("--es state 0")

    def test_set_power_on_already_on(self):
        self.set_initial_bt_state("STATE_ON")
        self._sut.set_bt_power("on")
        self.assertFalse(self._sut._internal_exec_v2.called)

    def test_set_power_off_already_off(self):
        self.set_initial_bt_state("STATE_OFF")
        self._sut.set_bt_power("off")
        self.assertFalse(self._sut._internal_exec_v2.called)

    def test_set_power_invalid_arg(self):
        self.set_initial_bt_state(None)
        with self.assertRaisesRegexp(AcsConfigException, "Parameter mode is not valid"):
            self._sut.set_bt_power("xxx")

    def test_reset_power(self):
        self.set_initial_bt_state(None)
        self._sut.bt_reset_device()
        self._sut._internal_exec_v2.assert_called_with(LocalConnectivity._BLUETOOTH_MODULE, "resetPower")

    def test_get_bt_power_status_eot_is_on(self):
        self._sut._uecmd_default_timeout = 0  # Speeds up the test
        self._get_bt_power_status_will_return(["STATE_ON"])
        self.assertEqual("STATE_ON", self._sut.get_bt_power_status_eot())

    def test_get_bt_power_status_eot_timeout(self):
        self._sut._uecmd_default_timeout = 0  # Speeds up the test
        self._get_bt_power_status_will_return(["STATE_OFF"])
        with self.assertRaisesRegexp(DeviceException, "power state transiton timeout:STATE_OFF"):
            self._sut.get_bt_power_status_eot("STATE_ON")

    def test_get_bt_power_status_eot_not_yet_on(self):
        self._sut._uecmd_default_timeout = 0  # Speeds up the test
        self._get_bt_power_status_will_return(["STATE_OFF", "STATE_TURNING_ON", "STATE_ON"])

        # Setting the timing to speed up the test
        self._sut._WAIT_FOR_POLL_SECS = 0
        time_out_secs = 1

        self.assertEqual("STATE_ON", self._sut.get_bt_power_status_eot("STATE_ON", time_out_secs))

    def test_bt_power_state_reached_ok(self):
        self._get_bt_power_status_will_return(["STATE_ON"])
        self.assertTrue(self._sut.bt_power_state_reached("STATE_ON"),
                        "bt_power_state_reached is supposed to return True")

    def test_bt_power_state_reached_fail(self):
        self._get_bt_power_status_will_return(["STATE_OFF"])
        time_out = 0
        self.assertFalse(self._sut.bt_power_state_reached("STATE_ON", time_out),
                        "bt_power_state_reached is supposed to return False")

    def test_bt_l2cap_ping_not_implemented(self):
        with self.assertRaisesRegexp(DeviceException, "bt_l2cap_ping not implemented on Android"):
            self._sut.bt_l2cap_ping(None, None, None)

    def test_set_bt_authentication(self):
        self._sut.set_bt_authentication(None, None)
        self._assert_not_implemented_by_warning("set_bt_authentication")

    def test_set_agent_property(self):
        self._sut.set_agent_property(None, None)
        self._assert_not_implemented_by_warning("set_agent_property")

    def test_set_bt_default_link_policy_unknown_mode(self):
        with self.assertRaisesRegexp(AcsConfigException, "Invalid parameter \(Unknown mode \(invalid\)\)"):
            self._sut.set_bt_default_link_policy("hci0", "invalid")

    def test_set_bt_default_link_policy_invalid_iface(self):
        with self.assertRaisesRegexp(AcsConfigException, "interface must be valid \(eg: hci0\)"):
            self._sut.set_bt_default_link_policy(None, "rswitch")

    def test_set_bt_default_link_policy_iface_not_found(self):
        self._exec_will_return(" ")
        with self.assertRaisesRegexp(DeviceException, "Operation failed \(Interface not found!\)"):
            self._sut.set_bt_default_link_policy("hci0", "rswitch")

    def test_set_bt_default_link_policy_ok(self):
        returned_from_exec = ["hci0", "", "RSWITCH"]
        self._sut._exec = mock.MagicMock(side_effect=lambda x: returned_from_exec.pop(0))
        self._sut.set_bt_default_link_policy("hci0", "rswitch")
        self._sut._exec.assert_called_with("adb shell hciconfig hci0 lp")

    def test_set_bt_default_link_policy_fail(self):
        returned_from_exec = ["hci0", "", ""]
        self._sut._exec = mock.MagicMock(side_effect=lambda x: returned_from_exec.pop(0))
        with self.assertRaisesRegexp(DeviceException, "Fail to set bluetooth default link policy"):
            self._sut.set_bt_default_link_policy("hci0", "rswitch")

    def test_activate_bt_test_mode_fail(self):
        self._exec_will_return("Example:")
        with self.assertRaisesRegexp(DeviceException, "Fail to activate bluetooth  test mode"):
            self._sut.activate_bt_test_mode("hci0")

    def test_set_bt_ctrl_event_mask_fail(self):
        self._exec_will_return("Example:")
        with self.assertRaisesRegexp(DeviceException, "Fail to set bluetooth  controler event mask"):
            self._sut.set_bt_ctrl_event_mask("hci0", "0x0001")

    def test_set_bt_ctrl_event_filter_fail(self):
        self._exec_will_return("Example:")
        with self.assertRaisesRegexp(DeviceException, "Fail to set bluetooth  controler event filter"):
            self._sut.set_bt_ctrl_event_filter("hci0", "0x01", "0x02", "0x03")

    def test_get_bt_adapter_address(self):
        self._internal_exec_v2_will_return({"Address": self.BDADDRESS})
        self.assertEqual(self.BDADDRESS, self._sut.get_bt_adapter_address())

    def test_get_default_addr_ok(self):
        self._sut._device.get_config = mock.MagicMock(return_value=self.BDADDRESS)
        self.assertEqual(self.BDADDRESS, self._sut.get_default_addr())

    def test_get_default_addr_ignore_bytes(self):
        self._sut._device.get_config = mock.MagicMock(return_value="11:22:XX:44:XX:66")
        self.assertEqual("11:22:XX:44:XX:66", self._sut.get_default_addr())

    def test_get_default_addr_invalid_addr(self):
        self._sut._device.get_config = mock.MagicMock(return_value="11:22:XX:")
        self.assertEqual("", self._sut.get_default_addr(),
                         "If invalid address in the bench config, get_default_addr() should return an empty string")

    def _get_bt_power_status_will_return(self, states):
        """
        Set up what get_bt_power_status will return when called.
        :type states: array
        :param states:  an array of states (or strings in general)
                        each of the values in states array will be returned when get_bt_power_status() is called.
                        In this way it's possibile to program the way get_bt_power_status() will behave.
        """
        self._sut.get_bt_power_status = mock.MagicMock(side_effect=lambda: states.pop(0))

    def _assert_set_power_success(self, expected_args):
        """
        Check that call to the UECmd API was done with the correct arguments
        """
        self._sut._internal_exec_v2.assert_called_with(LocalConnectivity._BLUETOOTH_MODULE, "setPower",
                                                       expected_args)

    def _uecmd_will_return(self, expected):
        self._internal_exec_v2_will_return({"Power_state": expected})

    def set_initial_bt_state(self, expected):
        """
        Create sut for set power command
        """
        self._sut._internal_exec_v2 = mock.MagicMock(name="_internal_exec_v2")
        self._sut.get_bt_power_status = mock.MagicMock(name="bt_power_status", return_value=expected)

if __name__ == "__main__":
    # import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
