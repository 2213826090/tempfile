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
from ErrorHandling.AcsConfigException import AcsConfigException
from unit_test.UtDevice.UtLocalConnectivity.BaseTestCase import BaseTestCase
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.Device.UECmd.UECmdTypes import BtProfile
from acs_test_scripts.Device.UECmd.UECmdTypes import BtConState, BtAudioState


# Ignore W0212 has mocking / stubbing is needed for testing purpose (hence, accessing protected members)
# Ignore C0111 has methods names should be self describing.
# Ignore W0212 has some times names longer than 30 chars are needed to well express what the method does
# pylint: disable=C0111,C0103,W0212
class ProfileTest(BaseTestCase):
    """
    Test suite for profiles APIs
    """
    def test_connect_invalid_profile(self):
        with self.assertRaisesRegexp(AcsConfigException, "connectProfile: not supported profile INVALID"):
            self._sut.connect_bt_device(self.BDADDRESS, "invalid")

    def test_connect_invalid_result(self):
        # return empty result
        self._uecmd_will_return({})
        self._assert_connect_raise_dev_exc(self.BDADDRESS, "A2DP", "connectProfile: No profile returned")

    def test_connect_wrong_profile(self):
        self._uecmd_will_return({BtProfile.NAME: BtProfile.HSP})
        self._assert_connect_raise_dev_exc(self.BDADDRESS, "A2DP", "connectProfile: Bad profile returned HSP")

    def test_connect_no_status_no_output(self):
        self._uecmd_will_return({BtProfile.NAME: BtProfile.A2DP})
        self._assert_connect_raise_dev_exc(self.BDADDRESS, "A2DP", "connectProfile ERROR")

    def test_connect_no_status_with_output(self):
        self._uecmd_will_return({BtProfile.NAME: BtProfile.A2DP, "output": "generic error"})
        self._assert_connect_raise_dev_exc(self.BDADDRESS, "A2DP", "connectProfile ERROR: generic error")

    def test_connect_invalid_status(self):
        self._uecmd_will_return({BtProfile.NAME: BtProfile.HSP, BtConState.NAME: "invalid"})
        self._assert_connect_raise_dev_exc(self.BDADDRESS, "HSP", "connectProfile: Bad connection return value invalid")

    def test_connect_ok(self):
        self._uecmd_will_return({BtProfile.NAME: BtProfile.HSP, BtConState.NAME: BtConState.CONNECTED})
        self.assertTrue(self._sut.connect_bt_device(self.BDADDRESS, "HSP"),
                                                    "connect_bt_device() is supposed to return True")

    def test_connect_fails(self):
        self._uecmd_will_return({BtProfile.NAME: BtProfile.HSP, BtConState.NAME: BtConState.DISCONNECTED})
        self.assertFalse(self._sut.connect_bt_device(self.BDADDRESS, "HSP"),
                                                    "connect_bt_device() is supposed to return False")

    def test_connect_bt_hid_device_not_implemented(self):
        with self.assertRaisesRegexp(DeviceException, "connect_bt_hid_device not implemented on Android"):
            self._sut.connect_bt_hid_device(self.BDADDRESS)

    def test_get_bt_connection_state_invalid_profile(self):
        with self.assertRaisesRegexp(AcsConfigException, "getProfileConnectionState: not supported profile OPP"):
            self._sut.get_bt_connection_state(self.BDADDRESS, "OPP")

    def test_get_bt_connection_state_no_output_tag(self):
        self._uecmd_will_return({})
        self._assert_get_bt_connection_state_raise_dev_exc(self.BDADDRESS, BtProfile.HSP,
                                                           "getProfileConnectionState ERROR")

    def test_get_bt_connection_state_no_state_tag(self):
        self._uecmd_will_return({"output": "some error"})
        self._assert_get_bt_connection_state_raise_dev_exc(self.BDADDRESS, BtProfile.HSP,
                                                           "getProfileConnectionState ERROR: some error")

    def test_get_bt_connection_state_invalid_state(self):
        self._uecmd_will_return({BtConState.NAME: "invalid"})
        self._assert_get_bt_connection_state_raise_dev_exc(self.BDADDRESS, BtProfile.HSP,
                                                    "getProfileConnectionState: Bad connection return value invalid")

    def test_get_bt_connection_state_unknown_state(self):
        self._uecmd_will_return({BtConState.NAME: BtConState.UNKNOWN})
        self._assert_get_bt_connection_state_raise_dev_exc(self.BDADDRESS, BtProfile.HSP, "unknown connection state")

    def test_get_bt_connection_state_connected(self):
        self._uecmd_will_return({BtConState.NAME: BtConState.CONNECTED})
        self.assertEqual(1, self._sut.get_bt_connection_state(self.BDADDRESS, BtProfile.HSP))

    def test_get_bt_connection_state_disconnected(self):
        self._uecmd_will_return({BtConState.NAME: BtConState.DISCONNECTED})
        self.assertEqual(0, self._sut.get_bt_connection_state(self.BDADDRESS, BtProfile.HSP))

    def test_disconnect_bt_device_invalid_profile(self):
        with self.assertRaisesRegexp(AcsConfigException, "disconnectProfile: not supported profile OPP"):
            self._sut.disconnect_bt_device(self.BDADDRESS, BtProfile.OPP)

    def test_disconnect_bt_device_hid_profile(self):
        self._uecmd_will_return({BtProfile.NAME: BtProfile.HID, BtConState.NAME: BtConState.DISCONNECTED})
        self.assertTrue(self._sut.disconnect_bt_device(self.BDADDRESS, "HID"),
                                                    "disconnect_bt_device() is supposed to return True")

    def test_disconnect_bt_device_no_output(self):
        self._assert_disconnect_raise_dev_exc(self.BDADDRESS, BtProfile.A2DP, "disconnectProfile: No profile returned")

    def test_disconnect_bt_device_wrong_profile(self):
        self._uecmd_will_return({BtProfile.NAME: BtProfile.OPP})
        self._assert_disconnect_raise_dev_exc(self.BDADDRESS, BtProfile.A2DP,
                                              "disconnectProfile: Bad profile returned OPP")

    def test_disconnect_bt_device_invalid_state(self):
        self._uecmd_will_return({BtProfile.NAME: BtProfile.A2DP, BtConState.NAME: "invalid"})
        self._assert_disconnect_raise_dev_exc(self.BDADDRESS, BtProfile.A2DP,
                                              "disconnectProfile: Bad connection return value invalid")

    def test_disconnect_bt_device_no_output_state(self):
        self._uecmd_will_return({BtProfile.NAME: BtProfile.A2DP})
        self._assert_disconnect_raise_dev_exc(self.BDADDRESS, BtProfile.A2DP, "disconnectProfile ERROR")

    def test_disconnect_bt_device_specific_error(self):
        self._uecmd_will_return({BtProfile.NAME: BtProfile.A2DP, "output": "specific error"})
        self._assert_disconnect_raise_dev_exc(self.BDADDRESS, BtProfile.A2DP, "disconnectProfile ERROR: specific error")

    def test_disconnect_bt_device_ok(self):
        self._uecmd_will_return({BtProfile.NAME: BtProfile.A2DP, BtConState.NAME: BtConState.DISCONNECTED})
        self.assertTrue(self._sut.disconnect_bt_device(self.BDADDRESS, BtProfile.A2DP),
                        "disconnect_bt_device is supposed to return True")

    def test_disconnect_bt_device_fail(self):
        self._uecmd_will_return({BtProfile.NAME: BtProfile.A2DP, BtConState.NAME: BtConState.CONNECTED})
        self.assertFalse(self._sut.disconnect_bt_device(self.BDADDRESS, BtProfile.A2DP),
                        "disconnect_bt_device is supposed to return False")

    def test_get_bt_audio_state_wrong_profile(self):
        with self.assertRaisesRegexp(AcsConfigException, "getAudioState: not supported profile OPP"):
            self._sut.get_bt_audio_state(self.BDADDRESS, BtProfile.OPP)

    def test_get_bt_audio_state_no_output(self):
        self._assert_get_bt_audio_state_raise_dev_exc(self.BDADDRESS, BtProfile.A2DP, "getAudioState ERROR")

    def test_get_bt_audio_state_specific_error(self):
        self._uecmd_will_return({BtProfile.NAME: BtProfile.A2DP, "output": "specific error"})
        self._assert_get_bt_audio_state_raise_dev_exc(self.BDADDRESS, BtProfile.A2DP,
                                                      "getAudioState ERROR: specific error")

    def test_get_bt_audio_state_unknown(self):
        self._uecmd_will_return({BtProfile.NAME: BtProfile.A2DP, BtAudioState.NAME: "unknown"})
        self._assert_get_bt_audio_state_raise_dev_exc(self.BDADDRESS, BtProfile.A2DP,
                                                      "getAudioState: Bad audio state return value unknown")

    def test_get_bt_audio_state_playing(self):
        self._uecmd_will_return({BtProfile.NAME: BtProfile.A2DP, BtAudioState.NAME: BtAudioState.PLAYING})
        self.assertEqual(1, self._sut.get_bt_audio_state(self.BDADDRESS, BtProfile.A2DP))

    def test_get_bt_audio_state_stopped(self):
        self._uecmd_will_return({BtProfile.NAME: BtProfile.A2DP, BtAudioState.NAME: BtAudioState.STOPPED})
        self.assertEqual(0, self._sut.get_bt_audio_state(self.BDADDRESS, BtProfile.A2DP))

    def test_start_a2dp_media_player(self):
        self._sut.start_a2dp_media_player("file.mp3", 10)
        self._assert_bluetooth_api_was_called_with("startA2dpPlayer", "--es fileName file.mp3 --ei timeout 10", timeout=10)

    def test_stop_a2dp_media_player(self):
        self._sut.stop_a2dp_media_player()
        self._assert_bluetooth_api_was_called_with("stopA2dpPlayer")

    def _assert_get_bt_connection_state_raise_dev_exc(self, address, profile, msg):
        with self.assertRaisesRegexp(DeviceException, msg):
            self._sut.get_bt_connection_state(address, profile)

    def _assert_get_bt_audio_state_raise_dev_exc(self, address, profile, msg):
        with self.assertRaisesRegexp(DeviceException, msg):
            self._sut.get_bt_audio_state(address, profile)

    def _assert_disconnect_raise_dev_exc(self, address, profile, msg):
        with self.assertRaisesRegexp(DeviceException, msg):
            self._sut.disconnect_bt_device(address, profile)

    def _assert_connect_raise_dev_exc(self, address, profile, msg):
        with self.assertRaisesRegexp(DeviceException, msg):
            self._sut.connect_bt_device(address, profile)

    def _uecmd_will_return(self, expected):
        self._internal_exec_v2_will_return(expected)
