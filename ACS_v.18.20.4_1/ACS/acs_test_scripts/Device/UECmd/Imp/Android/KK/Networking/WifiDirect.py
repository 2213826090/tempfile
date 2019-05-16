"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

:organization: UMG PSI Validation
:summary: This file implements the Wifi Direct specific methods.
:since: 2014-07-18
:author: emarchan

"""
import re
from acs_test_scripts.Device.UECmd.Imp.Android.Common.BaseV2 import BaseV2
from acs_test_scripts.Device.UECmd.Interface.Networking.IWifiDirect import IWifiDirect
from re import match
from time import sleep
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.Device.UECmd.Imp.Android.Common.Networking import Networking


class WifiDirect(BaseV2, IWifiDirect):
    """
    Implementation of the Wifi Direct specific methods.
    """

    KEYCODE_ENTER = "66"
    KEYCODE_DPAD_DOWN = "20"
    KEYCODE_MENU = "82"
    KEYCODE_BACK = "4"

    def __init__(self, device):
        """
        Constructor
        """
        BaseV2.__init__(self, device)
        self._wifidirect_module = "acscmd.connectivity.wifi.WifiDirectModule"

    def open_wifi_direct_menu_settings(self):
        """
        Display the Wifi direct settings page
        """
        # Go back to HOME to ensure there's no other activity in the front.
        self._device.get_uecmd("PhoneSystem").home_page_menu()

        # Open the Wifi settings
        self._exec("adb shell am start -n com.android.settings/com.android.settings.SubSettings -e :android:show_fragment com.android.settings.wifi.p2p.WifiP2pSettings")

        # Register the ACS listeners to get the P2P updates from android.
        method = "wifiDirectRegisterListeners"
        self._internal_exec_v2(self._wifidirect_module, method, is_system=True)

    def set_wifi_direct_dev_name(self, name=""):
        """
        Sets the name of the device seen in wifi direct

        :type name: String
        :param name: The name you want to set
        """
        method = "setWifiDirectDevName"
        cmd_args = "--es name %s" % name
        self._internal_exec_v2(self._wifidirect_module, method, cmd_args, is_system=True)

    def wifi_direct_scan_peer_devices(self):
        """
        initiates a wifi direct discovery of the peers (scan)
        """
        # Register the ACS listeners to get the P2P updates from android.
        method = "wifiDirectScanPeerDevices"
        self._internal_exec_v2(self._wifidirect_module, method, is_system=True)

    def wifi_direct_remove_remembered_groups(self):
        """
        Removes Wifi direct remembered groups
        """
        method = "wifiDirectRemoveRememberedGroups"
        self._internal_exec_v2(self._wifidirect_module, method, is_system=True)

    def get_wifi_direct_peer_devices(self, state="all"):
        """
        Gets the list of wifi direct peer devices scanned

        :type state: string (possible values all,available,invited,connected)
        :param state: If you want to get only the devices in a given state, use this field for filtering.
        :rtype: List of String
        :return name: list of wifi direct peer devices scanned
        """
        state = state.lower()
        if state not in ["all", "available", "invited", "connected"]:
            msg = "state value (%s) is invalid" % state
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # Register the ACS listeners to get the P2P updates from android.
        method = "wifiDirectRegisterListeners"
        self._internal_exec_v2(self._wifidirect_module, method, is_system=True)

        method = "getWifiDirectPeerDevices"
        params = "--es state %s" % state
        raw_peers = self._internal_exec_multiple_v2(self._wifidirect_module, method, params, is_system=True)

        dev_names = self._build_list_from_dict(raw_peers, "wifidirect_peer_device")
        if dev_names == []:
            self._logger.info("get_wifi_direct_peer_devices found 0 devices")
        for cur_dev in dev_names:
            self._logger.info("get_wifi_direct_peer_devices found: " + cur_dev)

        return dev_names

    def get_wifi_direct_dev_address_from_name(self, wifi_direct_name=""):
        """
        Gets a Wifi Direct device address from its name.

        :type wifi_direct_name: String
        :param wifi_direct_name: The name of the peer device you want the address of.
        :rtype: string
        :return: The peer device address
        """
        method = "getWifiDirectDevAddressFromName"
        cmd_args = "--es wifi_direct_name %s" % wifi_direct_name
        output = self._internal_exec_v2(self._wifidirect_module, method, cmd_args, is_system=True)
        wifi_direct_address = output["wifi_direct_address"]
        self._logger.info("get_wifi_direct_dev_address_from_name: %s = %s " % (wifi_direct_name, wifi_direct_address))
        return wifi_direct_address


    def wifi_direct_connect(self, peer_device_address="", timeout=20, go_intent=8, force_freq=0, connect_type="",
                            authentication_type="", pin=""):
        """
        Connect to a wifi direct peer device.

        :type peer_device_address: String
        :param peer_device_address: The address of the device you want to connect to.
        :type timeout: int
        :param timeout: Connection timeout.
        :type timeout: int
        :param go_intent: Group owner intent.
        :type timeout: int
        :param force_freq: Frequency used for the communication (forced).
        """
        if force_freq != 0:
            cmd = "adb shell setprop wifi.p2p.force_freq %s" % (force_freq)
            self._exec(cmd, timeout=5, wait_for_response=False)

        method = "wifiDirectConnect"
        cmd_args = "--es peer_device_address %s" % (peer_device_address)
        cmd_args += " --ei go_intent %s" % (go_intent)

        self._internal_exec_v2(self._wifidirect_module, method, cmd_args, is_system=True)

        self._logger.info("Waiting up to %d s for the peer to be connected" % timeout)
        remaining_time = timeout
        dev_connected = False
        while (remaining_time > 0) and (dev_connected == False):
            connected_peers = self.get_wifi_direct_peer_devices("connected")
            for cur_peer in connected_peers:
                peer_addr = self.get_wifi_direct_dev_address_from_name(cur_peer)
                if peer_addr.lower() == peer_device_address:
                    self._logger.info("Peer Device %s is connected after %s s" % \
                                      (peer_device_address, (timeout - remaining_time)))
                    dev_connected = True
                    break
            sleep(1)
            remaining_time -= 1
        if dev_connected is not True:
            msg = "wifi_direct_connect failed, timeout reached!"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    def wifi_direct_accept_connection(self, incoming_peer=""):
        """
        Grants a connection request from a peer device.

        :type incoming_peer: string
        :param incoming_peer: MAC address of the peer you want
        """

        # Allow the DUT to do the GO negotiation
        cmd = "adb shell wpa_cli p2p_connect %s pbc auth" % (incoming_peer)
        result = self._exec(cmd, timeout=5, wait_for_response=True)
        if "OK" not in result:
            msg = "Error executing p2p_connect command"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # Set properties to disable the popup notification on the DUT that receives the incoming connection request.
        cmd = "adb shell setprop wifi.p2p.wps_method pbc"
        self._exec(cmd, timeout=5, wait_for_response=False)
        cmd = "adb shell setprop sigma.wps_pin 00000000"
        self._exec(cmd, timeout=5, wait_for_response=False)

    def wifi_direct_disconnect(self, wifi_direct_name=""):
        """
        Disconnect from a wifi direct peer device.

        :type wifi_direct_name: String
        :param wifi_direct_name: The address of the device you want to disconnect from.
        """
        peer_device_address = None
        connected_peers = self.get_wifi_direct_peer_devices("connected")
        for cur_peer in connected_peers:
            if cur_peer == wifi_direct_name:
                peer_device_address = self.get_wifi_direct_dev_address_from_name(cur_peer)
                break
        if peer_device_address is None:
            msg = "wifi_direct_disconnect failed, the peer device is not connected to the DUT!"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        method = "wifiDirectDisconnect"
        cmd_args = "--es peer_device_address {0}".format(peer_device_address)
        self._internal_exec_v2(self._wifidirect_module, method, cmd_args, is_system=True)

    def get_wifi_direct_remembered_groups(self):
        """
        Gets the Wifi direct remembered groups.
        :rtype: List of String
        :return: list of wifi direct remembered groups.
        """
        # Register the ACS listeners to get the P2P updates from android.
        method = "wifiDirectRegisterListeners"
        self._internal_exec_v2(self._wifidirect_module, method, is_system=True)

        method = "getWifiDirectRememberedGroups"
        raw_groups = self._internal_exec_multiple_v2(self._wifidirect_module, method, is_system=True)

        rmbr_groups = self._build_list_from_dict(raw_groups, "wifidirect_remembered_group")
        for cur_group in rmbr_groups:
            self._logger.info("get_wifi_direct_remembered_groups found: " + cur_group)

        return rmbr_groups

    def check_wifi_direct_groups(self, group_owner="", groups_list=""):
        """
        Checks if the name of the group (regarding the group owner) is included in a list of groups.

        :type group_owner: String
        :param group_owner: Name of the peer device that initiated the group creation.
        :type groups_list: String
        :param groups_list: List of group names you check the name of. Separator is |

        :rtype: Boolean
        :return True if the group owner's group is in the list. False else
        """
        result = False

        expected_group_name = "DIRECT-[0-9A-Za-z]{2}-%s$" % group_owner
        groups_list = groups_list.split('|')

        for cur_group in groups_list:
            result_match = match(expected_group_name, cur_group)
            if result_match is not None:
                result = True
                break
        return result

    def get_wifi_direct_interface(self):
        """
        Gets the WiFi direct interface.
        :rtype: str
        :return: WiFi direct interface
        """

        cmd = "adb shell netcfg"
        output = self._exec(cmd)

        wifi_direct_interface = re.findall(r"(p2p-[wlan0p2]*-[0-9]*)", output)
        if wifi_direct_interface is None:
            msg = "WiFi Direct interface not found : %s" % output
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        return wifi_direct_interface[0]

    def get_wifi_direct_channel_freq(self):
        """
        Gets the current channel frequency used for WiFi Direct.
        :rtype: int
        :return: Current WiFi direct channel frequency .
        """

        # Register the ACS listeners to get the P2P updates from android.
        method = "wifiDirectRegisterListeners"
        self._internal_exec_v2(self._wifidirect_module, method, is_system=True)

        method = "getWifiDirectChannelFreq"

        result = self._internal_exec_v2(self._wifidirect_module, method, is_system=True)
        cur_channel = result["wifidirect_channel_freq"]
        return int(cur_channel)

    def get_wifi_direct_mode(self, wifi_direct_interface):
        """
        Gets the WiFi direct mode (CLI/GO).
        :type wifi_direct_interface: String
        :param wifi_direct_interface: Name of the WiFi Direct interface.
        :rtype: str
        :return: WiFi Direct mode
        """
        cmd = "adb shell wpa_cli IFNAME=%s status" % wifi_direct_interface
        output = self._exec(cmd)

        mode = re.findall(r"mode=([a-zA-Z0-9 ]+)", output)
        if mode is None:
            msg = "WiFi Direct mode can't be find : %s" % output
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        if len(mode) != 1:
            msg = "Multiple mode found for WiFi Direct : %s" % str(mode)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        if mode[0].lower() == "station":
            return "CLI"
        elif mode[0].lower() == "p2p go":
            return "GO"
        else:
            msg = "Unknown WiFi Direct mode : %s" % str(mode)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    def set_wifi_direct_mode(self, wifi_direct_mode):
        """
        Sets the WiFi direct mode (CLI/GO).
        :type wifi_direct_mode: String
        :param wifi_direct_mode: Mode to set. Possible values : GO or CLI.
        """
        if wifi_direct_mode == "GO":
            mode_value = 15
        elif wifi_direct_mode == "CLI":
            mode_value = 0
        else:
            msg = "ERROR set_wifi_direct_mode, invalid parameter value %s" % wifi_direct_mode
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        cmd = "adb shell setprop wifi.p2p.go_intent %s" % mode_value
        self._exec(cmd)

    def set_wifi_direct_frequency(self, wifi_direct_frequency):
        """
        Sets the WiFi direct frequency.
        :type wifi_direct_frequency: int
        :param wifi_direct_frequency: Frequency to force. Set a valid WiFi frequency in MHz.
        """
        if wifi_direct_frequency not in [2412, 2417, 2422, 2427, 2432, 2437, 2442, 2447, 2452, 2457, 2462, 2467, 2472,
                                         5180, 5200, 5220, 5240]:
            msg = "WiFi Direct set frequency - Invalid frequency value %s" % wifi_direct_frequency
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        cmd = "adb shell setprop wifi.p2p.force_freq %s" % wifi_direct_frequency
        self._exec(cmd)

    def get_wifi_direct_rssi(self, interface):
        """
        Gets the WiFi direct RSSI.
        :type interface: str
        :param interface: P2P interface to check
        :rtype: str
        :return: WiFi direct RSSI in dBm
        """
        networking_api = self._device.get_uecmd("Networking")
        wifi_chipset = networking_api._get_wifi_chipset_manufacturer()

        if wifi_chipset == networking_api.CHIPSET_INTEL:
            cmd = "adb shell iw %s link" % interface
            output = self._exec(cmd)
            wifi_direct_rssi = re.findall(r"signal: (.[0-9]*) dBm", output)

        elif wifi_chipset == networking_api.CHIPSET_BROADCOM:
            cmd = "adb shell wlx -i %s status" % interface
            output = self._exec(cmd)
            wifi_direct_rssi = re.findall(r"RSSI: (.[0-9]*) dBm", output)

        if wifi_direct_rssi is None:
            msg = "WiFi Direct RSSI not found : %s" % output
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        return wifi_direct_rssi[0]
