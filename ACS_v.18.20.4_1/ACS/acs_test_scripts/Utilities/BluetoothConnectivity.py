"""

:copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
The source code contained or described herein and all documents related
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

:organization: INTEL OTC ANDROID QA

:since: 2/10/15
:author: mmaraci
"""
from ErrorHandling.AcsConfigException import AcsConfigException
import time
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.Device.UECmd.UECmdTypes import BT_BOND_STATE, BtConState
from acs_test_scripts.Utilities.LocalConnectivityUtilities import disconnect_tethering
from acs_test_scripts.TestStep.Device.Wireless.BT.Constants import Constants

class BluetoothConnectivity():

    def __init__(self, device, wait_btwn_cmd):
        self._device = device
        self._wait_btwn_cmd = wait_btwn_cmd

    def handle_external_connection(self, connection_to_share, wifi_api, global_config, wifi_access_point):
        if connection_to_share == "WIFI":
            wifi_api.set_wifi_power("on")
            time.sleep(self._wait_btwn_cmd)
            self.wifi_connect(global_config, wifi_access_point, wifi_api)
        else:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "You did not choose WIFI to share as a parameter")


    def search_for_device(self, searching_device_api, searched_device_api, searched_device_addr, must_find):
        """
        The searched device is the one that looks for another device
        """
        try:
            device_found = searching_device_api.bt_find_device(searched_device_addr)
            time.sleep(self._wait_btwn_cmd)
            if not device_found and must_find is True:
                msg = "Device %s not found" % searched_device_addr
                self._device._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

            elif device_found and must_find is False:
                msg = "Device %s found" % searched_device_addr
                self._device._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        except DeviceException as acs_exception:
            if must_find is None:
                self._device._logger.debug("Error/Timeout on scan devices")
            else:
                raise acs_exception

    def search_for_device_interleave(self, searching_device_api, searched_device_api, searched_device_addr, must_find):
        """
        The searched device is the one that looks for another device
        """
        try:
            searched_device_api.set_bt_discoverable("both", 0)
            time.sleep(20)

            device_found = searching_device_api.bt_find_device_interleave_search(searched_device_addr)
            time.sleep(self._wait_btwn_cmd)
            if not device_found and must_find is True:
                msg = "Device %s not found" % searched_device_addr
                self._device._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

            elif device_found and must_find is False:
                msg = "Device %s found" % searched_device_addr
                self._device._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        except DeviceException as acs_exception:
            if must_find is None:
                self._device._logger.debug("Error/Timeout on scan devices")
            else:
                raise acs_exception

    def search_for_device_until_found(self, searching_device_api, searched_device_api, searched_device_addr, must_find):
        try:
            found = False
            searched_device_api.set_bt_discoverable("both", 0)
            time.sleep(20)
            searching_device_api.set_bt_discoverable("both", 0)
            while found is False:
                device_found = searching_device_api.bt_find_device(searched_device_addr)
                time.sleep(self._wait_btwn_cmd)
                if device_found and must_find is True:
                    found = True
                if not device_found and must_find is True:
                    found = False
                    msg = "Device %s not found" % searched_device_addr
                    self._device._logger.error(msg)

                elif device_found and must_find is False:
                    found = False
                    msg = "Device %s found" % searched_device_addr
                    self._device._logger.error(msg)

        except DeviceException as acs_exception:
            if must_find is None:
                self._device._logger.debug("Error/Timeout on scan devices")
            else:
                raise acs_exception

    def pair_devices(self, pairing_device_api, paired_device_api, pairing_device_addr, paired_device_addr, replyval, accept_connection):
        time.sleep(self._wait_btwn_cmd)
        bond_state = paired_device_api.wait_for_pairing(pairing_device_addr, reconnect=1, replyval=replyval, timeout=30)
        pair_result = pairing_device_api.pair_to_device(paired_device_addr, reconnect=1, replyval=replyval, passkey=0, pincode="0000")

        if pair_result[0] == BT_BOND_STATE.BOND_NONE:
            if accept_connection:
                msg = "Pairing with device failed"
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)
                # return False
            else:
                return_msg = "Pairing with device rejected"
                self._device._logger.info(return_msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, return_msg)

        if pair_result[0] == BT_BOND_STATE.BOND_BONDED:
            if accept_connection:
                return_msg = "Pairing with device succeeded"
                self._device._logger.info(return_msg)
            else:
                msg = "Pairing with device succeeded but it should not"
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    def try_until_paired(self, pairing_device_api, paired_device_api, pairing_device_addr, paired_device_addr, replyval, accept_connection):
        """
        The pairing_device is the one initiating the process
        The pairded_device is the one accepting the request or not
        """
        paired = False
        while(paired is False):
            time.sleep(self._wait_btwn_cmd)
            bond_state = paired_device_api.wait_for_pairing(pairing_device_addr, reconnect=1, replyval=replyval, timeout=30)
            pair_result = pairing_device_api.pair_to_device(paired_device_addr, reconnect=1, replyval=replyval, passkey=0, pincode="0000")
            if pair_result[0] == BT_BOND_STATE.BOND_NONE:
                paired = False
                if accept_connection:
                    msg = "Pairing with device failed"
                    self._device._logger.info(msg)
                else:
                    return_msg = "Pairing with device rejected"
                    self._device._logger.info(return_msg)

            if pair_result[0] == BT_BOND_STATE.BOND_BONDED:
                paired = True
                if accept_connection:
                    return_msg = "Pairing with device succeeded"
                    self._device._logger.info(return_msg)
                else:
                    msg = "Pairing with device succeeded but it should not"
                    self._device._logger.info(msg)

    def check_paired(self, pairing_device_api, paired_device_addr):
        paired = False

        list_paired_devices = pairing_device_api.list_paired_device()
        for element in list_paired_devices:
            if str(element.address).upper() == str(paired_device_addr).upper():
                paired = True
        if not paired:
            msg = "Device %s is not paired" % paired_device_addr
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    def wifi_connect(self, global_config, wifi_access_point, wifi_api):
        """
        Connect to the LIVE WiFi Access Point
        global_config is to be instantiated with self.__global_config when in an UseCase
        """
        # Connect to WiFi if required
        if not global_config.benchConfig.has_parameter(wifi_access_point):
            msg = "Wrong TC parameter WIFI_ACCESS_POINT:" + wifi_access_point
            self._device._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        wifi_router = global_config.benchConfig.get_parameters(wifi_access_point)
        ssid = wifi_router.get_param_value("SSID")
        security = wifi_router.get_param_value("WIFI_SECURITY").upper()
        passphrase = wifi_router.get_param_value("passphrase")
        # self._server_to_ping = wifi_router.get_param_value("IP")

        # WiFi connect
        if security in ("WEP", "WPA", "WPA2"):
            wifi_api.set_wificonfiguration(ssid, passphrase, security)
            time.sleep(self._wait_btwn_cmd)
        wifi_api.wifi_connect(ssid)
        wifi_api.wifi_connect(ssid)


    def disconnect_bt_tethering_and_pan(self, bt_tethering_deactivation_test, nap_api, who_disconnect, panu_api, nap_addr, panu_addr):
        """
        Disconnect the BT tethering link
        """
        # Handle "PANU got kicked" test
        if bt_tethering_deactivation_test:
            # Disable BT Tethering on NAP Device
            nap_api.set_bt_tethering_power("off")

            # Checks that the connection is broken
            # check_connection_lost(self._panu_net_api,
            #                       self._server_to_ping,
            #                       self._packet_size,
            #                       self._packet_count,
            #                       self._logger)

            # Enable BT Tethering on NAP Device
            nap_api.set_bt_tethering_power("on")
            time.sleep(self._wait_btwn_cmd)

        # Disconnect Bluetooth PAN profile
        disconnect_tethering(who_disconnect, nap_api, panu_api,
                             nap_addr, panu_addr)
        time.sleep(self._wait_btwn_cmd)

    def _raise_error_if_fail_when_profiles_not_connected(self, status, fail_if):
        """
        Raises an error if must fail
        """
        fail_if = str(fail_if).lower()
        if fail_if == Constants.PROFILE_CONNECTED and status == BtConState.d[BtConState.CONNECTED]:
            raise DeviceException(DeviceException.OPERATION_FAILED, "Profile connection state is CONNECTED")
        elif fail_if == Constants.PROFILE_DISCONNECTED and status == BtConState.d[BtConState.DISCONNECTED]:
            raise DeviceException(DeviceException.OPERATION_FAILED,"Profile connection state is DISCONNECTED")


    def _keep_going(self, status, start_time, timeout):
        """
        Decides whether to keep polling the device
        """
        keep_going = not status in [BtConState.d[BtConState.CONNECTED], BtConState.d[BtConState.DISCONNECTED]]
        if keep_going and time.time() - start_time > timeout:
            raise DeviceException(DeviceException.OPERATION_FAILED, "Expected status hasn't been reached before timeout")
        return keep_going

