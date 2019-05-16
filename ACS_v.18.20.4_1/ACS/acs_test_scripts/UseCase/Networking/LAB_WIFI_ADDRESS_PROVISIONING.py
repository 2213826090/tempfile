"""
:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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

:organization: INTEL MCG PSI
:summary: This file implements the Lab WIFI ADDRESS PROVISIONING UC
(Changing Wifi MAC address)
:since: 05/07/2012
:author: rneux,emarchan
"""
import time
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global, str_to_bool
import random
from string import hexdigits  # pylint: disable=W0402
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class LabWifiAddressProvisioning(UseCaseBase):

    """
    Lab Wifi MAC address provisioning test.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        UseCaseBase.__init__(self, tc_name, global_config)

        # Get MAC Address to use during the test
        self._expected_mac_address = str(self._tc_parameters.get_param_value("DUT_MAC_ADDRESS"))
        # Genuine MAC address
        self._start_mac_address = None
        self._is_random_mac = False

        self._isConnectionNeeded = str_to_bool(self._tc_parameters.get_param_value("WIFI_CNX_NEEDED"))

        if self._isConnectionNeeded:
            default_ap = "CONFIGURABLE_AP1"
            self._current_ap = str(self._tc_parameters.get_param_value(
                                   "CONFIGURABLE_AP_TO_USE",
                                   default_value=default_ap))
            if self._current_ap == "":
                self._current_ap = default_ap
            self._logger.info('Using access point "%s"' % self._current_ap)
            self._configurable_ap = global_config.benchConfig.\
                get_parameters(self._current_ap)

            self._ssid = self._configurable_ap.get_param_value("SSID")
            if self._configurable_ap.\
                    get_param_value("Model") == "AP_CONTROLLER" \
                    and self._configurable_ap.\
                    get_param_value("APC_IP", "") != "":
                self._wifirouter_ip = self._configurable_ap.\
                    get_param_value("APC_IP")
            else:
                self._wifirouter_ip = \
                    self._configurable_ap.get_param_value("IP")

            # Get WIFI Router configuration according WIFI SECURITY
            self._security = self._tc_parameters.get_param_value("WIFI_SECURITY")

            # initial wifi equipment
            self._ns = self._em.get_configurable_ap(self._current_ap)

            self._dut_wlan_iface = str(self._dut_config.get("wlanInterface"))
        self._phonesystem_api = None
        self._networking_api = None
        self._bt_api = None

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        UseCaseBase.set_up(self)

        # Get UECmdLayer
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")
        self._networking_api = self._device.get_uecmd("Networking")
        self._bt_api = self._device.get_uecmd("LocalConnectivity")

        # Power on the wifi to do the following operations
        self._networking_api.set_wifi_power("on")
        time.sleep(self._wait_btwn_cmd)


        # Store current MAC address to restore it at the end of the test.
        self._start_mac_address = self._networking_api.get_interface_mac_addr().upper()

        self._networking_api.set_wifi_power("off")
        time.sleep(self._wait_btwn_cmd)
        self._uecmd_default_timeout = int(self._device.get_uecmd_timeout())  # pylint: disable=W0201

        # Check and format mac_address
        if (self._expected_mac_address == "") or (self._expected_mac_address is None):
            new_msg = "Wrong MAC address %s" % self._expected_mac_address
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, new_msg)
        # Check if there's a random
        self._expected_mac_address = self._expected_mac_address.strip().upper().replace('-', ':')
        if 'X' in self._expected_mac_address:
            self._is_random_mac = True
            # Replace next x by a random char
            while 'X' in self._expected_mac_address:
                random_char = random.choice(hexdigits).upper()
                self._expected_mac_address = self._expected_mac_address.replace('X', random_char, 1)
        else:
            self._is_random_mac = False

        if self._isConnectionNeeded:
            # Configure the equipment
            self._ns.init()
            self._ns.set_wifi_config(self._ssid,
                                     hidden=False,
                                     standard_type='bg',
                                     authentication_type=self._security,
                                     passphrase='')
            self._ns.enable_wireless()
            # Close the connection
            self._ns.release()

        return Global.SUCCESS, "No error"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        UseCaseBase.run_test(self)

        # Set DUT MAC ADDRESS
        # If it's not random, we erase the MAC to ensure the new address is actually set.
        if not self._is_random_mac:
            self._set_mac_addr("08:00:28:00:00:00")
        self._set_mac_addr(self._expected_mac_address)

        # Get MAC Address from CLI
        self._networking_api.set_wifi_power("on")
        time.sleep(self._wait_btwn_cmd)
        dut_mac = self._networking_api.get_interface_mac_addr().upper()
        if self._expected_mac_address in dut_mac:
            return_msg = "MAC address OK."
        else:
            return_msg = "MAC address %s is not the expected one  %s" \
                % (dut_mac, self._expected_mac_address)
            self._logger.error(return_msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, return_msg)

        if self._isConnectionNeeded:
            for mac_address_blacklisted in [False, True, False]:
                # Connect to the AP to set the MAC ACL.
                self._ns.init()
                if mac_address_blacklisted:
                    self._ns.add_mac_address_to_acl(str(self._expected_mac_address))
                else:
                    self._ns.del_mac_address_from_acl(str(self._expected_mac_address))

                # Close the connection
                self._ns.release()

                # Configure the DUT

                # Disconnect from Wifi Network
                self._networking_api.wifi_disconnect_all()
                time.sleep(self._wait_btwn_cmd)

                # Set frequency band to "auto"
                self._networking_api.set_wifi_frequency_band("auto", True, self._dut_wlan_iface)
                time.sleep(self._wait_btwn_cmd)

                # Connect the DUT on the Wifi network
                self._networking_api.wifi_connect(self._ssid, False)
                connection_state = False

                start_time = time.time()
                while (not connection_state) and ((time.time() - start_time) < self._uecmd_default_timeout):
                    # List connected SSIDs to check if the right ssid is connected
                    connected_wifi_list = self._networking_api.list_connected_wifi()
                    state = self._networking_api.get_wifi_dhcp_state()
                    if (self._ssid in connected_wifi_list) and \
                            (state == "CONNECTED"):
                        connection_state = True
                        break
                if mac_address_blacklisted:
                    if not connection_state:
                        new_msg = "OK - Connection not established with blacklisted MAC address."
                        self._logger.info(new_msg)
                    else:
                        new_msg = "ERROR - Connection established with blacklisted MAC address."
                        self._logger.error(new_msg)
                        raise DeviceException(DeviceException.OPERATION_FAILED, new_msg)
                else:  # MAC is in white list
                    if connection_state:
                        new_msg = "OK - Connection established with white-listed MAC address."
                        self._logger.info(new_msg)
                    else:
                        new_msg = "ERROR - Connection not established with white-listed MAC address."
                        self._logger.error(new_msg)
                        raise DeviceException(DeviceException.OPERATION_FAILED, new_msg)

        return Global.SUCCESS, return_msg

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        UseCaseBase.tear_down(self)

        if self._isConnectionNeeded:
            # Clean AP's ACL list
            self._ns.init()
            time.sleep(self._wait_btwn_cmd)

            self._ns.del_mac_address_from_acl(str(self._expected_mac_address))
            time.sleep(self._wait_btwn_cmd)
            self._ns.disable_wireless()

            self._ns.release()

            # Clean DUT
            self._networking_api.wifi_remove_config(self._ssid)
            time.sleep(self._wait_btwn_cmd)

        self._networking_api.set_wifi_power("off")

        # Restore genuine mac_address.
        if self._start_mac_address:
            self._logger.info("Restoring genuine MAC address.")
            self._set_mac_addr(self._start_mac_address)
        else:
            self._logger.error("Genuine MAC address not stored. Can't restore.")

        # Disable radio for all Wifi APs
        self._disable_all_radios()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def _set_mac_addr(self, mac_address):
        self._bt_api.set_mac_addr('wifi', mac_address)
        # Reboot device to take MAC address into account.
        # reboot board
        self._device.reboot()

    def _disable_all_radios(self):
        """
        Disable radios of all Controllable APs
        """
        self._logger.info("Disable all radios")

        aps = self._em.get_all_configurable_aps()
        for ns in aps:
            if not ns.is_radio_off():
                ns.init()
                ns.disable_wireless()
                ns.release()
