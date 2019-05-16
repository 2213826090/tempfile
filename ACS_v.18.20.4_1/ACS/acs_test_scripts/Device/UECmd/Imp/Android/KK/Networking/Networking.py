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
:summary: This file implements Networking UECmds for Android JB device
:since: 09 dec 2013
:author: emarchan
"""
import time
import random
import re

from acs_test_scripts.Device.UECmd.Imp.Android.JB_MR2.Networking.Networking import \
                                                                        Networking as NetworkingJBMR2
from acs_test_scripts.Device.UECmd.Imp.Android.Common.Networking.Networking import \
                                                                        Networking as NetworkCommon
from acs_test_scripts.Device.UECmd.UECmdDecorator import need
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.Equipment.ConfigurableAP.Common.Common import WifiKeyExchangeTypes
from string import digits  # pylint: disable=W0402
import acs_test_scripts.Utilities.NetworkingUtilities as NetworkingUtil
from acs_test_scripts.Utilities.RegistrationUtilities import ImsRegistrationStatus, get_dict_key_from_value


class Networking(NetworkingJBMR2):

    """
    Class that handle all networking operations
    """

    def __init__(self, phone):
        """
        Constructor
        """
        NetworkingJBMR2.__init__(self, phone)

    @need('wifi')
    def set_wifi_frequency_band(self, freq_band, silent_mode=False, interface="wlan0"):
        """
        Set the Wifi Frequency Band

        :type freq_band: String
        :param freq_band: Frequency Band to set (auto, 2.4GHz, 5GHz)

        :type silent_mode: boolean
        :param silent_mode: if True, do not raise an exception
                            if the device does not support this method

        :type interface: str
        :param interface: interface name (wlan0/wlan1 etc...)
        """
        self._logger.info("Set Wifi frequency band to: %s" % str(freq_band))
        # Send the intent to set the Wifi Frequency Band selection
        if freq_band not in self.SUPPORTED_WIFI_BANDS:
            msg = "Invalid value for frequency band"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        cmd = "adb shell wpa_cli IFNAME=%s DRIVER SETBAND %s" % (interface, self.SUPPORTED_WIFI_BANDS[freq_band])
        result = self._exec(cmd, timeout=5, wait_for_response=True)
        if "OK" not in result:
            msg = "Error executing setband command"
            self._logger.error(msg)
            if not silent_mode:
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    @need('wifi')
    def get_wifi_frequency_band(self, silent_mode=False, interface="wlan0"):
        """
        Gets the band selection (bands of frequencies)
        0 means dual
        1 means 5Ghz
        2 means 2.4Ghz

        :type silent_mode: boolean
        :param silent_mode: if True, do not raise an exception if the device does not support this method
        :type interface: str
        :param interface: interface name (wlan0/wlan1 etc...)

        :rtype: String
        :return: The band text (JB or later).

        """
        cmd = "adb shell wpa_cli IFNAME=%s DRIVER GETBAND" % interface
        results = self._exec(cmd, timeout=5, wait_for_response=True)
        band_id = None
        for line in results.split("\n"):
            m = re.search(r'^Band ([0-2])', line)
            if m is not None:
                band_id = int(m.group(1))
                break

        match = [k for k, v in self.SUPPORTED_WIFI_BANDS.iteritems() if v == band_id]
        if match:
            result = match[0]
        else:
            result = None

        self._logger.debug("Wifi frequency band: %s" % str(result))
        return result

    @need('wifi')
    def set_regulatorydomain(self, regulatory_domain, interface="wlan0"):
        """
        Set the Wifi Regulatory Domain

        :type regulatory_domain: String
        :param regulatory_domain: the regulatory domain to set (FR, GB, US...)

        :type interface: str
        :param interface: interface name (wlan0/wlan1 etc...)
        """
        if regulatory_domain.lower() == "none":
            regulatory_domain = "00"

        # Force the root because sometimes it is lost and the set
        # regulatory domain command fails.
        self._exec("adb root", force_execution=True, timeout=10)
        time.sleep(2)

        wifi_chipset = self._get_wifi_chipset_manufacturer()

        if wifi_chipset == self.CHIPSET_BROADCOM:

            # Check Wifi chipset is ready to be set
            self.get_regulatorydomain()

            cmd = "adb shell wpa_cli IFNAME=%s driver country %s" % (interface, regulatory_domain)
            output = self._exec(cmd)
            if "failed" in output.lower():
                msg = "Unable to set regulatory domain - %s" % str(output)
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        else:
            NetworkingJBMR2.set_regulatorydomain(self, regulatory_domain, interface)

        # Store set value for later restoration.
        self.__last_set_reg_domain = regulatory_domain

        # Control the value set
        value_set = self.get_regulatorydomain()

        if value_set != regulatory_domain:
            msg = "Regulatory domain set fails. Try to set: %s. Read: %s" \
                % (regulatory_domain, value_set)
            self._logger.warning(msg)

    @need('wifi')
    def wifi_setkeyexchange(self, ssid, key_exchange_mode, key_exchange_pin=None,
                            simulate_faulty_connection=False, interface="wlan0"):
        """
        Sets key exchange mode (WPS for instance) for secured WIFI network.

        :type key_exchange_mode: str
        :param key_exchange_mode: Key exchange mode

        :type key_exchange_pin: str
        :param key_exchange_pin: PIN used by some key exchange modes (WPS_PIN_FROM_AP
        for instance)

        :type ssid: str
        :param ssid: WIFI network SSID

        :type simulate_faulty_connection: bool
        :param simulate_faulty_connection: If True, corrupts the data or wait for the
        AP's timeout to simulate a faulty simulation request

        :type interface: str
        :param interface: interface name (wlan0/wlan1 etc...)

        :return: None
        """
        wpa_cli_param = ""
        wpa_cli_pin = ""

        if (key_exchange_mode == WifiKeyExchangeTypes.WPS_PBC):
            wpa_cli_param = "wps_pbc"
        elif (key_exchange_mode == WifiKeyExchangeTypes.WPS_PIN_FROM_AP):
            wpa_cli_param = "wps_reg"
            wpa_cli_pin = str(key_exchange_pin)
        elif (key_exchange_mode == WifiKeyExchangeTypes.WPS_PIN_FROM_DUT):
            wpa_cli_param = "wps_pin"
            wpa_cli_pin = str(key_exchange_pin)
        else:
            output = "wifi_setkeyexchange: %s is not a good mode" % key_exchange_mode
            self._logger.error(output)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, output)

        bssid = self._get_bssid_from_ssid(ssid)

        # Corrupt the data to simulate a faulty connection request.
        if(simulate_faulty_connection):
            self._logger.info("Simulating faulty connection request...")

            if (key_exchange_mode == WifiKeyExchangeTypes.WPS_PBC):
                # Make the AP go to timeout
                ap_timeout = 120
                self._logger.info("Waiting for the AP timeout %ds" % (ap_timeout))
                time.sleep(ap_timeout)
            elif ((key_exchange_mode == WifiKeyExchangeTypes.WPS_PIN_FROM_AP) or
                  (key_exchange_mode == WifiKeyExchangeTypes.WPS_PIN_FROM_DUT)):
                # Corrupt the PIN code
                self._logger.info("Corrupting the PIN code")
                new_digit = "0"
                old_digit = wpa_cli_pin[0]
                while(new_digit == old_digit):
                    new_digit = random.choice(digits)
                wpa_cli_pin = wpa_cli_pin.replace(str(old_digit), str(new_digit))

        self._logger.info("Setting key exchange %s (pin: \"%s\")" % (key_exchange_mode, wpa_cli_pin))
        # Remove previous configuration
        cmd = "adb shell wpa_cli IFNAME=%s wps_cancel" % interface
        self._exec(cmd, timeout=5, wait_for_response=True)
        cmd = "adb shell wpa_cli IFNAME=%s remove_network %s" % (interface, ssid)
        self._exec(cmd, timeout=5, wait_for_response=True)

        # Start WPS on DUT.
        cmd = "adb shell wpa_cli IFNAME=%s %s %s %s"\
              % (interface, wpa_cli_param, bssid, wpa_cli_pin)
        self._logger.debug("Launching supplicant " + cmd)
        results = self._exec(cmd, timeout=5, wait_for_response=True)
        if (len(results.split()) < 3):
            raise DeviceException(DeviceException.OPERATION_FAILED, results)
        if (results.split()[3] not in ["OK", wpa_cli_pin]):
            raise DeviceException(DeviceException.OPERATION_FAILED, results)
        # Wait for the command to be taken into account into the DUT.
        time.sleep(15)

    @need('wifi')
    def _get_bssid_from_ssid(self, ssid, interface="wlan0"):
        """
        Get BSSID from SSID
        :type ssid: str
        :param ssid: The access point's SSID

        :rtype: str
        :return: The access point's mac address (BSSID)

        :type interface: str
        :param interface: interface name (wlan0/wlan1 etc...)

        """
        if (not ssid):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, ssid)

        cmd = "adb shell wpa_cli IFNAME=%s scan_result" % interface
        results = self._exec(cmd, timeout=5, wait_for_response=True)
        # Align all the carriage returns to \n to make it work on all OS and all adb connect method
        results = results.replace('\r', '\n')
        bssid = ''
        for line in results.split('\n'):
            if line.endswith(ssid):
                bssid = str(line).split()[0]
                break

        if(not NetworkingUtil.is_valid_mac_address(bssid)):
            msg = "Unable to get BSSID from SSID '%s' (result = %s)" % (ssid, bssid)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        return bssid

    @need('wifi', False)
    def load_wpa_certificate(self, certificate_name=None,
                             certificate_file=None,
                             eap_password=None,
                             credential_password=None):
        """
        Load the WPA certificate file from the SDCARD
        Prerequisite: A certificate file ".p12" should have been pushed into
        the folder /sdcard/.
        Warning, only 1 .p12 file must be present on the SDCARD
        Warning 2: if a credential password is set, it should be the PIN code
                    specified in the benchConfig (Credential_password)

        :type certificate_name: str
        :param certificate_name: Name to give to the certificate after loading

        :type certificate_file: str
        :param certificate_file: Name of the certificate file to load

        :type eap_password: str
        :param eap_password: password to open the certificate file

        :type credential_password: str
        :param credential_password: password to set for the Android credential
                                    passwords

        :return: None
        """

        KEYCODE_DPAD_UP = "19"
        KEYCODE_DPAD_DOWN = "20"
        KEYCODE_DPAD_RIGHT = "22"
        KEYCODE_DPAD_CENTER = "23"
        KEYCODE_BACK = "4"
        KEYCODE_MOVE_HOME = "122"
        KEYCODE_MOVE_END = "123"
        KEYCODE_TAB = "61"
        KEYCODE_ENTER = "66"
        display = self._device.get_uecmd("Display")

        """
        On ICS, credential password is linked to screen phone lock password.
        Which was not the case on GingerBread.
        This make the UI script for loading the certificate much more complicate.
        This UI script should work in every circomstances:
        - With a certificate already loaded (so with a pin code set - we assume
         that this PIN code is the credential password mentionned in BencConfig)
        - With no certificate loaded and no pin code set to lock screen phone.
        - With no certificate but with PIN code to lock screen phone (We assume
         that this PIN code is the credential password mentionned in BencConfig)
        """

        # Unlock the device
        self._phone_system.set_phone_screen_lock_on(1)
        self._phone_system.set_phone_lock(0)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_RIGHT)

        # Force the screen orientation to portrait
        display.set_display_orientation("portrait")

        # Step 1: Remove potential existing certificate
        self._exec(
            "adb shell am start -n com.android.settings/.SecuritySettings")
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_RIGHT)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_DOWN)
        self._exec("adb shell input keyevent " + KEYCODE_MOVE_END)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_CENTER)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_RIGHT)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_CENTER)
        self._exec("adb shell input keyevent " + KEYCODE_BACK)
        self._exec("adb shell input keyevent " + KEYCODE_BACK)
        self._phone_system.set_phone_lock(1)

        # Step 2: Reset current screen phone lock PIN code if exists
        self._phone_system.set_phone_lock(0)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_RIGHT)
        self._exec(
            "adb shell am start -n com.android.settings/.SecuritySettings")
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_RIGHT)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_DOWN)
        self._exec("adb shell input keyevent " + KEYCODE_MOVE_HOME)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_CENTER)
        self._exec("adb shell input text " + credential_password)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_DOWN)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_CENTER)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_RIGHT)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_DOWN)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_CENTER)
        self._exec("adb shell input keyevent " + KEYCODE_BACK)
        self._exec("adb shell input keyevent " + KEYCODE_BACK)
        self._phone_system.set_phone_lock(1)

        # Step 3: Set credential password as PIN screen lock code
        self._phone_system.set_phone_lock(0)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_RIGHT)
        self._exec(
            "adb shell am start -n com.android.settings/.SecuritySettings")
        self._logger.info("WPA load certificate: set credential password (%s)"
                          % credential_password)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_RIGHT)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_DOWN)
        self._exec("adb shell input keyevent " + KEYCODE_MOVE_HOME)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_CENTER)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_RIGHT)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_DOWN)
        self._exec("adb shell input keyevent " + KEYCODE_MOVE_END)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_UP)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_CENTER)
        self._exec("adb shell input text " + credential_password)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_DOWN)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_CENTER)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_UP)
        self._exec("adb shell input text " + credential_password)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_DOWN)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_CENTER)
        self._exec("adb shell input keyevent " + KEYCODE_BACK)
        self._phone_system.set_phone_lock(1)

        # Step 4: Load the certificate
        self._phone_system.set_phone_lock(0)
        self._exec("adb shell input keyevent " + KEYCODE_BACK)
        self._exec("adb shell input keyevent " + KEYCODE_BACK)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_RIGHT)
        self._exec("adb shell am force-stop com.android.settings")
        time.sleep(2)
        self._exec("adb shell am start -n com.android.settings/.SecuritySettings")
        time.sleep(1)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_RIGHT)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_DOWN)
        self._exec("adb shell input keyevent " + KEYCODE_MOVE_END)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_UP)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_CENTER)
        self._logger.info("WPA load certificate: type certif password (%s)"
                          % eap_password)
        # Go to the menu
        time.sleep(3)
        self._exec("adb shell input keyevent " + KEYCODE_TAB)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_DOWN)
        self._exec("adb shell input keyevent " + KEYCODE_MOVE_END)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_CENTER)
        time.sleep(2)
        self._exec("adb shell input keyevent " + KEYCODE_TAB)
        self._exec("adb shell input keyevent " + KEYCODE_TAB)
        self._exec("adb shell input keyevent " + KEYCODE_TAB)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_DOWN)
        self._exec("adb shell input keyevent " + KEYCODE_MOVE_END)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_UP)
        self._exec("adb shell input keyevent " + KEYCODE_MOVE_END)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_CENTER)
        # Configure and install the certificate
        time.sleep(2)
        self._exec("adb shell input text " + eap_password)
        self._exec("adb shell input keyevent " + KEYCODE_TAB)
        self._exec("adb shell input keyevent " + KEYCODE_TAB)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_CENTER)
        time.sleep(2)
        self._exec("adb shell input text " + certificate_name)
        self._exec("adb shell input keyevent " + KEYCODE_TAB)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_CENTER)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_DOWN)
        self._exec("adb shell input keyevent " + KEYCODE_ENTER)
        self._exec("adb shell input keyevent " + KEYCODE_TAB)
        self._exec("adb shell input keyevent " + KEYCODE_TAB)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_CENTER)

        # Go back to home screen
        self._exec("adb shell input keyevent " + KEYCODE_BACK)
        self._exec("adb shell input keyevent " + KEYCODE_BACK)
        self._exec("adb shell input keyevent " + KEYCODE_BACK)

        # Re-allow phone locking
        self._phone_system.set_phone_lock(1)
        self._phone_system.set_phone_screen_lock_on(0)
        display.set_display_orientation("auto")

    @need('modem')
    def get_preferred_network_type(self):
        """
        Returns the Preferred Network Type.

        :rtype: str
        :return:
            "2G_ONLY"       for # 1: "GSM_ONLY",
            "3G_PREF"       for # 0: "WCDMA_PREF",
            "4G_PREF"       for # 9: "LTE_GSM_WCDMA",
            "3G_ONLY"       for # 2: "WCDMA_ONLY",
            "2G_3G"         for # 3: "GSM_UMTS",
            "CDMA_PREF"     for # 4: "CDMA",
            "CDMA_ONLY"     for # 5: "CDMA_NO_EVDO"
            "EVDO_ONLY"     for # 6: "EVDO_NO_CDMA"
            "GLOBAL"        for # 7: "GLOBAL",
            "4G_PREF_US"    for # 8: "LTE_CDMA_EVDO",
            "WORLD_MODE"    for # 10: "LTE_CMDA_EVDO_GSM_WCDMA"
            "4G_ONLY"       for # 11: "LTE_ONLY"
        """
        # Send UE command
        method = "getPreferredNetworkType"
        net_type = self._internal_exec_v2(self.network_type_module, method, is_system=True)
        if net_type is None:
            error_msg = "getPreferredNetworkType : Parameter network value is not valid"
            self._logger.error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        modem_net_type_value = int(net_type.get("networkTypeValue", None))
        modem_net_type_name = net_type.get("networkTypeName", None)
        msg = \
            "Actual Modem preferred network type is %s and it's value is %s  " % (modem_net_type_name, str(modem_net_type_value))
        self._logger.info(msg)


        settings_net_type_value = self._get_preferred_network_db_value()
        settings_net_type_name = NetworkCommon.NETWORK_TYPE_CONVERSION_TABLE[int(settings_net_type_value)]
        msg = \
            "Actual Settings preferred network type is %s and it's value is %s  " % (settings_net_type_name, str(settings_net_type_value))
        self._logger.info(msg)

        # if modem and settings network values are different, update settings database
        if modem_net_type_value != settings_net_type_value:
            warning_msg = \
                "getPreferredNetworkType : Settings network value %s and modem network value %s are not the same!" \
                % (str(settings_net_type_value), str(modem_net_type_value))
            self._logger.warning(warning_msg)

            # Update setting database to preferred network type we get from java
            self._set_preferred_network_db_value(modem_net_type_value)
            # check value is properly set
            settings_net_type_value = self._get_preferred_network_db_value()
            # return the modem value
            settings_net_type_name = NetworkCommon.NETWORK_TYPE_CONVERSION_TABLE[int(settings_net_type_value)]

        # finally return modem network value
        return settings_net_type_name

    @need('modem')
    def set_preferred_network_type(self, preferred_type):
        """
        Sets the Preferred Network Type .

        :type preferred_type: str
        :param preferred_type: can be:
            "2G_ONLY"       for # 1: "GSM_ONLY",
            "3G_PREF"       for # 0: "WCDMA_PREF",
            "4G_PREF"       for # 9: "LTE_GSM_WCDMA",
            "3G_ONLY"       for # 2: "WCDMA_ONLY",
            "2G_3G"         for # 3: "GSM_UMTS",
            "CDMA_PREF"     for # 4: "CDMA",
            "CDMA_ONLY"     for # 5: "CDMA_NO_EVDO"
            "EVDO_ONLY"     for # 6: "EVDO_NO_CDMA"
            "GLOBAL"        for # 7: "GLOBAL",
            "4G_PREF_US"    for # 8: "LTE_CDMA_EVDO",
            "WORLD_MODE"    for # 10: "LTE_CMDA_EVDO_GSM_WCDMA"
            "4G_ONLY"       for # 11: "LTE_ONLY"

        :return: None
        """
        self._logger.info(
            "Setting preferred network type on dut ...")
        current_type = self.get_preferred_network_type()
        preferred_type_index = get_dict_key_from_value(NetworkCommon.NETWORK_TYPE_CONVERSION_TABLE, preferred_type)

        if current_type == preferred_type:
            warning_msg = "the preferred network %s (%s) type is already enabled" % (preferred_type_index, str(preferred_type))
            self._logger.info(warning_msg)
        else:
            info_msg = "Setting the wanted preferred network type: %s (%s) " % (preferred_type_index, str(preferred_type))
            self._logger.info(info_msg)

            # Send UE command
            method = "setPreferredNetworkType"
            cmd = " --ei networkType %s" \
                  % str(preferred_type_index)
            # As set in RIL config, timeout is 300 s for SetPreferredNetworkType RIL Request.
            self._internal_exec_v2(self.network_type_module, method, cmd, 300, is_system=True)

            # Update setting database
            self._logger.info("Update preferred network mode (%d) to database ..." % int(preferred_type_index))
            self._set_preferred_network_db_value(preferred_type_index)


    def _set_preferred_network_db_value(self, preferred_type_index):
        preferred_type_index = int(preferred_type_index)
        self._logger.info("Update preferred network mode (%d) to database ..." % preferred_type_index)

        # if the xml file exists, we update it and exit
        xmlglobal = "/data/system/users/0/settings_global.xml"
        if self.update_xml_setting(xmlglobal, "preferred_network_mode", preferred_type_index, "setting"):
            return

        adb_sqlite3 = "adb shell sqlite3"
        db = "/data/data/com.android.providers.settings/databases/settings.db"
        request = '"update global set value=\'%d\' where name=\'preferred_network_mode\';"' % preferred_type_index
        cmd = adb_sqlite3 + ' ' + db + ' ' + request
        output = self._exec(cmd)
        output = str(output).strip()
        self._logger.debug("output: (%s)" % str(output))

    def _get_preferred_network_db_value(self):
        # adb shell sqlite3 /data/data/com.android.providers.settings/databases/settings.db
        # "select value from global where name='preferred_network_mode';"
        self._logger.debug("Request preferred network mode to database ...")

        # if the xml file exists, read from it
        xmlglobal = "/data/system/users/0/settings_global.xml"
        output = self.read_xml_setting(xmlglobal, "preferred_network_mode")

        # else read from database
        if not output:
            adb_sqlite3 = "adb shell sqlite3"
            db = "/data/data/com.android.providers.settings/databases/settings.db"
            request = '"select value from global where name=\'preferred_network_mode\';"'
            cmd = adb_sqlite3 + ' ' + db + ' ' + request
            output = str(self._exec(cmd)).strip()
        self._logger.debug("output: (%s)" % output)
        if not output.isdigit():
            error_msg = \
                "getPreferredNetworkType : Parameter network value is not valid"
            self._logger.error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)
        settings_net_type_value = int(output)
        return settings_net_type_value

    @need("wifi or modem_LTE")
    def get_ims_registration_status(self):
        """
        Returns an integer description the IMS registration
        state of the device.
        :return: an integer describing the IMS registration status
            (-1 if a non-integer value has been returned by the API).
        :rtype: int
        """
        method = "getImsRegStatus"
        output = self._internal_exec_v2(self._cellular_networking_module, method, is_system=True)
        ims_registration_status_str = output["ims_registration_status"]
        ims_registration_status = -1
        if ims_registration_status_str and ims_registration_status_str.isdigit():
            ims_registration_status = int(ims_registration_status_str)
        return ims_registration_status

    def set_wifi_hotspot(self, state, hotspot_ssid="", hotspot_security="", hotspot_passphrase="", hotspot_standard="",
                         hotspot_channel="AUTO", hotspot_hidden="false"):
        """
        Set the SSID, security and password for the Configurable wifi hotspot.

        :type state: str
        :param state: wifi hotspot enable or disable: on|off
        :type hotspot_ssid: str
        :param hotspot_ssid: ssid of hotspot (required only if state=on)
        :type hotspot_security: str
        :param hotspot_security: security of hotspot, can be
                         OPEN|WPA-PSK|WPA2-PSK (required only if state=on)
        :type hotspot_passphrase: str
        :param hotspot_passphrase: password of hotspot,
                         (required only if state=on and hotspot_security!=OPEN)
        :type hotspot_standard: str
        :param hotspot_standard: standard of hotspot, optional, can be (2_4GHZ_20MHZ;5GHZ_20MHZ;5GHZ_40MHZ;5GHZ_80MHZ)
        :type hotspot_channel: str
        :param hotspot_channel: channel of hotspot, value "AUTO" by default (only if standard is used, optional)
        :type hotspot_hidden: str
        :param hotspot_hidden: hidden SSID
        """
        self._logger.info("Trying to set wifi hotspot")
        final_channel = None
        final_security = None
        state = str(state).lower()
        if state not in ("on", "off"):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "'state' Parameter must be 'on' or 'off'.")
        if state == "on":
            if hotspot_ssid == "" or hotspot_security == "":
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                         "'hotspot_ssid' and 'hotspot_security' Parameters must be defined")
            if hotspot_security != "OPEN" and hotspot_passphrase == "":
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                         "'hotspot_passphrase' Parameter must be defined")
            if hotspot_standard != "" and hotspot_standard not in ["2_4GHZ_20MHZ", "5GHZ_20MHZ", "5GHZ_40MHZ",
                                                                   "5GHZ_80MHZ"]:
                msg = "Parameter hotspot standard invalid : %s" % hotspot_standard
                self._logger.error(msg)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

            # Check channel value is valid with standard
            if hotspot_standard != "" and hotspot_channel != "AUTO":
                if hotspot_standard == "2_4GHZ_20MHZ" and hotspot_channel not in ["1", "2", "3", "4", "5", "6", "7",
                                                                                  "8", "9", "10", "11", "12", "13",
                                                                                  "14"]:
                    msg = "Parameter hotspot channel invalid %s - Standard is %s" % (hotspot_channel, hotspot_standard)
                    self._logger.error(msg)
                    raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
                elif hotspot_standard in ["5GHZ_20MHZ", "5GHZ_40MHZ", "5GHZ_80MHZ"] and hotspot_channel not in ["36",
                                                                                                                "40",
                                                                                                                "44",
                                                                                                                "48"]:
                    msg = "Parameter hotspot channel invalid %s - Standard is %s" % (hotspot_channel, hotspot_standard)
                    self._logger.error(msg)
                    raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

            if hotspot_security == "OPEN":
                # Set variable to be used for Embd
                final_security = "None"
            else:
                final_security = hotspot_security

            if hotspot_channel == "AUTO":
                if hotspot_standard == "2_4GHZ_20MHZ":
                    final_channel = str(random.randrange(1, 11))
                else:
                    final_channel = "36"
            else:
                final_channel = hotspot_channel

        method = "setWifiHotspot"
        cmd_args = "--es ssid '%s' --es security '%s'  --es passphrase '%s' --es switch '%s' " % (
            hotspot_ssid, final_security, hotspot_passphrase, state)
        cmd_args = "%s --es standard '%s' --es channel '%s' --es hidden '%s'" % (
            cmd_args, hotspot_standard, final_channel, hotspot_hidden)
        self._internal_exec_v2(self._tethering_module, method, cmd_args, is_system=True)

    def get_wifi_hotspot_parameters(self):
        """
        Get all parameters of the Wifi hotspot feature.

        :type state: str
        :param state: wifi hotspot parameter, can be SSID;SECURITY;PASSPHRASE;STANDARD;CHANNEL;HIDDEN
        :rtype: dict
        :return: all currents parameters : SSID, SECURITY, PASSPHRASE, STANDARD, CHANNEL, HIDDEN
        """
        method = "getWifiHotspotParameters"
        output = self._internal_exec_v2(self._tethering_module, method, is_system=True)

        hotspot_parameters = {}
        hotspot_parameters['SSID'] = str(output["wifi_hotspot_ssid"])
        hotspot_parameters['SECURITY'] = str(output["wifi_hotspot_security"])
        hotspot_parameters['PASSPHRASE'] = str(output["wifi_hotspot_passphrase"])

        if output.has_key("wifi_hotspot_standard"):
            hotspot_parameters['STANDARD'] = str(output["wifi_hotspot_standard"])
        else:
            hotspot_parameters['STANDARD'] = "AUTO"
        if output.has_key("wifi_hotspot_channel"):
            hotspot_parameters['CHANNEL'] = str(output["wifi_hotspot_channel"])
        else:
            hotspot_parameters['CHANNEL'] = "AUTO"

        if output["wifi_hotspot_hidden"] == "true":
            hotspot_parameters['HIDDEN'] = "on"
        else:
            hotspot_parameters['HIDDEN'] = "off"

        return hotspot_parameters

    @need("wifi or modem_LTE")
    def answerIMS(self):
        """
        Answers all voice calls.

        :return: None
        """
        self._logger.info("Answering incoming call...")
        self._exec("adb shell input swipe 350 900 600 900 500")

        self.wait_for_state(self._vc_state.get("active"), self._call_setup_timeout)

    def check_ims_registration_before_timeout(self, timeout=30):
        """
        Waits a maximum of the given timeout (in seconds) for IMS registration.

        If the IMS registration does not occur before the timeout, a
        DeviceException is raised.
        """
        start = time.time()
        ims_registered_state = ImsRegistrationStatus.in_service()
        current_status = None
        registration_time = 0
        while time.time() - start < timeout:
            time.sleep(1)
            current_status = ImsRegistrationStatus(
                self.get_ims_registration_status())
            if current_status == ims_registered_state:
                registration_time = time.time()
                break
        if current_status is None or current_status != ims_registered_state:
            return_msg = "The device could not register to IMS before " \
                "%d seconds." % timeout
            raise DeviceException(DeviceException.TIMEOUT_REACHED, return_msg)
        else:
            # Update the time needed for registration
            registration_time -= start
            # Log the computed time
            self._logger.info("IMS registration complete after %ds"
                % int(registration_time))

    def set_wifi_scan_always_available(self, mode):
        """
        Set the WiFi Scan always available option in WiFi settings menu.
        WARNING : this function use UI !

        :type mode: str
        :param mode: mode to set. Possible values : "ON", "OFF"
        """
        KEYCODE_DPAD_DOWN = "20"
        KEYCODE_MOVE_HOME = "122"
        KEYCODE_ENTER = "66"

        if mode not in ["ON", "OFF"]:
            msg = "Invalid parameter mode : %s" % mode
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        if self.get_wifi_scan_always_available() == mode:
            self._logger.debug("WiFi Scan Always Available is already %s" % mode)
            return

        # Go to WiFi Advanced Settings Menu
        self.wifi_menu_advanced_settings()

        # Switch WiFi Scan always available option
        self._exec("adb shell input keyevent " + KEYCODE_MOVE_HOME)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_DOWN)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_DOWN)
        self._exec("adb shell input keyevent " + KEYCODE_ENTER)

    def get_wifi_scan_always_available(self):
        """
        Get the WiFi Scan always available option in WiFi settings menu.

        :rtype: str
        :return: WiFi Scan always available option state. Possible values are "ON" or "OFF".
        """
        method = "getWifiScanAlwaysAvailable"
        output = self._internal_exec_v2(self._wifi_module, method, is_system=True)
        self._logger.info("Wifi Scan always available status: " + output["wifi_scan_always_available_state"])

        if str(output["wifi_scan_always_available_state"]).lower() == "false":
            return "OFF"
        elif str(output["wifi_scan_always_available_state"]).lower() == "true":
            return "ON"
        else:
            msg = "ERROR - Invalid getWifiScanAlwaysAvailable return : %s" % str(output["wifi_scan_always_available_state"])
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    @need('wifi', False)
    def remove_wpa_certificates(self, credential_password=None, pin_code_removal=False):
        """
        Remove wpa certificates already installed on device.
        Possibility to remove pin code already set.

        :type pin_code_removal: boolean
        :param pin_code_removal: Reset Pin Code to None

        :return: None
        """
        KEYCODE_DPAD_DOWN = "20"
        KEYCODE_DPAD_RIGHT = "22"
        KEYCODE_DPAD_CENTER = "23"
        KEYCODE_BACK = "4"
        KEYCODE_MOVE_HOME = "122"
        KEYCODE_MOVE_END = "123"
        display = self._device.get_uecmd("Display")

        # Unlock the device
        self._phone_system.set_phone_screen_lock_on(1)
        self._phone_system.set_phone_lock(0)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_RIGHT)

        # Force the screen orientation to portrait
        display.set_display_orientation("portrait")

        # Step 1: Remove potential existing certificate
        self._exec(
            "adb shell am start -n com.android.settings/.SecuritySettings")
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_RIGHT)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_DOWN)
        self._exec("adb shell input keyevent " + KEYCODE_MOVE_END)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_CENTER)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_RIGHT)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_CENTER)
        self._exec("adb shell input keyevent " + KEYCODE_BACK)
        self._exec("adb shell input keyevent " + KEYCODE_BACK)
        self._phone_system.set_phone_lock(1)

        if pin_code_removal:
            # Remove Pin CODE
            msg = "Removing PIN CODE on Device"
            self._logger.info(msg)
            # Step 2: Reset current screen phone lock PIN code if exists
            self._phone_system.set_phone_lock(0)
            self._exec("adb shell input keyevent " + KEYCODE_DPAD_RIGHT)
            self._exec(
                "adb shell am start -n com.android.settings/.SecuritySettings")
            self._exec("adb shell input keyevent " + KEYCODE_DPAD_RIGHT)
            self._exec("adb shell input keyevent " + KEYCODE_DPAD_DOWN)
            self._exec("adb shell input keyevent " + KEYCODE_MOVE_HOME)
            self._exec("adb shell input keyevent " + KEYCODE_DPAD_CENTER)
            self._exec("adb shell input text " + credential_password)
            self._exec("adb shell input keyevent " + KEYCODE_DPAD_DOWN)
            self._exec("adb shell input keyevent " + KEYCODE_DPAD_CENTER)
            self._exec("adb shell input keyevent " + KEYCODE_DPAD_RIGHT)
            self._exec("adb shell input keyevent " + KEYCODE_DPAD_DOWN)
            self._exec("adb shell input keyevent " + KEYCODE_DPAD_CENTER)
            self._exec("adb shell input keyevent " + KEYCODE_BACK)
            self._exec("adb shell input keyevent " + KEYCODE_BACK)
            self._phone_system.set_phone_lock(1)

        # Go back to home screen
        self._exec("adb shell input keyevent " + KEYCODE_BACK)
        self._exec("adb shell input keyevent " + KEYCODE_BACK)
        self._exec("adb shell input keyevent " + KEYCODE_BACK)

        # Re-allow phone locking
        self._phone_system.set_phone_lock(1)
        self._phone_system.set_phone_screen_lock_on(0)
        display.set_display_orientation("auto")
