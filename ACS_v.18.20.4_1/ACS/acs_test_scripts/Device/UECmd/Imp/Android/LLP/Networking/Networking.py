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
import os
import tempfile

from acs_test_scripts.Utilities.RegistrationUtilities import get_dict_key_from_value
from acs_test_scripts.Device.UECmd.UECmdDecorator import need
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.Device.UECmd.Imp.Android.KK.Networking.Networking import Networking as NetworkingKK


class Networking(NetworkingKK):

    """
    Class that handle all networking operations
    """

    def __init__(self, phone):
        """
        Constructor
        """
        NetworkingKK.__init__(self, phone)
        self._device_serial_number = phone.retrieve_serial_number()
        self.dut = None

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
        # uiautomator is imported here to avoid crashes when uiautomator is not installed on the computer
        # uiautomator is not in the ACS installer for the moment
        # Should be removed when uiautomator will be installed by ACS installer
        from uiautomator import Device
        if self.dut is None:
            self.dut = Device(self._device_serial_number)

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

        # Force the screen orientation to portrait
        display.set_display_orientation("portrait")

        # Step 1: Remove potential existing certificate
        self._exec(
            "adb shell am start -n com.android.settings/.SecuritySettings")
        time.sleep(1)

        self.dut(scrollable=True).scroll.to(text="Clear credentials")
        if self.dut(text="Clear credentials").enabled:
            msg = "Removing old Certificate"
            self._logger.info(msg)
            self.dut(text="Clear credentials").click()
            self.dut(text="OK").click()

        self.dut(scrollable=True).scroll.to(text="Screen lock")

        if self.dut.exists(text="PIN"):
            msg = "PIN CODE already set"
            self._logger.info(msg)

        else:
            # set PIN CODE
            self.dut(text="Screen lock").click()
            self.dut(text="PIN").click()
            if self.dut(text="No thanks").exists:
                # since LLP, Pin code can be required for starting up
                self.dut(text="No thanks").click()
                self.dut(text="Continue").click()
            # setting Pin code
            self.dut(text="Choose your PIN").set_text(credential_password)
            self.dut(text="Continue").click()
            self.dut(text="Choose your PIN").set_text(credential_password)
            self.dut(text="OK").click()
            if self.dut(text="Don't show notifications at all").exists:
                self.dut(text="Don't show notifications at all").click()
                self.dut(text="Done").click()
            self.dut.press.back()
            self.dut.press.back()

        self._phone_system.set_phone_lock(1)
        time.sleep(1)
        # Load the certificate
        self._phone_system.set_phone_lock(0)

        self._exec("adb shell am force-stop com.android.settings")
        time.sleep(3)
        self._exec("adb shell am start -n com.android.settings/.SecuritySettings")
        time.sleep(2)

        self.dut(scrollable=True).scroll.to(text="Clear credentials")

        # Specific to platforms ( internal storage or sd card need to be chosen)
        if self.dut(text="Install from storage").exists:
            self.dut(text="Install from storage").click()
        elif self.dut(text="Install from SD card").exists:
            self.dut(text="Install from SD card").click()
        else:
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                      "install from storage or from sd card is not available, possible api changes")
        time.sleep(2)

        # after first certificate installation this window disappear
        if self.dut(text="Internal storage").exists:
            self.dut(text="Internal storage").click()

        if self.dut(text=certificate_file).exists:
            self.dut(text=certificate_file).click()

        else:
            try:
                # try to scroll to search for certificate file
                self._logger.info("the certificat %s does not exisit on display trying to scroll" % certificate_file)
                self.dut(scrollable=True).scroll.to(text=certificate_file)
                self.dut(text=certificate_file).click()
            except Exception as error:
                # raise execption for cetifcate file not found
                raise DeviceException(DeviceException.OPERATION_FAILED,
                                      "Error %s : Certificate file %s not found on DUT "
                                      % (str(error), certificate_file))
        time.sleep(2)
        self.dut(text="Extract certificate").set_text(eap_password)
        self.dut(text="OK").click()
        self.dut(text="Certificate name:").set_text(certificate_name)
        if self.dut(text="VPN and apps").exists:
            self.dut(text="VPN and apps").click()
            self.dut(text="Wi-Fi").click()
            self.dut(text="OK").click()
            if self.dut(text="Confirm your PIN").exists:
                # sometimes need to confirm PIN CODE
                self.dut(text="Confirm your PIN").set_text(credential_password)
                if self.dut(text="Next").exists:
                    self.dut(text="Next").click()
                elif self.dut(text="Continue").exists:
                    self.dut(text="Continue").click()
                else:
                    raise DeviceException(DeviceException.OPERATION_FAILED,
                                          "Unable to confirm PIN CODE , check if display has been modified ")

        elif self.dut(text="Wi-Fi").exists:
                self.dut(text="OK").click()

        else:
            msg = "Credential use not supported"
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                      "Credential use not supported ")

        self.dut(scrollable=True).scroll.to(text="Clear credentials")
        if self.dut(text="Clear credentials").enabled:
            msg = "Wifi Certifiacte is successfully installed"
            self._logger.info(msg)
        else:
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                      "Wifi Certifiacte is not successfully installed")

        # Re-allow phone locking
        self._phone_system.set_phone_lock(1)
        self._phone_system.set_phone_screen_lock_on(0)
        display.set_display_orientation("auto")

    def open_web_browser(self, website_url, browser_type="chrome", timeout=None, skip_eula=False):
        """
        Open the Android Browser on the web page
        passed as parameter.

        :type website_url: str
        :param website_url: URL to open

        :type browser_type: str
        :param browser_type: "native" will open the default browser,
                             "acs_agent" will use the browser of the acs agent

        :type timeout: int
        :param timeout: timeout to open the page

        :type skip_eula: boolean
        :param skip_eula: skip EULA on 1st start

        :rtype: tuple
        :return: operation status & output log
        """
        if browser_type == "chrome" or browser_type == "acs_agent":
            error_code, error_msg = NetworkingKK.open_web_browser(self, website_url, browser_type, timeout, skip_eula)
            return error_code, error_msg
        else:
            error_msg = "Unsupported browser type : %s" % str(browser_type)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

    def close_web_browser(self, browser_type="chrome"):
        """
        Close the Android Browser.

        :type browser_type: str
        :param browser_type: "native" will open the default browser,
            other type can be added depending on the os

        :return: None
        """
        self._logger.info("Closing the %s web browser" % str(browser_type))
        if browser_type == "chrome" or browser_type == "acs_agent":
            NetworkingKK.close_web_browser(self, browser_type)
        else:
            error_msg = "Unsupported browser type : %s" % str(browser_type)
            raise AcsConfigException(
                            AcsConfigException.INVALID_PARAMETER, error_msg)

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
        self._exec("adb shell input keyevent " + KEYCODE_ENTER)

    def delete_apn(self, apn_name):
        """
        Deletes the APN with the given name.

        :type apn_name: str
        :param apn_name: the name of the APN to delete

        :rtype: None
        """
        # Initialize some local variables
        method = "deleteApn"
        args = "--es name \"%s\"" % apn_name
        # Execute the command
        self._internal_exec_v2(self._cellular_networking_module, method, args, is_system=True)

    def update_apn(self, new_parameters):
        """
        Updates the APN configuration with the given new parameters.

        :type new_parameters: dict
        :param new_parameters: the key/value mapping of the parameters
            to update

        :rtype: None
        """
        # Initialize some local variables
        method = "updateApnWithParameters"
        args = ""
        # Update the command arguments
        for parameter_name in new_parameters:
            value = new_parameters[parameter_name]
            args = "%s %s \"%s\" \"%s\"" % (args, "--es", parameter_name, value)
        # Execute the command
        self._internal_exec_v2(self._cellular_networking_module, method, args, is_system=True)

    def wifi_remove_config(self, ssid):
        """
        Remove a wifi configuration for the device (screen will be awaken during the process).

        :type ssid: str
        :param ssid: the ssid of the wifi configuration to be removed or "all"
        """
        # retrieve the initial wifi power status
        wifi_initial_status = self.get_wifi_power_status()

        # Enable wifi if necessary, to remove a known wifi network
        if wifi_initial_status == 0:
            self._logger.info("In order to remove remembered wifi "
                              + "networks, we must power on the wifi")
            self.set_wifi_power("on")

        # Wake up screen
        self._phone_system_api = self._device.get_uecmd("PhoneSystem")
        self._phone_system_api.wake_screen()

        # Remove the requested SSID
        self._logger.info("Trying to remove wifi config [%s]" % ssid)
        method = "removeWifiConfig"
        cmd_args = "--es ssid %s" % ssid
        self._internal_exec_v2(self._wifi_module, method, cmd_args, is_system=True)

        # reset wifi state to its original state
        if wifi_initial_status == 0:
            self._logger.info("Set the wifi power to its original state")
            self.set_wifi_power("off")

    def usb_tether(self, wifi_off, unplug_usb, use_flight_mode):
        """
        push and run a script that will execute the test in adb disconnected mode

        :type wifi_off: int
        :param wifi_off: 1 to turn wifi off during usb tethering

        :type unplug_usb: int
        :param unplug_usb: 1 to unplug usb during usb tethering

        :type use_flight_mode: int
        :param use_flight_mode: 1 to turn on flight mode during usb tethering

        :return: None
        """
        script_name = "/data/usb_tether.sh"
        script_output = "/data/usb_tether.log"

        if wifi_off:
            # case 3: activate tethering, ping AP, turn wifi off
            # ping AP (fail) turn wifi on, ping AP, deactivate
            # tethering
            script_data = \
                """#!/system/bin/sh
sleep 10
am broadcast -a intel.intent.action.acs.cmd.system --es class %s --es method setUsbTethering --es switch on -e opcode %s
sleep 60
am broadcast -a intel.intent.action.acs.cmd.system --es class %s --es method setWifiPower --ei mode 0 -e opcode %s
sleep 30
am broadcast -a intel.intent.action.acs.cmd.system --es class %s --es method setWifiPower --ei mode 1 -e opcode %s
sleep 30
am broadcast -a intel.intent.action.acs.cmd.system --es class %s --es method setUsbTethering --es switch off -e opcode %s
sleep 30
""" \
                % (self._tethering_module, self._generate_key(),
                   self._wifi_module, self._generate_key(),
                   self._wifi_module, self._generate_key(),
                   self._tethering_module, self._generate_key())
        elif unplug_usb:
            # case 2: activate tethering, ping AP, unplug USB
            # ping AP (fail) plug USB, ping AP (fail), deactivate
            # tethering
            script_data = \
                """#!/system/bin/sh
sleep 10
am broadcast -a intel.intent.action.acs.cmd.system --es class %s --es method setUsbTethering --es switch on -e opcode %s
sleep 120
am broadcast -a intel.intent.action.acs.cmd.system --es class %s --es method setUsbTethering --es switch off -e opcode %s
sleep 30
""" \
                % (self._tethering_module, self._generate_key(),
                   self._tethering_module, self._generate_key())
        elif not use_flight_mode:
            # case 1: activate tethering, ping AP, activate flight mode
            # ping AP (fail) deactivate flight mode, ping AP, deactivate
            # tethering
            script_data = \
                """#!/system/bin/sh
sleep 10
am broadcast -a intel.intent.action.acs.cmd.system --es class %s --es method setUsbTethering  --es switch on -e opcode %s
sleep 60
am broadcast -a intel.intent.action.acs.cmd.system --es class %s --es method setFlightMode --ei mode 1 -e opcode %s
sleep 30
am broadcast -a intel.intent.action.acs.cmd.system --es class %s --es method setFlightMode --ei mode 0 -e opcode %s
sleep 30
am broadcast -a intel.intent.action.acs.cmd.system --es class %s --es method setUsbTethering  --es switch off -e opcode %s
sleep 30
""" \
                % (self._tethering_module, self._generate_key(),
                   self._connectivity_module, self._generate_key(),
                   self._connectivity_module, self._generate_key(),
                   self._tethering_module, self._generate_key())
        else:
            # case 0: activate tethering, ping AP, deactivate tethering
            script_data = \
                """#!/system/bin/sh
sleep 10
am broadcast -a intel.intent.action.acs.cmd.system --es class %s --es method setUsbTethering --es switch on -e opcode %s
sleep 60
am broadcast -a intel.intent.action.acs.cmd.system --es class %s --es method setUsbTethering --es switch off -e opcode %s
sleep 30
""" \
                % (self._tethering_module, self._generate_key(),
                   self._tethering_module, self._generate_key())

        # Copy the script file on DUT file system
        tmp_file = tempfile.NamedTemporaryFile(mode="w+b", delete=False)
        tmp_file.write(script_data)
        tmp_file.flush()
        tmp_file.close()
        cmd = "adb push %s %s" % (tmp_file.name, script_name)
        self._exec(cmd)
        os.unlink(tmp_file.name)

        # Set execution permissions
        cmd = "adb shell chmod 777 %s" % script_name
        self._exec(cmd)

        # Run script detached on DUT
        cmd = "adb shell exec nohup %s > %s && echo -n" \
              % (script_name, script_output)
        self._exec(cmd, wait_for_response=False)

    def set_preferred_network_mode(self, preferred_mode):
        """
        Sets the Preferred Network Type on user menu .

        :type preferred_mode:  str
        :param preferred_mode: can be:
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
            "Set preferred network type on dut ...")
        current_mode = self.get_preferred_network_type()
        if current_mode == preferred_mode:
            warning_msg = "the preferred network '%s' type is already enabled" % str(preferred_mode)
            self._logger.info(warning_msg)
        else:
            preferred_mode = get_dict_key_from_value(self.NETWORK_TYPE_CONVERSION_TABLE, preferred_mode)
            self._logger.info("Setting the wanted preferred network type: '%s' " % str(preferred_mode))
            method = "setPreferredNetworkMode"
            cmd = " --ei networkMode %d" \
                  % preferred_mode
            self._internal_exec_v2(self.network_type_module, method, cmd, is_system=True)

            self._logger.info("Update preferred network mode (%d) to database ..." % int(preferred_mode))
            self._set_preferred_network_db_value(preferred_mode)

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

        # Force the root because sometimes it is lost and the set regulatory domain command fails.
        self._exec("adb root", force_execution=True, timeout=10)
        time.sleep(2)

        wifi_chipset = self._get_wifi_chipset_manufacturer()

        if wifi_chipset == self.CHIPSET_INTEL:

            # Check Wifi chipset is ready to be set
            self.get_regulatorydomain()

            cmd = "adb shell iw reg set %s" % regulatory_domain
            output = self._exec(cmd)
            if "failed" in output.lower():
                msg = "Unable to set regulatory domain - %s" % str(output)
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        else:
            NetworkingKK.set_regulatorydomain(self, regulatory_domain, interface)

        # Store set value for later restoration.
        self.__last_set_reg_domain = regulatory_domain

        # Control the value set
        value_set = self.get_regulatorydomain()

        if value_set != regulatory_domain:
            msg = "Regulatory domain set fails. Try to set: %s. Read: %s" \
                % (regulatory_domain, value_set)
            self._logger.warning(msg)

    def remove_wpa_certificates(self, credential_password=None, pin_code_removal=False):
        """
        Remove wpa certificates already installed on device.
        Possibility to remove pin code already set.

        :type pin_code_removal: boolean
        :param pin_code_removal: Remove Pin Code

        :return: None
        """
        # uiautomator is imported here to avoid crashes when uiautomator is not installed on the computer
        # uiautomator is not in the ACS installer for the moment
        # Should be removed when uiautomator will be installed by ACS installer
        from uiautomator import Device
        if self.dut is None:
            self.dut = Device(self._device_serial_number)

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

        # Force the screen orientation to portrait
        display.set_display_orientation("portrait")

        # Step 1: Remove potential existing certificate
        self._exec(
            "adb shell am start -n com.android.settings/.SecuritySettings")
        time.sleep(1)
        self.dut(scrollable=True).scroll.to(text="Clear credentials")
        if self.dut(text="Clear credentials").enabled:
            msg = "Removing old Certificate"
            self._logger.info(msg)
            self.dut(text="Clear credentials").click()
            self.dut(text="OK").click()

        if pin_code_removal:
            # Remove Pin CODE
            msg = "Trying to remove PIN CODE on Device"
            self._logger.info(msg)
            # Step 2: Reset current screen phone lock PIN code if exists
            self._phone_system.set_phone_lock(0)
            self._exec(
                "adb shell am start -n com.android.settings/.SecuritySettings")
            time.sleep(1)

            self.dut(scrollable=True).scroll.to(text="Screen lock")

            if self.dut.exists(text="PIN"):
                msg = "Removing PIN CODE already set"
                self._logger.info(msg)
                self.dut(text="Screen lock").click()
                # setting Pin code
                self.dut(text="Confirm your PIN").set_text(credential_password)
                if self.dut(text="Next").exists:
                    self.dut(text="Next").click()
                elif self.dut(text="Continue").exists:
                    self.dut(text="Continue").click()
                else:
                    raise DeviceException(DeviceException.OPERATION_FAILED,
                                          "Unable to confirm PIN CODE , check if display has been modified ")

                self.dut(text="Swipe").click()
                self.dut(text="OK").click()
                if self.dut.exists(text="Swipe"):
                    msg = "PIN CODE successfully removed"
                    self._logger.info(msg)

                else:
                    msg = "PIN CODE was not successfully removed"
                    self._logger.error(msg)

                self.dut.press.back()
                self.dut.press.back()

            else:
                # Remove Pin CODE
                msg = "PIN CODE is already not set"
                self._logger.info(msg)

        # Re-allow phone locking
        self._phone_system.set_phone_lock(1)
        self._phone_system.set_phone_screen_lock_on(0)
        display.set_display_orientation("auto")

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

        method = "setWifiFrequencyBand"
        cmd_args = "--ei wifi_frequency_band %s" % self.SUPPORTED_WIFI_BANDS[freq_band]
        self._internal_exec_v2(self._wifi_module, method, cmd_args, is_system=True)

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
        method = "getWifiFrequencyBand"
        output = self._internal_exec_v2(self._wifi_module, method, is_system=True)

        status = int(output.get("wifi_frequency_band", None))
        self._logger.debug("STATUS VALUE = %s" % status)

        match = [k for k, v in self.SUPPORTED_WIFI_BANDS.iteritems() if v == status]
        if match:
            result = match[0]
        else:
            result = None

        self._logger.debug("Wifi frequency band: %s" % str(result))
        return result
