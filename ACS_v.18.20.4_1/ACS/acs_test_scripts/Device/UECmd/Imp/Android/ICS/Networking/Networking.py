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
:summary: This file implements Networking UECmds for Android ICS device
:since: 02/12/2011
:author: ssavrimoutou
"""
import time
from acs_test_scripts.Device.UECmd.Imp.Android.Common.Networking.Networking import Networking as NetworkingCommon
from acs_test_scripts.Device.UECmd.UECmdTypes import AUTO_CONNECT_STATE
from acs_test_scripts.Device.UECmd.UECmdDecorator import need
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException


class Networking(NetworkingCommon):

    """
    Class that handle all networking operations
    """

    def __init__(self, phone):
        """
        Constructor
        """
        NetworkingCommon.__init__(self, phone)

    @need('wifi')
    def _toggle_wifi_disconnection_policy(self):
        """
        Toogle wifi disconnection policy in order to avoid or enable wifi disconnection
        if the network is too far away (wifi power too low).
        """
        KEYCODE_DPAD_UP = "19"
        KEYCODE_DPAD_DOWN = "20"
        KEYCODE_DPAD_CENTER = "23"
        KEYCODE_HOME = "3"
        KEYCODE_MENU = "82"

        # Unlock the phone
        self._phone_system.set_phone_lock(0)

        self._logger.info("Toggle Avoid poor connection")
        # Open Network Settings directly
        output = self._exec(
            "adb shell am start -n com.android.settings/.Settings")

        if output.find("Error:") != -1:
            error_msg = output[output.find("Error: ") + len("Error: "):]
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, error_msg)

        if output.find("Warning: ") != -1:
            warning_msg = output[output.find("Warning: ") + len("Warning: "):]
            self._logger.warning(warning_msg)

        # Ensure we are at the bottom of the current view
        count = 1
        while count < 19:
            self._exec("adb shell input keyevent " + KEYCODE_DPAD_UP)
            count += 1

        # Down to Wifi enabled options
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_DOWN)
        # Go into wifi settings
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_CENTER)

        # Go into wifi settings menu
        self._exec("adb shell input keyevent " + KEYCODE_MENU)

        # Choose and enter to the advanced menu
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_UP)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_CENTER)

        # Ensure we are at the bottom of the current view
        count = 1
        while count < 6:
            self._exec("adb shell input keyevent " + KEYCODE_DPAD_UP)
            count += 1

        # Choose the wifi disconnection policy button
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_DOWN)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_DOWN)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_DOWN)

        # Toogle the button
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_CENTER)

        # Exit the com.android.settings/.SubSettings application
        self._exec("adb shell input keyevent " + KEYCODE_HOME)
        # Re-allow phone locking
        self._phone_system.set_phone_lock(1)

    @need('wifi')
    def __get_wifi_avoid_connection_status(self):
        """
        """
        method = "getWifiAvoidConnectionStatus"
        output = self._internal_exec_v2(self._wifi_module, method, is_system=True)

        retrieved_state = output["state"]
        if str(retrieved_state).isdigit():
            retrieved_state = int(retrieved_state)
        self._logger.info("Wifi avoid connection status: " +
                          {0: "OFF", 1: "ON"}[retrieved_state])

        return retrieved_state

    @need('wifi')
    def set_autoconnect_mode(self, interface, state):
        """
        Sets the autoconnect mode to on/off for a specific interface or all

        :type interface: String
        :param interface: interface to modify or 'all' for all interfaces

        :type state: str or int
        :param state: Can be AUTO_CONNECT_STATE.on  to enable autoconnect
                             AUTO_CONNECT_STATE.off to disable autoconnect

        :return: None
        """
        # pylint: disable=E1101
        self._logger.info(
            "Setting autoconnect mode for %s interfaces to %s"
            % (str(interface), str(state)))

        # Check that we are asking for a known state
        if state not in AUTO_CONNECT_STATE:
            # Unknown request state
            output = "set_autoconnect_mode : %s is not in known state" % state
            self._logger.error(output)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, output)
        elif state == AUTO_CONNECT_STATE.off:
            mode = "0"
        else:
            mode = "1"

        # retrieve the initial wifi power status
        wifi_initial_status = self.get_wifi_power_status()

        # Enable wifi if necessary, because on ICS
        # we can't disable remembered wifi networks
        if wifi_initial_status == 0:
            self._logger.warning("In order to disable remembered wifi "
                                 + "networks, we must power on the wifi")
            self.set_wifi_power("on")

        # Get cellular interface
        cellular_ssid = self._phone_system.\
            get_property_value("gsm.sim.operator.alpha")

        # Set autoconnect mode to its original state if necessary
        if interface in (cellular_ssid, "all"):
            if mode == "0":
                self.deactivate_pdp_context()
            else:
                self.activate_pdp_context()

        # Set autoconnect mode on wifi interface
        if interface != cellular_ssid:
            method = "setAutoconnectMode"
            cmd_args = "--es ssid %s --ei mode %s" % (interface, mode)
            self._internal_exec_v2(self._wifi_module, method, cmd_args, is_system=True)

        # reset wifi state to its original state
        if wifi_initial_status == 0:
            self._logger.warning("Set the wifi power to its original state")
            self.set_wifi_power("off")

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
        self._exec(
            "adb shell am start -n com.android.settings/.SecuritySettings")
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_RIGHT)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_DOWN)
        self._exec("adb shell input keyevent " + KEYCODE_MOVE_END)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_UP)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_CENTER)
        self._logger.info("WPA load certificate: type certif password (%s)"
                          % eap_password)
        self._exec("adb shell input text " + eap_password)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_DOWN)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_RIGHT)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_CENTER)
        time.sleep(2)

        # Set the certificate name
        self._exec("adb shell input text " + certificate_name)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_DOWN)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_RIGHT)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_CENTER)

        # Go back to home screen
        self._exec("adb shell input keyevent " + KEYCODE_BACK)
        self._exec("adb shell input keyevent " + KEYCODE_BACK)
        self._exec("adb shell input keyevent " + KEYCODE_BACK)

        # Re-allow phone locking
        self._phone_system.set_phone_lock(1)
        self._phone_system.set_phone_screen_lock_on(0)
        display.set_display_orientation("auto")

    @need('wifi', False)
    def remove_wpa_certificates(self, credential_password=None, pin_code_removal=False):
        """
        Remove wpa certificates already installed on device.
        Possibility to remove pin code already set.

        :type pin_code_removal: boolean
        :param pin_code_removal: Remove Pin Code

        :return: None
        """
        KEYCODE_DPAD_DOWN = "20"
        KEYCODE_DPAD_RIGHT = "22"
        KEYCODE_DPAD_CENTER = "23"
        KEYCODE_BACK = "4"
        KEYCODE_MOVE_END = "123"
        KEYCODE_MOVE_HOME = "122"
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

        # Re-allow phone locking
        self._phone_system.set_phone_lock(1)
        self._phone_system.set_phone_screen_lock_on(0)
        display.set_display_orientation("auto")
