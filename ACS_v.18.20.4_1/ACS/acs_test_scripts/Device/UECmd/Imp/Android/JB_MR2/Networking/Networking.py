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
:summary: This file implements Networking UECmds for Android JB MR2 device
:since: 23 aug 2013
:author: smaurel
"""

from acs_test_scripts.Device.UECmd.Imp.Android.JB.Networking.Networking import Networking as NetworkingCommon
from acs_test_scripts.Device.UECmd.UECmdDecorator import need
import time


class Networking(NetworkingCommon):

    """
    Class that handle all networking operations
    """

    def __init__(self, phone):
        """
        Constructor
        """
        NetworkingCommon.__init__(self, phone)

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
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_CENTER)
        # select wifi certificate in a list
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_DOWN)
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_CENTER)
        # valide dialog
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

        if pin_code_removal:
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