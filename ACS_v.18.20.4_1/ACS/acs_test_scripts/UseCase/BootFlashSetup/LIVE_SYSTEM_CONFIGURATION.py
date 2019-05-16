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
:summary: This file implements the UC to configure some parameters on the device
:since: 06/06/2013
:author: pbluniex
"""
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global


class LiveSystemConfiguration(UseCaseBase):

    """
    Device configuration
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        self.__system = self._device.get_uecmd("System")
        self.__phonesystem = self._device.get_uecmd("PhoneSystem")
        self.__appmgmt_api = self._device.get_uecmd("AppMgmt")
        self.__reboot_mandatory = False

    def __skip_wizard(self):
        """
        Run skip wizard uecmd
        """
        if self._tc_parameters.get_param_value("SKIP_WIZARD", False, "str_to_bool"):
            self.__reboot_mandatory |= self.__system.skip_wizard()

    def __disable_verify_application(self):
        """
        Disable verify application
        """
        if self._tc_parameters.get_param_value("DISABLE_VERIFY_APPLICATION", False, "str_to_bool"):
            self.__phonesystem.set_verify_application(False)

    def __disable_lockscreen(self):
        """
        Disable lockscreen
        """
        if self._tc_parameters.get_param_value("DISABLE_LOCKSCREEN", False, "str_to_bool"):
            self.__reboot_mandatory |= self.__phonesystem.disable_lockscreen(reboot=False)

    def __configure_launcher(self):
        """
        Configure the launcher
        """
        if self._tc_parameters.get_param_value("CONFIGURE_LAUNCHER", False, "str_to_bool"):
            self.__system.configure_launcher()
            self.__reboot_mandatory = True

    def __set_display_brightness(self):
        """
        Set display brightness
        """
        if self._tc_parameters.get_param_value("SET_DISPLAY_BRIGHTNESS", False, "str_to_bool"):
            brightness = int(self._dut_config.get("displayBrightness"))
            new_brightness = int(self.__phonesystem.set_display_brightness(brightness))

            if abs(new_brightness - brightness) > 1:
                self._logger.warning("Requested brightness was %s, brightness has been set to %s" %
                                     (brightness, new_brightness))

    def __set_screen_timeout(self):
        """
        Configure the display timeout
        """
        screen_timeout = self._tc_parameters.get_param_value("DISPLAY_TIMEOUT", 0, int)
        if screen_timeout:
            self.__phonesystem.set_screen_timeout(screen_timeout)

    def __disable_vending_app(self):
        """
        Disable com.android.vending application
        """
        if self._tc_parameters.get_param_value("DISABLE_VENDING_APPLICATION", False, "str_to_bool"):
            self.__appmgmt_api.app_enable_disable("com.android.vending", False)

    def __disable_wifi_scan_always_enabled(self):
        """
        Disable wifi_scan_always_enabled option
        """
        self.__phonesystem.disable_wifi_scan_always_enable()

    def __close_welcome_screen(self):
        """
        Disable wifi_scan_always_enabled option
        """
        self.__phonesystem.close_welcome_screen()

    def __close_video_popup(self):
        """
        Disable wifi_scan_always_enabled option
        """
        self.__phonesystem.close_video_popup()
        self.__reboot_mandatory = True

    def __turn_off_location(self):
        """
        Turn off location
        """
        if self._tc_parameters.get_param_value("TURN_OFF_LOCATION", False, "str_to_bool"):
            self.__phonesystem.turn_off_location()

    def __disable_google_voice_hotword(self):
        # if the parameter is not set, we disable ok google
        if self._tc_parameters.get_param_value("DISABLE_OK_GOOGLE", False, "str_to_bool"):
            self.__system.disable_voiceinteraction()
            self.__system.disable_voicerecognition()
            self.__system.disable_google_voice_hotword()
            self.__reboot_mandatory = True

    def run_test(self):
        """
        Run the test
        """
        UseCaseBase.run_test(self)

        self.__skip_wizard()
        self.__disable_lockscreen()

        self.__disable_verify_application()
        self.__disable_vending_app()
        self.__set_screen_timeout()
        self.__set_display_brightness()
        self.__configure_launcher()
        self.__disable_wifi_scan_always_enabled()
        self.__disable_google_voice_hotword()
        self.__close_welcome_screen()
        self.__close_video_popup()
        self.__turn_off_location()
        if self.__reboot_mandatory:
            self._device.reboot(wait_settledown_duration=True)
        return Global.SUCCESS, "No error"
