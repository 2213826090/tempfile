"""

:copyright: (c)Copyright 2016, Intel Corporation All Rights Reserved.
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

:organization: INTEL OTC ANDROID CORE QA
:summary: This script implements the interface to python uiautomator actions
:since: 07/01/2016
:author: apalko
"""
from ErrorHandling.DeviceException import DeviceException


class IPyUiAutomator():
    """
    Abstract class that defines the interface to be implemented
    by Python uiautomator operations.

    All method that shall be redefined in sub-classes raise a
    I{DeviceException} error.
    """

    def __init__(self, device):
        """
        Constructor
        """
        pass

    def press_home(self):
        """
        Used to press the home button
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED, "press_home")

    def screen_on(self):
        """
        Used to set the screen on
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED, "screen_on")

    def dismiss_anr(self, max_attempts):
        """
        Used to dismiss ANR dialog(s) if any is found on the screen

        :type max_attempts: int
        :param max_attempts: maximum number of tries to dismiss
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED, "dismiss_anr")

    def set_live_wallpaper_auto(self):
        """
        Used to change the Live Wallpaper automatically and randomly
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED, "set_live_wallpaper_auto")

    def set_live_wallpaper_name(self, wp_name):
        """
        Used to change the Live Wallpaper by means of its given name

        :type wp_name: str
        :param wp_name: the name of the live wallpaper to set

        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED, "set_live_wallpaper_name")

    def dismiss_soft_keyboard(self):
        """
        Used to dismiss the soft keyboard on the screen

        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED, "dismiss_soft_keyboard")

    # ####################Bluetooth Settings pannel###############################

    def bt_scroll_to_paired_devices(self):
        """
        Used for scrolling to Paired devices list in the Bluetooth Settings panel

        :return: true if paired devices list tile is found, false otherwise

        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED, "bt_scroll_to_paired_devices")

    def bt_open_options_menu(self):
        """
        Used for opening the "More options" menu in the Bluetooth Settings panel

        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED, "open_bluetooth_options")

    def bt_is_refresh_option_enabled(self):
        """
        Used for checking whether the "Refresh" option is available in the Bluetooth menu

        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED, "bt_is_refresh_option_enabled")

    def bt_refresh_scan_ui(self):
        """
        Used for refreshing the Bluetooth scan from the UI

        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED, "bt_refresh_scan")

    def bt_wait_for_scan_end(self):
        """
        This method waits for the full bluetooth scan to end for a maximum of 60 seconds

        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED, "bt_wait_for_scan_end")

    def bt_look_for_device_in_list(self, device_name_to_find):
        """
        This method looks for a specific device name in the list of found bluetooth devices on the screen

        :param device_name_to_find: name of the device to find, as it is visible for others
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED, "bt_look_for_device_in_list")

    # ##############Bluetooth profile base #############################

    def _bt_disconnect_profiles(self, device_name):
        """
        This method disconnects the profiles from a paired device given the device name.
        This method happens in the opened pairing menu.

        :param device_name: name of the device, as visible to others
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED, "_bt_disconnect_profiles")

    def _bt_connect_profiles(self, device_name):
        """
        This method connects back the profiles of a paired device
        This method happens in the opened pairing menu.
        :param device_name: name of the device, as visible to others
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED, "_bt_connect_profiles")

    def _bt_open_pairing_menu(self, device_name):
        """
        This method will open the pairing menu by clicking on the settings wheel in the list.
        We assume there exists an instance of paired devices

        :param device_name: name of the device, as visible to others
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED, "_bt_open_pairing_menu")

    def _bt_dismiss_pairing_menu(self, device_name):
        """
        This method will dismiss the opened pairing menu
        This method happens in the opened pairing menu.

        :param device_name: name of the device, as visible to others
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED, "_bt_dismiss_pairing_menu")

    def _bt_check_profile_disabled(self, device_name, boolean_check=False):
        """
        This method will check if any given profile by name is disabled.
        When further implement it will be parametrised according to the wanted profile.
        This method happens in the opened pairing menu.

        :param device_name: name of the device, as visible to others
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED, "_bt_check_profile_disabled")

    def _bt_check_profile_enabled(self, device_name, boolean_check=False):
        """
        This method will check if any given profile by name is disabled.
        When further inherited, it will be parametrised according to the wanted profile.
        This method happens in the opened pairing menu.

        :param device_name: name of the device, as visible to others
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED, "_bt_check_profile_enabled")

    def _bt_disable_profile(self, profile_name, fail_if_already_disabled=True):
        """
        This method will deactivate any given profile by name.
        When further inherited, it will be parametrised according to the wanted profile.
        This method happens in the opened pairing menu.

        :param profile_name: name of the profile to disable
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED, "_bt_disable_profile")

    def _bt_enable_profile(self, profile_name, fail_if_already_enabled=True):
        """
        This method will deactivate any given profile by name.
        When further inherited, it will be parametrised according to the wanted profile.
        This method happens in the opened pairing menu.

        :param profile_name: name of the profile to enable
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED, "_bt_enable_profile")

    def _bt_allow_profile(self, resource_id, fail_if_already=True, must_exist=False):
        """
        Checks a checkbox profile into pair request window

        :param resource_id: resource id of the checkbox as it appears into pair request window
        :param fail_if_already: set to true to fail if the checkbox is already checked
        :param must_exist: set to true to fail if the checkbox is not shown
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED, "_bt_allow_profile")

    def bt_pbap_allow_contact_sharing(self, fail_if_already=False, must_exist=False):
        """
        Checks allow contact sharing into Pair request window

        :param fail_if_already: set to true to fail if contact sharing is already checked
        :param must_exist: set to true to fail if contact sharing is not present into pair request
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED, "bt_pbap_allow_contact_sharing")

    # ##############Applications Utilities #############################

    def reopen_upfront_app_in_recent(self, expected_title):
        """
        This method assumes that the Recent Apps were opened and looks, by name, for the latest app to reopen

        :param expected_title: the expected title of the recent app
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED, "reopen_upfront_app_in_recent")

    def open_screenlock_menu(self):
        """
        This method opens the 'Screen Lock' menu in the Settings Security panel. This method assumes that we are in the
        Settings Security panel

        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED, "open_screenlock_menu")

    def set_lockscreen(self, lockscreen_type):
        """
        This method selects a lock screen from the Choose lock screen list

        :param lockscreen_type: a String with the name of the lock screen to set
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED, "set_lockscreen")

    def check_screenlock(self):
        """
        This method checks the lock screen type from the Security Settings list

        :return: the text of the lock screen type
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED, "check_screenlock")
