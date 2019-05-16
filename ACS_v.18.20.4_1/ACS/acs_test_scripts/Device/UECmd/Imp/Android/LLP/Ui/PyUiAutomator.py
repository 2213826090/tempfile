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
:summary: This script implements the LLP PyUiAutomator actions
:since: 07/01/2016
:author: apalko, mmaraci
"""
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.Device.UECmd.Imp.Android.KK.Ui.PyUiAutomator import PyUiAutomator


class PyUiAutomator(PyUiAutomator):
    """
    Class that implements the LLP PyUiAutomator actions
    """

    def dismiss_anr(self, max_attempts):
        """
        Used to dismiss ANR dialog(s) if any is found on the screen

        :type max_attempts: int
        :param max_attempts: maximum number of tries to dismiss
        """

        self._logger.info("Dismiss ANR Dialog")

        for i in range(max_attempts):
            if self.d(textContains="has stopped").exists or self.d(textContains="isn't responding").exists:
                ok_button = self.d(text="OK", enabled=True)
                if not ok_button.wait.exists(timeout=5000):
                    raise DeviceException(DeviceException.OPERATION_FAILED,
                                          "ANR dialog does not have OK-enabled button")
                ok_button.click()
            else:
                break

        # final check if any ANR dialog is still on the screen
        if self.d(textContains="has stopped").exists or self.d(textContains="isn't responding").exists:
            raise DeviceException(DeviceException.OPERATION_FAILED, "ANR dialog not dismissed after %s attempts"
                                  % max_attempts)

    def bt_scroll_to_paired_devices(self):
        """
        Used for scrolling to Paired devices list in the Bluetooth Settings panel

        :return: true if paired devices list tile is found, false otherwise
        """
        list = self.d(resourceId='android:id/list')
        if not list.wait.exists(timeout=3000):
            msg = 'Bluetooth devices list was not found on the screen'
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        if list.scroll.to(text='Paired devices'):
            return True
        else:
            return False

    def bt_open_options_menu(self):
        """
        Used for opening the "More options" menu in the Bluetooth Settings panel

        """
        if self.d(description='More options').wait.exists(timeout=3000):
            self.d(description='More options').click()
        else:
            msg = 'The More Options menu was not found'
            raise DeviceException(DeviceException.OPERATION_SET_ERROR, msg)

    def bt_is_refresh_option_enabled(self):
        """
        Used for checking whether the "Refresh" option is available in the Bluetooth menu

        """
        if self.d(text='Refresh').wait.exists(timeout=3000):
            return True
        else:
            msg = 'The Refresh option could not be found on the screen'
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    def bt_refresh_scan_ui(self):
        """
        Used for refreshing the Bluetooth scan from the UI

        """
        self.bt_open_options_menu()
        if self.bt_is_refresh_option_enabled():
            self.d(text='Refresh').click()

        # check that a new search has started
        if self.d(resourceId='com.android.settings:id/scanning_progress').wait.exists(timeout=2000):
            return True
        else:
            msg = 'A new scan seems to not have started'
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    def bt_wait_for_scan_end(self):
        """
        This method waits for the full bluetooth scan to end for a maximum of 60 seconds

        """
        if self.d(resourceId='com.android.settings:id/scanning_progress').wait.gone(timeout=60000):
            return True
        else:
            msg = 'Somehow, the scan process did not end in the given timeout {0}s'.format(60)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    def bt_look_for_device_in_list(self, device_name_to_find):
        """
        This method looks for a specific device name in the list of found bluetooth devices on the screen

        :param device_name_to_find: name of the device to find, as it is visible for others
        """
        obj = self.d(resourceId='android:id/list')
        if not obj.exists:
            msg = 'Available devices list was not found on the screen'
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        if obj.scroll.to(text=device_name_to_find):
            return True
        else:
            msg = 'The device {0} was not found by the scan'.format(device_name_to_find)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    def _bt_disconnect_profiles(self, device_name):
        """
        This method disconnects the profiles from a paired device given the device name.
        This method happens in the opened pairing menu.

        :param device_name: name of the device, as visible to others
        """

        if not self.bt_scroll_to_paired_devices():
            msg = 'There are no paired devices in the list'
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        if not self.d(text=device_name).wait.exists(timeout=3000):
            msg = 'The given device is not present in the list'
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        if not self.d(className='android.widget.RelativeLayout').child_by_text(device_name,
                                                                               resourceId='android:id/title').child(
                textContains='Connected').wait.exists(timeout=3000):
            msg = 'Profile is already disconnected'
            raise DeviceException(DeviceException.OPERATION_SET_ERROR, msg)

        self.d(text=device_name).click()

        if self.d(className='android.widget.RelativeLayout').child_by_text(device_name,
                                                                           resourceId='android:id/title').child(
                textContains='Connected').wait.exists(timeout=3000):
            msg = 'Profile appears to still be connected'
            raise DeviceException(DeviceException.OPERATION_SET_ERROR, msg)

    def _bt_connect_profiles(self, device_name):
        """
        This method connects back the profiles of a paired device
        This method happens in the opened pairing menu.
        :param device_name: name of the device, as visible to others
        """
        if not self.bt_scroll_to_paired_devices():
            msg = 'There are no paired devices in the list'
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        if not self.d(text=device_name).wait.exists(timeout=3000):
            msg = 'The given device is not present in the list'
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        if self.d(className='android.widget.RelativeLayout').child_by_text(device_name,
                                                                           resourceId='android:id/title').child(
                textContains='Connected').wait.exists(timeout=3000):
            msg = 'Profile is already connected'
            raise DeviceException(DeviceException.OPERATION_SET_ERROR, msg)

        self.d(text=device_name).click()

        if not self.d(className='android.widget.RelativeLayout').child_by_text(device_name,
                                                                               resourceId='android:id/title').child(
                textContains='Connected').wait.exists(timeout=3000):
            msg = 'Profile appears to still be disconnected'
            raise DeviceException(DeviceException.OPERATION_SET_ERROR, msg)

    def _bt_open_pairing_menu(self, device_name):
        """
        This method will open the pairing menu by clicking on the settings wheel in the list.
        We assume there exists an instance of paired devices

        :param device_name: name of the device, as visible to others
        """
        if not self.bt_scroll_to_paired_devices():
            msg = 'There are no paired devices in the list'
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        device_details = self.d(resourceId='android:id/list').child_by_text(device_name,
                                                                            className='android.widget.LinearLayout').child(
                resourceId='com.android.settings:id/deviceDetails')
        if not device_details:
            msg = 'The device details for the given device name could not be found'
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        device_details.click()

        if not self.d(text='Use for').wait.exists(timeout=3000):
            msg = 'The pairing menu was not opened properly'
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    def _bt_dismiss_pairing_menu(self, device_name):
        """
        This method will dismiss the opened pairing menu
        This method happens in the opened pairing menu.

        :param device_name: name of the device, as visible to others
        """
        if self.d(text='Use for').wait.exists(timeout=3000):
            self.d(text='OK').click()
        else:
            msg = 'Could not dismiss paired menu, not found on the screen'
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        if not self.bt_scroll_to_paired_devices():
            msg = 'The Bluetooth menu was not reached back'
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    def _bt_check_profile_disabled(self, profile_name, boolean_check=False):
        """
        This method will check if any given profile by name is disabled.
        When further implement it will be parametrised according to the wanted profile.
        This method happens in the opened pairing menu.
        :return:
        """
        checkbox = self.d(resourceId='android:id/list').child_by_text(profile_name,
                                                                      className='android.widget.LinearLayout').child(
                resourceId='android:id/checkbox')
        if not checkbox.wait.exists(timeout=5000):
            msg = 'The profile checkbox was not found'
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        if checkbox.info['checked']:
            if boolean_check:
                return False
            else:
                msg = 'The profile checkbox is not unchecked, profile does not appear as disabled'
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        return True

    def _bt_check_profile_enabled(self, profile_name, boolean_check=False):
        """
        This method will check if any given profile by name is disabled.
        When further inherited, it will be parametrised according to the wanted profile.
        This method happens in the opened pairing menu.
        """
        checkbox = self.d(resourceId='android:id/list').child_by_text(profile_name,
                                                                      className='android.widget.LinearLayout').child(
                resourceId='android:id/checkbox')
        if not checkbox.wait.exists(timeout=5000):
            msg = 'The profile checkbox was not found'
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        if not checkbox.info['checked']:
            if boolean_check:
                return False
            else:
                msg = 'The profile checkbox is not checked, profile does not appear as enabled'
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        return True

    def _bt_disable_profile(self, profile_name, fail_if_already_disabled=True):
        """
        This method will deactivate any given profile by name.
        When further inherited, it will be parametrised according to the wanted profile.
        This method happens in the opened pairing menu.
        :param profile_name:
        """
        if not self.d(text=profile_name).wait.exists(timeout=3000):
            msg = 'The profile does not appear as an entry in the menu'
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        if fail_if_already_disabled:
            if self._bt_check_profile_disabled(profile_name, boolean_check=True):
                msg = 'The profile is already disabled and it should not have been in this state'
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        else:
            if self._bt_check_profile_disabled(profile_name, boolean_check=True):
                return

        checkbox = self.d(resourceId='android:id/list').child_by_text(profile_name,
                                                                      className='android.widget.LinearLayout').child(
                resourceId='android:id/checkbox')
        if not checkbox.wait.exists(timeout=5000):
            msg = 'The profile checkbox was not found'
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        checkbox.click()

        if not self.d(text='Disable profile?').wait.exists(timeout=3000):
            msg = 'The disable menu prompt was not opened for the user'
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        self.d(text='OK').click()

        if not self.d(text='Disable profile?').wait.gone(timeout=3000):
            msg = 'The disable menu prompt was not closed after the user interaction'
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        self._bt_check_profile_disabled(profile_name)

    def _bt_enable_profile(self, profile_name, fail_if_already_enabled=True):
        """
        This method will deactivate any given profile by name.
        When further inherited, it will be parametrised according to the wanted profile.
        This method happens in the opened pairing menu.
        :param profile_name:
        """
        if not self.d(text=profile_name).wait.exists(timeout=3000):
            msg = 'The profile does not appear as an entry in the menu'
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        if fail_if_already_enabled:
            if self._bt_check_profile_enabled(profile_name, boolean_check=True):
                msg = 'The profile is already enabled and it should not have been in this state'
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        else:
            if self._bt_check_profile_enabled(profile_name, boolean_check=True):
                return

        checkbox = self.d(resourceId='android:id/list').child_by_text(profile_name,
                                                                      className='android.widget.LinearLayout').child(
                resourceId='android:id/checkbox')
        if not checkbox.wait.exists(timeout=5000):
            msg = 'The profile checkbox was not found'
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        checkbox.click()

        self._bt_check_profile_enabled(profile_name)

    def reopen_upfront_app_in_recent(self, expected_title):
        """
        This method assumes that the Recent Apps were opened and looks, by name, for the latest app to reopen

        :param expected_title: the expected title of the recent app
        """

        self._logger.info("Opening {0} from recent apps list".format(expected_title))

        if self.d(resourceId='com.android.systemui:id/recents_view').wait.exists(timeout=10000):
            obj = self.d(resourceId='com.android.systemui:id/activity_description', text=expected_title)
            if obj.wait.exists(timeout=5000):
                obj.click()
            else:
                msg = 'The {0} app was not present in Recent Apps'.format(expected_title)
                raise DeviceException(DeviceException.OPERATION_SET_ERROR, msg)
        else:
            msg = 'The Recent Apps stack was not opened'
            raise DeviceException(DeviceException.OPERATION_SET_ERROR, msg)

    def open_screenlock_menu(self):
        """
        This method opens the 'Screen Lock' menu in the Settings Security panel. This method assumes that we are in the
        Settings Security panel

        """

        self._logger.info("Opening Screenlock menu")

        list_obj = self.d(resourceId='android:id/list')
        if not list_obj.wait.exists(timeout=3000):
            msg = 'The Settings Security menu list does not exist'
            raise DeviceException(DeviceException.OPERATION_SET_ERROR, msg)

        obj = self.d(text="Screen lock")
        if list_obj.scroll.to(text="Screen lock"):
            obj.click()
        else:
            msg = 'The Screen Lock option does not exist, cannot open the menu for selection'
            raise DeviceException(DeviceException.OPERATION_SET_ERROR, msg)

    def set_lockscreen(self, lockscreen_type):
        """
        This method selects a lock screen from the Choose lock screen list

        :param lockscreen_type: a String with the name of the lock screen to set
        """

        self._logger.info("Setting Screen lock type as {0}".format(lockscreen_type))

        if lockscreen_type in ['Pattern', 'PIN', 'Password']:
            msg = 'This type of lockscreen setting has not been implemented: {0}'.format(lockscreen_type)
            raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED, msg)

        # here, we assume the Screen Lock menu was opened
        if not self.d(text="Choose screen lock").wait.exists(timeout=3000):
            msg = "We are not in the 'Choose screen lock' menu"
            raise DeviceException(DeviceException.OPERATION_SET_ERROR, msg)

        obj = self.d(text=lockscreen_type)

        if obj.wait.exists(timeout=3000):
            obj.click()
        else:
            msg = 'The screen lock option {0} could not be found'.format(lockscreen_type)
            raise DeviceException(DeviceException.OPERATION_SET_ERROR, msg)

    def check_screenlock(self):
        """
        This method checks the lock screen type from the Security Settings list

        :return: the text of the lock screen type
        """

        self._logger.info("Checking Screen lock type")

        # check that we are in the Settings Security menu
        security_settings_obj = self.d(resourceId='android:id/action_bar').child(text='Security')
        if not security_settings_obj.wait.exists(timeout=3000):
            msg = 'We are not in the Settings Security activity'
            raise DeviceException(DeviceException.OPERATION_SET_ERROR, msg)

        list_obj = self.d(resourceId='android:id/list')
        if not list_obj.wait.exists(timeout=3000):
            msg = 'The Settings Security menu list does not exist'
            raise DeviceException(DeviceException.OPERATION_SET_ERROR, msg)

        obj = self.d(text="Screen lock")
        if list_obj.scroll.to(text="Screen lock"):
            screenlock_value_obj = obj.sibling(resourceId='android:id/summary')
        else:
            msg = 'The Screen Lock option does not exist in the menu'
            raise DeviceException(DeviceException.OPERATION_SET_ERROR, msg)

        if not screenlock_value_obj.wait.exists(timeout=3000):
            msg = 'The value for the current Screen Lock does not exist in the menu'
            raise DeviceException(DeviceException.OPERATION_SET_ERROR, msg)

        return screenlock_value_obj.text
