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
:summary: This script implements the N PyUiAutomator actions
:since: 07/01/2016
:author: apalko
"""
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.Device.UECmd.Imp.Android.M.Ui.PyUiAutomator import PyUiAutomator


class PyUiAutomator(PyUiAutomator):
    """
    Class that implements the N PyUiAutomator actions
    """

    def dismiss_anr(self, max_attempts):
        """
        Used to dismiss ANR dialog(s) if any is found on the screen

        :type max_attempts: int
        :param max_attempts: maximum number of tries to dismiss
        """

        self._logger.info("Dismiss ANR Dialog")

        for i in range(max_attempts):
            if self.d(textContains="has stopped").exists or self.d(textContains="keeps stopping").exists:
                close_button = self.d(text="Close", enabled=True)
                if not close_button.wait.exists(timeout=5000):
                    raise DeviceException(DeviceException.OPERATION_FAILED,
                                          "ANR dialog does not have CLOSE-enabled button")
                close_button.click()
            else:
                break

        # final check if any ANR dialog is still on the screen
        if self.d(textContains="has stopped").exists or self.d(textContains="keeps stopping").exists:
            raise DeviceException(DeviceException.OPERATION_FAILED, "ANR dialog not dismissed after %s attempts"
                                  % max_attempts)

    def bt_scroll_to_paired_devices(self):
        """
        Used for scrolling to Paired devices list in the Bluetooth Settings panel

        :return: true if paired devices list tile is found, false otherwise
        """
        list = self.d(resourceId='com.android.settings:id/list')
        if not list.wait.exists(timeout=3000):
            msg = 'Bluetooth devices list was not found on the screen'
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        if list.scroll.to(text='Paired devices'):
            return True
        else:
            return False

    def bt_look_for_device_in_list(self, device_name_to_find):
        """
        This method looks for a specific device name in the list of found bluetooth devices on the screen

        :param device_name_to_find: name of the device to find, as it is visible for others
        """
        obj = self.d(resourceId='com.android.settings:id/list')
        if not obj.exists:
            msg = 'Available devices list was not found on the screen'
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        if obj.scroll.to(text=device_name_to_find):
            return True
        else:
            msg = 'The device {0} was not found by the scan'.format(device_name_to_find)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    def _bt_open_pairing_menu(self, device_name):
        """
        This method will open the pairing menu by clicking on the settings wheel in the list.
        We assume there exists an instance of paired devices

        :param device_name: name of the device, as visible to others
        """
        if not self.bt_scroll_to_paired_devices():
            msg = 'There are no paired devices in the list'
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        device_details = self.d(resourceId='com.android.settings:id/list').child_by_text(device_name,
                                                                                         className='android.widget.LinearLayout').child(
                resourceId='com.android.settings:id/deviceDetails')
        if not device_details:
            msg = 'The device details for the given device name could not be found'
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        device_details.click()

        if not self.d(text='Use for').wait.exists(timeout=3000):
            msg = 'The pairing menu was not opened properly'
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    def reopen_upfront_app_in_recent(self, expected_title):
        """
        This method assumes that the Recent Apps were opened and looks, by name, for the latest app to reopen

        :param expected_title: the expected title of the recent app
        """

        self._logger.info("Opening {0} from recent apps list".format(expected_title))

        if self.d(resourceId='com.android.systemui:id/recents_view').wait.exists(timeout=10000):
            obj = self.d(resourceId='com.android.systemui:id/title', text=expected_title)
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

        list_obj = self.d(resourceId='com.android.settings:id/list')
        if not list_obj.wait.exists(timeout=3000):
            msg = 'The Settings Security menu list does not exist'
            raise DeviceException(DeviceException.OPERATION_SET_ERROR, msg)

        obj = self.d(text="Screen lock")
        if list_obj.scroll.to(text="Screen lock"):
            obj.click()
        else:
            msg = 'The Screen Lock option does not exist, cannot open the menu for selection'
            raise DeviceException(DeviceException.OPERATION_SET_ERROR, msg)

    def check_screenlock(self):
        """
        This method checks the lock screen type from the Security Settings list

        :return: the text of the lock screen type
        """

        self._logger.info("Checking Screen lock type")

        # check that we are in the Settings Security menu
        security_settings_obj = self.d(resourceId='com.android.settings:id/action_bar').child(text='Security')
        if not security_settings_obj.wait.exists(timeout=3000):
            msg = 'We are not in the Settings Security activity'
            raise DeviceException(DeviceException.OPERATION_SET_ERROR, msg)

        list_obj = self.d(resourceId='com.android.settings:id/list')
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
