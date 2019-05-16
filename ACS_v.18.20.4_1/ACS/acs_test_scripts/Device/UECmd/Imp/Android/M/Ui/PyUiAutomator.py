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
:summary: This script implements the M PyUiAutomator actions
:since: 07/18/2016
:author: mmaraci
"""
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.Device.UECmd.Imp.Android.LLP.Ui.PyUiAutomator import PyUiAutomator
import time


class PyUiAutomator(PyUiAutomator):
    """
    Class that implements the N PyUiAutomator actions
    """

    def _bt_dismiss_pairing_menu(self, enable=False):
        """
        This method will dismiss the opened pairing menu
        This method happens in the opened pairing menu.

        """
        self.dismiss_soft_keyboard()
        if not self.d(text='Use for').wait.exists(timeout=3000):
            msg = 'Could not dismiss paired menu, not found on the screen'
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        self.d(text='OK').click()

        if not self.bt_scroll_to_paired_devices():
            msg = 'The Bluetooth menu was not reached back'
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        if enable:
            for i in range(10):
                try :
                    if not self.d(textContains='Connecting').exists():
                        break
                except : pass
                time.sleep(6)

    def _bt_check_profile_disabled(self, profile_name, boolean_check=False):
        """
        This method will check if any given profile by name is disabled.
        When further implement it will be parametrised according to the wanted profile.
        This method happens in the opened pairing menu.

        """
        if profile_name != 'Input device':
            self.dismiss_soft_keyboard()
        checkbox = self.d(text=profile_name, enabled=True)
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
        :return:
        """
        self.dismiss_soft_keyboard()
        checkbox = self.d(text=profile_name, enabled=True)
        if not checkbox.wait.exists(timeout=40000):
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
        if profile_name != 'Input device':
            self.dismiss_soft_keyboard()
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

        checkbox = self.d(text=profile_name, enabled=True)
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
        self.dismiss_soft_keyboard()
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

        checkbox = self.d(text=profile_name, enabled=True)
        if not checkbox.wait.exists(timeout=5000):
            msg = 'The profile checkbox was not found'
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        checkbox.click()

        self._bt_check_profile_enabled(profile_name)

    def _bt_allow_profile(self, resource_id, fail_if_already=True, must_exist=False):
        """
        Checks a checkbox profile into pair request window

        :param resource_id: resource id of the checkbox as it appears into pair request window
        :param fail_if_already: set to true to fail if the checkbox is already checked
        :param must_exist: set to true to fail if the checkbox is not shown
        """

        if not self.d(resourceId="android:id/alertTitle").wait.exists(timeout=10000):
            msg = 'Pair request window is not on the screen, cannot check profile'
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        checkbox = self.d(resourceId=resource_id)
        if checkbox.wait.exists(timeout=5000):
            if not checkbox.checked:
                checkbox.click()
                if not self.d(resourceId=resource_id, checked=True).wait.exists(timeout=5000):
                    msg = 'Checkbox for profile was not checked into pair request window'
                    raise DeviceException(DeviceException.OPERATION_FAILED, msg)
            else:
                if fail_if_already:
                    msg = 'Checkbox for profile is already checked'
                    raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        else:
            if must_exist:
                msg = 'The checkbox for profile does not appear as an entry in the pair request window'
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    def bt_pbap_allow_contact_sharing(self, fail_if_already=False, must_exist=False):
        """
        Checks allow contact sharing into Pair request window

        :param fail_if_already: set to true to fail if contact sharing is already checked
        :param must_exist: set to true to fail if contact sharing is not present into pair request
        """

        self._logger.info("Allowing PBAP profile")

        resource_id = "com.android.settings:id/phonebook_sharing_message_confirm_pin"
        self._bt_allow_profile(self, resource_id, fail_if_already, must_exist)
