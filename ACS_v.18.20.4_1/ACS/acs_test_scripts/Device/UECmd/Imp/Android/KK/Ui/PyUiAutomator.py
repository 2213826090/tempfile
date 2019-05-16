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
:summary: This script implements the KK PyUiAutomator actions
:since: 07/01/2016
:author: apalko
"""
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.Device.UECmd.Imp.Android.Common.Ui.PyUiAutomator import PyUiAutomator
import random


class PyUiAutomator(PyUiAutomator):
    """
    Class that implements the KK PyUiAutomator actions
    """

    def set_live_wallpaper_auto(self):
        """
        Used to change the Live Wallpaper automatically and randomly
        """

        self._logger.info("Set random live wallpaper...")

        wp_list = list()

        if self.d(resourceId='com.google.android.googlequicksearchbox:id/workspace').wait.exists(timeout=2000):
            self.d.press.menu()
        else:
            err_msg = "HomeScreen workspace not found, cannot access options screen"
            raise DeviceException(DeviceException.OPERATION_FAILED, err_msg)
        if self.d(text='Wallpapers').wait.exists(timeout=5000):
            self.d(text='Wallpapers').click()
            if self.d(resourceId='com.google.android.googlequicksearchbox:id/wallpaper_scroll_container').wait.exists(
                    timeout=5000):
                self.d(
                        resourceId='com.google.android.googlequicksearchbox:id/wallpaper_scroll_container').fling.horiz.toEnd()
                if not self.d(resourceId='com.google.android.googlequicksearchbox:id/live_wallpaper_list').wait.exists:
                    err_msg = "Could not find the container at the end of wallpapers list"
                    raise DeviceException(DeviceException.OPERATION_FAILED, err_msg)
                j = 0
                while (
                        not self.d(
                                resourceId='com.google.android.googlequicksearchbox:id/wallpaper_list').exists) and j < 10:
                    for i in range(
                            self.d(resourceId='com.google.android.googlequicksearchbox:id/wallpaper_item_label').count):
                        if not self.d(
                                resourceId='com.google.android.googlequicksearchbox:id/live_wallpaper_list').child(
                                resourceId="com.google.android.googlequicksearchbox:id/wallpaper_item_label",
                                instance=i):
                            break
                        else:
                            list_text = \
                                self.d(
                                        resourceId='com.google.android.googlequicksearchbox:id/live_wallpaper_list').child(
                                        resourceId="com.google.android.googlequicksearchbox:id/wallpaper_item_label",
                                        instance=i).info['text']
                            if not list_text in wp_list:
                                wp_list.append(list_text)
                    self.d(
                            resourceId='com.google.android.googlequicksearchbox:id/wallpaper_scroll_container').scroll.horiz.backward()
                    j = j + 1
                for i in range(
                        self.d(resourceId='com.google.android.googlequicksearchbox:id/wallpaper_item_label').count):
                    if not self.d(resourceId='com.google.android.googlequicksearchbox:id/live_wallpaper_list').child(
                            resourceId="com.google.android.googlequicksearchbox:id/wallpaper_item_label",
                            instance=i):
                        break
                    else:
                        list_text = \
                            self.d(resourceId='com.google.android.googlequicksearchbox:id/live_wallpaper_list').child(
                                    resourceId="com.google.android.googlequicksearchbox:id/wallpaper_item_label",
                                    instance=i).info['text']
                        if not list_text in wp_list:
                            wp_list.append(list_text)
                if wp_list:
                    rand = random.choice(wp_list)
                else:
                    err_msg = "The Live Wallpapers list is empty"
                    raise DeviceException(DeviceException.OPERATION_FAILED, err_msg)
                while (not self.d(text=rand).exists) and j > 0:
                    self.d(
                            resourceId='com.google.android.googlequicksearchbox:id/wallpaper_scroll_container').scroll.horiz.forward()
                    j = j - 1
                if self.d(text=rand):
                    self.d(text=rand).click()
                    if not self.d(packageName='com.android.wallpaper.livepicker').wait.exists(timeout=5000):
                        err_msg = "Live Wallpaper picker screen not reached after tapping on %s Wallpaper" % rand
                        raise DeviceException(DeviceException.OPERATION_FAILED, err_msg)
                    if self.d(text='Set wallpaper').wait.exists(timeout=5000):
                        self.d(text='Set wallpaper').click()
                    else:
                        err_msg = "'Set wallpaper' button not found for %s Wallpaper" % rand
                        raise DeviceException(DeviceException.OPERATION_FAILED, err_msg)
                else:
                    err_msg = "The %s Live Wallpaper instance does not exist" % rand
                    raise DeviceException(DeviceException.OPERATION_FAILED, err_msg)
            else:
                err_msg = "The list of Wallpapers not reached"
                raise DeviceException(DeviceException.OPERATION_FAILED, err_msg)
        else:
            err_msg = "The Wallpaper option screen was not reached"
            raise DeviceException(DeviceException.OPERATION_FAILED, err_msg)
        if not self.d(resourceId='com.google.android.googlequicksearchbox:id/launcher').wait.exists(timeout=5000):
            err_msg = "Launcher screen was not reached after %s Wallpaper was set" % rand
            raise DeviceException(DeviceException.OPERATION_FAILED, err_msg)

        self._logger.info("Wallpaper %s set" % rand)

    def set_live_wallpaper_name(self, wp_name):
        """
        Used to change the Live Wallpaper by means of its given name

        :type wp_name: str
        :param wp_name: the name of the live wallpaper to set

        """

        self._logger.info("Set live wallpaper %s..." % wp_name)

        if self.d(resourceId='com.google.android.googlequicksearchbox:id/workspace').wait.exists(timeout=2000):
            self.d.press.menu()
        else:
            err_msg = "HomeScreen workspace not found, cannot access options screen"
            raise DeviceException(DeviceException.OPERATION_FAILED, err_msg)
        if self.d(text='Wallpapers').wait.exists(timeout=5000):
            self.d(text='Wallpapers').click()
        else:
            err_msg = "The Wallpaper option screen was not reached"
            raise DeviceException(DeviceException.OPERATION_FAILED, err_msg)
        if self.d(resourceId='com.google.android.googlequicksearchbox:id/wallpaper_scroll_container').wait.exists(
                timeout=5000):
            if self.d(
                    resourceId='com.google.android.googlequicksearchbox:id/wallpaper_scroll_container').scroll.horiz.to(
                    text=wp_name):
                self.d(text=wp_name).click()
                if not self.d(packageName='com.android.wallpaper.livepicker').wait.exists(timeout=5000):
                    err_msg = "Live Wallpaper picker screen not reached after tapping on %s Wallpaper" % wp_name
                    raise DeviceException(DeviceException.OPERATION_FAILED, err_msg)
                if self.d(text='Set wallpaper').wait.exists(timeout=5000):
                    self.d(text='Set wallpaper').click()
                else:
                    err_msg = "'Set wallpaper' button not found"
                    raise DeviceException(DeviceException.OPERATION_FAILED, err_msg)
            else:
                err_msg = "The Wallpaper name you provided %s does not exist" % wp_name
                raise DeviceException(DeviceException.OPERATION_FAILED, err_msg)
        else:
            err_msg = "The list of wallpapers not reached"
            raise DeviceException(DeviceException.OPERATION_FAILED, err_msg)
        if not self.d(resourceId='com.google.android.googlequicksearchbox:id/launcher').wait.exists(timeout=5000):
            err_msg = "Launcher screen was not reached after %s Wallpaper was set" % wp_name
            raise DeviceException(DeviceException.OPERATION_FAILED, err_msg)

        self._logger.info("Wallpaper %s set" % wp_name)
