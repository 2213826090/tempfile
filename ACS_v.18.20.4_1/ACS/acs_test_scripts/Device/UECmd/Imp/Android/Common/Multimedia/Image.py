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
:summary: This file implements interfaces from IImage, and run the test cases
:since: 26/04/2011
:author: Zhongyou
"""

import os

from acs_test_scripts.Device.UECmd.Interface.Multimedia.IImage import IImage
from acs_test_scripts.Device.UECmd.Imp.Android.Common.BaseV2 import BaseV2
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
import acs_test_scripts.Lib.ImageCheck.Imagecheck as Imagecheck


class Image(BaseV2, IImage):

    """
    Class that handle all image operations
    """

    def __init__(self, device):
        """
        Constructor
        """
        BaseV2.__init__(self, device)
        IImage.__init__(self, device)

    def set_wallpaper(self, wp_type, wp_name):
        """
        Set a static or live wallpaper

        :type wp_type: String
        :param wp_type: the type of wallpaper: live | static
        :type wp_name: String
        :param wp_name: the name or index of wallpaper
        :rtype: None
        :return: None
        :raise: AcsConfigException, DeviceException
        """
        wp_name = wp_name.strip()
        if (wp_name is None) or (wp_name in ''):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "wallpaper is not specified")

        module = 'acscmd.system.WallpaperModule'
        function = "setWallpaper"
        cmd_args = "--es wp_type {0} --es image_file_path {1}".format(wp_type, wp_name)
        self._internal_exec_v2(module, function, cmd_args)

        # set live wallpaper
        if wp_type in "live":
            KEYCODE_DPAD_UP = "19"
            KEYCODE_DPAD_DOWN = "20"
            KEYCODE_DPAD_CENTER = "23"
            KEYCODE_DPAD_BACK = "4"

            try:
                wp_index = int(wp_name)
            except ValueError:
                self._exec("adb shell input keyevent " + KEYCODE_DPAD_BACK)
                raise AcsConfigException(AcsConfigException.DEFAULT_ERROR_CODE, "live paper name should be the index")

            # set live wallpaper
            self._exec("adb shell input keyevent " + KEYCODE_DPAD_UP)
            count = 1
            while count < wp_index:
                self._exec("adb shell input keyevent " + KEYCODE_DPAD_DOWN)
                count += 1
            self._exec("adb shell input keyevent " + KEYCODE_DPAD_CENTER)
            self._exec("adb shell input keyevent " + KEYCODE_DPAD_CENTER)

    def restore_wallpaper(self):
        """
        restore to original wallpaper
        :rtype: None
        :return: None

        :raise: DeviceException
        """
        module = 'acscmd.system.WallpaperModule'
        function = "restoreWallpaper"
        self._internal_exec_v2(module, function)

    def take_screenshot_and_pull_on_host(self, filename, output_path):
        """
        Take a screenshot and save it on the host
        :type filename: str
        :param filename: the name we will give at the screenshot.
        :type output_path: str
        :param output_path: the path where you want pull the screenshot on the host
        :rtype: str
        :return: screenshot_path the complete name of the screenshot
        """
        self._logger.info("Take screenshot and pull it on host.")

        #take screenshot and store on the sdcard
        cmd = "adb shell screencap -p /sdcard/" + str(filename) + ".png"
        self._exec(cmd)
        #pull the screenshot on the host
        cmd = "adb pull /sdcard/" + str(filename) + ".png " + str(output_path)
        self._exec(cmd)
        cmd = "adb shell rm /sdcard/" + filename + ".png"
        self._exec(cmd)

        screenshot_path = os.path.join(output_path, filename + ".png")
        return screenshot_path

    def touch_template_on_screen(self, screenshot, template, tap_number=1):
        """
        This function search if the template is in screenshot. If the template is match, it realize an input tap on
        template coordinate.
        :type screenshot: str
        :param screenshot: path of the screenshot
        :type template: str
        :param template: path of the template
        :rtype: Bool
        :return: matching, True if match template or False if doesn't.
        """
        self._logger.info("Search template in DUT screenshot.")
        matching, x, y = Imagecheck.match_template_in_image(screenshot, template)
        if matching:
            count = 0
            while count < tap_number:
                self._logger.info("Touch screen on template coordinate.")
                cmd = "adb shell input tap " + str(x) + " " + str(y)
                self._exec(cmd)
                count += 1
        else:
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, "Doesn't match template in screenshot.")
        return matching

    def pull_on_host(self, file_path, output_path):
        """
        Get a file from host
        :type file_path: str
        :param file_path : path of the file to download
        :type output_path : str
        :param output_path : directory where to pull the file
        :rtype: str
        :return: path in host of the file pulled from the DUT
        """
        cmd = "adb pull " + str(file_path) + " " + str(output_path)
        self._exec(cmd)

        filename = file_path.split('/')[-1]

        return os.path.join(output_path, filename)
