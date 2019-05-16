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
:summary: This file implements UECmds for image operations for Android ICS device
:since: 06/12/2011
:author: dgonzalez
"""
from acs_test_scripts.Device.UECmd.Imp.Android.Common.Multimedia.Image import Image as ImageCommon
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class Image(ImageCommon):

    """
    Class that handle all image operations
    """

    def set_wallpaper(self, wp_type, wp_name):
        """
        Set a static or live wallpaper

        :type wp_type: String
        :param wp_type: the type of wallpaper: live | static
        :type wp_name: String
        :param wp_name: the name or index of wallpaper
        :rtype: None
        :return: None
        :raise: DeviceException
        """
        wp_name = wp_name.strip()
        if wp_name is None or wp_name in '':
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
            KEYCODE_DPAD_RIGHT = "22"

            try:
                wp_index = int(wp_name)
            except ValueError:
                self._exec("adb shell input keyevent " + KEYCODE_DPAD_BACK)
                raise DeviceException(DeviceException.DEFAULT_ERROR_CODE, "live paper name should be the index")

            # set live wallpaper
            self._exec("adb shell input keyevent " + KEYCODE_DPAD_UP)
            count = 1
            while count < wp_index:
                self._exec("adb shell input keyevent " + KEYCODE_DPAD_DOWN)
                count += 1
            self._exec("adb shell input keyevent " + KEYCODE_DPAD_CENTER)
            self._exec("adb shell input keyevent " + KEYCODE_DPAD_RIGHT)
            self._exec("adb shell input keyevent " + KEYCODE_DPAD_CENTER)
