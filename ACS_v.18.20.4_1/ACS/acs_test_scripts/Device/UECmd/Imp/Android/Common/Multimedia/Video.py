"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
:summary: This file implements UECmds for video playback
:since: 08/10/2014
:author: vdechefd
"""
import os
import re
import time
import re
import Lib.ImageCheck.Imagecheck as Imagecheck

from acs_test_scripts.Device.UECmd.Interface.Multimedia.IVideo import IVideo
from acs_test_scripts.Device.UECmd.Imp.Android.Common.BaseV2 import BaseV2

from ErrorHandling.AcsConfigException import AcsConfigException


class Video(BaseV2, IVideo):
    """
    Class that handle all video operations
    """

    def __init__(self, device):
        """
        Constructor
        """
        BaseV2.__init__(self, device)
        IVideo.__init__(self, device)

        self._target_video_class = 'acscmd.multimedia.VideoModuleActivity'
        self._multimedia_path = device.multimedia_path
        self._native_player = None

    def _is_screen_orientation_valid(self, screen_orientation):
        """
        :type screen_orientation: str
        :param screen_orientation: Possible values are portrait, landscape, reverse_portrait, reverse_landscape.
                                   If none, default device screen orientation will be used.
        """
        # Check screen_orientation value
        valid_screen_orientation = ["portrait", "landscape", "reverse_portrait", "reverse_landscape"]
        if screen_orientation not in (valid_screen_orientation + [None, ""]):
            error_msg = "Bad screen orientation value. Possible values are ".join(valid_screen_orientation)
            self._logger.error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)
        else:
            return True

    def play(self, filename, loop, screen_orientation=None):
        """
        Launch the video file playback.

        :type filename: str
        :param filename: can be an absolute path to a file on the device (such as /sdcard/myfile.avi), or a url to a
                         distant file, in http or rtsp protocol (such as http://myserver.com/myfile.mp4).
                         If it is a distant file, the call will be blocking until data is ready.
                         When using rtsp, loop parameter won't have any effect (e.g. playback won't loop)

        :type loop: bool
        :param loop: If True, video playback will loop until stop() is called.
                       If False, playback will stop once file's end is reached.

        :type screen_orientation: str
        :param screen_orientation: Possible values are portrait, landscape, reverse_portrait, reverse_landscape.
                                   If none, default device screen orientation will be used.

        :rtype: str
        :return: output log
        """
        str_filename = str(filename)
        self._logger.info("Start video playback on %s", str_filename)

        self._is_screen_orientation_valid(screen_orientation)

        # Check if file is a full path
        if os.path.dirname(str_filename) == "":
            filename = "%s%s" % (self._multimedia_path, str_filename)

        # Build the adb command to launch video playback
        target_method = "startVideoPlayback"
        args = " --es filename \"%s\"" % str_filename
        args += " --ez loop %s" % ("true" if loop else "false")

        # Add screen orientation extra
        if screen_orientation:
            self._logger.info("Video will be played in %s mode" % screen_orientation)
            args += " --es orientation %s" % screen_orientation
        else:
            self._logger.info("No screen orientation given : using device's default value.")

        # execute command on device
        output = self._internal_exec_v2(self._target_video_class, target_method, args)

        return output[Video.OUTPUT_MARKER]

    def stop(self):
        """
        Stop the video file playback.
        This is not meant to work with play_native().

        :rtype: float
        :return: playback duration in seconds.
        """
        self._logger.info("Stop video playback")

        # stop video playback
        target_method = "stopVideoPlayback"
        output = self._internal_exec_v2(self._target_video_class, target_method)
        duration = int(output["duration"]) / 1000.0

        # quit video player
        target_method = "quitPlayer"
        self._internal_exec_v2(self._target_video_class, target_method)

        return duration

    def is_playing(self):
        """
        Check if the video playback is running.
        This is not meant to work with play_native().

        :rtype: tuple
        :return: A tuple containing video playback state as a bool, and playback duration in seconds as a float.
                 If video is playing, it is the current playback's duration. If not, it is the duration of last
                 playback (or 0 if no last playback).
        """
        self._logger.info("Check video playback state")

        # Build the adb command to stop video playback
        target_class = "acscmd.multimedia.VideoModule"
        target_method = "isPlaying"

        # execute command on device
        output = self._internal_exec_v2(target_class, target_method)

        is_playing = (output["is_playing"] == "true")
        duration = int(output["duration"]) / 1000.0

        return is_playing, duration

    def play_native(self, filename, delay_time_s=0):
        """
        Launch the video file playback. This

        :type filename: str
        :param filename: can be an absolute path to a file on the device (such as /sdcard/myfile.avi), or a url to a
                         distant file, in http or rtsp protocol (such as http://myserver.com/myfile.mp4).
                         If it is a distant file, the call will be blocking until data is ready.

        :rtype: str
        :return: output log
        """
        app_1 = "com.google.android.apps.plus"
        app_2 = "com.google.android.apps.photos"
        app_3 = "com.android.gallery3d"

        if self.__check_player(app_1):
            #Up-to-date application
            self._native_player = app_1
            self._play_with_android_apps(filename, delay_time_s)
        elif self.__check_player(app_2):
            #Application for android M desert
            self._native_player = app_2
            self._play_with_android_apps(filename, delay_time_s)
        elif self.__check_player(app_3):
            #Old application (available for back-compatibility reasons)
            self._native_player = app_3
            self._play_with_android_gallery(filename, delay_time_s)
        else:
            msg = "No available player to play video!!!"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.FILE_NOT_FOUND, msg)

    def _play_with_android_gallery(self, filename, delay_time_s):
        """

        """
        self._logger.info("Start native video playback on %s (%s)" % (str(filename), self._native_player))

        # Check if file is a full path
        if os.path.dirname(filename) == "":
            filename = "%s%s" % (self._multimedia_path, str(filename))

        # Command creation
        start = "am start"
        param_1 = "-S -W -a android.intent.action.VIEW"
        param_2 = "-n com.android.gallery3d/.app.MovieActivity"
        param_3 = "-d %s" % filename
        param_4 = "-t \"video/*\""
        cmd = "%s %s %s %s %s" % (start, param_1, param_2, param_3, param_4)

        self.__play_common(cmd, delay_time_s)

    def _play_with_android_apps(self, filename, delay_time_s):
        """

        """
        self._logger.info("Start native video playback on %s (%s)" % (str(filename), self._native_player))

        # Check if file is a full path
        if os.path.dirname(filename) == "":
            filename = "%s%s" % (self._multimedia_path, str(filename))

        # Command creation
        start = "am start"
        param_1 = "-a android.intent.action.VIEW"
        param_2 = '-d "file://%s" -t "video/*"' % filename
        cmd = "%s %s %s" % (start, param_1, param_2)

        self.__play_common(cmd, delay_time_s)

    def __play_common(self, cmd, delay_time_s):
        """
        Common manner to launch video player, with or without delay.
        If delay_time_s is not 0, the player is launched asynchronously with shell script and nohup
        """
        if delay_time_s == 0:
            return self._exec("adb shell %s" % cmd)
        else:
            tmp_script_file = "/data/play_native_video.sh"
            self._exec("adb shell echo 'sleep %d; %s' > %s" %
                        (delay_time_s, cmd, tmp_script_file))

            self._exec("adb shell chmod 777 %s" % tmp_script_file)

            self._exec('adb shell "cd data; nohup sh play_native_video.sh"', wait_for_response=False, timeout=1)

    def __check_player(self, package_name):
        """

        """
        result = None
        self._logger.info("Try to find video app %s" % package_name)
        cmd_str = "adb shell pm list packages -f | grep %s" % package_name
        results_str = self._exec(cmd_str)

        results = results_str.split("\n")
        self._logger.info("packages: %s " % results)
        self._logger.info("packages length: %s " % len(results))

        regex_str = "(package):(?P<path>.*?)=(?P<package>.*)"
        regex = re.compile(regex_str)

        if len(results) > 1:
            self._logger.error("package manager list is greater than one")
            raise AcsConfigException(AcsConfigException.FILE_NOT_FOUND)

        result = results[0]
        self._logger.info("package result: %s" % result)

        match_object = regex.match(result)

        if match_object is None:
            return False

        package = match_object.group("package")
        self._logger.info("package: %s" % package)

        if package == package_name:
            result = True
        else:
            result = False

        return result

    def close_native(self):
        """
        Close native video player

        :return: None
        """

        if self._native_player is not None:
            exec_str = "adb shell am force-stop %s" % self._native_player
        else:
            app_1 = "com.google.android.apps.plus"
            app_2 = "com.google.android.apps.photos"
            app_3 = "com.android.gallery3d"

            if self.__check_player(app_1):
                exec_str = "adb shell am force-stop %s" % app_1
            elif self.__check_player(app_2):
                exec_str = "adb shell am force-stop %s" % app_2
            elif self.__check_player(app_3):
                exec_str = "adb shell am force-stop %s" % app_3
            else:
                msg = "No available player to play video!!!"
                self._logger.error(msg)
                raise AcsConfigException(AcsConfigException.FILE_NOT_FOUND, msg)
        self._exec(exec_str)

    def enable_post_processing_video_effect(self, wait_btwn_cmd, library_image_path, on_off_image):
        """
        Enable post processing video effect
        :type wait_btwn_cmd: int
        :param wait_btwn_cmd: time between command
        :type library_image_path: str
        :param library_image_path: path to the image library on host.
        :type on_off_image: dict
        :param on_off_image: Dictionary with two Key (ON : name of On image, OFF : nome of Off image)
        :rtype: None
        :return: None
        """
        image_filename = "DUTScreenshot"

        # Generate image dictionary
        dic_image = Imagecheck.generate_dictionary_image_path(library_image_path)

        # Open Intel Smart Video
        self._open_smart_video()
        time.sleep(wait_btwn_cmd)
        # Check if two effects is already On
        image_api = self._device.get_uecmd("Image", True)
        for _ in range(2):
            screenshot_path = image_api.take_screenshot_and_pull_on_host(image_filename, os.getcwd())
            time.sleep(wait_btwn_cmd)
            # Search On image in screen
            matching = Imagecheck.match_template_in_image(screenshot_path, dic_image[on_off_image["ON"]])
            # If On is displayed on the screen, set to Off
            if matching[0]:
                image_api.touch_template_on_screen(screenshot_path, dic_image[on_off_image["ON"]])
            os.remove(screenshot_path)
            # Click on two off button
        icons = [dic_image[on_off_image["OFF"]], dic_image[on_off_image["OFF"]]]

        multimedia_api = self._device.get_uecmd("Multimedia", True)
        multimedia_api.execute_icons_sequence(icons, os.getcwd())
        # Close Intel Smart Video
        self._close_smart_video()

    def disable_post_processing_video_effect(self, wait_btwn_cmd, library_image_path, on_off_image):
        """
        Disable post processing video effect
        :type wait_btwn_cmd: int
        :param wait_btwn_cmd: time between command
        :type library_image_path: str
        :param library_image_path: path to the image library on host.
        :type on_off_image: dict
        :param on_off_image: Dictionary with two Key (ON : name of On image, OFF : nome of Off image)
        :rtype: None
        :return: None
        """
        # Generate image dictionary
        dic_image = Imagecheck.generate_dictionary_image_path(library_image_path)

        # Open Intel Smart Video
        self._open_smart_video()
        time.sleep(wait_btwn_cmd)
        # Click on first off button
        # Click on two off button
        icons = [dic_image[on_off_image["ON"]], dic_image[on_off_image["ON"]]]
        multimedia_api = self._device.get_uecmd("Multimedia", True)
        multimedia_api.execute_icons_sequence(icons, os.getcwd())
        # Close Intel Smart Video
        self._close_smart_video()

    def _open_smart_video(self):
        """
        Open smart video settings
        :rtype: None
        :return: None
        """
        self._logger.info("Open Smart Video settings...")
        cmd = "adb shell am start -n com.intel.vpp/com.intel.vpp.VppSettings"
        self._exec(cmd)

    def _close_smart_video(self):
        """
        Close smart video settings
        :rtype: None
        :return: None
        """
        self._logger.info("Close Smart Video settings...")
        cmd = "adb shell am force-stop com.intel.vpp"
        self._exec(cmd)

