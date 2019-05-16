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
:summary: This file implements UECmds for multimedia purposes (audio, video ...)
:since: 18/01/2013
:author: ssavrimoutou
"""

import time
import os
import re
import xml.dom.minidom as minidom
import tempfile

from UtilitiesFWK.Utilities import Global
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException
import acs_test_scripts.Lib.ImageCheck.Imagecheck as Imagecheck

from acs_test_scripts.Device.UECmd.Imp.Android.Common.BaseV2 import BaseV2
from acs_test_scripts.Device.UECmd.Interface.Multimedia.IMultimedia import IMultimedia


class Multimedia(BaseV2, IMultimedia):
    """
    Class that handle all audio operations
    """
    KEYCODE_HOME = "3"
    KEYCODE_BACK = "4"
    KEYCODE_ENTER = "66"
    KEYCODE_DPAD_UP = "19"
    KEYCODE_DPAD_DOWN = "20"

    UECMD_ERROR_MEDIAPLAYER_ERROR = "MediaPlayer: error"
    UECMD_ERROR_ACTIVITY_NOT_STARTED = "Activity not started"

    EXTRA_SCREEN_ORIENTATION = "android.intent.extra.screenOrientation"

    VIDEO_DECODER_ERROR_MESSAGES = [
        "OMXVideoDecoder: Decoder returned DECODE_MEMORY_FAIL",
        "OMXVideoDecoder: Decoder returned DECODE_FAIL",
        "OMXVideoDecoder: Decoder returned DECODE_DRIVER_FAIL",
        "VideoDecoder: endDecodingFrame failed. status",
        "VideoDecoder: Reference frame 8 is missing for current frame 4"]
    VSP_ERROR_MESSAGES = [
        "pvr_drv_video: vsp_context_flush_cmdbuf fails with 'Bad address' at vendor/intel/hardware/psb_video/src/vsp_cmdbuf.c",
        "KERNEL: [drm:vsp_submit_cmdbuf] *ERROR* The VSP is hang abnormally!",
        "VPPWorker: vaEndPicture failed",
        "VPPProcThread: process error",
        "[drm:psb_fence_lockup] *ERROR* VSP timeout (probable lockup) detected, reset vsp",
        "[drm:psb_fence_lockup] *ERROR* fence sequence is 114"]

    def __init__(self, device):
        """
        Constructor
        """
        BaseV2.__init__(self, device)
        IMultimedia.__init__(self, device)

        self._system = device.get_uecmd("System")
        self._app_api = device.get_uecmd("AppMgmt")

        # Get path to multimedia files
        self._multimedia_path = device.multimedia_path

        # Initialize common intents
        self.android_action_view = "android.intent.action.VIEW"

        self.video_package = "com.android.gallery3d"
        self.audio_package = "com.android.music"
        self.video_component = self.video_package + "/.app.MovieActivity"
        self.audio_component = self.audio_package + "/.AudioPreview"
        self.gallery_component = self.video_package + "/.app.Gallery"
        self.__display_image_gallery3d_module = "acscmd.multimedia.ImageDisplayModuleActivity"

        self._target_class = "acscmd.multimedia.MediaModule"
        self.__image_api = self._device.get_uecmd("Image")

        # Instantiate Display UECmd for display
        self.__display = self._device.get_uecmd("Display")

    def __wait_for_triggered_message(self, message, timeout):
        """
        Wait for triggered message before timeout

        :type message: str
        :param message: Message to check

        :type timeout: integer
        :param timeout: script execution timeout

        :rtype: boolean
        :return: If messages have been received or not
        """
        # Wait before retrieving triggered messages
        msg_received = False
        received_messages = []
        start_time = time.time()

        while not msg_received and (time.time() - start_time) < timeout:
            time.sleep(0.1)
            received_messages = self._device_logger.get_message_triggered_status(message)
            if received_messages:
                msg_received = True

        if msg_received:
            for received_message in received_messages:
                self._logger.debug(received_message)
            return True
        else:
            self._logger.debug("Triggered message (%s) not received !" % message)
            return False

    def get_media_file_duration(self, filename):
        """
        Get the duration the audio/video file.

        :type filename: str
        :param filename: Audio/Video file used to get the duration

        :rtype: float
        :return: duration in second
        """
        target_method = "getMediaDuration"
        args = ' --es filename '
        if str(filename).startswith("http://") or os.path.dirname(filename):
            args += '"%s"' % str(filename)
        else:
            # In other case the file used will be from acs media path
            args += '"%s%s"' % (self._multimedia_path, str(filename))

        output = self._internal_exec_v2(self._target_class, target_method, args)
        media_duration = int(output[Multimedia.OUTPUT_MARKER]) / 1000.0
        return media_duration

    def launch_system_gallery_application(self):
        """
        Launch the gallery application.

        :rtype: None
        :return: None
        """
        video_component, gallery_component = self.retrieve_gallery_system_package()

        self._logger.info("Launch Gallery3d application")
        #Launch application Gallery3d
        self._exec("adb shell am start %s" % gallery_component)

    def get_video_id_from_path(self, filePath, mediaStoreName):
        """
        Get the video id from is path.

        :type filePath: str
        :param filePath: path of the video.
        :type mediaStoreName: str
        :param mediaStoreName: Use "external" if you find media in sdcard or "internal" if you find media in DUT.

        :rtype: str
        :return: video id
        """
        target_method = "getVideoIdFromPath"
        args = ' --es filePath %s --es mediaStoreName %s' % (str(filePath), str(mediaStoreName))

        output = self._internal_exec_v2(self._target_class, target_method, args)
        video_ID = output[Multimedia.OUTPUT_MARKER]
        return video_ID

    def play_video_from_ID(self, ID, screen_orientation=None):
        """
        Launch the video file playback.

        :type filename: str
        :param filename: can be only file stored in the Device file system

        :type screen_orientation: str
        :param screen_orientation: Possible values are portrait or landscape
            If none it will use default device screen orientation

        :rtype: None
        :return: None
        """
        video_component, gallery_component = self.retrieve_gallery_system_package()

        self._logger.info("Start video playback on content://media/external/video/media/%s", str(ID))

        # Check screen_orientation value
        if screen_orientation not in [None, "portrait", "landscape", ""]:
            error_msg = "Screen orientation value must be 'portrait' or 'landscape' !"
            self._logger.error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        # Build the adb command to launch video playback
        cmd = "adb shell am start -a %s" % self.android_action_view
        cmd += " -d \"content://media/external/video/media/%s\"" % str(ID)
        cmd += " -n %s" % video_component
        cmd += " -t \"video/*\""

        # Add screen orientation extra
        if screen_orientation == "landscape":
            self._logger.info("Video will be played in landscape mode !")
            cmd += " --ei %s 0" % Multimedia.EXTRA_SCREEN_ORIENTATION

        elif screen_orientation == "portrait":
            self._logger.info("Video will be played in portrait mode !")
            cmd += " --ei %s 1" % Multimedia.EXTRA_SCREEN_ORIENTATION

        else:
            self._logger.info("No screen orientation given ! Use default device value.")

        self._exec(cmd)

    def play_video_list(self,videoFile,videoTime,screen_orientation=None):
        """
        Play videos from a file(list).

        :type videoFile : str
        :param videoFile : list of video file paths seperated by comma. For Example : dir1\dir2\file1.mp4,dir1\dir2\file2.mp4
        :type videoTime : integer
        :param videoTime : runtime duration of the test_script in minutes

        :type screen_orientation: str
        :param screen_orientation: Possible values are portrait or landscape
            If none it will use default device screen orientation
        """
        self._logger.info("Start video playback /%s", str(videoFile))

        # Check screen_orientation value
        if screen_orientation not in [None, "portrait", "landscape", ""]:
            error_msg = "Screen orientation value must be 'None' or 'portrait' or 'landscape' !"
            self._logger.error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        # Build the adb command to launch video playback
        cmd = "adb shell am start"
        cmd += " -n com.example.gdc_cv_videoloop/.CVLoop_1"
        cmd += " -e vlist %s" % videoFile
        cmd += " -e time %s" % videoTime
        # Add screen orientation extra
        if screen_orientation == "landscape":
            self._logger.info("Video will be played in landscape mode !")
            cmd += " --ei %s 0" % Multimedia.EXTRA_SCREEN_ORIENTATION

        elif screen_orientation == "portrait":
            self._logger.info("Video will be played in portrait mode !")
            cmd += " --ei %s 1" % Multimedia.EXTRA_SCREEN_ORIENTATION

        else:
            self._logger.info("No screen orientation given ! Use default device value.")
        self._exec(cmd)

    def play_video_from_ID_with_Google_Photo(self, ID, screen_orientation=None):
        """
        Launch the video file playback with Google Photo.

        :type filename: str
        :param filename: can be only file stored in the Device file system

        :type screen_orientation: str
        :param screen_orientation: Possible values are portrait or landscape
            If none it will use default device screen orientation

        :rtype: None
        :return: None
        """
        video_component = "com.google.android.apps.plus/.phone.VideoViewActivity"

        self._logger.info("Start video playback on content://media/external/video/media/%s", str(ID))

        # Check screen_orientation value
        if screen_orientation not in [None, "portrait", "landscape", ""]:
            error_msg = "Screen orientation value must be 'portrait' or 'landscape' !"
            self._logger.error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        # Build the adb command to launch video playback
        cmd = "adb shell am start -a %s" % self.android_action_view
        cmd += " -d \"content://media/external/video/media/%s\"" % str(ID)
        cmd += " -n %s" % video_component
        cmd += " -t \"video/*\""

        # Add screen orientation extra
        if screen_orientation == "landscape":
            self._logger.info("Video will be played in landscape mode !")
            cmd += " --ei %s 0" % Multimedia.EXTRA_SCREEN_ORIENTATION

        elif screen_orientation == "portrait":
            self._logger.info("Video will be played in portrait mode !")
            cmd += " --ei %s 1" % Multimedia.EXTRA_SCREEN_ORIENTATION

        else:
            self._logger.info("No screen orientation given ! Use default device value.")

        self._exec(cmd)

    def Google_Photo_is_playing_video(self):
        """
        Check if video playback in Google photos.

        :rtype: bool
        :return: True if video is playing and False if video is not playing.
        """
        cmd = "adb shell dumpsys window | grep mCurrent"
        o = self._exec(cmd)
        if "VideoViewActivity" in o:
            return True
        else:
            return False

    def retrieve_gallery_system_package(self):
        """
        Retrieve gallery system package install on the DUT.

        :rtype: tuple
        :return: a tuple containing video component found on DUT as str and gallery component found on DUT as str.
        """

        #Find last version of gallery system application
        video_package = "com.android.gallery3d"
        if self._app_api.get_path_of_device_app(video_package) != "":
            video_component = video_package + "/.app.MovieActivity"
            gallery_component = video_package + "/.app.Gallery"

        #Find new version of gallery system application
        else:
            video_package = "com.google.android.gallery3d"
            if self._app_api.get_path_of_device_app(video_package) != "":
                video_component = video_package + "/com.android.gallery3d.app.MovieActivity"
                gallery_component = video_package + "/com.android.gallery3d.app.Gallery"
            #Raise exception if no gallery application found on DUT.
            else:
                raise DeviceException(DeviceException.INVALID_DEVICE_STATE, "No gallery system package found on DUT.")

        return video_component, gallery_component

    def display_image_photos(self, image_path, photos_icon, just_once_icon, pictures_save_directory):
        """
        Display image using photos
        :type image_path : str
        :param image_path : Path to the image to display
        :type photos_icon: str
        :param photos_icon: the name of gallery3d_icon
        :type just_once_icon: str
        :param just_once_icon: the name of just_once_icon
        :rtype: None
        :return: None
        """
        time_for_display = 2

        self._logger.info("Displaying %s using photos", image_path)
        if image_path not in (None, ""):
            method = "displayImage"
            cmd_args = "--es image %s" % image_path
            self._internal_exec_v2(self.__display_image_gallery3d_module, method, cmd_args)
            time.sleep(time_for_display)

            # select gallery3d application to display picture
            screen_shot_uri = self.__image_api.take_screenshot_and_pull_on_host("screen_shot", pictures_save_directory)
            self.__image_api.touch_template_on_screen(screen_shot_uri, photos_icon, 2)
            os.remove(screen_shot_uri)
            time.sleep(time_for_display)

            try:
                # tap on just_once
                screen_shot_uri = self.__image_api.take_screenshot_and_pull_on_host("screen_shot",
                                                                                    pictures_save_directory)
                self.__image_api.touch_template_on_screen(screen_shot_uri, just_once_icon)
                time.sleep(time_for_display)
            except:
                pass
            finally:
                os.remove(screen_shot_uri)

        else:
            error_msg = "Image path should be set"
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

    def launch_media_scanner(self, save_directory):
        """
        Force media scanner
        :type save_directory: str
        :param save_directory : directory where media files are saved in DUT
        :rtype: None
        :return: None
        """

        cmd = "adb shell am broadcast -a android.intent.action.MEDIA_MOUNTED -d file:///mnt/%s" % save_directory
        self._exec(cmd)

    def execute_icons_sequence(self, icons, pictures_save_directory, time_between_action=0.5, rotate=False):
        """
        Executes a sequence of tap on screen actions following a list of icons
        :type icons : list
        :param icons : icons to click on
        :type pictures_save_directory : str
        :param pictures_save_directory : directory where to save screen shots taken
        :type time_between_action : int
        :param time_between_action : time between tap on screen actions
        :type rotate : bool
        :param rotate : whether or not screen shots taken should be rotated.
        :rtype : None
        :return : None
        """

        for icon in icons:
            screen_shot_uri = self.__image_api.take_screenshot_and_pull_on_host("screen_shot", pictures_save_directory)
            screen_shot_rotated = Imagecheck.set_orientation(screen_shot_uri, "landscape", pictures_save_directory,
                                                             True)

            screen_shot_to_check = screen_shot_uri

            if rotate:
                screen_shot_to_check = screen_shot_rotated
            try:
                self.__image_api.touch_template_on_screen(screen_shot_to_check, icon)
            except:
                os.remove(screen_shot_uri)
                os.remove(screen_shot_rotated)
                raise DeviceException(DeviceException.INVALID_DEVICE_STATE, "Doesn't match template in screenshot.")

            os.remove(screen_shot_uri)
            os.remove(screen_shot_rotated)

            time.sleep(time_between_action)

    def start_run_run_3d(self):
        """
        Starts Run Run 3D
        :rtype : None
        :return : None
        """
        wait_for_loading = 15

        # Start run run application
        self._exec("adb shell am start -n com.timuzsolutions.runrun3d/com.prime31.UnityPlayerProxyActivity")
        time.sleep(wait_for_loading)

    def stop_run_run_3d(self):
        """
        Stops Run Run 3D
        :rtype : None
        :return : None
        """
        self._exec("adb shell am force-stop com.timuzsolutions.runrun3d")

    def set_hardware_composer(self, enable = True):
        """
        This does the same thing as going to Settings>Devloper options and enable/disable HW composer. The
        information for the command came from packages/apps/Settings/src/com/android/settings/DevelopmentSettings.java:writeDisableOverlaysOption()
        :rtype : None
        :return : None
        """
        if enable:
            hwc_key = 0
        else:
            hwc_key = 1

        cmd = "adb shell service call SurfaceFlinger 1008 i32 {0}".format(hwc_key)
        (status, output) =  self._device.run_cmd(cmd, self._uecmd_default_timeout)

        if status != Global.SUCCESS:
            self._logger.error(output)
            raise DeviceException(DeviceException.OPERATION_FAILED, output)

    def set_intel_smart_video(self, enable = True):
        """
        Turn on VSP features by writing to the Intel Smart Video shared preferences file.
        :rtype : None
        :return : None
        """
        if enable:
            vsp_key = 1
        else:
            vsp_key = 0

        cmd = 'adb shell sed -i "s/0frc/{0}frc/" /data/data/com.intel.vpp/shared_prefs/vpp_settings.xml'.format(vsp_key)
        (status, output) =  self._device.run_cmd(cmd, self._uecmd_default_timeout)

        if status != Global.SUCCESS:
            self._logger.error(output)
            raise DeviceException(DeviceException.OPERATION_FAILED, output)

        cmd = 'adb shell sed -i "s/0vpp/{0}vpp/" /data/data/com.intel.vpp/shared_prefs/vpp_settings.xml'.format(vsp_key)
        (status, output) =  self._device.run_cmd(cmd, self._uecmd_default_timeout)

        if status != Global.SUCCESS:
            self._logger.error(output)
            raise DeviceException(DeviceException.OPERATION_FAILED, output)

    def get_number_of_dropped_frames(self):
        """
        Get number of dropped video frames from dumpsys
        :rtype output : string
        :return output : dumpsys output
        :rtype no_total_frames : string
        :return no_total_frames : Total number of played frames
        :rtype  no_dropped_frames : string
        :return no_dropped_frames : number of dropped frames
        """
        cmd = 'adb shell dumpsys media.player | grep numVideoFramesDropped'
        (status, output) = self._device.run_cmd(cmd, self._uecmd_default_timeout)

        if status != Global.SUCCESS or output is None:
            self._logger.error(output)
            raise DeviceException(DeviceException.OPERATION_FAILED, output)

        re_match = re.match(".*numVideoFramesDecoded\(([0-9]*)\).*numVideoFramesDropped\(([0-9]*)\).*", output)

        if re_match and len(re_match.groups()) == 2:
            no_total_frames = re_match.group(1)
            no_dropped_frames = re_match.group(2)
            return (output, no_total_frames, no_dropped_frames)
        else:
            msg = "'dumpsys media.player | grep numVideoFramesDropped' returned unexpected result as following: " + output
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    def enable_youtube_cmdline_launch(self, version):
        """
        Add a permission to play Youtube from command-line
        :type file_path : string
        :param file_path : Path to a temporary directory to pull the permission file
        :rtype : None
        :return : None
        """
        test_files_root = os.path.join(tempfile.gettempdir(), 'package-restrictions.xml')
        self._device.run_cmd(cmd="adb pull /data/system/users/0/package-restrictions.xml {0}".format(test_files_root), timeout=120)
        if self.insert_youtube_item_if_doesnt_exist(test_files_root, version):
            self._device.run_cmd(cmd="adb push {0} /data/system/users/0/package-restrictions.xml".format(test_files_root), timeout=120)
            self._device.reboot(wait_settledown_duration=True)

    def insert_youtube_item_if_doesnt_exist(self, file_path, version):
        """
        Insert Youtube permission to permission XML. This is an internal method
        :type file_path : string
        :param file_path : Path to a temporary directory to pull the permission file
        :rtype : None
        :return : None
        """
        if version == "LLP":
            insert_node = """
                <item name="com.google.android.youtube/.UrlActivity" match="500000" always="true" set="2">
                    <set name="com.google.android.youtube/.UrlActivity" />
                    <set name="com.android.chrome/com.google.android.apps.chrome.Main" />
                    <filter>
                        <action name="android.intent.action.VIEW" />
                        <cat name="android.intent.category.DEFAULT" />
                        <scheme name="http" />
                        <auth host="www.youtube.com" />
                        <path sglob=".*" />
                    </filter>
                </item>
            """
        else:
            insert_node = """
                    <item name="com.google.android.youtube/com.google.android.apps.youtube.app.honeycomb.Shell$WatchActivity" match="500000" always="true" set="3">
                        <set name="com.android.browser/.BrowserActivity" />
                        <set name="com.android.chrome/com.google.android.apps.chrome.Main" />
                        <set name="com.google.android.youtube/com.google.android.apps.youtube.app.honeycomb.Shell$WatchActivity" />
                        <filter>
                            <action name="android.intent.action.VIEW" />
                            <cat name="android.intent.category.DEFAULT" />
                            <scheme name="http" />
                            <auth host="www.youtube.com" />
                            <path prefix="/watch" />
                        </filter>
                    </item>
            """

        package_restrictions = minidom.parse(file_path)
        preferred_activities = package_restrictions.getElementsByTagName('preferred-activities')[0]
        youtube_item_node = minidom.parseString(insert_node).firstChild
        for item_node in preferred_activities.childNodes:
            if self.xml_nodes_are_equal(item_node, youtube_item_node):
                return False
        preferred_activities.appendChild(youtube_item_node)
        out_file = open(file_path, 'wb')
        #print(package_restrictions.toprettyxml(indent="    "))
        #out_file.write(package_restrictions.toprettyxml(indent="    ", newl=""))
        package_restrictions.writexml(out_file)
        out_file.close()
        return True

    def xml_nodes_are_equal(self, left_node, right_node, check_children=True):
        """
        Compare XML nodes. This is an internal method
        :type file_path : string
        :param file_path : Path to a temporary directory to pull the permission file
        :rtype : None
        :return : None
        """
        if left_node.nodeType!=left_node.ELEMENT_NODE and right_node.nodeType!=right_node.ELEMENT_NODE:
            return True
        if left_node.nodeName != right_node.nodeName:
            return False
        if ( (left_node.attributes==None and right_node.attributes!=None) or
           (left_node.attributes!=None and right_node.attributes==None) ):
            return False
        if left_node.attributes!=None and right_node.attributes!=None and left_node.attributes.items()!=right_node.attributes.items(): # Compare lists
            return False
        if check_children:
            if len(left_node.childNodes) != len(right_node.childNodes):
                return False
            for index in range(len(left_node.childNodes)):
                if self.xml_nodes_are_equal(left_node.childNodes[index], right_node.childNodes[index]) == False:
                    return False
        return True

    def open_google_photos(self):
        """
        Open google photos
        :rtype: None
        :return: None
        """
        self._logger.info("Open Google Photos...")
        cmd = "adb shell am start -n " \
              "com.google.android.apps.plus/com.google.android.apps.photos.phone.PhotosLauncherActivity"
        self._exec(cmd)

    def check_screen(self, icon, type):
        """
        Check screen
        :type icon: str
        :param icon : id of icon
        :type type: str
        :param type: Type of id
        :rtype: bool
        :return: True if icon_id is in UI or False if icon_id is not in UI
        """
        status = False
        cmd = "adb shell uiautomator dump"
        self._exec(cmd)

        cmd = "adb shell cat /storage/emulated/legacy/window_dump.xml"
        output = self._exec(cmd)
        if type == "id":
            text = "id/" + icon
        elif type == "text":
            text = "text=\"" + icon + "\""
        else:
            text = None

        if text in output:
            status = True

        return status

    def open_google_camera(self):
        """
        Open google camera
        :rtype: None
        :return: None
        """
        self._logger.info("Open Google Camera...")
        # grant runtime permissions to google camera
        self._device.grant_runtime_permissions("com.google.android.GoogleCamera")
        cmd = "adb shell am start -n com.google.android.GoogleCamera/com.android.camera.CameraActivity"
        self._exec(cmd)
