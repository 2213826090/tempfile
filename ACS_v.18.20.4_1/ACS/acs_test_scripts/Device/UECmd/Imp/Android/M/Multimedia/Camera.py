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

:organization: INTEL MCG PSI, PEG SVE DSV and TMT TTD AN
:summary: camera implementation for MM. This UECMD depends on device model to have button coordinates at get_camera_touchcmds.
          toggle_burst_mode and change_camera in the Camera UEcmd require these coordinates.
:since: 11/15/2015
:authors: imoren2x, jongyoon, ahkhowaj
"""

from acs_test_scripts.Device.UECmd.Imp.Android.Common.Multimedia.Camera import Camera as CameraCommon
from acs_test_scripts.Device.UECmd.Interface.Multimedia.ICamera import ICamera
from acs_test_scripts.Device.UECmd.UECmdDecorator import need
from UtilitiesFWK.Utilities import Global
import time
import os
import posixpath

class Camera(CameraCommon, ICamera):

    """
    Class that handles all Camera operations
    """

    # 'Camera 0: Opened' is a message shared by Google camera app and refCam app for logs when they are started.
    action_messages = dict([
                ('cameraAppStarting', 'Camera 0: Opened')
                   ])

    @need('camera')
    def __init__(self, device):
        """
        Constructor
        """
        CameraCommon.__init__(self, device)
        ICamera.__init__(self, device)
        self.camera_package = "com.google.android.GoogleCamera"
        self.camera_component = "com.android.camera.CameraLauncher"

    def launch(self):
        """
        Launches the camera.
        """
        self._logger.info("Camera MM to be launched")

        # stop camera in case of one is already running
        self.legacy_shutdown()

        #Set Preference configuration file
        self.__set_preference_conf_file()

        # grant runtime permissions to the camera app
        self._device.grant_runtime_permissions(self.camera_package)

        # -W to wait for completion and -S to kill the camera first
        start_cmd = "am start -W -S -n com.google.android.GoogleCamera/com.android.camera.CameraLauncher"
        cmd = "adb shell %s" % start_cmd
        self._exec(cmd, 5)
        self.__camera_started = 1

        time.sleep(2)

        self._logger.info("Camera MM launched")

    def launch_system_camera_application(self, checkTriglogMsg = False, reset = False):
        """
        Launches the camera.

        :rtype: None
        :return: None
        """

        self._logger.info("Camera MM to be launched")

        # grant runtime permissions to the camera app
        self._device.grant_runtime_permissions(self.camera_package)

        #Set Preference configuration file
        self.__set_preference_conf_file()

        if checkTriglogMsg is True:
            # Reset trigger message first in case it has already occurred.
            self._device_logger.reset_trigger_message(self.action_messages['cameraAppStarting'])
            # Launch camera application
            if reset is True:
                # -W to wait for completion and -S to kill the camera first
                self._exec("adb shell am start -S -W -a android.intent.action.MAIN -n " + self.camera_package + '/' + self.camera_component)
            else:
                self._exec("adb shell am start -W -a android.intent.action.MAIN -n " + self.camera_package + '/' + self.camera_component)

            self.__camera_started = 1

            startMsg = self._device_logger.get_message_triggered_status(self.action_messages['cameraAppStarting'])
            if len(startMsg) == 0:
                self._logger.error("Unable to start the camera app.")
                timestamp = datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d_%H-%M-%S')
                self._exec("adb shell screencap -p  " + posixpath.join(self.error_dir, "screen_startfail_%s.png"%timestamp))
            else:
                self._logger.info("Camera MM launched")
        else:
            if reset is True:
                # -W to wait for completion and -S to kill the camera first
                self._exec("adb shell am start -S -W -a android.intent.action.MAIN -n " + self.camera_package + '/' + self.camera_component)
            else:
                self._exec("adb shell am start -W -a android.intent.action.MAIN -n " + self.camera_package + '/' + self.camera_component)

            self.__camera_started = 1

    def legacy_shutdown(self):
        """
        Stop the camera
        """
        stop_cmd = "am force-stop com.google.android.GoogleCamera"
        cmd = "adb shell %s" % stop_cmd
        self._exec(cmd, 3)
        self.__camera_started = 0

    def __set_preference_conf_file(self):
        """
        Set Preference configuration file.
        """
        self._logger.debug("Setting configuration file begin")

        folder = "script_Camera"
        basefilename = "com.google.android.GoogleCamera_preferences.xml"
        report_dir = self._device.get_report_tree()
        report_dir.create_subfolder(folder)
        report_base = report_dir.get_subfolder_path(folder)
        filename = os.path.join(report_base, basefilename)
        self._logger.debug("MM Camera - Configuration script filename: %s " % str(filename))

        config_file = "/data/data/com.google.android.GoogleCamera/" \
                      "shared_prefs/" + basefilename

        # Create local file in the host computer before pushing to DUT
        f = open(filename, "w")
        lines = ["<?xml version=\'1.0\' encoding=\'utf-8\' standalone=\'yes\' ?>\n",
                "<map>\n",
                "    <string name=\"panorama_upgrade_version\">1</string>\n",
                "    <string name=\"pref_video_quality_back_key\">large</string>\n",
                "    <long name=\"client_first_use_time_millis\" value=\"1416494900782\" />\n",
                "    <string name=\"pref_user_selected_aspect_ratio\">1</string>\n",
                "    <string name=\"CachedSupportedPictureSizes_Sizes_Camera0\">" +
                "320,240,640,480,1024,768,1280,720,1920,1080,2048,1536,2560,1920" +
                ",3264,1836,3264,2448,4160,3104</string>\n",
                "    <string name=\"pref_upgrade_version\">6</string>\n",
                "    <string name=\"CachedSupportedPictureSizes_Build_Camera0\">" +
                "mofd_v1_64-userdebug 5.0 LRX21M imin_legacy-lst-260 dev-keys</string>\n",
                "    <string name=\"camera.startup_module\">0</string>\n",
                "    <string name=\"pref_camera_recordlocation_key\">1</string>\n",
                "    <string name=\"pref_camera_picturesize_front_key\">1920x1080</string>\n",
                "    <string name=\"pref_camera_hdr_plus_key\">0</string>\n",
                "    <string name=\"pref_camera_picturesize_back_key\">4160x3104</string>\n",
                "    <string name=\"CachedSupportedPictureSizes_Sizes_Camera1\">" +
                "320,240,640,480,1280,720,1920,1080</string>\n",
                "    <string name=\"refocus_upgrade_version\">1</string>\n",
                "    <string name=\"pref_video_quality_front_key\">large</string>\n",
                "    <string name=\"pref_camera_grid_lines\">0</string>\n",
                "    <string name=\"CachedSupportedPictureSizes_Build_Camera1\">\n" +
                "mofd_v1_64-userdebug 5.0 LRX21M imin_legacy-lst-260 dev-keys</string>\n",
                "    <string name=\"pref_flash_supported_back_camera\">1</string>\n",
                "</map>\n"]

        f.writelines(lines)
        f.close()

        # Push file to DUT
        origin = filename
        destination = config_file
        cmd = "adb push %s %s" % (origin, destination)
        self._exec(cmd, 3)

        self._logger.debug("Setting configuration file end")

    def _get_camera_name(self, camera_name):
        """
        @type camera_name: string
        @param camera_name: name of the camera

        @rtype: string
        @return: name of the component to launch
        """
        if camera_name == "camera2":
            cmd = "grep -i camera /data/system/packages.list | " \
                  "grep -v .test | grep google |" \
                  "sed 's/[[:space:]]\+/ /g' |cut -d' ' -f1"
        else:
            cmd = "grep gallery3d /data/system/packages.list | " \
                  "grep -v .test | grep -v intel | " \
                  "sed 's/[[:space:]]\+/ /g' |cut -d' ' -f1"

        exec_cmd = "adb shell %s" % cmd
        self.__camera_name = self._exec(exec_cmd, 3)

        # check response from adb command
        if "No response" in self.__camera_name:
            self.__camera_name = None
            self._logger.info("Camera %s not found in file packages.list" % camera_name)
        else:
            self._logger.info("Camera name is %s" % self.__camera_name)

    def camera_app_setup(self, camera_app, mode):
        """
        set camera defaults for device_save_directory, camera_package, camera_component
        This UECMD depends on device model to have button coordinates at get_camera_touchcmds.
        toggle_burst_mode and change_camera in the Camera UEcmd require these coordinates.

        @type camera_app: string
        @param camera_app: Camera app to be used.  Currently supports "Android_Intel_Refcam2_v0.9" and "Android_Google_Camera_v2.4"

        @type mode: string
        @param mode: mode of app to use.  Supports "video" and "camera".
        """
        self._logger.info("Setting camera api defaults - device_save_directory, camera_package, and camera_component.")
        self.camera_app = self.get_camera_version(camera_app)
        if self.camera_app == "Android_Intel_Refcam_v1.0":
            # Set touch command
            _status, self.touchCmds = self._device.get_camera_touchcmds('intel')
            # Set camera package
            self.camera_package = "com.intel.refcam"
            # Set save directories
            self.device_save_directory = ['/storage/emulated/0/DCIM/Camera/']
            # Set camera component.
            self.camera_component = ".CameraActivity"
        elif self.camera_app == "Android_Google_Camera_v2.4":
            # Set touch command
            _status, self.touchCmds = self._device.get_camera_touchcmds('google')
            # Set camera package
            self.camera_package = "com.google.android.GoogleCamera"
            self.device_save_directory = ['/storage/emulated/0/DCIM/Camera/']
            # Set camera component.
            self.camera_component = "com.android.camera.CameraLauncher"
        else:
            self._logger.error("Selected camera app: {0} is not supported".format(self.camera_app))
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Wrong camera_app name")

        if _status == Global.FAILURE:
            self._logger.error("Failed to get display information")
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Failed to get display information")

        if mode == 'camera':
            # Backup directory location
            self.backup_dir = posixpath.join(posixpath.dirname(self.device_save_directory[0]), 'pic_backup')
            self.error_dir = posixpath.join(posixpath.dirname(self.device_save_directory[0]), 'img_capture_failures')
        else:
            self.backup_dir = posixpath.join(posixpath.dirname(self.device_save_directory[0]), 'video_backup')
            self.error_dir = posixpath.join(posixpath.dirname(self.device_save_directory[0]), 'video_capture_failures')

        self._device_logger = self._device.get_device_logger()
        # Add trigger messages
        for message in self.error_messages:
            self._device_logger.add_trigger_message(self.error_messages[message])
        for message in self.action_messages:
            self._device_logger.add_trigger_message(self.action_messages[message])

        self._logger.info("Setting camera api defaults end")

    def toggle_burst_mode(self):
        """

        """
        if self.touchCmds is None:
            self._logger.error("Display resolution {0}x{1} is not supported.".format(self.disp_width, self.disp_height))
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Display resolution is not supported.")

        if self.camera_package =="com.intel.refcam" :
            self._keyevent_api.tap_on_screen(self.touchCmds['select_mode'][0], self.touchCmds['select_mode'][1])
            time.sleep(2)
            self._keyevent_api.tap_on_screen(self.touchCmds['select_burst'][0], self.touchCmds['select_burst'][1])
            time.sleep(2)
        else:
            self._logger.error("Selected camera app, {0}, is not supported.".format(self.camera_app))
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Selected camera_app is not supported. ")

    def change_camera(self, cam_to_use):
        """
        Switches the camera in use.  Currently supports com.intel.camera22 camera package only.  If 25x16 display it will select the camera
        specified by cam_to_use: 'back' is facing the world, 'front' is facing the user.  For other displays with supporting coordinates in
        touchCmds it simply toggles the camera in use, without knowledge of which one is actually being selected.

        @type cam_to_use: string
        @param cam_to_use: name of the camera to select.
        """
        if self.touchCmds is None:
            self._logger.error("Display resolution {0}x{1} is not supported.".format(self.disp_width, self_disp_height))
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Display resolution is not supported.")

        if self.camera_package == "com.intel.refcam":
            self._logger.info("Toggle intel camera")
            self._keyevent_api.tap_on_screen(self.touchCmds['change_camera'][0], self.touchCmds['change_camera'][1])
        elif self.camera_package == "com.google.android.GoogleCamera":
            self._logger.info("Toggle google camera")
            self._keyevent_api.tap_on_screen(self.touchCmds['select_settings'][0], self.touchCmds['select_settings'][1])
            time.sleep(2)
            self._keyevent_api.tap_on_screen(self.touchCmds['change_camera'][0], self.touchCmds['change_camera'][1])
            time.sleep(2)
        else:
            error_str = "Selected camera app, {0}, is not supported.".format(self.camera_app)
            self._logger.error(error_str)
            error_str = "Selected camera_app is not supported. "
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_str)

    def camera_refcam_take_picture(self, number_of_pics = 1, sleep_interval = 5):
        """
        Use input to take picture with Intel refcam
        """
        if self.camera_package =="com.intel.refcam":
            for n in range(0, number_of_pics):
                self._keyevent_api.tap_on_screen(self.touchCmds['take_picture'][0], self.touchCmds['take_picture'][1])
                time.sleep(sleep_interval)
        else:
            self._logger.error("Selected camera app, {0}, is not supported.".format(self.camera_app))
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Selected camera_app is not supported. ")

    def get_camera_in_use(self) :
        """
        @rtype: string
        @return: Camera currently in use.  "BACK" for world facing camera, "FRONT" for user facing camera and "UNKNOWN" if neither world or user facing camera.
        """
        cam_stat = "UNKNOWN"
        cmd = "adb shell dumpsys media.camera | grep 'open' | cut -d' ' -f4"
        stat = self._exec(cmd, 3)
        time.sleep(2)
        if '0' in stat:
            cam_stat = "BACK"
        elif '1' in stat:
            cam_stat = "FRONT"
        return cam_stat

    def get_camera_version(self, camera_app):
        """
        @type camera_app: string
        @param camera_app: Camera app to be used
        @rtype: string
        @return: Camera version supported by Android-M.
        """
        if camera_app == "Android_Google_Camera":
            camera_app = "Android_Google_Camera_v2.4"
        elif camera_app == "Android_Intel_Camera":
            camera_app = "Android_Intel_Refcam_v1.0"

        self._logger.info("Using camera app: %s" % camera_app)
        return camera_app
