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

:organization: INTEL MCG PSI, TMT TTD AN
:summary: camera implementation
:since: 16/12/2013
:authors: mmorchex, sasmith2, ahkhowaj
"""
from acs_test_scripts.Device.UECmd.Imp.Android.Common.BaseV2 import BaseV2
from acs_test_scripts.Device.UECmd.Interface.Multimedia.ICamera import ICamera
from acs_test_scripts.Device.UECmd.UECmdDecorator import need
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.AcsBaseException import AcsBaseException
import os
import re
import time
from datetime import datetime
import posixpath


class Camera(BaseV2, ICamera):

    """
    Class that handles all Camera operations
    """
    #This supports only Android_Intel_Camera_v2.2 app
    touchCmds_720P = {
        'change_camera': [650, 60],
        'select_options': [660, 1095],
        'select_settings': [58, 58],
        'select_burst': [370, 1090],
        'select_fast_type': [366, 930]
                    }
    touchCmds_1080P = {
        'change_camera': [1026, 57],
        'select_options': [1000, 1748],
        'select_settings': [81,94],
        'select_burst': [720, 1716],
        'select_fast_type': [530, 1551]
                   }
    touchCmds_25x16 = {
        'change_camera_b_to_f': [91, 400],
        'change_camera_f_to_b': [91, 485],
        'select_options': [85, 1116],
        'select_settings': [85, 1116],
        'select_burst': [394, 1458],
        'select_fast_type': [530, 1551]
                   }

    # Logcat messages
    error_messages = dict([
                ('camConnectionErr', ".*E/AndroidRuntime\([ 0-9]*\): FATAL EXCEPTION: main\nE/AndroidRuntime\([0-9]*\): Process: com.intel.camera22"),
                ('camStopped', ".*E/LooooooG\([ 0-9]*\): \[ CameraHolder.java:158 \] - fail to connect Camera"),
                ('camNotResponding',  ".*E/ActivityManager\([ 0-9]*\): ANR in com.intel.camera22"),
                ('camRecProcess',  "E AndroidRuntime: Process: com.intel.camera22"),
                ('camRecConnectErr', "E LooooooG: [ CameraHolder.java:158 ] - fail to connect Camera"),
                ('camRecNotRespond', "E ActivityManager: ANR in com.intel.camera22 (com.intel.camera22/.VideoCamera)"),
                ('camRecStopFail', "E LooooooG: [ VideoCamera.java:1763 ] - stop fail")
                   ])

    action_messages = dict([
                ('cameraAppStarting', 'Opening camera')
                   ])

    # Time to sleep after pressing stop recording.
    SLEEP_AFTER_STOP = 10
    # Time to sleep after issuing a tap input.
    TAP_SLEEP = 1

    @need('camera')
    def __init__(self, device):
        """
        Constructor
        """
        BaseV2.__init__(self, device)
        ICamera.__init__(self, device)

        self._logger = device.get_logger()
        self._camera_module = "acscmd.multimedia.CameraModuleActivity"

        self.camera_package = "com.android.camera2"
        self.camera_component = "com.android.camera.CameraLauncher"

        self._keyevent_api = self._device.get_uecmd("KeyEvent")
        self._phone_system = self._device.get_uecmd("PhoneSystem")
        phone_resolution = self._phone_system.get_screen_resolution()
        self.disp_width = phone_resolution.split('x', 1)[0]
        self.disp_height = phone_resolution.split('x', 1)[1]
        self.touchCmds = None
        if self.disp_width == '720':
            self.touchCmds = Camera.touchCmds_720P
        elif self.disp_width == '1080':
            self.touchCmds = Camera.touchCmds_1080P
        elif self.disp_width == '2560':
            self.touchCmds = Camera.touchCmds_25x16
        elif self.disp_width == '1200':
            self._logger.info('touch commands for display resolution {0} are included in device model and will be setup in camera API camera_app_setup.'.format(self.disp_width))
        else:
            self._logger.error('Camera API: Display resolution {0} is not supported. Some UECmds might not work.'.format(self.disp_width))

        self.backup_dir = None
        self.device_save_directory = None
        self.error_dir = None
        self.device_save_directory = None
        self.camera_app = None

        #Legacy attributes
        self.__flash_mode = None
        self.__screen_mode = None
        self.__camera_name = None
        self.__camera_started = 0

    def device_orientation(self):
        """
        Returns the device orientation
        """
        if int(self.disp_width) > int(self.disp_height):
            disp_natural_orient = "landscape"
        else:
            disp_natural_orient = "portrait"

        return disp_natural_orient

    def __close_geolocation_popup(self):
        """
        Close popup on first launch
        """
        keyevent_api = self._device.get_uecmd("KeyEvent")
        keyevent_api.scenario(["move_home", "tab", "tab", "enter"])

    def launch_camera(self, camera):
        """
        Launch camera
        :type camera: str
        :param camera: the back or front camera will be launched, should be one of('FRONT','BACK')
        :rtype: None
        :return: None
        """
        self._logger.info("Starting camera")
        camera = camera.upper()
        if camera not in ('FRONT', 'BACK'):
            self._logger.error("Launch_camera : Parameter camera %s is not valid" % camera)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Parameter camera is not valid !")
        self._logger.info("Opening Camera")
        method = "openCamera"
        cmd_args = "--es camera %s" % camera
        self._internal_exec_v2(self._camera_module, method, cmd_args)

    def launch(self):
        """
        Launch the camera
        This function is added for legacy reasons.
        """
        self._logger.info("Camera to be launched (legacy function)")

        component_name = self.__camera_name
        if "gallery3d" in self.__camera_name:
            component_name = "com.android.camera"

        # -W to wait for completion and -S to kill the camera first
        start_cmd = "am start -W -S -n %s/%s.Camera" % (self.__camera_name, component_name)
        cmd = "adb shell %s" % start_cmd
        self._exec(cmd, 3)
        self.__camera_started = 1

        time.sleep(2)

        self._logger.info("Camera launched (legacy function)")

    def set_scene_mode(self, scene_mode):
        """
        Sets the scene mode
        :type scene_mode: str
        :param scene_mode: the scene mode to set
        :rtype: None
        :return: None
        """
        scene_mode = scene_mode.lower()
        if scene_mode not in ('action', 'auto', 'barcode', 'beach', 'candlelight',
                                'fireworks', 'hdr', 'landscape', 'night', 'night-portrait',
                                'party', 'portrait', 'snow', 'steadyphoto', 'sunset', 'theatre'):
            self._logger.error("set scene mode : Parameter scene_mode %s is not valid" % scene_mode)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Parameter scene_mode is not valid !")
        self._logger.info("Setting scene mode")
        method = "setSceneMode"
        cmd_args = "--es scenemode %s" % scene_mode
        self._internal_exec_v2(self._camera_module, method, cmd_args)

    def setup(self, flash_mode, screen_mode, camera_name):
        """
        set camera preferences

        @type flash_mode: string
        @param flash_mode: mode of flash (on, off or auto)

        @type screen_mode: string
        @param screen_mode: mode of screen (WideScreen or Standard)
        @type camera_name: string
        @param camera_name: name of the camera
        """
        self._logger.info("Camera setup begin")

        if camera_name not in ("camera2", "camera"):
            self._logger.error("Invalid camera name:%s" % camera_name)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Wrong camera app name")

        self._logger.info("Flash Mode is set to is %s" % flash_mode)
        self._logger.info("Screen mode is set to %s" % screen_mode)
        self._get_camera_name(camera_name)
        if (self.__camera_name is None) or (self.__camera_name == ""):
            msg_str = "There is no camera name, probably no camera app neither. Please, check both"
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg_str)

        self.__flash_mode = flash_mode
        self.__screen_mode = screen_mode
        self.__configure()
        self._logger.info("Camera setup end")

    def camera_app_setup(self, camera_app, mode):
        """
        set camera defaults for device_save_directory, camera_package, camera_component

        @type camera_app: string
        @param camera_app: Camera app to be used.  Currently supports "Android_Intel_Camera_v2.2" and "Android_Google_Camera_v2.0"

        @type mode: string
        @param mode: mode of app to use.  Supports "video" and "camera".
        """
        self._logger.info("Setting camera api defaults - device_save_directory, camera_package, and camera_component.")
        self.camera_app = self.get_camera_version(camera_app)

        if self.camera_app == "Android_Intel_Camera_v2.2":
            # Set camera package
            self.camera_package = "com.intel.camera22"
            # Set save directories
            self.device_save_directory = []
            self.device_save_directory.append('/storage/sdcard0/DCIM/100ANDRO/')
            self.device_save_directory.append('/storage/sdcard1/DCIM/100ANDRO/')
            # Set camera component.
            if mode == 'video':
                self.camera_component = ".VideoCamera"
            elif mode == 'camera':
                self.camera_component = ".Camera"
            else:
                self._logger.error("Selected camera mode: {0} is not a supported mode".format(mode))
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Wrong camera mode type")

        elif self.camera_app == "Android_Google_Camera_v2.0":
            # Set camera package
            self.camera_package = "com.android.camera2"
            self.device_save_directory = ['/storage/emulated/legacy/DCIM/Camera/']
            # Set camera component.
            if mode == 'video':
                self.camera_component = "com.android.camera.CameraLauncher ; input tap 165 1110 ; input tap 170 970"
            elif mode == 'camera':
                self.camera_component = "com.android.camera.CameraLauncher ; input tap 165 1110 ; input tap 165 1110"
            else:
                self._logger.error("Selected camera mode: {0} is not a supported mode".format(mode))
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Wrong camera mode type")
        else:
            self._logger.error("Selected camera app: {0} is not supported".format(self.camera_app))
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Wrong camera_app name")

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

    def get_camera_in_use(self) :
        """
        @rtype: string
        @return: Camera currently in use.  "BACK" for world facing camera, "FRONT" for user facing camera and "UNKNOWN" if neither world or user facing camera.
        """
        cam_stat = "UNKNOWN"
        cmd = "adb shell dumpsys media.camera | grep 'Device is' | sed -n '1p' | cut -d' ' -f5"
        stat = self._exec(cmd, 3)
        time.sleep(2)
        if 'open' in stat:
            cam_stat = "BACK"
        elif 'closed' in stat:
            cam_stat = "FRONT"
        return cam_stat

    def toggle_burst_mode(self):
        """

        """
        if self.touchCmds is None:
            self._logger.error("Display resolution {0}x{1} is not supported.".format(self.disp_width, self.disp_height))
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Display resolution is not supported.")

        if self.camera_package =="com.intel.camera22" :
            self._keyevent_api.tap_on_screen(self.touchCmds['select_options'][0], self.touchCmds['select_options'][1])
            time.sleep(2)
            self._keyevent_api.tap_on_screen(self.touchCmds['select_burst'][0], self.touchCmds['select_burst'][1])
            time.sleep(2)
            self._keyevent_api.tap_on_screen(self.touchCmds['select_fast_type'][0], self.touchCmds['select_fast_type'][1])
            time.sleep(2)
        else:
            self._logger.error("Selected camera app, {0}, is not supported.".format(self.camera_app))
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Selected camera_app is not supported. ")

    def select_camera_settings(self):
        if self.touchCmds is None:
            self._logger.error("Display resolution {0}x{1} is not supported.".format(self.disp_width, self.disp_height))
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Display resolution is not supported.")

        if self.camera_package == "com.intel.camera22":
            self._keyevent_api.tap_on_screen(self.touchCmds['select_settings'][0], self.touchCmds['select_settings'][1])
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

        if self.camera_package == "com.intel.camera22":
            if self.disp_width == '2560':
                if cam_to_use == "back":
                    self._keyevent_api.tap_on_screen(self.touchCmds['change_camera_f_to_b'][0],
                                                     self.touchCmds['change_camera_f_to_b'][1])
                else:
                    self._keyevent_api.tap_on_screen(self.touchCmds['change_camera_b_to_f'][0],
                                                     self.touchCmds['change_camera_b_to_f'][1])
            else:
                self._keyevent_api.tap_on_screen(self.touchCmds['change_camera'][0],
                                                 self.touchCmds['change_camera'][1])
        else:
            error_str = "Selected camera app, {0}, is not supported.".format(self.camera_app)
            self._logger.error(error_str)
            error_str = "Selected camera_app is not supported. "
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_str)

    def _get_camera_name(self, camera_name):
        """
        @type camera_name: string
        @param camera_name: name of the camera

        @rtype: string
        @return: name of the component to launch
        """
        if camera_name == "camera2":
            cmd = "grep camera /data/system/packages.list | " \
                  "grep -v .test | grep -v android |" \
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

    def __configure(self):
        """
        configure camera
        """
        self._logger.info("Camera configuration begin")
        data_path = "/data/data/%s/shared_prefs/" % self.__camera_name

        #remove configuration files
        cmd = "adb shell rm -r -f %s && mkdir %s" % (data_path, data_path)
        self._device.run_cmd(cmd, 3)

        #launch camera
        self.launch()
        time.sleep(5)

        #remove geo location popup
        self.__close_geolocation_popup()
        time.sleep(5)

        #close camera
        self.legacy_shutdown()

        #set flash mode and screen mode in configuration files
        self.__write_setup_in_file()

        self._logger.info("Camera configuration end")

    def __write_setup_in_file(self):
        """
        Write setup configuration in xml file
        """
        data_path = "/data/data/%s/shared_prefs/" % self.__camera_name

        conf_filename = "%s_preferences_0_0.xml" % self.__camera_name

        cmd = "adb shell sed -i \"3i\ <string name='pref_camera_flashmode_key'>%s</string>\" %s%s" \
            % (self.__flash_mode, data_path, conf_filename)

        self._device.run_cmd(cmd, 3)

        cmd = "adb shell sed -i \"3i\ <string name='pref_camera_screenmode_key'>%s</string>\" %s%s" \
            % (self.__screen_mode, data_path, conf_filename)
        self._device.run_cmd(cmd, 3)

    def set_color_effect(self, color_effect):
        """
        Sets the color effect
        :type color_effect: str
        :param color_effect: the color effect mode to set should be one of ('aqua', 'blackboard', 'mono', 'negative',
         'posterize', 'sepia', 'solarize', 'whiteboard')
        :rtype: None
        :return: None
        """
        color_effect = color_effect.lower()
        if color_effect not in ('aqua', 'blackboard', 'mono', 'negative', 'posterize', 'sepia', 'solarize', 'whiteboard'):
            self._logger.error("set color effect : Parameter color_effect %s is not valid" % color_effect)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Parameter color_effect is not valid !")

        self._logger.info("Setting color effect")
        method = "setColorEffect"
        cmd_args = "--es coloreffect %s" % color_effect
        self._internal_exec_v2(self._camera_module, method, cmd_args)

    def set_flash_mode(self, flash_mode):
        """
        Sets the flash mode
        :type flash_mode: str
        :param flash_mode: the flash mode to set should be one of ('auto', 'off', 'on', 'torch', 'red-eye')
        :rtype: None
        :return: None
        """
        flash_mode = flash_mode.lower()
        if flash_mode not in ('auto', 'off', 'on', 'torch', 'red-eye'):
            self._logger.error("set flash mode : Parameter flash_mode %s is not valid" % flash_mode)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Parameter flash_mode is not valid !")
        self._logger.info("Setting flash mode")
        method = "setFlashMode"
        cmd_args = "--es flashmode %s" % flash_mode
        self._internal_exec_v2(self._camera_module, method, cmd_args)

    def set_white_balance(self, white_balance):
        """
        Sets the white balance
        :type white_balance: str
        :param white_balance: the white balance mode to set should be one of ('auto', 'cloudy-daylight', 'daylight',
        'fluorescent', 'incandescent','shade','twilight','warm-fluorescent')
        :rtype: None
        :return: None
        """
        white_balance = white_balance.lower()
        if white_balance not in ('auto', 'cloudy-daylight', 'daylight', \
                                 'fluorescent', 'incandescent', 'shade', 'twilight', 'warm-fluorescent'):
            self._logger.error("set white balance : Parameter white_balance %s is not valid" % white_balance)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Parameter white_balance is not valid !")
        self._logger.info("Setting white balance")
        method = "setWhiteBalance"
        cmd_args = "--es whitebalance %s" % white_balance
        self._internal_exec_v2(self._camera_module, method, cmd_args)

    def set_exposure_mode(self, exposure_value):
        """
        Sets exposure mode value
        :type exposure_value: int
        :param exposure_value: exposure mode value to set
        :rtype: None
        :return: None
        """
        self._logger.info("Setting exposure mode to :" + str(exposure_value))
        actual_value, maximum_value, minimum_value = self.get_exposure_mode_values()
        if exposure_value > maximum_value or exposure_value < minimum_value:
            error_msg = "exposure value should be =< %s and => %s" % (str(maximum_value), str(minimum_value))
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)
        elif exposure_value == actual_value:
            error_msg = " exposure mode already set to %s" % str(actual_value)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)
        else:
            method = "setExposureMode"
            cmd_args = "--ei exposure %s" % exposure_value
            self._internal_exec_v2(self._camera_module, method, cmd_args)

    def get_exposure_mode_values(self):
        # This is a default implementation.
        """
        Get the actual, maximum and minimum exposure value
        :rtype: tuple
        :return: A tuple containing exposure value of the DUT (actual exposure, maximum exposure, minimum exposure)
        """
        self._logger.info("Getting exposure mode values : actual , munimum and maximum value")
        method = "getExposureModeDetails"
        output = self._internal_exec_v2(self._camera_module, method)
        exposure_actual_value = int(output['exposure_actual_value'])
        exposure_maximum_value = int(output['exposure_maximum_value'])
        exposure_minimum_value = int(output['exposure_minimum_value'])

        return exposure_actual_value, exposure_maximum_value, exposure_minimum_value

    def set_picture_size(self, width, height):
        """
        Sets picture size
        :type width: int
        :param width: picture width to set
        :type height: int
        :param height: picture height to set
        :rtype: None
        :return: None
        """
        self._logger.info("Setting picture size")
        method = "setPictureSize"
        cmd_args = "--ei width %s --ei height %s" % (width, height)
        self._internal_exec_v2(self._camera_module, method, cmd_args)

    def take_picture(self, save_directory, pictures_number=1):
        """
        Take picture
        :type save_directory : str
        :param save_directory: directory where to save picture taken on the DUT
        :type pictures_number: int
        :param pictures_number: the number of picture to take in case of burst mode, if picture_number > 1 , we are in
        burst mode.
        :rtype: str
        :return: MediaFile uri
        """
        # check picture number
        if pictures_number < 1:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "mode should be => 1")
        self._logger.info("Taking picture")
        method = "takePicture"
        cmd_args = "--es savedirectory %s --ei picturesnumber %s" % (save_directory, str(pictures_number))
        output = self._internal_exec_v2(self._camera_module, method, cmd_args)
        return output["picture_uri"]

    def record_video(self, duration, save_directory, mode=0, resolution=4):
        """
        Record a video
        :type duration: int
        :param duration: duration of the video to record
        :type save_directory : str
        :param save_directory: directory where to save video recorded on the DUT
        :type mode: int
        :param mode: when mode is 1, picture is taken while video recording.
        :type resolution: int
        :param resolution: resolution of the video to record
        :rtype: str or tuple
        :return: Video recorded uri or video recorded and picture file uri
        """
        recording_output = ""

        # check mode
        if mode not in (0, 1):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "mode should be 1 or 0")

        self._logger.info("Video record")
        method = "videoRecord"
        cmd_args = "--ei duration %s --es savedirectory %s --ei mode %s --ei resolution %s"\
                   % (duration, save_directory, str(mode), str(resolution))
        output = self._internal_exec_v2(self._camera_module, method, cmd_args)
        if mode == 0:
            recording_output = output["video_uri"]
        else:
            recording_output = output["video_uri"], output["picture_uri"]

        return recording_output

    def shutdown(self):
        """
        Stop the camera
        :rtype: None
        :return: None
        """
        self._logger.info("Releasing camera")
        method = "releaseCamera"
        self._internal_exec_v2(self._camera_module, method)

    def legacy_shutdown(self):
        """
        Stop the camera
        """
        stop_cmd = "am force-stop %s" % self.__camera_name
        cmd = "adb shell %s" % stop_cmd
        self._exec(cmd, 3)
        self.__camera_started = 0

    def download_media_file(self, remote_file_uri, local_file_uri):
        """
        download media file from DUT to host
        :type remote_file_uri: str
        :param remote_file_uri: uri of media file uri on the DUT
        :rtype: str
        :return: media file uri on host
        """
        self._logger.info("Downloading media file from DUT")
        # Create folder if it doesn't exist
        report_path = self._device.get_report_tree().get_report_path()
        local_file_uri = os.path.join(report_path, local_file_uri)
        if not os.path.exists(local_file_uri):
            os.makedirs(local_file_uri)
        # Copy picture from DUT to host
        local_picture = os.path.join(local_file_uri, remote_file_uri.split('/')[-1])
        cmd = "adb pull %s %s" % (remote_file_uri, local_picture)
        self._exec(cmd)
        return local_picture

    def upload_output_files(self, include_error_dir, local_file_dir, timeout = 180):
        """
        upload output files from DUT backup directory to host and removes files from DUT directory.
        :type include_error_dir: boolean
        :param include_error_dir: if true, upload error_dir as well
        :rtype: str
        """
        self._logger.info("Uploading backup directory's files from DUT")
        if not os.path.exists(local_file_dir):
            try:
                os.makedirs(local_file_dir)
            except OSError as e:
                self._logger.error("upload_output_files failed to create directory: {0}".format(local_file_dir))
                raise
        self._device.pull(self.backup_dir, local_file_dir, timeout)
        cmd = "adb shell rm -rf " + self.backup_dir
        self._exec(cmd)
        if include_error_dir:
            cmd = 'adb shell "if [ ! -d %s ]; then mkdir %s; fi"'%(self.error_dir, self.error_dir)
            self._exec(cmd)
            self._logger.info("Uploading error directory's files from DUT")
            self._device.pull(self.error_dir, local_file_dir, timeout)
            cmd = "adb shell rm -rf " + self.error_dir
            self._exec(cmd)

    def launch_media_scanner(self, save_directory):
        """
        force media scanner
        :type save_directory: str
        :param save_directory : directory where media files are saved in DUT
        :rtype: None
        :return: None
        """
        cmd = "adb shell am broadcast -a android.intent.action.MEDIA_MOUNTED -d file:///mnt/%s/AcsAgent/" \
              % save_directory
        self._exec(cmd)

    def launch_system_camera_application(self, checkTriglogMsg = False, reset = False):
        """
        Launch camera application.

        :rtype: None
        :return: None
        """

        self._logger.info("Launch camera application")

        # grant runtime permissions to the camera app
        self._device.grant_runtime_permissions(self.camera_package)

        if checkTriglogMsg is True:
            # Reset trigger message first in case it has already occurred.
            self._device_logger.reset_trigger_message(self.action_messages['cameraAppStarting'])
            # Launch camera application
            if reset is True:
                self._exec("adb shell am start -S -a android.intent.action.MAIN -n " + posixpath.join(self.camera_package, self.camera_component))
            else:
                self._exec("adb shell am start -a android.intent.action.MAIN -n " + posixpath.join(self.camera_package, self.camera_component))

            # Wait for 2 second to for status update
            time.sleep(2)
            startMsg = self._device_logger.get_message_triggered_status(self.action_messages['cameraAppStarting'])
            if len(startMsg) == 0:
                self._logger.error("Unable to start the camera app.")
                timestamp = datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d_%H-%M-%S')
                self._exec("adb shell screencap -p  " + posixpath.join(self.error_dir, "screen_startfail_%s.png"%timestamp))
            else:
                self._logger.info("Camera application started...")
        else:
            if reset is True:
                self._exec("adb shell am start -S -a android.intent.action.MAIN -n " + posixpath.join(self.camera_package, self.camera_component))
            else:
                self._exec("adb shell am start -a android.intent.action.MAIN -n " + posixpath.join(self.camera_package, self.camera_component))

    def stop_system_camera_application(self):
        """
        Stop the camera application
        """
        self._logger.info("Stopping camera application")
        self._exec("adb shell am force-stop " + self.camera_package)

    def camera_application_take_picture(self, number_of_pics = 1, sleep_interval = 5):
        """
        Use keyevent to take picture with camera application
        """
        for n in range(0, number_of_pics):
            self._exec("adb shell input keyevent 27")
            time.sleep(sleep_interval)

    def camera_application_start_stop_recording(self, record_time = 5):
        """
        Use keyevent to make recording with camera application
        """
        self._exec("adb shell input keyevent 27")
        time.sleep(record_time)
        self._exec("adb shell input keyevent 27")
        # Can take extra time before recording actually stops.
        self._logger.info("Sleeping {0} more seconds after pressing stop recording to make sure process is done.".format(self.SLEEP_AFTER_STOP))
        time.sleep(self.SLEEP_AFTER_STOP)

    def get_focused_input_window_name(self):
        """
           Obtains the name of the window that currently has input focus.
        """
        system_api = self._device.get_uecmd("System")
        dumpsys_output = system_api.get_dumpsys_focus()
        re_match = re.match(".* (.*/.*)}", dumpsys_output)
        if (re_match is None) or (len(re_match.groups()) < 1):
            self._logger.debug('Unable to find a app/activity name in the dumpsys input output: '+dumpsys_output)
            return ''

        return re_match.group(1)

    def move_files_to_backup_dir(self, filepaths, backup_directory = None):
        """
        Moves the files in the filepath to the directory
        :type filepaths: str list
        :param filepaths: list of path where the files to be moved are
        :type backup_directory: string
        :param backup_directory: Optional.  If passed in, use this path to move the files to.
        :rtype: str
        :return: number of files found in backup_directory
        """
        if backup_directory is None:
            backup_directory = self.backup_dir
        cmd_list = list([])
        cmd_list.append('adb shell "if [ ! -d %s ]; then mkdir %s; fi"'%(backup_directory, backup_directory))

        for directory in filepaths:
            cmd_list.append('adb shell "if [ ! -d %s ]; then mkdir %s; fi"'%(directory, directory))
            self._logger.info("Moving file directory ({0}) to backup directory ({1})".format(directory, backup_directory))
            # Add command to copy files from directory to backup_directory
            cmd_list.append("adb shell cp " + directory + posixpath.sep + '* ' + backup_directory)
            # Add command to remove files in filepath
            cmd_list.append("adb shell rm " + directory + posixpath.sep + '*')

        for cmd in cmd_list:
            self._exec(cmd)
        (return_code, output) = self._device.run_cmd(cmd = "adb shell ls " + backup_directory + " | wc -l", timeout = 20)

        return output

    def check_for_camera_issues(self, errorCount = None):
        """
        Check if a known camera issue has occurred
        Increment the errorCount dictionary, if provided, and return one of the
        following codes:

        unclassifiedFail - Unclassified error
        camConnectionErr - "Unfortunately, camera has stopped"
        camStopped       - "Can't connect to the camera."
        camNotResponding - "Camera isn't responding. Do you want to close it?"
        """
        timestamp = datetime.fromtimestamp(time.time()).strftime('%Y_%m_%d_%H_%M_%S')

        adbCmd = 'adb shell "if [ ! -d %s ]; then mkdir %s; fi"' % (self.error_dir, self.error_dir)
        self._device.run_cmd(cmd = adbCmd, timeout = 20)
        # Check for any of the known camera errors
        for k, v in self.error_messages.iteritems():
            msg = self._device_logger.get_message_triggered_status(v)
            if msg is not None:
                if len(msg) != 0:
                    # Message was triggered
                    adbCmd = "adb shell screencap -p " + posixpath.join(self.error_dir, "screen_%s_%s.png" % (self.error_messages[k], timestamp))
                    self._device.run_cmd(cmd = adbCmd, timeout = 20)
                    self._device_logger.reset_trigger_message(v)
                    if errorCount is not None:
                        if k not in errorCount:
                            errorCount[k] = 1
                        else:
                            errorCount[k] += 1
                    return self.error_messages[k]

        # If none of the known errors were detected above, this is an unclassified error
        adbCmd = "adb shell screencap -p " + posixpath.join(self.error_dir, "screen_unclassifiedFail_%s.png" % timestamp)
        self._device.run_cmd(cmd = adbCmd, timeout = 20)
        if errorCount is not None:
            if 'unclassifiedFail' not in errorCount:
                errorCount['unclassifiedFail'] = 1
            else:
                errorCount['unclassifiedFail'] += 1

        return 'unclassifiedFail'

    def start_face_detection(self):
        """
        Starts the face detection
        :rtype: int
        :return: number of faces detected
        """
        self._logger.info("Start face detection")

        method = "faceDetection"
        try:
            result = self._internal_exec_v2(self._camera_module, method)
            faces_number = int(result['faces'])
        except:
            msg = "No face detected"
            raise AcsBaseException(AcsBaseException.OPERATION_FAILED, msg)

        return faces_number

    def get_camera_current_parameters(self):
        """
        Returns the current camera parameters configuration from the DUT.
        :rtype: dictionary
        :return: a map with current camera parameters with parameter:value
        """
        self._logger.info("Ask DUT for camera parameters.")

        method = "getCurrentCameraParameters"
        try:
            result = self._internal_exec_v2(self._camera_module, method)
            camera_current_params = {}
            param_list = re.split(";", result['output'])
            for param in param_list:
                if param:
                    setting = param.split("=")[0]
                    value = param.split("=")[1]
                    camera_current_params[setting] = value
        except:
            msg = "Could not get current camera parameters."
            raise AcsBaseException(AcsBaseException.OPERATION_FAILED, msg)

        return camera_current_params

    def take_raw_image_no_preview(self, v4l2n_path, download_dir, width, height):
        """
        Takes a picture without preview using V4L2n, and this requires v4l2n to be installed.
        v4l2n can be found in Artifactory at acs_test_artifacts/CONCURRENCY/TESTS/nopreview_imaging.
        Project Home can be found at https://teamforge-amr-01.devtools.intel.com/sf/projects/v4l2n/
        :type v4l2n_path: string
        :param v4l2n_path: path to the v4l2n binary, with filename
        :type download_dir: string
        :param download_dir: directory to download the captured picture
        :type width: int
        :param width: picture width to set
        :type height: int
        :param height: picture height to set
        :rtype: None
        :return: None
        """

        captured_file_name = "testimage_000.raw"

        cmd = "adb shell rm /cache/{0}".format(captured_file_name)
        self._exec(cmd)

        cmd = "adb shell {0} -o /cache/testimage_@.raw " \
              "--device /dev/video0 --input 0 --parm type=1,capturemode=CI_MODE_STILL_CAPTURE " \
              "--cvf_parm=0,0,0 --fmt type=1,width={1},height={2},pixelformat=SGRBG10 " \
              "--reqbufs count=2,memory=USERPTR --exposure=5400,0,0,256 --capture=1".format(v4l2n_path, width, height)
        self._exec(cmd)

        cmd = "adb pull /cache/{0} {1}".format(captured_file_name, download_dir)
        self._exec(cmd)

        return captured_file_name

    def prepare_video_capture(self, error_counts = {}, restart_app = False, checkTriglogMsg = True):
        """
        Launch and make sure we have correct camera package in focus.  Log errors into error_counts as encountered.

        :type error_counts: dict
        :param error_counts: dictionary of errors encountered and number of each occurrence.
        :type restart_app: bool
        :param restart_app: True if we will restart app, False if not.
        :type checkTriglogMsg: bool
        :param checkTriglogMsg: True if we want to reset triglog messages to be checked later, otherwise set to False.
        :rtype: tuple (int, reset_loop (bool))
        :return: int - Returns 0 if known issue or no error occurs, -1 if 'unclassifiedFail'.  bool - return True or False.
        """
        # Launch the camera app.
        self.launch_system_camera_application(checkTriglogMsg, reset = restart_app)
        # Wait for the app to have the input focus
        focused_window_name = self.get_focused_input_window_name()
        wait_focus_start_time = time.time()
        reset_loop = False
        while self.camera_package not in focused_window_name and self.camera_component not in focused_window_name:
            if time.time()-wait_focus_start_time > 60:
                self._logger.error("The camera app took longer than 60 seconds to obtain input focus.")
                error_hit = self.check_for_camera_issues(errorCount=error_counts)
                if error_hit != "unclassifiedFail":
                    # We hit a known issue. We will set reset_loop true to retry this loop iteration since it was problem starting and gaining focus.
                    self._logger.info("Known issue hit (does not fail test): "+str(error_hit))
                    reset_loop = True
                    break
                return (-1, reset_loop)
            focused_window_name = self.get_focused_input_window_name()
            if 'com.google.android.googlequicksearchbox' in focused_window_name:
                self._logger.info("Google QuickSearch issue occurred. Stopping camera app and trying again.")
                if 'Google QuickSearch Appeared' not in error_counts:
                    error_counts['Google QuickSearch Appeared'] = 1
                else:
                    error_counts['Google QuickSearch Appeared'] += 1
                reset_loop = True
                break
        if reset_loop:
            self.stop_system_camera_application()
            return (0, reset_loop)
        self._logger.debug("Time for camera app to obtain input focus: %.02f seconds"%(time.time()-wait_focus_start_time))
        return (0, reset_loop)

    def verify_video_creation(self, error_counts = {}, videos_saved = 0):
        """
        Checks to see if more videos have been created compared to current count of videos_saved.  If not, checks for errors
        to reset loop if known error has occurred or fail test if unclassifiedFail has occurred.

        :type error_counts: dict
        :param error_counts: dictionary of errors encountered and number of each occurrence.
        :type videos_saved: int
        :param videos_saved: Number of videos saved in backup directory before backing up anything new.
        :rtype: tuple - (video_found (bool), reset_loop (bool), videos_saved (int))
        :return: video_found -> True if more videos found in backup directory than videos_saved, False if not.
                 reset_loop -> True if known error has occurred and False otherwise.
                 videos_saved -> Number of videos saved in backup directory after moving any new files.
        """
        attempts_remaining = 3
        video_found = False
        reset_loop = False
        while ( attempts_remaining > 0): # Verification loop looking for new video file
            retVal = int(self.move_files_to_backup_dir(self.device_save_directory))
            if (retVal - videos_saved) > 0:
                self._logger.info('Moved {0} videos.'.format(retVal - videos_saved))
                video_found = True
                # Record number of video files
                videos_saved = retVal
                break
            else:
                self._logger.info('No video file found')
            attempts_remaining -= 1
            if (attempts_remaining == 0):
                self._logger.info("No video file was found. Collecting debug data and checking if it's a real failure.")
                error_hit = self.check_for_camera_issues(errorCount=error_counts)
                if error_hit != "unclassifiedFail":
                    # We hit a known issue.  We will not fail test.
                    self._logger.info("Known issue hit (does not fail test): "+str(error_hit))
                    reset_loop = True
                    break
                # We didn't hit known issue.
                self._logger.error("Logcat output does not match any known issues. Failing test.")
                return (video_found, reset_loop)
            self._logger.info('Waiting 5 seconds before checking again.')
            time.sleep(5)
        return (video_found, reset_loop, videos_saved)

def get_camera_version(self, camera_app):
        """
        @type camera_app: string
        @param camera_app: Camera app to be used
        @rtype: string
        @return: Camera version supported by Android-KK.
        """
        if camera_app == "Android_Google_Camera":
            camera_app = "Android_Google_Camera_v2.0"
        elif camera_app == "Android_Intel_Camera":
            camera_app = "Android_Intel_Camera_v2.2"

        self._logger.info("Using camera app: %s" % camera_app)
        return camera_app
