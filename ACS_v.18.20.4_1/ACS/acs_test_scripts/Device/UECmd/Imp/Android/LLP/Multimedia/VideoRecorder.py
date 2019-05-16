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
:summary: This file implements UECmds for video playback for Android JB device
:since: 10 dic 2014
:author: imoren2x
"""

import time
import os
import re
import tempfile
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.Device.UECmd.UECmdDecorator import need
from acs_test_scripts.Device.UECmd.Imp.Android.Common.Multimedia.VideoRecorder import VideoRecorder as VideoRecorderCommon
from _testcapi import raise_exception


class VideoRecorder(VideoRecorderCommon):
    """
    Class that handle all video recording operations
    """

    camera_packages = {
        'camera': 'com.android.camera',
        'testcamera': 'com.android.testcamera',
        'camera_google': 'com.google.android.GoogleCamera'
    }

    def __init__(self, device):
        """
        Constructor
        """
        VideoRecorderCommon.__init__(self, device)

        # add try catch to avoid crashing other people code due to a missing CameraModule path on their device model
        self.CAMERA_TAG = {}
        try:
            self._camera_module = self._device.get_device_module("CameraModule")
            self._camera_module.init()
            self.CAMERA_TAG = self._camera_module.camera_properties.CAMERA_TAG
        except AcsConfigException as e:
            self._logger.warning("fail to load camera module, some uecmds may be impacted :%s" % str(e))
            self._camera_module = None

        self._device = device
        self.__existing_files = None
        self.__camera_name = None
        self.__recorded_video_file_name = None
        self.__quality = None

    def __configure(self):
        """
        configure camera
        """
        self._logger.info("Camera configuration begin")

        # launch camera
        self.launch()

        # remove geo location popup
        self.__close_geolocation_popup()

        self._logger.info("Camera configuration end")

    @need('camera')
    def native_video_record(self,
                            video_path,
                            camera_name,
                            camera,
                            quality,
                            flash_mode,
                            color_effect,
                            white_balance,
                            dvs,
                            noise_reduction):
        """
        Start the video record

        :type video_path: str
        :param video_path: Path to video file to play
        :type camera_name: str
        :param camera_name: which camera app to use
        :type camera: str
        :param camera: which camera to user, could be front or back
        :type quality: str
        :param quality: quality of the record, could be high, high1080p and etc
        :type flash_mode: str
        :param flash_mode: flash mode on or off
        :type color_effect: str
        :param color_effect: which color effect to apply, could be mono, auto and etc
        :type white_balance: str
        :param white_balance: white balance setting, could be auto or etc
        :type dvs: str
        :param dvs: dvs set to true or false
        :type noise_reduction: str
        :param noise_reduction: enable noise reduction, could be on or off

        :rtype: String
        :return: The created video filename
        """
        self.__camera_name = camera_name
        filename = None

        if camera_name == "camera_google":
            filename = self.__native_video_record_google(video_path, camera_name, camera, quality,
                                                         flash_mode, color_effect, white_balance,
                                                         dvs, noise_reduction)
            self._logger.debug("Google camera filename: %s" % filename)
        else:
            filename = VideoRecorderCommon.native_video_record(self, video_path, camera_name, camera,
                                                               quality, flash_mode, color_effect,
                                                               white_balance, dvs, noise_reduction)
        return filename

    @need('camera')
    def __native_video_record_google(self,
                            video_path,
                            camera_name,
                            camera,
                            quality,
                            flash_mode,
                            color_effect,
                            white_balance,
                            dvs,
                            noise_reduction):
        """
        Start the video record

        :type video_path: str
        :param video_path: Path to video file to play
        :type camera_name: str
        :param camera_name: which camera app to use
        :type camera: str
        :param camera: which camera to user, could be front or back
        :type quality: str
        :param quality: quality of the record, could be high, high1080p and etc
        :type flash_mode: str
        :param flash_mode: flash mode on or off
        :type color_effect: str
        :param color_effect: which color effect to apply, could be mono, auto and etc
        :type white_balance: str
        :param white_balance: white balance setting, could be auto or etc
        :type dvs: str
        :param dvs: dvs set to true or false
        :type noise_reduction: str
        :param noise_reduction: enable noise reduction, could be on or off

        :rtype: String
        :return: The created video filename
        """
        self.__quality = quality
        self._logger.info("LLP native_video_record Google start")

        # stop camera in case of one is already running
        self.__shutdown()
        time.sleep(2)

        # cleanup the record directory
        self._logger.info("Delete old video files (MP4)")
        self._exec("adb shell rm " + video_path + "*.mp4 ")
        self._logger.info("Delete old video files (3GPP)")
        self._exec("adb shell rm " + video_path + "*.3gp ")
        self._logger.info("Delete old picture files")
        self._exec("adb shell rm " + video_path + "*.jpg ")

        # Set Preference configuration file
        self.__set_preference_conf_file_google()
        time.sleep(1)

        # Get list of existing files
        self.__existing_files = self.__get_existing_video_files(video_path)

        # grant runtime permissions to the camera app
        self._device.grant_runtime_permissions("com.google.android.GoogleCamera")

        # Launch video recorder
        self._logger.debug("LLP start video camera google")
        start_cmd = "am start -W -S -n com.google.android.GoogleCamera/com.android.camera.CameraLauncher"
        cmd = "adb shell %s" % start_cmd
        self._exec(cmd, 3)
        self._logger.debug("LLP Google video camera started")
        time.sleep(5)
        # start the recording by pressing the camera key
        self._logger.info("Start recording by pressing ENTER...")
        cmd_str = "adb shell input keyevent 122"
        self._exec(cmd_str, 5)
        time.sleep(2)
        self._logger.info("Start recording by pressing ENTER...")
        cmd_str = "adb shell input keyevent 27"
        self._exec(cmd_str, 5)
        # starting twice to be sure that the record has started
        time.sleep(2)
        self._exec(cmd_str, 5)
        time.sleep(2)
        self._logger.info("LLP Google video camera recording...")

        # Get the recorded file
        full_path = None
        filename = self.__get_recorded_file_name(video_path)
        if (type(filename) is str) and (len(filename) > 0):
            self.__recorded_video_file_name = filename
            full_path = video_path + filename
        else:
            txt = "LLP native_video_record Google : Cant find where the video has been stored!"
            self._logger.error(txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, txt)
        self._logger.info("LLP native_video_record Google end")
        return full_path

    def __get_existing_video_files(self, video_path):
        """
        Retrieves a list of existing video files as .mp4.tmp which
            includes current file under recording state.
        """
        cmd_str = "adb shell ls %s" % video_path
        (ret_code, file_list) = self._device.run_cmd(cmd_str, 10)

        self._logger.debug("Files in folder %s: %s" % (video_path, file_list))

        # RegEx: VID_20141210_175246.mp4.tmp
        file_regex_str = r"VID_(?P<date>\d{8})_(?P<hour>\d{6})\.mp4\.tmp"

        iterator = re.finditer(file_regex_str, file_list)
        output_list = [element.group(0) for element in iterator]

        # RegEx: VID_20141210_175246.3gp.tmp
        file_regex_str = r"VID_(?P<date>\d{8})_(?P<hour>\d{6})\.3gp\.tmp"

        iterator = re.finditer(file_regex_str, file_list)
        output_list += [element.group(0) for element in iterator]

        self._logger.debug("Output files in folder %s: %s" % (video_path, output_list))

        return output_list

    def __get_recorded_file_name(self, video_path):
        """
        It compares the list of .mp4.tmp files to take the
            file to be recorded.
        """
        file_list = self.__get_existing_video_files(video_path)

        filename_list = list(set(file_list) - set(self.__existing_files))

        # Prevention of wrong cases
        if len(filename_list) == 0:
            self._logger.warning("No video file created. Continue.")
            return ""
        elif len(filename_list) > 1:
            self._logger.warning("file list: %s" % str(filename_list))
            self._logger.warning("System can't extract which one is user_mode_image_capture \
                                 pid. Continue.")
            return filename_list[0]

        filename = filename_list[0]

        regex_str = "(?P<main>.*?)\.tmp"
        filename_output = re.match(regex_str, filename).group("main")
        self._logger.debug("Video recorded filename: %s" % str(filename_output))

        return filename_output

    def __shutdown(self):
        """
        Stop the camera
        """
        stop_cmd = "am force-stop com.google.android.GoogleCamera"
        cmd = "adb shell %s" % stop_cmd
        self._exec(cmd, 3)

    def __set_preference_conf_file_google(self):
        """
        Set Preference configuration file.
        """
        self._logger.debug("Setting configuration file begin")

        folder = "script_VideoRecorder"
        basefilename = "com.google.android.GoogleCamera_preferences.xml"
        report_dir = self._device.get_report_tree()
        report_dir.create_subfolder(folder)
        report_base = report_dir.get_subfolder_path(folder)
        filename = os.path.join(report_base, basefilename)
        self._logger.debug("LLP Camera - Configuration script filename: %s " % str(filename))

        config_file = "/data/data/com.google.android.GoogleCamera/" \
                      "shared_prefs/" + basefilename

        # Create local file in the host computer before pushing to DUT
        f = open(filename, "w")
        lines = ["<?xml version=\'1.0\' encoding=\'utf-8\' standalone=\'yes\' ?>\n",
                "<map>\n",
                "    <string name=\"pref_camera_exposure_compensation_key\">0</string>\n",
                "    <string name=\"pref_lightcycle_quality_key\">hq</string>\n",
                "    <string name=\"pref_video_quality_back_key\">large</string>\n",
                "    <string name=\"CachedSupportedPictureSizes_Sizes_Camera0\">" +
                "320,240,640,480,1024,768,1280,720,1280,960,1920,1080,2048,1536," +
                "2560,1920,3264,1836,3264,2448,4096,3072</string>\n",
                "    <string name=\"pref_upgrade_version\">6</string>\n",
                "    <string name=\"pref_should_show_settings_button_cling\">0</string>\n",
                "    <string name=\"CachedSupportedPictureSizes_Build_Camera0\">" +
                "mofd_v1_64-userdebug 5.0 LRX21V imin_legacy-lst-290 dev-keys</string>\n",
                "    <string name=\"pref_video_quality_front_key\">%s</string>\n" % self.__quality,
                "    <string name=\"pref_camera_grid_lines\">0</string>\n",
                "    <string name=\"panorama_upgrade_version\">1</string>\n",
                "    <long name=\"client_first_use_time_millis\" value=\"1417624141504\" />\n",
                "    <string name=\"pref_user_selected_aspect_ratio\">1</string>\n",
                "    <string name=\"camera.startup_module\">1</string>\n",
                "    <string name=\"pref_camera_picturesize_front_key\">1280x960</string>\n",
                "    <string name=\"pref_camera_recordlocation_key\">1</string>\n",
                "    <string name=\"pref_refocus_quality_key\">hq</string>\n",
                "    <string name=\"CachedSupportedPictureSizes_Sizes_Camera1\">" +
                "320,240,640,480,1280,720,1280,960</string>\n",
                "    <string name=\"pref_camera_picturesize_back_key\">4096x3072</string>\n",
                "    <string name=\"pref_camera_hdr_plus_key\">0</string>\n",
                "    <string name=\"refocus_upgrade_version\">1</string>\n",
                "    <string name=\"CachedSupportedPictureSizes_Build_Camera1\">" +
                "mofd_v1_64-userdebug 5.0 LRX21V imin_legacy-lst-290 dev-keys</string>\n",
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

    @need('camera')
    def stop_video_record(self, video_path, video_file_name):
        """
        Stop the video record

        :type video_path: str
        :param video_path: Path to video file to play
        :type video_file_name: str
        :param video_file_name: Record video in continuous mode or not

        :return: None
        """
        if self.__camera_name == "camera_google":
            self.__stop_video_record_google(video_path, video_file_name)
        else:
            VideoRecorderCommon.stop_video_record(self, video_path, video_file_name)

    @need('camera')
    def __stop_video_record_google(self, video_path, video_file_name):
        # stop recording by pressing the camera key
        self._logger.info("Stop recording LLP ...")
        self._exec("adb shell input keyevent 66")
        time.sleep(5)

        # go back to home screen
        # self._logger.info("Go back to home LLP ...")
        # self._exec("adb shell input keyevent 4")
        # self._exec("adb shell input keyevent 3")

        self._logger.info("Force stop")
        stop_cmd = "am force-stop com.google.android.GoogleCamera"
        cmd = "adb shell %s" % stop_cmd
        self._exec(cmd, 3)

        # rename the recorded file
        self._logger.info("Rename the recorded file LLP ...")

        origin_video_file = video_path + self.__recorded_video_file_name
        self._exec("adb shell cp " + origin_video_file + " " + video_file_name)

    #-------------------uecmd above are legacy uecmds -----------------------------------#

    def clean_video_storage(self, camera="default"):
        """
        empty the video storage

        :type camera: str
        :param camera: the camera package name, like "camera2", "camera_google"
        """
        # cleanup the record directory
        self._logger.info("Delete old video files")
        self.__check_camera_module()
        store_path = self._camera_module.camera_properties.video_storage_path.get(camera)
        if store_path is None and camera != "default":
            self._logger.info("no specific storage path set for %s, taking the default one" % camera)
            store_path = self._camera_module.camera_properties.video_storage_path.get("default")
        # may be enhanced to include file type to avoid removing important file
        cmd = "adb shell rm %s/*" % store_path
        self._exec(cmd.replace("//", "/", 1), raise_error=False)

    def __check_camera_module(self):
        if self._camera_module is None:
            error = "no camera module found , you need to edit the device model in use to define one!"
            self._logger.error(error)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error)

    @need('camera')
    def setup_camera(self, camera="default", back_quality=None, skip_wizard=None):
        """
        Setup the camera
        :type camera: str
        :param camera: the camera package name, like "camera2", "camera_google"

        :type back_quality: str
        :param back_quality: the back camera quality to be choose between : 1080P, 720P, 480P, LOW (lowest supported) , MAX (max supported)

        :type skip_wizard: str
        :param skip_wizard: some camera got a wizard the first time you open ,this allow to skip it, can be ON or OFF
        """
        self._logger.debug("Setup camera")
        if back_quality is None and skip_wizard is None:
            self._logger.info("no camera setting has been specified, skip the uecmd")
            return

        self.__check_camera_module()
        camera = self._camera_module.get_camera(camera, raise_error=False)
        error = ""
        # try to get the default way to stop the camera
        pref_setting = self._camera_module.camera_properties.prefs_setting_path.get(camera)
        if pref_setting is None:
                error = "there is no default key on CameraModule prefs_setting_path entry"
                self._logger.error(error)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error)

        # Grant runtime permissions
        camera_pkg = self._camera_module.camera_properties.camera_package.get(camera)
        self._device.grant_runtime_permissions(camera_pkg)
        # check that the file exist
        file_api = self._device.get_uecmd("File", True)
        if not file_api.exist(pref_setting)[0]:

            # try to open then close the camera to force the file to be generated
            # Launch video recorder
            self._logger.debug("Open video view to force preference settings generation")
            self._open_camera_view(camera)
            # wait some sec to let camera view animation finish
            time.sleep(5)

            if not file_api.exist(pref_setting)[0]:
                txt = "no pref settings found for camera %s at location %s" % (camera, pref_setting)
                self._logger.error(txt)
                raise DeviceException(DeviceException.OPERATION_FAILED, txt)

        # force the camera to close before editing the setup
        self.force_stop(camera)
        # load pref file
        tmpd = tempfile.gettempdir()
        tmp_file = "%s_temp_prefs_file.xml" % camera
        tmp_file = os.path.join(tmpd, tmp_file)
        cmd = "adb pull %s %s" % (pref_setting, tmp_file)
        output = self._exec(cmd, force_execution=True)
        if not self.is_shell_output_ok(output):
            txt = "fail to load camera %s pref settings: %s" % (camera, output)
            self._logger.error(txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, txt)

        # edit it
        modification = self._camera_module.edit_camera_setting(camera, tmp_file, back_quality, wizard_skip=skip_wizard)

        if modification:
            # Push file to DUT
            # Set /system partition to read/write
            self._device.set_filesystem_rw()
            # create a secure copy of camera pref file first
            secure_file = pref_setting + self._camera_module.orig_suffix
            if not file_api.exist(secure_file)[0]:
                self._logger.info("Make a secure copy of %s" % pref_setting)
                file_api.copy(pref_setting, secure_file)
                # copy the file permission
                permission = file_api.get_file_permissions(pref_setting, "root")[1]
                file_api.set_file_permission(secure_file, permission)
                # copy owner and group
                owner, group_owner = file_api.get_file_owner(pref_setting)
                file_api.set_file_owner(secure_file, owner, group_owner)
            else:
                permission = file_api.get_file_permissions(secure_file, "root")[1]
                owner, group_owner = file_api.get_file_owner(pref_setting)

            # push it on the board
            time.sleep(2)
            self._exec("adb push %s %s" % (tmp_file, pref_setting), force_execution=True)
            # dos2unix commented to avoid problem when camera apps try to upload setttings
            # self._exec("adb shell dos2unix %s" % pref_setting, force_execution=True)
            file_api.set_file_permission(pref_setting, permission)
            file_api.set_file_owner(pref_setting, owner, group_owner)
        else:
            self._logger.info("there is no camera setup modification needed after analysis")

    @need('camera')
    def restore_camera_setup(self, camera="default"):
        """
        Restore the camera setup from a ref file when it is present on the DUT
        :type camera: str
        :param camera: the camera package name, like "camera2", "camera_google"

        :rtype: boolean
        :return: True if the file has been restored, False otherwise
        """
        result = False
        self._logger.debug("Restore camera settings")
        self.__check_camera_module()
        camera = self._camera_module.get_camera(camera, raise_error=False)
        error = ""
        pref_setting = self._camera_module.camera_properties.prefs_setting_path.get(camera)
        if pref_setting is None:
            def_pref_setting = self._camera_module.camera_properties.prefs_setting_path.get("default")
            if  def_pref_setting is None:
                error = "there is no default key on CameraModule prefs_setting_path entry"
                self._logger.error(error)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error)
            else:
                # TODO: dynamicly create the pref settings when default is call
                pref_setting = def_pref_setting

        # check that the file exist
        secure_file = pref_setting + self._camera_module.orig_suffix
        file_api = self._device.get_uecmd("File", True)
        if file_api.exist(secure_file)[0]:
            self._logger.info("restore %s to .orig value" % pref_setting)
            # enable root
            self._device.enable_adb_root()
            # Set /system partition to read/write
            self._device.set_filesystem_rw()
            time.sleep(5)
            result, _ = file_api.copy(secure_file, pref_setting)
        else:
            self._logger.warning("restore failed because file %s is missing" % secure_file)

        return result

    @need('camera')
    def record(self, camera="default"):
        """
        Open video view, start video recording and return the raw path of the video

        :type camera: str
        :param camera: the camera package name, like "camera2", "camera_google"

        :rtype: tuple
        :return: ( temporary file path , final file path)
        """
        # check that the camera exist
        self.__check_camera_module()
        camera = self._camera_module.get_camera(camera)
        # Get list of existing files
        files_before = self._list_video_storage_files(camera)
        # Launch video recorder
        self._logger.debug("Open video view")
        self._open_camera_view(camera)
        # wait some sec to let camera view animation finish
        time.sleep(5)
        # try to push button to start the camera
        self._logger.debug("starting the recording")
        keycode_list = self._camera_module.camera_properties.keycode_start_recording.get(camera)
        if keycode_list is None:
            keycode_list = self._camera_module.camera_properties.keycode_start_recording.get("default")

        # tape to touch the screen
        self._exec("adb shell input tap 0 0")
        if keycode_list is not None:
            for keycode in keycode_list:
                self._exec("adb shell input keyevent %s" % keycode)
                time.sleep(2)
        else:
            error = "No keycode was defined to start camera recording"
            self._logger.error(error)
            raise AcsConfigException(DeviceException.OPERATION_FAILED, error)

        # Get the recorded file
        time.sleep(2)
        files_after = self._list_video_storage_files(camera)
        filename_list = list(set(files_after) - set(files_before))

        # Prevention of wrong cases
        new_file_name = ""
        if len(filename_list) == 0:
            self._logger.warning("No video file was created")

        elif len(filename_list) > 1:
            self._logger.warning("too many video file was created")
        else:
            new_file_name = filename_list[0]
            self._logger.debug("Video will be reccorded in raw file: %s" % str(new_file_name))

        raw_full_path = None
        final_path = None
        if type(new_file_name) is str and len(new_file_name) > 0:
                raw_full_path = new_file_name
                final_path = new_file_name.replace(".tmp", "", 1)
        else:
            self._logger.warning("Cant find where the video has been stored!")

        return raw_full_path, final_path

    def stop_record(self, camera="default"):
        """
        stop the video recording and leave the video view

        :type camera: str
        :param camera: the camera package name, like "camera2", "camera_google"
        """
        self._logger.info("Stop video and leave video view...")
        self.__check_camera_module()
        camera = self._camera_module.get_camera(camera, raise_error=False)
        error = ""
        # try to get the default way to stop the camera
        if camera is None or (camera not in self._camera_module.camera_properties.keycode_stop_recording.keys()):
            if  "default" not in self._camera_module.camera_properties.keycode_stop_recording:
                error = "there is no default key from camera module keycode_stop_recording entry"
                self._logger.error(error)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error)
            else:
                camera = "default"

        keycode_list = self._camera_module.camera_properties.keycode_stop_recording.get(camera)
        if keycode_list is not None:
            for keycode in keycode_list:
                self._exec("adb shell input keyevent %s" % keycode)
                time.sleep(1)

    def check_video_exist(self, video_path):
        """
        Check if a video file exist
        this uecmd take into account the fact that ongoing recorded video my have a name different from
        its final name like having a .tmp at the end of the file name

        :type video_path: str
        :param video_path: Path of the video file generated by video record uecmd

        :rtype: boolean
        :return: True if video exist, False otherwise
        """
        result = False
        if video_path is not None:
            # format video name
            split_video_path = video_path.rsplit(".", 1)
            # if video path end with a tmp it means that the video was on going when its path was return
            if split_video_path[-1].strip() == "tmp":
                video_path = split_video_path[0]

            cmd = "adb shell ls %s*" % video_path
            output = self._exec(cmd, force_execution=True, raise_error=False).strip()
            result = self.is_shell_output_ok(output)

        return result

    def force_stop(self, camera="default"):
        """
        force camera application to stop

        :type camera: str
        :param camera: the camera package name, like "camera2", "camera_google"
        """
        self._logger.info("Force camera apps to stop")
        self.__check_camera_module()
        camera = self._camera_module.get_camera(camera)
        camera_pkg = self._camera_module.camera_properties.camera_package.get(camera)
        exec_cmd = 'adb shell am force-stop {0}'.format(camera_pkg)
        self._exec(exec_cmd)

    def _list_video_storage_files(self, camera="default"):
        """
        Retrieves a list of existing video

        :type camera: str
        :param camera: the camera package name, like "camera2", "camera_google"
        """
        result = []
        store_path = self._camera_module.camera_properties.video_storage_path.get(camera)
        if store_path is None and camera != "default":
            self._logger.info("no specific storage path set for %s, taking the default one" % camera)
            store_path = self._camera_module.camera_properties.video_storage_path.get("default")

        cmd = "adb shell ls -d %s/*" % store_path
        cmd = cmd.replace("//", "/", 1)
        output = self._exec(cmd, raise_error=False).strip()
        if self.is_shell_output_ok(output):
            for element in output.split("\n"):
                element = element.strip()
                if len(element) > 0:
                    result.append(element)

        return result

    def _open_camera_view(self, camera):
        """
        open the camera view
        """
        start_cmd = "adb shell am start %s" % self._camera_module.camera_properties.open_camera_view_intent.get(camera)
        output = self._exec(start_cmd)
        if (not self.is_shell_output_ok(output)) or ("does not exist" in output.lower()) or ("unable to resolve intent" in output.lower()):
            error = "Failed to start camera view : %s" % output
            self._logger.error(error)
            raise AcsConfigException(DeviceException.OPERATION_FAILED, error)

    def check_video_state(self, video_path):
        """
        Check if a video is being recorded
        this uecmd take into account the fact that ongoing recorded video my have a name different from
        its final name like having a .tmp at the end of the file name

        :type video_path: str
        :param video_path: Path of the video file generated by video record uecmd

        :rtype: boolean
        :return: True if video is being recorded
        """
        result = False
        if video_path is not None:
            # format video name
            split_video_path = video_path.rsplit(".", 1)
            # if video path end with a tmp it means that the video was on going when its path was return
            if split_video_path[-1].strip() == "tmp":
                video_path = split_video_path[0]

            cmd2 = "adb shell \"ls -l %s | awk '{print $4}'\"" % video_path
            output2 = self._exec(cmd2).strip()
            no_error = self.is_shell_output_ok(output2)
            if not no_error:
                message = "Failed to check video record status : %s" % output2
                self._logger.error(message)
                raise AcsConfigException(DeviceException.OPERATION_FAILED, message)
            time.sleep(0.5)
            # while count <= 3:
            output3 = self._exec(cmd2).strip()
            no_error3 = self.is_shell_output_ok(output3)
            if not no_error3:
                message = "Failed to check video record status : %s" % output3
                self._logger.error(message)
                raise AcsConfigException(DeviceException.OPERATION_FAILED, message)

            if output2.isdigit() and output3.isdigit():
                output2 = int(output2)
                output3 = int(output3)
                output = output3 - output2
                if output > 0:
                    result = True
        return result
