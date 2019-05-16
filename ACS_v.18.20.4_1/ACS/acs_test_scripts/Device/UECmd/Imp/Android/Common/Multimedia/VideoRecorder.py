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
:summary: This file implements UECmds for video record
:since: 08/04/2011
:author: fhu2
"""
import os
import time

from acs_test_scripts.Device.UECmd.Interface.Multimedia.IVideoRecorder import IVideoRecorder
from acs_test_scripts.Device.UECmd.UECmdDecorator import need
from acs_test_scripts.Device.UECmd.Imp.Android.Common.BaseV2 import BaseV2
from ErrorHandling.AcsConfigException import AcsConfigException
from xml.dom.minidom import Document


class VideoRecorder(BaseV2, IVideoRecorder):
    """
    Class that handle all video recording operations
    """

    camera_packages = {
        'camera': 'com.android.camera',
        'testcamera': 'com.android.testcamera'
    }

    def __init__(self, device):
        """
        Constructor
        """
        BaseV2.__init__(self, device)
        IVideoRecorder.__init__(self, device)

        self.__camera_packages = self.camera_packages

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
        self._logger.info("Start video recording ...")

        # cleanup the record directory
        self._logger.info("Delete old video files")
        self._exec("adb shell rm " + video_path + "*.mp4 ")

        video_file_name = video_path + camera + "_" + quality \
            + "_" + white_balance + "_" + color_effect + "_" + flash_mode \
            + "_" + dvs + "_" + noise_reduction + ".mp4"

        camera_id = "0"
        if camera == "back":
            camera_id = "0"
        elif camera == "front":
            camera_id = "1"
        else:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "camera can only be front or back")

        # create a global conf file
        self._logger.info("Create camera preference files ...")
        config_path = "/data/data/com.android." + camera_name + "/shared_prefs/"
        global_config_file_name = "com.android." + camera_name + "_preferences.xml"
        camera_config_file_name = "com.android." + camera_name + "_preferences_" + camera_id + ".xml"

        app_conf_file = open(global_config_file_name, "w")
        app_conf_document = Document()

        base_element = app_conf_document.createElement("map")
        app_conf_document.appendChild(base_element)

        global_conf_entries = {"pref_camera_recordlocation_key": "none",
                               "pref_camera_jpegquality_key": "superfine",
                               "pref_camera_id_key": camera_id}

        for key, value in global_conf_entries.iteritems():
            element = app_conf_document.createElement("string")
            element.setAttribute("name", key)
            element.appendChild(app_conf_document.createTextNode(value))
            base_element.appendChild(element)

        ele_pref_version = app_conf_document.createElement("int")
        ele_pref_version.setAttribute("name", "pref_version_key")
        ele_pref_version.setAttribute("value", "4")
        base_element.appendChild(ele_pref_version)

        app_conf_file.write(app_conf_document.toprettyxml(indent="", newl="", encoding="utf-8"))
        app_conf_file.close()

        # create the camera configuration file
        camera_conf_file = open(camera_config_file_name, "w")
        camera_conf_document = Document()
        ele_map = camera_conf_document.createElement("map")
        camera_conf_document.appendChild(ele_map)

        # Change quality to DUT "language", now in default camera
        # there is only 3 options, can add more else here if more options been add in this camera
        if quality == "1080p":
            qual_key = "6"
        elif quality == "720p":
            qual_key = "5"
        elif quality == "480p":
            qual_key = "4"
        else:
            qual_key = quality

        if camera_id == "0":
            camera_conf_entries = {"pref_camera_whitebalance_key": white_balance,
                                   "pref_camera_video_flashmode_key": flash_mode,
                                   "pref_video_quality_key": qual_key,
                                   "pref_camera_temporal_noise_reduction_key": noise_reduction,
                                   "pref_camera_coloreffect_key": color_effect,
                                   "pref_camera_picturesize_key": "3264x2448",
                                   "pref_camera_dvs_key": dvs
                                   }
        else:
            camera_conf_entries = {"pref_camera_antibanding_key": "50hz",
                                   "pref_video_quality_key": quality,
                                   "pref_camera_picturesize_key": "1280x960"
                                   }
        for key, value in camera_conf_entries.iteritems():
            element = camera_conf_document.createElement("string")
            element.setAttribute("name", key)
            element.appendChild(camera_conf_document.createTextNode(value))
            ele_map.appendChild(element)

        ele_pref_version = app_conf_document.createElement("int")
        ele_pref_version.setAttribute("name", "pref_local_version_key")
        ele_pref_version.setAttribute("value", "2")
        ele_map.appendChild(ele_pref_version)

        camera_conf_file.write(camera_conf_document.toprettyxml(indent="", newl="", encoding="utf-8"))
        camera_conf_file.close()

        # push it to phone
        self._logger.info("Upload preference files ...")
        push_global_conf_cmd = "adb push \"%s\" \"%s\"" % \
            (global_config_file_name, config_path + global_config_file_name)
        push_camera_conf_cmd = "adb push \"%s\" \"%s\"" % \
            (camera_config_file_name, config_path + camera_config_file_name)

        self._device.run_cmd(push_global_conf_cmd, 1000)
        self._device.run_cmd(push_camera_conf_cmd, 1000)

        os.unlink(global_config_file_name)
        os.unlink(camera_config_file_name)

        # am start the video camera
        if "camera2" not in self.__camera_packages.keys() is None:
            self.__get_package_camera2()

        if self.__camera_packages[camera_name] == "":
            msg_str = "There is no camera name, probably no camera app neither. Please, check both."
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg_str)

        # grant runtime permissions to the app
        self._device.grant_runtime_permissions(self.__camera_packages[camera_name])

        self._logger.info("Launch camera ...")
        self._exec("adb shell am start -a android.media.action.VIDEO_CAMERA "
                   "-n {0}/.VideoCamera".format(self.__camera_packages[camera_name]))
        time.sleep(1)

        # start the recording by pressing the camera key
        self._logger.info("Start recording ...")
        self._exec("adb shell input keyevent 27")

        return video_file_name

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
        # stop recording by pressing the camera key
        self._logger.info("Stop recording ...")
        self._exec("adb shell input keyevent 27")
        time.sleep(1)
        # go back to home screen
        self._logger.info("Go back to home ...")
        self._exec("adb shell input keyevent 4")
        self._exec("adb shell input keyevent 3")

        # rename the recorded file
        self._logger.info("Rename the recorded file ...")
        self._exec("adb shell mv " + video_path + "*.mp4 " + video_file_name)

    @need('camera')
    def check_recording(self):
        """
        Check that video recording on going

        :rtype: boolean
        :return: return True if video is running False else.
        """
        self._logger.warning("check_recording is not implemented on Android !")

        return False

    @need('camera')
    def kill_camera(self, packagename):
        """
        Kill Camera.

        :type packagename: str
        :param packagename: have to be a effective camera name.
                     if the camera name is invalid, nothing would happen
        """

        # Seen with S. Savrimoutou for the moment
        # Platforms genericity handling mechanism is
        # not yet implemented, therefore we use mapping.
        if "camera2" not in self.__camera_packages.keys():
            self.__get_package_camera2()
        if (packagename not in self.__camera_packages) or \
                (self.__camera_packages[packagename] == ""):
            error = "Wrong camera app name ({0})".format(packagename)
            self._logger.error(error)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error)

        exec_cmd = 'adb shell am force-stop {0}'.format(self.__camera_packages[packagename])
        # adb shell am force-stop `packagename`
        output = self._exec(exec_cmd)
        if output:
            self._logger.info(output)

    def __get_package_camera2(self):
        """
        get the package name for camera2
        """
        cmd = "grep camera /data/system/packages.list | "\
              "grep -v .test | grep -v android |"\
              "sed 's/[[:space:]]\+/ /g' |cut -d' ' -f1"
        exec_cmd = "adb shell %s" % cmd
        package = self._exec(exec_cmd, 3)
        self.__camera_packages["camera2"] = package
