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
:since: 12 sep 2012
:author: sfusilie
"""

import time
import os
from lxml import etree


from acs_test_scripts.Device.UECmd.UECmdDecorator import need
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.Device.UECmd.Imp.Android.Common.Multimedia.VideoRecorder \
    import VideoRecorder as VideoRecorderICS


class VideoRecorder(VideoRecorderICS):

    """
    Class that handle all video recording operations
    """

    camera_packages = {
        'camera': 'com.android.gallery3d',
        'camera2': 'com.intel.camera2'}

    def __init__(self, device):
        """
        Constructor
        """
        VideoRecorderICS.__init__(self, device)
        self._device = device
        self._google_video_camera_action = "android.intent.action.Main"
        self._google_video_camera_component = "com.android.gallery3d/com.android.camera.VideoCamera"
        self._intel_video_camera_action = "android.intent.action.Main"
        self._intel_video_camera_component = "com.intel.camera2/.VideoCamera"
        self._video_camera_pref_version_key = "4"

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

        # Initial values
        camera_id = "0"

        if camera == "back":
            camera_id = "0"
        elif camera == "front":
            camera_id = "1"
        else:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "camera can only be front or back")

        # Define intent action to use for video recording
        camera_launcher_intent = "adb shell am start "
        if camera_name == "camera":
            # Rename into the right camera name for google camera
            camera_name = "gallery3d"
            camera_launcher_intent += "-a %s -n %s" % \
                (self._google_video_camera_action, self._google_video_camera_component)

        elif camera_name == "camera2":
            cmd = "grep camera2 /data/system/packages.list"\
                  "|grep -v 'tests'|grep -v 'android'|sed 's/[[:space:]]\+/ /g' |cut -d' ' -f1"
            camera = self._exec("adb shell %s || echo nok" % cmd)
            if camera != "nok":
                self._intel_video_camera_component = camera + "/.VideoCamera"

            camera_launcher_intent += "-a %s -n %s" % \
                (self._intel_video_camera_action, self._intel_video_camera_component)

        else:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Unknown camera name '%s' !" % camera_name)

        # create a global conf file
        self._logger.info("Create camera preference files ...")
        config_path = "/data/data/com.android." + camera_name + "/shared_prefs/"
        global_config_file_name = \
            "com.android." + camera_name + "_preferences.xml"
        camera_config_file_name = \
            "com.android." + camera_name + "_preferences_" + camera_id + ".xml"

        global_conf = etree.Element('map')
        etree.ElementTree(global_conf)
        etree.SubElement(global_conf, 'string', name='pref_camera_recordlocation_key').text = "1"
        etree.SubElement(global_conf, 'string', name='pref_camera_jpegquality_key').text = "superfine"
        etree.SubElement(global_conf, 'string', name='pref_camera_id_key').text = camera_id
        etree.SubElement(global_conf, 'boolean', name='has-editor-image/*', value="true")
        etree.SubElement(global_conf, 'boolean', name='pref_camera_first_use_hint_shown_key', value="false")
        etree.SubElement(global_conf, 'boolean', name='pref_video_first_use_hint_shown_key', value="false")
        etree.SubElement(global_conf, 'int', name='pref_version_key', value=self._video_camera_pref_version_key)
        etree.SubElement(global_conf, 'int', name='cache-up-to-date', value="1")
        etree.SubElement(global_conf, 'int', name='editor-update-image/*', value="1")

        app_conf_file = open(global_config_file_name, "w")
        app_conf_file.writelines(etree.tostring(global_conf,
                                                pretty_print=True,
                                                xml_declaration=True,
                                                encoding='utf-8'))
        app_conf_file.close()

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

        camera_conf = etree.Element('map')
        etree.ElementTree(camera_conf)
        etree.SubElement(camera_conf, 'int', name='pref_local_version_key', value="2")

        if camera_id == "0":
            etree.SubElement(camera_conf, 'string', name='pref_camera_whitebalance_key').text = white_balance
            etree.SubElement(camera_conf, 'string', name='pref_camera_video_flashmode_key').text = flash_mode
            etree.SubElement(camera_conf, 'string', name='pref_video_quality_key').text = qual_key
            etree.SubElement(camera_conf, 'string',
                             name='pref_camera_temporal_noise_reduction_key').text = noise_reduction
            etree.SubElement(camera_conf, 'string', name='pref_camera_coloreffect_key').text = color_effect
            etree.SubElement(camera_conf, 'string', name='pref_camera_picturesize_key').text = "3264x2448"
            etree.SubElement(camera_conf, 'string', name='pref_camera_dvs_key').text = dvs

        else:
            etree.SubElement(camera_conf, 'string', name='pref_camera_antibanding_key').text = "50hz"
            etree.SubElement(camera_conf, 'string', name='pref_video_quality_key').text = quality
            etree.SubElement(camera_conf, 'string', name='pref_camera_picturesize_key').text = "1280x960"

        camera_conf_file = open(camera_config_file_name, "w")
        camera_conf_file.writelines(etree.tostring(camera_conf,
                                                   pretty_print=True,
                                                   xml_declaration=True,
                                                   encoding='utf-8'))
        camera_conf_file.close()

        # push it to phone
        self._logger.info("Upload preference files ...")

        self._device.push(global_config_file_name, os.path.join(config_path, global_config_file_name), 1000)
        self._device.push(camera_config_file_name, os.path.join(config_path, camera_config_file_name), 1000)

        try:
            os.unlink(global_config_file_name)
            os.unlink(camera_config_file_name)
        except (IOError, OSError):
            self._logger.exception("Exception while deleting files: `{0}` & `{1}`".format(global_config_file_name,
                                                                                          camera_config_file_name))
        time.sleep(1)

        # am start the video camera
        self._logger.info("Launch camera %s ..." % str(camera_name))
        self._exec(camera_launcher_intent)
        time.sleep(1)

        # start the recording by pressing the camera key
        self._logger.info("Start recording ...")
        self._exec("adb shell input keyevent 27")

        return video_file_name
