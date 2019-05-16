"""
@summary: Repeatedly plays an audio file for the specified amount of time.  This uses a device-side
script to maintain the playback for an extended amount of time without intervention from the host.  This
is done to avoid adding traffic to the host-device connection (e.g. ADB for Android) throughout the test.
PREREQUISITES:
Android:
    * AudioPlayback.apk must be installed (Artifactory: acs_test_artifacts/CONCURRENCY/TESTS/audio_playback).
    * loopAudioPlayback.sh must be located in the directory specified by SCRIPTS_PATH (from acs_test_scripts/Lib/ShellScripts/Android/Multimedia/audio_playback, can use INSTALL_SCRIPTS_FROM_LIB).
    * at least one music file in the /sdcard/Music directory.
Windows: Device-side script not yet created.
@since 13 Feb 2015
@author: mcarriex
@organization: INTEL QCTV

@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
otherwise. Any license under such intellectual property rights must be expressed
and approved by Intel in writing.
"""

from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException
from UtilitiesFWK.Utilities import Global
from lxml import etree
import os
import tempfile


class SetGoogleMusicDefaultAudioPlayer(DeviceTestStepBase):
    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._phone_name = str(self._pars.device)
        self._file_type = str(self._pars.file_type)

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        self._logger.info(self._pars.id + ": Run")

        if self._file_type == "MP3":
            option_file_type = "audio/mp3"
        elif self._file_type == "M4A":
            option_file_type = "audio/m4a"
        elif self._file_type == "WMA":
            option_file_type = "audio/wma"
        elif self._file_type == "FLAC":
            option_file_type = "audio/flac"
        elif self._file_type == "OGG":
            option_file_type = "audio/ogg"
        elif self._file_type == "M4P":
            option_file_type = "audio/m4p"
        elif self._file_type is None:
            err_msg = "No File type specified"
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)
        else:
            err_msg = "File type unsupported"
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)

        pref_setting_file = "/data/system/users/0/package-restrictions.xml"
        packages_restrictions = "package-restrictions"

        # Load package-restrictions file
        tmp_dir = tempfile.gettempdir()
        tmp_file = "%s_temp_file.xml" % packages_restrictions
        tmp_file = os.path.join(tmp_dir, tmp_file)
        cmd = 'adb pull %s %s' % (pref_setting_file, tmp_file)
        (status, output) = self._device.run_cmd(cmd, 50)
        if status != Global.SUCCESS:
            self._logger.error(output)
            raise DeviceException(DeviceException.OPERATION_FAILED, output)

        # Edit package-restrictions file
        xml_pref = etree.parse(tmp_file, parser=None)
        doc = xml_pref.getroot()

        # Find AudioPreview
        audio_preview = doc.findall("./preferred-activities/item/[@name='com.google.android.music/.AudioPreview']")
        audio_preview_set_name1 = doc.findall(
            "./preferred-activities/item/set/[@name='com.google.android.music/.AudioPreview']")
        audio_preview_set_name2 = doc.findall(
            "./preferred-activities/item/set/[@name='com.android.music/.AudioPreview']")
        audio_preview_type = doc.findall("./preferred-activities/item/filter/type[@name='audio/mp3']")
        len_audio_preview = len(audio_preview)
        len_audio_preview_set_name1 = len(audio_preview_set_name1)
        len_audio_preview_set_name2 = len(audio_preview_set_name2)
        len_audio_preview_type = len(audio_preview_type)

        # AudioPreview not found
        if len_audio_preview == 0:
            if len_audio_preview_set_name1 == 0:
                if len_audio_preview_set_name2 == 0:
                    if len_audio_preview_type == 0:
                        self._logger.debug("AudioPreview not found in package-restrictions.xml")
                        self._logger.debug("Set Google Play Music default program to open %s", str(option_file_type))

                        # Add all elements needed
                        node_preferred_activities = doc.find("preferred-activities")
                        elt_audio_preview = etree.Element("item", name="com.google.android.music/.AudioPreview",
                                                          match="600000", always="true", set="2")
                        etree.SubElement(elt_audio_preview, 'set', name='com.google.android.music/.AudioPreview')
                        etree.SubElement(elt_audio_preview, 'set', name='com.android.music/.AudioPreview')
                        elt_filter = etree.SubElement(elt_audio_preview, 'filter')
                        etree.SubElement(elt_filter, 'action', name='android.intent.action.VIEW')
                        etree.SubElement(elt_filter, 'cat', name='android.intent.category.DEFAULT')
                        etree.SubElement(elt_filter, 'type', name=str(option_file_type))
                        node_preferred_activities.append(elt_audio_preview)

                        # Write package-restrictions temp file
                        doc_info = xml_pref.docinfo
                        with open(tmp_file, "w") as xml_file:
                            xml_pref.write(xml_file,
                                           pretty_print=False,
                                           encoding=doc_info.encoding,
                                           standalone=doc_info.standalone,
                                           xml_declaration=True)

                        # Push the file on the device
                        cmd = 'adb push %s %s' % (tmp_file, pref_setting_file)
                        (status, output) = self._device.run_cmd(cmd, 50)
                        if status != Global.SUCCESS:
                            self._logger.error(output)
                            raise DeviceException(DeviceException.OPERATION_FAILED, output)



