"""
PREREQUISITES:
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


class SetPhotosAppDefaultVideoPlayer(DeviceTestStepBase):
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

        if self._file_type == "MP4":
            option_file_type = "video/mp4"
        elif self._file_type == "3GP":
            option_file_type = "video/3gp"
        elif self._file_type == "ALL":
            option_file_type = "video/*"
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

        # Find Video node
        video_preview = doc.findall("./preferred-activities/item/[@name='com.google.android.apps.plus/.phone.VideoViewActivity']")
        video_preview_type = doc.findall("./preferred-activities/item/filter/type[@name='video/mp3']")
        len_video_preview = len(video_preview)
        len_video_preview_type = len(video_preview_type)

        # Video node not found
        if len_video_preview == 0:
            if len_video_preview_type == 0:
                self._logger.debug("VideoViewActivity not found in package-restrictions.xml")
                self._logger.debug("Set Photos App default program to open %s", str(option_file_type))

                # Add all elements needed
                node_preferred_activities = doc.find("preferred-activities")
                elt_video_preview = etree.Element("item", name="com.google.android.apps.plus/.phone.VideoViewActivity",
                                                  match="600000", always="false", set="0")
                elt_filter = etree.SubElement(elt_video_preview, 'filter')
                etree.SubElement(elt_filter, 'action', name='android.intent.action.VIEW')
                etree.SubElement(elt_filter, 'cat', name='android.intent.category.DEFAULT')
                etree.SubElement(elt_filter, 'type', name=str(option_file_type))
                node_preferred_activities.append(elt_video_preview)

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
