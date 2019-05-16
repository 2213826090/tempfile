"""
@summary: Play an audio file with google play music.
Format supported : MP3, M4A, WMA, FLAC, OGG, M4P
PREREQUISITES: none
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


class PlayAudioFileWithGooglePlayMusic(DeviceTestStepBase):
    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._phone_name = str(self._pars.device)
        self._file_path = str(self._pars.file_path)
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

        # Play Music
        cmd = 'adb shell am start -a android.intent.action.VIEW -d ' \
                                                     'file://' + str(self._file_path) + ' -t ' + str(option_file_type)
        (status, output) = self._device.run_cmd(cmd, 50)
        if status != Global.SUCCESS:
            self._logger.error(output)
            raise DeviceException(DeviceException.OPERATION_FAILED, output)
