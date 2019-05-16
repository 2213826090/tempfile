"""
@summary: Playback audio-file on DUT, and record it on the host.
@since 2 Feb 2016
@author: olilja
@organization: INTEL OTC/ACE

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

import time
import subprocess
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException
from UtilitiesFWK.Utilities import Global

class RecordAudio_Host(DeviceTestStepBase):

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

        self.duration = self._pars.duration
        self.fname_rec = self._pars.fname_rec

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """

        self.fname_artifact = context.get_info(self._pars.fname_artifact)

        self._logger.info("RecordAudio_Host - Parameters ----------------------------")
        self._logger.info("RecordAudio_Host - FNAME_ARTIFACT = " + self.fname_artifact)
        self._logger.info("RecordAudio_Host - DURATION = " + str(self.duration))
        self._logger.info("RecordAudio_Host - FNAME_REC = " + self.fname_rec)
        self._logger.info("RecordAudio_Host - ---------------------------- Parameters")

	self._logger.info("RecordAudio_Host - Starting music-player")
	self._device.run_cmd("adb shell am start -W com.google.android.music/com.android.music.activitymanagement.TopLevelActivity -D /sdcard/tmp.wav", 10)

	self._logger.info("RecordAudio_Host - Starting playback")
	self._device.run_cmd("adb shell input keyevent 85", 10)
        time.sleep(2)

        self._logger.info("RecordAudio_Host - Start recording on host")
	cmd = 'arecord -D hw:1,0 -f S24_3LE -c 2 -r 48000 -d ' + self.duration + ' ' + self.fname_rec
        subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        time.sleep(5)

	self._logger.info("RecordAudio_Host - Stopping audio")
	self._device.run_cmd("adb shell input keyevent 85", 5)

        return Global.SUCCESS
