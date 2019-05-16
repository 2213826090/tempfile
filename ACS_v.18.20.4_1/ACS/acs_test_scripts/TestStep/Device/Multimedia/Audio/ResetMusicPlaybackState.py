"""
@summary: Reset Google Play Music app to a defined state for a specific
audio file.
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

from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException
from UtilitiesFWK.Utilities import Global
import os

class ResetMusicPlaybackState(DeviceTestStepBase):

    fname_artifact = ""

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """

        self.fname_artifact = context.get_info(self._pars.fname_artifact)

        self._logger.info("ResetMusicPlayerState - Parameters ----------------------------")
        self._logger.info("ResetMusicPlayerState - FNAME_ARTIFACT = " + self.fname_artifact)
        self._logger.info("ResetMusicPlayerState - ---------------------------- Parameters")
        self._logger.info(os.getcwd())
        self._logger.info("ResetMusicPlayerState - Force-stop music-player\n")
	self._device.run_cmd("adb shell am force-stop com.google.android.music", 10)

	self._logger.info("ResetMusicPlayerState - Remove existing audio-file\n")
        self._device.run_cmd("adb shell rm /sdcard/tmp.wav", 10)

	self._logger.info("ResetMusicPlayerState - Updating sdcard\n")
	self._device.run_cmd("adb shell am broadcast -a android.intent.action.MEDIA_MOUNTED -d file:///mnt/shell/emulated/0", 10)

        self._logger.info("ResetMusicPlayerSta*te - Remove music-player settings\n")
        self._device.run_cmd("adb shell rm /data/data/com.google.android.music/shared_prefs/Music.xml", 10)

        self._logger.info("ResetMusicPlayerState - Pushing wav-file to target\n")
        self._device.run_cmd("adb push " + self.fname_artifact + " /sdcard/tmp.wav", 10)

	self._logger.info("ResetMusicPlayerState - Updating sdcard\n")
	self._device.run_cmd("adb shell am broadcast -a android.intent.action.MEDIA_MOUNTED -d file:///mnt/shell/emulated/0", 10)

        return Global.SUCCESS
