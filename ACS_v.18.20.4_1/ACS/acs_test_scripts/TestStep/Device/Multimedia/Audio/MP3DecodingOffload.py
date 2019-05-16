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
@since 9 Mar 2015
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
from UtilitiesFWK.Utilities import Global


class MP3DecodingOffload(DeviceTestStepBase):
    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._phone_name = str(self._pars.device)
        self._file_type = str(self._pars.file_type)
        self._mode = self._pars.mode

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        self._logger.info(self._pars.id + ": Run")

        if self._mode is "enable":
            cmd = "adb shell \"stop media & setprop audio.offload.disable 0 & start media\""
            (status, output) = self._device.run_cmd(cmd, 50)
            if status != Global.SUCCESS:
                self._logger.error(output)
                raise DeviceException(DeviceException.OPERATION_FAILED, output)
        else:
            cmd = "adb shell \"stop media & setprop audio.offload.disable 1 & start media\""
            (status, output) = self._device.run_cmd(cmd, 50)
            if status != Global.SUCCESS:
                self._logger.error(output)
                raise DeviceException(DeviceException.OPERATION_FAILED, output)
