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
@since 25 June 2014
@author: Val Peterson
@organization: INTEL PEG-SVE-DSV

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
import re

class PlayAudioLoop(DeviceTestStepBase):
    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        self._logger.info(self._pars.id + ": Run")

        self.system_api = self._device.get_uecmd("System")
        # If this test step is used along with an audio docking solution, such as the RUN_UMG_AUDIO_DOCK_TOOLKIT test step, then a message may pop up when accessory mode is started.  Look for and clear the message.
        # This is only an issue on Android.  For other OSes this will raise an exception, and in such cases
        # we will simply move on and execute the rest of this test step.
        try:
            dumpsys_output = self.system_api.get_dumpsys_focus()
            re_match = re.match(".* (.*/.*)}", dumpsys_output)
            if (re_match is not None) and ('com.android.systemui.usb.UsbAccessoryUriActivity' in re_match.group(1)):
                self._device.get_uecmd("KeyEvent").scenario(['escape'])

        except DeviceException as de:
            if de._error_msg == 'DeviceException: Feature not implemented':
                pass
            else:
                raise

        basename = "loopAudioPlayback"
        #To make this OS-independent, look for different possible filenames.
        #If anyone enables this in Windows, I assume they will create a script to do what loopAudioPlayback.sh does.
        #Add to this list if required for any other OS.
        possible_extensions = [".sh", ".ps1"]
        for ext in possible_extensions:
            #uecmd or underlying code should take care of changing path delimiters as needed
            script = self._pars.scripts_path + "/" + basename + ext
            found = self._device.get_uecmd("File").exist(script)
            if found:
                break
        if not found:
            msg = "PlayAudioLoop: Could not find a " + basename + ".* script."
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        args = str(self._pars.duration)

        """run_shell_executable doesn't return anything, but will raise an exception if it fails"""
        """Adding a extra small amount of time to the timeout, in case something goes a little long"""
        self.system_api.run_shell_executable(script, args, io_redirection=2, timeout=self._pars.duration*60 + 60)

        self._logger.info(self._pars.id + ": Done")
