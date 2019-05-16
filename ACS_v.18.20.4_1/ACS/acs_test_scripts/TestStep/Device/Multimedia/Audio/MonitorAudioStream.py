"""
@summary: This file implements a Test Step that monitors audio stream on host
          PC for silence for the test duration.  Continously takes samples of
          line in audio on host PC to monitor for silence.
@since 25 August 2014
@author: Stephen A Smith
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
import acs_test_scripts.Utilities.AudioUtilities as audio_verification
# Python Imports
import time
import os
import traceback
import thread

class MonitorAudioStream(DeviceTestStepBase):
    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        self._logger.info(self._pars.id + ": Starting")
        self.report_path = self._device.get_report_tree().create_subfolder('monitor_audio_stream')
        self._image_api = self._device.get_uecmd("Image")

        try:
            self.audio_streaming_monitor_function()
        except:
            self._logger.debug(self._pars.id + ": Unexpected error occured\n" + traceback.format_exc())
            raise

        self._logger.info(self._pars.id + ": Done")

    def audio_streaming_monitor_function(self):
        # Listen for silence
        # Do it again
        start_time = time.time()

        # Subtract 30 seconds from runtime because we generally have to wait some time to ensure audio playback has started.
        runtime_seconds = (self._pars.duration * 60) - 30
        loop_iteration = 0
        test_passed = True # Start with True. Change to false on failure.

        while time.time()-start_time < runtime_seconds:
            loop_iteration += 1
            self._logger.info(self._pars.id + ": Loop %d"%loop_iteration)

            self._logger.debug(self._pars.id + ": Listening for silence on the PC line in.")
            audio_output_file = os.path.join(self.report_path, "captured_audio_%d.mp3"%loop_iteration)
            self._logger.info(self._pars.id + ": Saving audio capture data to %s"%audio_output_file)
            # To keep from timing out in the app and to keep our wav files at a manageable size, limit listen_time to something less than 30 minutes.  Especially in longer test runs.
            listen_time = runtime_seconds-(time.time()-start_time)
            if listen_time > self._pars.listen_length * 60:
                listen_time = self._pars.listen_length * 60
            try:
                    silence_offsets = audio_verification.verify_audio_input( output_file_path=audio_output_file,
                        silence_threshold_seconds=self._pars.silence_threshold_length, silence_callback_func=self.debug_silence_in_new_thread,
                        audio_resume_callback_func=None, listen_len_seconds=listen_time, silence_threshold=self._pars.silence_threshold_raw )
            except IOError as e:
                self._logger.debug(self._pars.id + ": I/o error({0}) from verify_audio_input:  {1}".format(e.errno, e.strerror))
                raise
            except:
                self._logger.debug(self._pars.id + ": Unexpected error occured from verify_audio_input\n" + traceback.format_exc())
                raise
            if len(silence_offsets)>0:
                silence_offset_str = ""
                for offset_pair in silence_offsets:
                    silence_begin_minutes = int(offset_pair[0]/60)
                    silence_begin_seconds = int(offset_pair[0]-(silence_begin_minutes*60))
                    silence_end_minutes = int(offset_pair[1]/60)
                    silence_end_seconds = int(offset_pair[1]-(silence_end_minutes*60))
                    silence_offset_str += "%d:%02d-%d:%02d, "%(silence_begin_minutes, silence_begin_seconds, silence_end_minutes, silence_end_seconds)
                # Remove the comma at the end
                silence_offset_str = silence_offset_str[:-2]
                self._logger.error(self._pars.id + ": Silence detected at offset "+silence_offset_str)
                self._logger.error(self._pars.id + ": Setting test result to FAIL due to silence detected.")
                test_passed = False
            self._logger.debug(self._pars.id + ": End of monitoring loop.")
        if test_passed:
            self._logger.info(self._pars.id + ": Monitoring loop passed!")
        else:
            self._logger.error(self._pars.id + ": Monitoring loop failed!!")
            self.ts_verdict_msg = self._pars.id + ":  Monitoring loop failed!!"
            raise DeviceException(DeviceException.OPERATION_FAILED, self._pars.id + ": Test has failed while monitoring audio!")

    def debug_silence_in_new_thread(self, silence_start_offset):
        thread.start_new_thread(self.debug_silence, (silence_start_offset,))

    def debug_silence(self, silence_start_offset=0.0):
        offset_hours = int(silence_start_offset/60/60)
        offset_minutes = int(silence_start_offset/60)
        offset_seconds = silence_start_offset-(offset_minutes*60)
        offset_str = "%d:%02d:%.2f"%(offset_hours,offset_minutes,offset_seconds)
        self._logger.debug(self._pars.id + ": START debug for silence at offset "+offset_str)

        local_screencap_file_path = os.path.join( self.report_path, "AudioSilenceScreenshot_%dh%dm%ds.png"%(offset_hours,offset_minutes,int(offset_seconds)) )
        self._image_api.take_screenshot_and_pull_on_host(os.path.split(local_screencap_file_path)[1], os.path.split(local_screencap_file_path)[0])

        self._logger.debug(self._pars.id + ": Screen capture saved at "+local_screencap_file_path)
        self._logger.debug(self._pars.id + ": END debug for silence at offset "+offset_str)
