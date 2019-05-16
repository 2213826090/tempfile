"""
@summary: This file implements a Test Step to stream audio on the device under
          test using an application like Pandora
@since 2 July 2014
@authors: Val Peterson, Carl Sapp, Jacob Bowen
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

import time
import os
import logging
import thread
import acs_test_scripts.Utilities.AudioUtilities as audio_verification
import acs_test_scripts.Utilities.OSBVUtilities as osbv_utils
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException

"""This list should be in sync with the PossibleValues of the AUDIO_APP parameter in the catalog XML"""
android_internet_audio_apps = {
    #<app name passed in via test step parameters>:[<package>, <activity>]
    "RadioFM":["com.radio.fmradio", "RadioFM"],
    "Pandora":["com.pandora.android", "Main"],
}

class RunAudioStreaming(DeviceTestStepBase):
    """
        This class implements the test step functionality for streaming audio
        from the internet using an Android app, like Pandora.
    """
    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self.system_api = self._device.get_uecmd("System")
        self.app_api = self._device.get_uecmd("AppMgmt")
        self.file_api = self._device.get_uecmd("File")
        self.network_api = self._device.get_uecmd("Networking")

    def launch_android_app(self):
        if self._pars.audio_app not in android_internet_audio_apps:
            #we should not get here if the catalog XML has a list of PossibleValues that matches the entries in android_internet_audio_apps
            self._logger.error("RunAudioStreaming: '%s' is not a supported application."%self._pars.audio_app)

        self._logger.debug("RunAudioStreaming: Starting " + self._pars.audio_app)
        pkg = android_internet_audio_apps[self._pars.audio_app][0]
        activity = android_internet_audio_apps[self._pars.audio_app][1]
        intent = pkg + "/." + activity
        self.app_api.launch_app(intent=intent)

    def stop_android_app(self):
        self._logger.debug("RunAudioStreaming: Stopping Internet audio application")
        pkg = android_internet_audio_apps[self._pars.audio_app][0]
        self._device.run_cmd("adb shell am force-stop " + pkg, timeout=10)

    def audio_streaming_loop(self):
        self.runtime_seconds = self._pars.duration*60
        loop_iteration = 0

        #Do we launch the app one time now, or each time through the loop?
        if self._pars.continuous_stream:
            self.launch_android_app()
        self.start_time = time.time()
        chunk_passed = True
        while time.time()-self.start_time < self.runtime_seconds and chunk_passed:
            loop_iteration += 1
            self._logger.info("RunAudioStreaming: Loop %d"%loop_iteration)
            if not self._pars.continuous_stream:
                self.launch_android_app()
            # Wait 30 seconds for the app to open and start playing
            time.sleep(30)
            chunk_passed = self.monitor_audio(loop_iteration)
            if not self._pars.continuous_stream:
                self.stop_android_app()

        #Time's up!  Close the app if we were doing continuous streaming
        if self._pars.continuous_stream:
            self.stop_android_app()
        return chunk_passed

    def monitor_audio(self, loop_iteration):
        self._logger.debug("RunAudioStreaming: Listening for silence on the PC line in.")
        audio_output_file = os.path.join(self.log_directory, "captured_audio_%d.mp3"%loop_iteration)
        self._logger.info("RunAudioStreaming: Saving audio capture data to %s"%audio_output_file)

        #calculate how much time is left in the test run
        listen_time = self.runtime_seconds-(time.time()-self.start_time)
        #if greater than MONITORING_CHUNK_SIZE, then limit this chunk to that size
        if listen_time > self._pars.monitoring_chunk_size*60:
            listen_time = self._pars.monitoring_chunk_size*60
        #Use AudioUtilities function that records audio and checks for unexpected silence.  Use the
        #silence_threshold_seconds to account for expected breaks between songs, and silence_threshold to adjust for the noise floor.
        silence_offsets = audio_verification.verify_audio_input( output_file_path=audio_output_file, silence_threshold_seconds=10,
            silence_callback_func=self.debug_silence_thread, audio_resume_callback_func=None,
            listen_len_seconds=listen_time, silence_threshold=300 )
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
            self._logger.error("RunAudioStreaming: Silence detected at offset "+silence_offset_str)
            self._logger.error("RunAudioStreaming: FAILED -- detected excessive silence.")
            return False

        return True

    def debug_silence_thread(self, silence_start_offset):
        thread.start_new_thread(self.debug_silence, (silence_start_offset,))

    def debug_silence(self, silence_start_offset=0.0):
        offset_hours = int(silence_start_offset/60/60)
        offset_minutes = int(silence_start_offset/60)
        offset_seconds = silence_start_offset-(offset_minutes*60)
        offset_str = "%d:%02d:%.2f"%(offset_hours,offset_minutes,offset_seconds)
        self._logger.debug("START debug for silence at offset "+offset_str)
        self._logger.debug("Wifi State: "+osbv_utils.wifi_state(self._device))
        cell_props = self.network_api.cellular_signal_properties()
        for prop in sorted(cell_props):
            self._logger.debug("%s: %s"%(str(prop), str(cell_props[prop])))
        # Grab a screen capture
        device_screencap_file_path = "/data/screenshots/screenshot.png"
        self._device.run_cmd("adb shell if [ ! -d %s ]; then mkdir %s; fi;"%("/data/screenshots","/data/screenshots"), timeout=10)
        self._device.run_cmd("adb shell screencap -p %s"%device_screencap_file_path, timeout=30)
        local_screencap_file_path = os.path.join( self.log_directory,
            "AudioSilenceScreenshot_%dh%dm%ds.png"%(offset_hours,offset_minutes,int(offset_seconds)) )
        file_size = self.file_api.size(device_screencap_file_path)
        cmd = "adb pull " + device_screencap_file_path + " " + local_screencap_file_path
        self._device.run_cmd(cmd, timeout=file_size)    #timeout assumes throughput >= 1KB/s
        self._logger.debug("Screen capture saved at "+local_screencap_file_path)
        self._logger.debug("END debug for silence at offset "+offset_str)

    def run(self, context):
        """
            Runs the InternetAudioStreaming test step

            @type context: TestStepContext
            @param context: test case context
        """
        DeviceTestStepBase.run(self,context)

        self._logger.info("RunAudioStreaming: Run")

        report_root = self._device.get_report_tree().get_report_path()
        testcase = self._testcase_name.split("\\")[-1]
        self.log_directory = os.path.join(report_root, testcase, "RunAudioStreaming")
        if os.path.exists(self.log_directory)==False:
            os.makedirs(self.log_directory)

        log_file_path = os.path.join(self.log_directory, "monitor_thread.log")
        log_format = logging.Formatter('%(asctime)s - %(name)s - %(message)s',"%Y-%m-%d %H:%M:%S")
        log_file_handler = logging.FileHandler(filename=log_file_path)
        log_file_handler.setLevel(logging.DEBUG)
        log_file_handler.setFormatter(log_format)
        self._logger.addHandler(log_file_handler)


        test_passed = self.audio_streaming_loop()
        log_file_handler.close()

        if not test_passed:
            raise DeviceException(DeviceException.OPERATION_FAILED, "RunAudioStreaming: FAILED")
        else:
            self._logger.info("RunAudioStreaming: Done")
