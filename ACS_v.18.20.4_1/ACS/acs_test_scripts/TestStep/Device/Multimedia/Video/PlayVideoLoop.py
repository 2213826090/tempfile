"""
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
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

:organization: INTEL PEG SVE DSV
:summary: This file implements a Test Step to repeatedly play video files for the specified amount of time.
:since: 07/14/14
:author: jongyoon
"""

import os
import time
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException
from Device.DeviceManager import DeviceManager

class PlayVideoLoop(DeviceTestStepBase):

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Constructor
        """
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

        # Load Multimedia ue command.
        self._multimedia_api = self._device.get_uecmd("Multimedia")
        self._video_api = self._device.get_uecmd("Video")
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")
        self._system_api = self._device.get_uecmd("System")

        self._device_manager = DeviceManager()

        self._device_logger = self._device.get_device_logger()

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        DeviceTestStepBase.run(self, context)

        self._logger.debug("START PLAY_VIDEO_LOOP Run Phase")

        #Get Video Playback parameters
        videoFile          = self._pars.video_file
        runtime_seconds     = self._pars.duration * 60
        vsp_enabled         = self._pars.intel_smart_video_enabled
        hwc_enabled         = self._pars.hwc_enabled

        monitor_dropped_frames = self._pars.monitor_dropped_frames

        report_path =  self._device_manager.get_global_config().campaignConfig.get("campaignReportTree").get_report_path()
        tc_name = self._tc_parameters.get_name()
        test_files_root = os.path.join(report_path, tc_name, 'play_video_loop')
        vlist = videoFile.replace(', ', ',')
        num_of_videos = len(videoFile.split(','))

        # Start to monitor video decoder error conditions
        video_decoder_error_messages = self._device.get_video_decoder_error_messages()
        self._system_api.start_log_monitoring(device_logger=self._device_logger, target_messages=video_decoder_error_messages)

        # Start to monitor VSP error conditions
        if vsp_enabled == True:
            video_enhancer_error_messages = self._device.get_video_decoder_error_messages()
            self._system_api.start_log_monitoring(device_logger=self._device_logger, target_messages=video_enhancer_error_messages)

        #Disable hardware composer if the HWC_ENABLED parameter is false
        if hwc_enabled == False:
            self._multimedia_api.set_hardware_composer(enable=False)

        #Enabled Intel Smart Video if the INTEL_SMART_VIDEO_ENABLED parameter is true
        if vsp_enabled == True:
            self._multimedia_api.set_intel_smart_video(enable=True)

        start_time = time.time()
        if (num_of_videos== 1) :

            self._video_api.play(filename=videoFile, loop=True, screen_orientation='landscape')
        else:
            Package = "com.example.gdc_cv_videoloop"
            triglogAppmsg = [["PackageActivity " + Package ,"CV Video Loop: Video execution path: file:"]]

            # Add triglogs for applications and drivers
            for triglog in triglogAppmsg:
                self._device_logger.add_trigger_message(triglog[1])
            self._multimedia_api.play_video_list(vlist,self._pars.duration,screen_orientation='landscape')

            #Check logs for messages that indicate successful start of the activity
            for triglog in triglogAppmsg:
                time.sleep(1)
                messages= self._device_logger.get_message_triggered_status(triglog[1])
                if len(messages) == 0:
                    msg = triglog[0] + " failed to start properly. Could not find this message in the log: " + triglog[1]
                    raise DeviceException(DeviceException.OPERATION_FAILED, msg)

            #Remove trigger messages for errors
            for triglog in triglogAppmsg:
               self._device_logger.remove_trigger_message(triglog[1])

        #Wait for 10 seconds to check play back status
        time.sleep(10)

        while time.time()-start_time < runtime_seconds:
            if (num_of_videos== 1) :

                (is_playing, played_duration) = self._video_api.is_playing()

            else :
                #Check whether the application is running.
                is_playing =self._phonesystem_api.check_process(Package)

            if not is_playing:
                msg = "Video playback is not working.\n"
                self.ts_verdict_msg = msg
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)
            # If set to true, we'll not be able to rely on this thread because USB connection for ADB will be disconnected periodically and therefore won't start it.
            if not monitor_dropped_frames:
                self._logger.debug("Starting to check for dropped frames.\n")

                (cmd_output, no_total_frames, no_dropped_frames) = self._multimedia_api.get_number_of_dropped_frames()

                percentage_frames_dropped = float(no_dropped_frames)/float(no_total_frames)
                if percentage_frames_dropped > 0.1:
                    self._logger.error("Exceeded dropped frame rate of 10%. Dropped rate is %.1f%%. Failing test"%percentage_frames_dropped)
                    frames_dropped_log_file = open(os.path.join(test_files_root, "frames_dropped.log"), 'a')
                    frames_dropped_log_file.write(cmd_output)
                    frames_dropped_log_file.close()

            self._logger.debug("Videoplayback: Passed {0} seconds".format(round(time.time()-start_time, 0)))
            time.sleep(30)

        # Check for system UI
        systemui_ok =self._phonesystem_api.check_process("systemui")
        if systemui_ok == False:
            self._logger.error("Systemui is not active.")
            raise DeviceException(DeviceException.OPERATION_FAILED, "Systemui is not active.")
        else:
            self._logger.info("Systemui is active")
        if (num_of_videos== 1) :

            self._video_api.stop()
        else:
            self._system_api.stop_app(Package)
        #Verify if frame drop occurred
        if os.path.exists(os.path.join(test_files_root, 'frame_dropped.log')):
            msg = "Too many frames were dropped. Failing test."
            self.ts_verdict_msg = msg
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        #Verify if logs for error messages were triggered
        self._system_api.check_logged_errors(device_logger=self._device_logger, target_messages=video_decoder_error_messages)

        #Stop to monitor video decoder error conditions
        self._system_api.stop_log_monitoring(device_logger=self._device_logger, target_messages=video_decoder_error_messages)

        if vsp_enabled == True:
            # verify VSP error messages are triggered
            self._system_api.check_logged_errors(device_logger=self._device_logger, target_messages=video_enhancer_error_messages)

            # Stop to monitor VSP error conditions
            self._system_api.stop_log_monitoring(device_logger=self._device_logger, target_messages=video_enhancer_error_messages)

            # Turn off VSP features by writing to the Intel Smart Video shared preferences file.
            self._multimedia_api.set_intel_smart_video(enable=False)

        # Turn on HWC features
        if hwc_enabled == False:
            self._multimedia_api.set_hardware_composer(enable=True)


        self._logger.info("Video playback PASSED")

        self._logger.debug("FINISHED PLAY_VIDEO_LOOP Run Phase")
