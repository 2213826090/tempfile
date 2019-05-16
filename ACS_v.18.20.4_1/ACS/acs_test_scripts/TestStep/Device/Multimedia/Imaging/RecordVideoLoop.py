"""
@summary: Repeatedly take videos for the specified amount of time.
          PREREQUISITES:
          Installed application: a Camera application (currently only supports the Intel and Google Camera apps in Android)
@since 17 July 2014
@author: Stephen A Smith, Aamir H Khowaja
@organization: INTEL PEG-SVE-DSV, TMT TTD AN

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

from ErrorHandling.AcsConfigException import AcsConfigException
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException
import acs_test_scripts.Utilities.OSBVUtilities as osbv_utils
from Device.DeviceManager import DeviceManager
import os
import sys
from datetime import datetime as datetimestamp
import datetime
import random
import time

class RecordVideoLoop(DeviceTestStepBase):
    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        self._logger.info(self._pars.id + ": Test step starting.")
        # App signature to use with gui focus lock file
        appSignature = 'record_video_loop'
        try:
            # Delete any focus-lock file that may not have been released during the previous test run.
            osbv_utils.cleanup_focus_lock(appSignature)
        except:
            raise DeviceException(DeviceException.OPERATION_FAILED, self._pars.id + ": Issue trying to remove previous focus lock file.")
        loop_count = 1
        # variable for the different resolution video recording.
        loop_cnt =0
        self.videos_saved = 0
        self.total_videos_saved = 0
        error_counts = {}
        self.video_file_type = "mp4"
        # Timeout value to use that determines amount of time to wait on gui focus lock file to be removed.
        self.gui_lock_wait_time = self._pars.gui_lock_wait_time
        self._dut_os = self._device.get_device_os_path()
        # Set report path
        self._device_manager = DeviceManager()
        # Create folder under report path to put files into when there is an issue.
        self.report_path = self._device_manager.get_global_config().campaignConfig.get("campaignReportTree").create_subfolder('record_video_loop')
        # Following call creates directory to store files created from TS that will generally not be needed unless an error is hit.
        self.temp_dir = osbv_utils.test_step_temp_dir(self)
        # Create folder under temp_dir to put videos from device into.
        self.host_save_folder = os.path.join(self.temp_dir, 'saved_videos')
        if not os.path.exists(self.host_save_folder):
            os.makedirs(self.host_save_folder)
        # Get UECmdLayer
        self._camera_api = self._device.get_uecmd("Camera")
        self._keyevent_api = self._device.get_uecmd("KeyEvent")
        self._file_api = self._device.get_uecmd("File")
        self._phone_system_api = self._device.get_uecmd("PhoneSystem")
        self._camera_app = self._camera_api.get_camera_version(self._pars.camera_app)
        if self._camera_app == "Android_Intel_Camera_v2.2" or self._camera_app == "Android_Google_Camera_v2.0":
            self._camera_api.camera_app_setup(self._camera_app, 'video')
        else:
            error_msg = self._pars.id + ":  Selected camera app: {0} is not supported".format(self._camera_app)
            self._logger.error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        start_time = float(time.time())
        end_time = start_time + (float(self._pars.duration)) * 60
        try:
            # Following calls will remove all files in the save directory(ies).
            for directory in self._camera_api.device_save_directory:
                try:
                    self._phone_system_api.delete(directory + self._dut_os.sep + '*.*')
                except DeviceException:
                    self._logger.info(self._pars.id + ":  Directory {0} was already empty.".format(directory))
            while (time.time() < end_time): # Primary Test Loop
                # The following actions are executed in every loop:
                #  1. Open the camera app
                #  2. Start video capture
                #  3. Wait a random time period defined by user params
                #  4. Stop video capture
                #  5. Verify video file was created
                #  6. Move the video to our save directory.
                #  7. If loop%self._pars.video_upload_modulus==0, pull save directory to the PC and delete them from the device
                return_code = 0
                video_duration = random.randint(self._pars.video_interval_min, self._pars.video_interval_max)
                self._logger.info(self._pars.id + ":  Loop %d."%(loop_count))
                loop_start_time = datetime.datetime.now()
                # Lock the GUI focus before starting video_capture so that video capture doesn't conflict with other apps.
                if not osbv_utils.set_focus_lock(appSignature, timeout_sec=self._pars.gui_lock_wait_time):
                    # Could not set the focus-lock
                    raise DeviceException(DeviceException.OPERATION_FAILED, self._pars.id + ":  Video Capture failed to set the focus-lock!")
                self._logger.debug(self._pars.id + ":  Starting camera application")

                # Prepare video capture by making sure it is correctly launched and in gui focus.
                (return_code, reset_loop) = self._camera_api.prepare_video_capture(error_counts = error_counts, restart_app = self._pars.restart_app, checkTriglogMsg = True)

                if return_code == -1:
                    # Test has failed
                    #[12/19/14: Jong ] After failure, there does not exist a directory to clean out
                    #self.cleanup_test(error_counts)
                    self.ts_verdict_msg = self._pars.id + ":  Test has failed after trying to capture video!"
                    raise DeviceException(DeviceException.OPERATION_FAILED, self._pars.id + ": Test has failed after trying to capture video!")

                if reset_loop:
                    # Check if focus lock still exists with video capture's appSignature and remove if so.
                    if appSignature == osbv_utils.get_lock_signature():
                        if not osbv_utils.release_focus_lock(appSignature):
                            # Unable to release the focus-lock
                            raise DeviceException(DeviceException.OPERATION_FAILED, self._pars.id + ":  Video_Capture failed to release the focus-lock!")
                    continue
                # Recording videos of different resolution.
                if self._pars.record_type == 'DIFFRES':
                    device_orient=self._camera_api.device_orientation()
                    # Check camera in use.  We want "BACK" (world facing) camera.
                    stat = self._camera_api.get_camera_in_use()
                    if stat == "FRONT":
                        # Change from "FRONT" to "BACK".
                        self._camera_api.change_camera("back")
                        # Verify whether the camera is changed or not.
                        check_f = self._camera_api.get_camera_in_use()
                        if check_f == "BACK":
                            self._logger.debug("{0}:  'BACK' camera is now selected.".format(self._pars.id))
                        else:
                            self._logger.error("{0}:  Camera is not changed to 'BACK'".format(self._pars.id))
                            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, "{0}: Camera in use should be 'BACK' but is still {1}.".format(self._pars.id, check_f))
                    elif stat == "BACK":
                        self._logger.debug("{0}:  Back camera is selected.".format(self._pars.id))
                    else:
                        self._logger.error("{0}:  Camera is not properly selected.".format(self._pars.id))
                        raise DeviceException(DeviceException.INVALID_DEVICE_STATE, self._pars.id + ":  Camera is not properly selected.")
                    self._camera_api.select_camera_settings()
                    # Switch between different resolution options:  SD, HD, HD(HS), Full HD, Full HD(HS)
                    if device_orient== 'portrait' :
                        keyevent_list= ["DPAD_RIGHT","DPAD_RIGHT","DPAD_RIGHT","ENTER"]
                        for i in range(0,loop_cnt%5) :
                            keyevent_list.append("DPAD_RIGHT")

                    elif device_orient== 'landscape' :
                        keyevent_list= ["DPAD_DOWN","DPAD_DOWN","DPAD_DOWN","ENTER"]
                        for i in range(0,loop_cnt%5) :
                            keyevent_list.append("DPAD_DOWN")
                    else:
                        self._logger.error(self._pars.id + ":  " + device_orient + " is not a proper orientation. ")
                        raise DeviceException(DeviceException.OPERATION_FAILED, self._pars.id + ":  " + device_orient + " is not a proper orientation.")
                    keyevent_list.append("ENTER")
                    self._keyevent_api.scenario(keyevent_list,1)
                    time.sleep(2)
                    loop_cnt+=1
                self._logger.debug(self._pars.id + ":  Recording started. Waiting %d seconds until stopping."%video_duration)
                self._camera_api.camera_application_start_stop_recording(video_duration)
                self._logger.info(self._pars.id + ":  Video recording done.")
                # Release the GUI focus lock so that other apps can take focus
                if not osbv_utils.release_focus_lock(appSignature):
                    # Unable to release the focus-lock
                    raise DeviceException(DeviceException.OPERATION_FAILED, self._pars.id + ":  Video Capture failed to release the focus-lock!")
                # Verify that the video was captured in SD card or eMMC.
                self._logger.info(self._pars.id + ':  Checking for video files in save directory(ies).')
                (video_found, reset_loop, self.videos_saved) = self._camera_api.verify_video_creation(error_counts = error_counts, videos_saved = self.videos_saved)
                if reset_loop:
                    # We must have hit a known issue.  Try to stop app to see if it helps and retry this iteration of the loop.
                    self._camera_api.stop_system_camera_application()
                    continue
                if not video_found:
                    # No video file was found and no known issues were hit. Video capture failed and now exiting thread.
                    self.cleanup_test(error_counts)
                    self.ts_verdict_msg = self._pars.id + ":  No video file was found and no known issues were hit, test has failed."
                    raise DeviceException(DeviceException.OPERATION_FAILED, self._pars.id + ": Failed to execute correctly.")
                if loop_count%self._pars.video_upload_modulus == 0:
                    # Pull the video files to the PC
                    self._logger.info(self._pars.id + ':  Pulling video files from device to the PC log directory')
                    self._camera_api.upload_output_files(False, self.host_save_folder)
                    self.total_videos_saved += self.videos_saved
                    self.videos_saved = 0
                loop_time_delta = datetime.datetime.now() - loop_start_time
                loop_time_delta_hours = loop_time_delta.seconds/(60*60)
                loop_time_delta_minutes = (loop_time_delta.seconds-(loop_time_delta_hours*(60*60)))/60
                loop_time_delta_seconds = loop_time_delta.seconds-(loop_time_delta_hours*(60*60))-(loop_time_delta_minutes*60)
                self._logger.info(self._pars.id + ":  Loop %d duration: %02d:%02d:%02d"%(loop_count, loop_time_delta_hours, loop_time_delta_minutes, loop_time_delta_seconds))
                loop_count += 1
            self._logger.info(self._pars.id + ':  Test complete successfully. {0} videos recorded'.format(self.total_videos_saved))
            self.cleanup_test(error_counts)
            # Kill the camera app
            self._camera_api.stop_system_camera_application()
        except OSError as e:
            self._logger.error(self._pars.id + ":  OS Error({0}):  {1}".format(e.errno, e.strerror))
            raise
        except:
            import traceback
            self._logger.error(self._pars.id + ":  Unexpected exception -> " + str(sys.exc_info()[0]))
            self._logger.error(traceback.format_exc())
            self.ts_verdict_msg = self._pars.id + ":  Unexpected exception being raised"
            raise
        finally:
            # Check if focus lock still exists with video capture's appSignature and remove if so.
            if appSignature == osbv_utils.get_lock_signature():
                if not osbv_utils.release_focus_lock(appSignature):
                    # Unable to release the focus-lock
                    raise DeviceException(DeviceException.OPERATION_FAILED, self._pars.id + ":  Video_Capture failed to release the focus-lock!")
        self._logger.info(self._pars.id + ": Test step finished.")

    def cleanup_test(self, error_counts):
        (result, output) = self._file_api.exist(self._camera_api.backup_dir + self._dut_os.sep + "*." + self.video_file_type)
        if result:
            self._camera_api.upload_output_files(False, self.host_save_folder)
        if len(error_counts) != 0:
            self._logger.info(self._pars.id + ":  Issues encountered:")
            for error in error_counts:
                self._logger.info("  %s: %d"%(error, error_counts[error]))
            self._logger.error(self._pars.id + ":  Pulling error screenshots and logcat files from DUT...")
            imageDir = os.path.join(self.report_path,'error_images_%s'%datetimestamp.fromtimestamp(time.time()).strftime('%Y-%m-%d_%H-%M-%S'))
            if not os.path.exists(imageDir):
                os.makedirs(imageDir)
            self._camera_api.upload_output_files(True, imageDir)

