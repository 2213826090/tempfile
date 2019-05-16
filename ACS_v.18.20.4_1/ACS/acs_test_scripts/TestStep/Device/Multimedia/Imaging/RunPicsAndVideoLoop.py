"""
@summary: Swiches between Video Recording and Image capturing for the specified amount of time.
          PREREQUISITES:
          Installed application: a Camera application (currently only supports the Intel app in Android)
@since 6 October 2014
@author: Santhosh Reddy Dubbaka, Aamir H Khowaja
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

from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException
import acs_test_scripts.Utilities.OSBVUtilities as osbv_utils
from Device.DeviceManager import DeviceManager
import os
import sys
from datetime import datetime as datetimestamp
import datetime
import time

class RunPicsAndVideoLoop(DeviceTestStepBase):
    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        self._logger.debug(self._pars.id + ": Test step starting.")
        loop_count = 1
        num_pics_to_take = 2
        video_duration = 5
        # variable for the different resolution video recording.
        loop_cnt =0
        numPicsTaken = 0
        max_retries = 50
        retry = max_retries
        self.videos_saved = 0
        self.total_videos_saved = 0
        error_counts = {}
        video_file_type = "mp4"
        pic_file_type = "jpg"
        dut_os = self._device.get_device_os_path()
        # Set report path
        self.report_path = DeviceManager().get_global_config().campaignConfig.get("campaignReportTree").create_subfolder('Image_video_switch')
        # Create folder under temp_dir to put videos and pictures from device into.
        self.temp_dir = osbv_utils.test_step_temp_dir(self)
        self.host_save_folder = os.path.join(self.temp_dir, 'saved_videos_pictures')
        if not os.path.exists(self.host_save_folder):
            os.makedirs(self.host_save_folder)
        # The API is assigned to different variables to make use of VideoCamera and Camera features of Camera API easily.
        self._camera_api = self._device.get_uecmd("Camera")
        self._video_api = self._device.get_uecmd("Camera")
        self._file_api = self._device.get_uecmd("File")
        self._phone_system_api = self._device.get_uecmd("PhoneSystem")

        errorCount = {
                'camConnectionErr':0,
                'camStopped'      :0,
                'camNotResponding':0,
                'unclassifiedFail':0
                    }

        end_time = float(time.time()) + (float(self._pars.duration)) * 60
        try:

            self._camera_app = self._video_api.get_camera_version(self._pars.camera_app)
            self._video_api.camera_app_setup(self._camera_app, 'video')
            # Following calls will remove all files in the save directory(ies).
            for directory in self._video_api.device_save_directory:
                try:
                    self._phone_system_api.delete(directory + dut_os.sep + '*.*')
                except DeviceException:
                    self._logger.debug("{0}:  Directory {1} was already empty.".format(self._pars.id,directory))
            self._camera_api.camera_app_setup(self._camera_app, 'camera')
            # Following calls will remove all files in the save directory(ies).
            for directory in self._camera_api.device_save_directory:
                try:
                    self._phone_system_api.delete(directory + dut_os.sep + '*.*')
                except DeviceException:
                    self._logger.debug("{0}:  Directory {1} was already empty.".format(self._pars.id,directory))
            self._logger.debug(self._pars.id + ":  Starting camera application")
            while (time.time() < end_time): # Primary Test Loop
                # The following actions are executed in every loop:
                #  1. Open the camera app
                #  2. Start video capture for 5 sec
                #  3. Stop recording and switch to camera
                #  4. take pictures(2)
                #  4. Stop camera app
                #  6. Move the files to our save directory.
                return_code = 0
                self._logger.debug("%s:  Loop %d."%(self._pars.id,loop_count))
                loop_start_time = datetime.datetime.now()

                # Starts the VideoCamera app with and without restart based on the restart_app.
                self._video_api.launch_system_camera_application(checkTriglogMsg = True, reset = self._pars.restart_app)

                (return_code, reset_loop) = self._video_api.prepare_video_capture(error_counts=error_counts)

                if return_code == -1:
                    # Test has failed
                    self.cleanup_test(error_counts)
                    self.ts_verdict_msg = self._pars.id + ":  Test has failed after trying to capture video!"
                    raise DeviceException(DeviceException.OPERATION_FAILED, self._pars.id + ": Test has failed after trying to capture video!")
                if reset_loop:
                    # We must have hit a known issue.  Try to stop app to see if it helps and retry this iteration of the loop.
                    self._camera_api.stop_system_camera_application()
                    continue
                #Check for camera using front or back
                stat= self._camera_api.get_camera_in_use()
                if 'FRONT' in stat:
                    self._camera_api.change_camera("back")
                    self._logger.debug(self._pars.id +" : Switched to back camera. ")
                    time.sleep(2)
                elif 'BACK' in stat:
                    self._logger.debug(self._pars.id +" : Back camera was already selected. ")
                else :
                    self._logger.error(self._pars.id +" :Camera is not properly selected. ")
                    raise DeviceException(DeviceException.OPERATION_FAILED, self._pars.id +" :Camera is not properly selected. ")
                self._logger.debug( "%s :  Recording started. Waiting %d seconds until stopping."%(self._pars.id,video_duration))
                self._video_api.camera_application_start_stop_recording(video_duration)
                self._logger.debug(self._pars.id + ":  Video recording done.")
                # Starts the VideoCamera app with and without restart based on the restart_app.
                self._camera_api.launch_system_camera_application(checkTriglogMsg = True, reset = self._pars.restart_app)
                self._camera_api.camera_application_take_picture(num_pics_to_take)
                self._logger.debug(self._pars.id + ":  Image capture is done.")

                # Verify that the video was captured in SD card or eMMC.
                self._logger.debug(self._pars.id +" : Checking for video files in save directory(ies).")
                (video_found, reset_loop, self.videos_saved) = self._video_api.verify_video_creation(error_counts=error_counts, videos_saved = self.videos_saved)
                if reset_loop:
                    # We must have hit a known issue.  Try to stop app to see if it helps and retry this iteration of the loop.
                    self._camera_api.stop_system_camera_application()
                    continue
                if not video_found:
                    # No video file was found and no known issues were hit. Video capture failed and now exiting thread.
                    self.cleanup_test(error_counts)
                    self.ts_verdict_msg = self._pars.id + ":  No video file was found and no known issues were hit, test has failed."
                    raise DeviceException(DeviceException.OPERATION_FAILED, self._pars.id + ": Failed to execute correctly.")
                #Clase the camera app.
                self._camera_api.stop_system_camera_application()
                retVal = int(self._camera_api.move_files_to_backup_dir(self._camera_api.device_save_directory))
                if (retVal - numPicsTaken) == 0:
                    # Check the error type and increment errorCount appropriately
                    errorCode = self._camera_api.check_for_camera_issues(errorCount)
                    # Reset the retry counter if this is a known issue
                    if errorCode != 'unclassifiedFail':
                        retry = max_retries
                    if retry < 1:
                        # The test failed
                        self.fail_test_cleanup(numPicsTaken, errorCount)
                        self.ts_verdict_msg = self._pars.id + ":  Too many loop retries, test has failed."
                        raise DeviceException(DeviceException.OPERATION_FAILED, self._pars.id + ": Failed to execute correctly.")
                    self._logger.debug(self._pars.id + ":  This iteration of image capture failed.\nAllowing %d more attempts..."%(retry-1))
                    retry -= 1
                    continue
                elif retry < max_retries:
                    # image_capture recovered, so reset the retry counter
                    retry = max_retries
                numPicsTaken = retVal
                # Pull the video files to the PC
                self._logger.debug(self._pars.id +" : Pulling files from device to the PC log directory")
                self._camera_api.upload_output_files(False, self.host_save_folder)
                self._video_api.upload_output_files(False, self.host_save_folder)
                self.total_videos_saved += self.videos_saved
                self.videos_saved = 0

                loop_time_delta = datetime.datetime.now() - loop_start_time
                self._logger.debug(self._pars.id + ": Loop"+ loop_count+" duration:"+ loop_time_delta.strftime("%H:%M:%S"))
                loop_count += 1
            self._logger.debug(" {2}:Test complete successfully. {0} videos recorded and {1} images captured ".format(self.total_videos_saved,numPicsTaken,self._pars.id))
            (result, output) = self._file_api.exist(self._video_api.backup_dir + dut_os.sep + "*." + video_file_type)

            if result:
                self._video_api.upload_output_files(False, self.host_save_folder)
            self.cleanup_test(error_counts)
            (result, output) = self._file_api.exist(self._camera_api.backup_dir + dut_os.sep + "*." + pic_file_type)
            if result:
                self._camera_api.upload_output_files(False, self.host_save_folder)

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
        self._logger.debug(self._pars.id + ": Test step finished.")

    def cleanup_test(self, error_counts):
        if len(error_counts) != 0:
            self._logger.debug(self._pars.id + ":  Known issues encountered:")
            for error in error_counts:
                self._logger.debug("%s  %s: %d"%(self._pars.id,error, error_counts[error]))
            self._logger.error(self._pars.id +" :  Pulling error screenshots and logcat files from DUT...")
            imageDir = os.path.join(self.report_path,'error_images_%s'%datetimestamp.fromtimestamp(time.time()).strftime('%Y-%m-%d_%H-%M-%S'))
            if not os.path.exists(imageDir):
                os.makedirs(imageDir)
            self._video_api.upload_output_files(True, imageDir)

