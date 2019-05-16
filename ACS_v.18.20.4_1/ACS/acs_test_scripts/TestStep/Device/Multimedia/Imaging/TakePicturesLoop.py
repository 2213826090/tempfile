"""
@summary: Repeatedly take pictures for the specified amount of time.
            PREREQUISITES:
                Installed application: a Camera application (currently only supports the Intel and Google Camera apps on Android)
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

from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException
import acs_test_scripts.Utilities.OSBVUtilities as osbv_utils
from Device.DeviceManager import DeviceManager
import os
import sys
import time
import random
from datetime import datetime

class TakePicturesLoop(DeviceTestStepBase):
    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        self._logger.info(self._pars.id + ": Test step starting.")
        # App signature to use with gui focus lock file
        appSignature = 'take_pictures_loop'
        try:
            # Delete any focus-lock file that may not have been released during the previous test run.
            osbv_utils.cleanup_focus_lock(appSignature)
        except:
            raise DeviceException(DeviceException.OPERATION_FAILED, self._pars.id + ": Issue trying to remove previous focus lock file.")
        loop = 1
        numPicsTaken = 0
        num_pics_to_take = 2
        # Number of consecutive retries in loop before it would fail due to too many.
        max_retries = 50
        retry = max_retries
        self.pic_file_type = "jpg"
        self._dut_os = self._device.get_device_os_path()
        # Set report path
        self._device_manager = DeviceManager()
        self.report_path = self._device_manager.get_global_config().campaignConfig.get("campaignReportTree").create_subfolder('take_pictures_loop')
        # Following call creates directory to store files created from TS that will generally not be needed unless an error is hit.
        self.temp_dir = osbv_utils.test_step_temp_dir(self)
        # Create folder under temp_dir to put pictures from device into.
        self.host_save_folder = os.path.join(self.temp_dir, 'saved_pics')
        # Timeout value to use that determines amount of time to wait on gui focus lock file to be removed.
        self.gui_lock_wait_time = self._pars.gui_lock_wait_time
        if not os.path.exists(self.host_save_folder):
            os.makedirs(self.host_save_folder)
        # Get UECmdLayer
        self._camera_api = self._device.get_uecmd("Camera")
        self._file_api = self._device.get_uecmd("File")
        self._camera_app = self._camera_api.get_camera_version(self._pars.camera_app)
        # Set camera application to image capture mode
        self._camera_api.camera_app_setup(self._camera_app, 'camera')

        start_time = float(time.time())
        end_time = start_time + (float(self._pars.duration)) * 60

        errorCount = {
                'camConnectionErr':0,
                'camStopped'      :0,
                'camNotResponding':0,
                'unclassifiedFail':0
                    }

        try:
            # Following calls will remove all files in the save directory(ies).
            for directory in self._camera_api.device_save_directory:
                try:
                    self._device.get_uecmd("PhoneSystem").delete(directory + self._dut_os.sep + '*.*')
                except DeviceException:
                    self._logger.info(self._pars.id + ":  Directory {0} was already empty.".format(directory))
            # Verify which camera app is used and run the below commands only if Intel camera is used.
            if self._camera_app in ("Android_Intel_Camera_v2.2", "Android_Intel_Refcam2_v0.9", "Android_Google_Camera_v2.4", "Android_Intel_RefCam_v1.0"):
                #Open the camera app to check the initial camera used(Front or Back)
                self._camera_api.launch_system_camera_application(checkTriglogMsg = True, reset = True)

                # Wait for 4 second to load the camera app. the first launch takes longer due to driver initialization
                time.sleep(4)

                # Check camera in use and if it is not what user has chosen to be used, change it.
                stat = self._camera_api.get_camera_in_use()
                if stat != self._pars.camera_to_use:
                    # We need to switch the camera in use.
                    if stat == "BACK":
                        self._camera_api.change_camera("front")
                    elif stat == "FRONT":
                        self._camera_api.change_camera("back")
                    else:
                        raise DeviceException(DeviceException.INVALID_DEVICE_STATE, self._pars.id + ": Camera used is " + stat + ", failing test.")

                    # Wait for 2 second to switch camera
                    time.sleep(2)

                    # Verify whether the camera is changed or not.
                    check_f = self._camera_api.get_camera_in_use()
                    if self._pars.camera_to_use == check_f:
                        self._logger.info("{0}:  {1} camera is now selected.".format(self._pars.id, self._pars.camera_to_use))
                    else:
                        self._logger.error("{0}:  Camera is not changed".format(self._pars.id))
                        raise DeviceException(DeviceException.INVALID_DEVICE_STATE, "{0}: Camera in use should be {1} but is still {2}.".format(self._pars.id, self._pars.camera_to_use, check_f))

            while (time.time() < end_time):
                self._logger.info(self._pars.id + ":  Starting loop {0}".format(loop))
                time_start1 = float(time.time())
                self._logger.info(self._pars.id + ":  Start image capturing:")
                if not osbv_utils.set_focus_lock(appSignature, timeout_sec=self.gui_lock_wait_time):
                    # Could not set the focus-lock
                    raise DeviceException(DeviceException.OPERATION_FAILED, self._pars.id + ": Failed to set focus lock.")
                if self._pars.restart_app:
                    # Stops and starts app.
                    self._camera_api.launch_system_camera_application(checkTriglogMsg = True, reset = True)
                else:
                    # Displays app if not started.
                    self._camera_api.launch_system_camera_application(checkTriglogMsg = True)

                # Wait for 1 second to re-load the camera app
                time.sleep(1)

                if self._camera_app in ("Android_Intel_Refcam2_v0.9", "Android_Intel_RefCam_v1.0"):
                    # take picture
                    if self._pars.camera_mode == "BURST":
                        #Settings for selecting burst mode
                        self._camera_api.toggle_burst_mode()
                        self._camera_api.camera_refcam_take_picture(1)
                    else:
                        self._camera_api.camera_refcam_take_picture(num_pics_to_take)
                else :
                    # Sleeps default of 5 seconds between each picture for a non-burst mode.
                    self._camera_api.camera_application_take_picture(num_pics_to_take)

                if not osbv_utils.release_focus_lock(appSignature):
                    # Could not release the focus-lock
                    raise DeviceException(DeviceException.OPERATION_FAILED, self._pars.id + ": Failed to release focus lock.")
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
                    self._logger.info(self._pars.id + ":  This iteration of image capture failed.  Allowing %d more attempts..."%(retry-1))
                    retry -= 1
                    continue
                elif retry < max_retries:
                    # image_capture recovered, so reset the retry counter
                    retry = max_retries
                numPicsTaken = retVal
                # Count iteration time
                self.delta(time_start1,1, loop)
                loop += 1
                if ((loop % 20) == 0):
                    self._logger.info(self._pars.id + ':  Copy newly captured images to host dir: {0}'.format(self.host_save_folder))
                    time_start2 = float(time.time())
                    #Pull pics to host every 20 iterations of loop.
                    self._camera_api.upload_output_files(False, self.host_save_folder)
                    self.delta(time_start2, 2, loop)
                # Sleep at random interval between min and max before moving towards next loop iteration.
                time.sleep(random.randint(self._pars.picture_interval_min, self._pars.picture_interval_max))
                self._logger.info(self._pars.id + ":  Number of pictures taken = %d"%numPicsTaken)
            # Times up! The test passed if it made it this far without encountering a failure and we've taken pictures.
            time_start2= float(time.time())
            self.delta(time_start2,2, loop)
            # The test passed unless 0 pictures were taken.
            if numPicsTaken < 1:
                self._logger.info(self._pars.id + ' is at the end of the test but has {0} pictures were taken... faling test.'.format(numPicsTaken))
                self.fail_test_cleanup(numPicsTaken, errorCount)
                self.ts_verdict_msg = self._pars.id + ":  Finished loop without any pictures taken!"
                raise DeviceException(DeviceException.OPERATION_FAILED, self._pars.id + ": Failed to execute correctly.")
            else:
                self.pass_test_cleanup(numPicsTaken, errorCount)
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
                    raise DeviceException(DeviceException.OPERATION_FAILED, self._pars.id + ": Failed to release focus lock.")
        self._logger.info(self._pars.id + ": Test step finished.")
        #Close the camera after the test is completed.
        self._camera_api.stop_system_camera_application()

    def delta(self, time_start, flag, loop):
        time_stop = float(time.time())
        sr = round((time_stop - time_start),0)
        s = int("%d"%sr)
        hours = s // 3600
        s = s - (hours*3600)
        minutes = s//60
        seconds = s - (minutes * 60)
        run_duration = '%sh:%sm:%ss'%(hours,minutes,seconds)
        if flag==1:
            self._logger.info(self._pars.id + ":  Test duration for Loop " + str(loop) + ": " + str(run_duration))
        if flag==2:
            self._logger.info(self._pars.id + ":  duration of pulling pics to host: " + str(run_duration))

    def fail_test_cleanup(self, numPicsTaken, errorCount):
        '''
            Perform any needed cleanup and summary logging
        '''
        (result, output) = self._file_api.exist(self._camera_api.backup_dir + self._dut_os.sep + "*." + self.pic_file_type)
        if result:
            self._logger.info(self._pars.id + ':  Copy the remaining captured images to save directory')
            self._camera_api.upload_output_files(False, self.host_save_folder)
        pull_screenshots = False
        self._logger.error(self._pars.id + ":  The test FAILED due to unresolvable camera failures!")
        self._logger.error(self._pars.id + ":  Summary:")
        self._logger.error(self._pars.id + ":  Images captured successfully before failure: %d"%numPicsTaken)
        self._logger.error(self._pars.id + ":  Errors encountered by type:")
        for camErr in errorCount:
            self._logger.error(self._pars.id + ":  %s: %d"%(camErr,errorCount[camErr]))
            if errorCount[camErr] > 0:
                pull_screenshots = True
        if pull_screenshots == True:
            self._logger.error(self._pars.id + ":  Pulling error screenshots and logcat files from DUT...")
            imageDir = os.path.join(self.report_path,'error_images_%s'%datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d_%H-%M-%S'))
            if not os.path.exists(imageDir):
                os.makedirs(imageDir)
            self._camera_api.move_files_to_backup_dir(self._camera_api.device_save_directory)
            self._camera_api.upload_output_files(True, imageDir)


    def pass_test_cleanup(self, numPicsTaken, errorCount):
        '''
            Perform any needed cleanup and summary logging
        '''
        (result, output) = self._file_api.exist(self._camera_api.backup_dir + self._dut_os.sep + "*." + self.pic_file_type)
        if result:
            self._logger.info(self._pars.id + ':  Copy the remaining captured images to save directory')
            self._camera_api.upload_output_files(False, self.host_save_folder)
        pull_screenshots = False
        self._logger.info(self._pars.id + ':  Test complete successfully')
        self._logger.info(self._pars.id + ":  Summary:")
        self._logger.info(self._pars.id + ":  Images captured successfully: %d"%numPicsTaken)
        self._logger.error(self._pars.id + ":  Errors encountered by type:")
        for camErr in errorCount:
            self._logger.error(self._pars.id + ":  %s: %d"%(camErr,errorCount[camErr]))
            if errorCount[camErr] > 0:
                pull_screenshots = True
        if pull_screenshots == True:
            self._logger.error(self._pars.id + ":  Pulling error screenshots and logcat files from DUT...")
            imageDir = os.path.join(self.report_path,'error_images_%s'%datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d_%H-%M-%S'))
            if not os.path.exists(imageDir):
                os.makedirs(imageDir)
            self._camera_api.move_files_to_backup_dir(self._camera_api.device_save_directory)
            self._camera_api.upload_output_files(True, imageDir)
