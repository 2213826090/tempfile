"""
@summary: This file implements a Test Step to run CV_Different_Format_Video_Capture.apk on the device.  This
    application was created by the DSV FV (functional validation) team, and can be
    found in Artifactory under acs_test_artifacts/CONCURRENCY/TESTS/Different_Format_Video_Capture.
    This Test records different format videos and move the videos to host.
@since 5 October 2014
@author: Santhosh Reddy D
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
import acs_test_scripts.Utilities.OSBVUtilities as osbv_utils
from Device.DeviceManager import DeviceManager
import os
import time


class RunVideoEncodingDiffFormat(DeviceTestStepBase):
    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self.system_api = self._device.get_uecmd("System")
        self._keyevent_api = self._device.get_uecmd("KeyEvent")
        self._phone_system_api = self._device.get_uecmd("PhoneSystem")
        self._appmgmt = self._device.get_uecmd("AppMgmt")

        self.left= ["DPAD_LEFT"]
        self.enter_twice =["ENTER","ENTER"]


    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)

        self._logger.debug(self._pars.id+" : Run")
        _dut_os = self._device.get_device_os_path()
        self.video_save_directory="/sdcard/Pictures/MyCameraApp/"
        PackageActivity = self._pars.app_package+"/."+self._pars.activity

        # Starting the application intially to make sure the MyCameraApp directory is present before trying to empty the directory.
        self._appmgmt.launch_app(intent =PackageActivity)
        time.sleep(2)
        # input Pad Left key to focus the capture button
        self._keyevent_api.scenario(self.left,5)
        # The enter's are for starting and stopping the video record, 5sec timeout between them to record video for 5 sec.
        self._keyevent_api.scenario(self.enter_twice,5)
        # Following will remove all files in the save directory
        self._phone_system_api.delete(self.video_save_directory + _dut_os.sep + '*.*')
        # Create folder under temp_dir to put videos from device into.
        self.temp_dir = osbv_utils.test_step_temp_dir(self)
        self.host_save_folder = os.path.join(self.temp_dir, 'saved_videos')
        if not os.path.exists(self.host_save_folder):
            os.makedirs(self.host_save_folder)
            self._logger.debug(self._pars.id+" :  Directory was already empty.")

        # Calculate test end time
        end_time = float(time.time()) + (float(self._pars.duration)) * 60

        while (time.time() < end_time):
            # The enter's are for starting and stopping the video record, 5sec timeout between them to record video for 5 sec.
            self._keyevent_api.scenario(self.enter_twice,5)
            time.sleep(2)

            # Check for system UI
            systemui_ok =self._phone_system_api.check_process("systemui")
            if systemui_ok:
                self._logger.debug(self._pars.id+" : Systemui is active")
            else:
                self._logger.error(self._pars.id+" : Systemui is not active.")
                raise DeviceException(DeviceException.OPERATION_FAILED, self._pars.id+" :Video Encoding Diff Format test : Systemui is not active.")

        #Closing the app
        self.system_api.stop_app(self._pars.app_package)
        self._logger.debug(self._pars.id+" :  Pulling video files from device to the PC log directory %s"%(self.host_save_folder))
        # copy all the video files to host
        # Pull the video files to the PC
        self._device.pull(self.video_save_directory,self.host_save_folder,timeout=180)


        self._logger.debug(self._pars.id+" : Done")
