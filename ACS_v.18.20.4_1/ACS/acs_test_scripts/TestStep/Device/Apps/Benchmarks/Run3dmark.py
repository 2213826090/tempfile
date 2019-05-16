"""
@summary: This file implements a Test Step to run the 3D mark for a specified duration.
PREREQUISITES: The 3Dmark application should be downloaded and installed before
running the test. The application can be found at CONCURRENCY/TESTS/3d_mark/com.futuremark.dmandroid.application-1.apk
@since 19 October 2014
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
import time


class Run3dmark (DeviceTestStepBase):

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self.phonesystem_api=self._device.get_uecmd("PhoneSystem")
        self._keyevent_api = self._device.get_uecmd("KeyEvent")
        self.system_api=self._device.get_uecmd("System")
        self.app_api = self._device.get_uecmd("AppMgmt")

    # Override run method, this is the execution loop for the test.
    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)

        self._logger.debug(self._pars.id + " : Test step starting.")
        self._3dmark_package="com.futuremark.dmandroid.application"
        self._3dmark_activity="activity.MainActivity"

        # Start test control loop.
        # Number of seconds to run the bench mark for before restarting it.
        benchRun = 100

        pkg_activity = self._3dmark_package+"/."+ self._3dmark_activity
        # Making sure that 3d mark application is installed.
        self.app_api.launch_app(intent=pkg_activity)
        app_ok =self.phonesystem_api.check_process(self._3dmark_package)
        if app_ok :
            self._logger.debug(self._pars.id + " : 3D Mark is installed")
            self.system_api.stop_app(self._3dmark_package)
        else:
            self._logger.error(self._pars.id + " : 3D Mark is not installed.")
            raise DeviceException(DeviceException.OPERATION_FAILED,self._pars.id + " :3D Mark is not installed.")
        # Start the 3D mark test control loop.
        end_time = float(time.time()) + (float(self._pars.duration)) * 60
        while (time.time() < end_time):
            # Launch the 3D mark application.
            self.app_api.launch_app(intent=pkg_activity)
            time.sleep(15)

            # For the drop down option by sending keyboard presses
            # At this point the pull down menu is showing and then selecting the run.
            keyevent_list = ["DPAD_DOWN","DPAD_RIGHT","ENTER","DPAD_DOWN","DPAD_DOWN","ENTER"]
            self._keyevent_api.scenario(keyevent_list,5)

            # Calculate bench mark run end time
            benchRunEndTime = time.time() + benchRun

            # Start the bench run control loop
            while((benchRunEndTime > time.time()) and (end_time > time.time())):
                # Check the status every few seconds
                time.sleep(10)

                # Check for system UI
                systemui_ok =self.phonesystem_api.check_process("systemui")
                if systemui_ok :
                    self._logger.debug(self._pars.id + ": Systemui is active")

                else:
                    self._logger.error(self._pars.id + ": Systemui is not active.")
                    raise DeviceException(DeviceException.OPERATION_FAILED,self._pars.id + " : Systemui is not active.")

                # Check that the app is still alive
                app_ok =self.phonesystem_api.check_process(self._3dmark_package)
                if app_ok :
                    self._logger.debug(self._pars.id + " :3D Mark apk is ok")
                else:
                    self._logger.error(self._pars.id + " :3D Mark apk Crashed.")
                    raise DeviceException(DeviceException.OPERATION_FAILED, self._pars.id + " :3D Mark apk Crashed.")

            # Close out the application.
            self.system_api.stop_app(self._3dmark_package)
            #Sleep time for the application to close and start the application in the next iteration.
            time.sleep(10)

        self._logger.debug(self._pars.id + " : test finished.")