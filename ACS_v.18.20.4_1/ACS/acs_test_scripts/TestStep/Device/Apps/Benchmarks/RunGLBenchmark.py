"""
@summary: This file implements a Test Step to run the GLBenchmark application (either GLBenchmark 2.7 or GLBench3.0) for a specified duration.
Prerequisites for running GLbenchmark 2.7: the following script must be located in the path specified by SCRIPTS_PATH:
    run_glbenchmark_27.sh
This can be found in Artifactory at acs_test_artifacts/CONCURRENCY/TESTS/GLBenchmark.
In addition, the following APK should be installed on the DUT:
    GLBenchmark_2_7_0_Release_a68901_Android_Corporate_Package.apk or GFXBenchGL30.apk.
They can be found in the Artifactory at acs_test_artifacts/BENCHMARKS/GLBENCHMARK and acs_test_artifacts/CONCURRENCY/TESTS/GFXBENCH30
respectively.@since 26 June 2014
@since 1 October 2014
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
from Device.DeviceManager import DeviceManager
from ErrorHandling.AcsConfigException import AcsConfigException
import time
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import Global


class RunGLBenchmark(DeviceTestStepBase):


    touchCmds_720P = {
        'start_btn': [360, 590]
                    }
    touchCmds_1080P = {
        'start_btn': [534, 894]
         }
    touchCmds_25x16 = {
        'start_btn': [1283, 763]
                   }

    def run(self, context):

        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        self._logger.debug(self._pars.id +" : Test starting")

        if self._pars.version == 3.0 :

            self.gfx_package = "com.glbenchmark.glbenchmark27.corporate"
            self.gfx_activity = "net.kishonti.benchui.corporate.SplashScreenCorporate"

            phone_system_api = self._device.get_uecmd("PhoneSystem")
            keyevent_api = self._device.get_uecmd("KeyEvent")
            _system_api = self._device.get_uecmd("System")
            _app_api = self._device.get_uecmd("AppMgmt")
            disp_width_height = phone_system_api.get_screen_resolution()
            disp_width = disp_width_height.split("x")[0]
            touchCmds = None
            if disp_width == "720":
                touchCmds = self.touchCmds_720P
            elif disp_width == "1080":
                touchCmds = self.touchCmds_1080P
            elif disp_width == "2560":
                touchCmds = self.touchCmds_25x16
            else:
                self._logger.error("{1} : Display resolution {0} is not supported. Some UECmds might not work.".format(disp_width,self._pars.id))
                raise DeviceException(DeviceException.OPERATION_FAILED, self._pars.id+ " : Display resolution is not supported. ")

            end_time = float(time.time()) + (float(self._pars.duration)) * 60

            while (time.time() < end_time):

                package_activity = self.gfx_package+"/"+self.gfx_activity
                _app_api.launch_app(intent=package_activity)
                time.sleep(15)
                keyevent_api.tap_on_screen(touchCmds['start_btn'][0], touchCmds['start_btn'][1])

            # Check for system UI
            systemui_ok =phone_system_api.check_process("systemui")
            if systemui_ok :
                self._logger.debug(self._pars.id+" : Systemui is active")
            else:
                self._logger.error(self._pars.id+" : Systemui is not active.")
                raise DeviceException(DeviceException.OPERATION_FAILED, self._pars.id+" : Systemui is not active.")
            # Check for the gfx status.
            gfx_stat = phone_system_api.check_process("com.glbenchmark.glbenchmark27.corporate")
            if gfx_stat :
                self._logger.debug(self._pars.id+" : benchmark app is still running")
            else:
                self._logger.error(self._pars.id+" : benchmark app is closed unexpectedly .")
                raise DeviceException(DeviceException.OPERATION_FAILED, self._pars.id+" : benchmark app is closed unexpectedly .")
            _system_api.stop_app(self.gfx_package)
            self._logger.debug(self._pars.id+" : Done")
        elif self._pars.version == 2.7 :
            cmd = "adb shell cd %s; sh run_glbenchmark_27.sh %d"%(self._pars.scripts_path, self._pars.duration)

            #Added generous amount to timeout since the app may go over the desired time.
            verdict, output = self._device.run_cmd(cmd, (self._pars.duration+8)*60)
            if verdict == Global.FAILURE:
                raise DeviceException(DeviceException.OPERATION_FAILED," %s : run_cmd failed with message '%s'"%(self._pars.id,output))
            else:
                self._logger.debug(self._pars.id+" : Done")

        else:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, self._pars.id+" : Version entered is not supported")

