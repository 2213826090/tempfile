"""
@summary: This file implements a Test Step that exercises some basic UI functions.
It opens and closes the Settings app, swipes up/down/right/left, and opens and closes the Calculator app.
*** This test currently runs only on android.
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
import time
import math
from ErrorHandling.DeviceException import DeviceException

class UiNavigation(DeviceTestStepBase):


    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

        self._api = self._device.get_uecmd("PhoneSystem")
        self._system_api = self._device.get_uecmd("System")
        self._appmgmt = self._device.get_uecmd("AppMgmt")

        self._keyevent_api = self._device.get_uecmd("KeyEvent")
        phone_resolution = self._api.get_screen_resolution()
        self.disp_width = phone_resolution.split('x', 1)[0]
        self.disp_height = phone_resolution.split('x', 1)[1]
        self.swipe_vert_coord = [
            math.ceil(int(self.disp_width)/2),
            math.ceil(int(self.disp_height)*.25),
            math.ceil(int(self.disp_width)/2),
            math.floor(int(self.disp_height)*.75)
            ]

        self.swipe_hori_coord = [
            math.ceil(int(self.disp_width)*.25),
            math.ceil(int(self.disp_height)/2),
            math.floor(int(self.disp_width)*.75),
            math.ceil(int(self.disp_height)/2)
            ]

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """

        DeviceTestStepBase.run(self, context)
        end_time = float(time.time()) + (float(self._pars.duration)) * 60
        while (time.time() < end_time):
            self._keyevent_api.scenario(["HOME"],1)
            #Swipe Right
            for i in range(0,5):
                self._keyevent_api.swipe(self.swipe_hori_coord[2],self.swipe_hori_coord[3],95,self.swipe_hori_coord[0],self.swipe_hori_coord[1])
            #Swipe Left
            for i in range(5):
                self._keyevent_api.swipe(self.swipe_hori_coord[0],self.swipe_hori_coord[1],95,self.swipe_hori_coord[2],self.swipe_hori_coord[3])
            #Open Settings menu
            self._api.settings_menu()
            time.sleep(3)
            #Swipe Down
            for i in range(3):
                self._keyevent_api.swipe(self.swipe_vert_coord[2],self.swipe_vert_coord[3],95,self.swipe_vert_coord[0],self.swipe_vert_coord[1])
            #Swipe up
            for i in range(3):
                self._keyevent_api.swipe(self.swipe_vert_coord[0],self.swipe_vert_coord[1],95,self.swipe_vert_coord[2],self.swipe_vert_coord[3])
            #Close Settings menu
            self._system_api.stop_app("com.android.settings")
            #Open Calculator
            self._appmgmt.launch_app(intent ="com.android.calculator2/.Calculator")
            time.sleep(5)
            #Close Calculator
            self._system_api.stop_app("com.android.calculator2")

            # Check for system UI
            systemui_ok =self._api.check_process("systemui")
            if systemui_ok :
                self._logger.debug("Ui Navigation test: Systemui is active")

            else:
                self._logger.error("Ui Navigation test: Systemui is not active.")
                raise DeviceException(DeviceException.OPERATION_FAILED, "Ui Navigation test:Systemui is not active.")

        self._logger.debug("Ui Navigation test completed. ")

