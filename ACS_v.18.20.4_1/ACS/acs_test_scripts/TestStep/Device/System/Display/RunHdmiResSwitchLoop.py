
"""
@summary: This file implements a Test Step to open the HDMI settings and repeatedly
switches the HDMI display resolution between 480 and 1080.
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

class RunHdmiResSwitchLoop(DeviceTestStepBase):
    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._hdmi_res = 1080
        self.phonesystem_api = self._device.get_uecmd("PhoneSystem")
        self._system_api = self._device.get_uecmd("System")
        self._app_api = self._device.get_uecmd("AppMgmt")
        self._keyevent_api = self._device.get_uecmd("KeyEvent")
        self.up= ["DPAD_UP"]
        self.hdmi_package = "com.intel.hdmi"
        self.hdmi_settings ="HDMISettings"
        self.PackageActivity = self.hdmi_package+"/."+self.hdmi_settings


    # Highlights the resolution to change to
    def __highlight_resolution(self):
        key_presses = 10
        if self._hdmi_res == 1080:
            for i in range(0,key_presses):
                time.sleep(1)
                self._keyevent_api.scenario(["DPAD_DOWN"],1)
            self._hdmi_res = 480
        else:
            for i in range(0,key_presses):
                time.sleep(1)

                self._keyevent_api.scenario(self.up,1)
            self._hdmi_res = 1080

    # Selects the Hdmi modes option.
    def __toggle_resolution(self):
        # Press up to select the Modes option
        self._keyevent_api.scenario(self.up,1)
        time.sleep(2)
        # Press Enter to open the mode select window
        self.enter = ["ENTER"]
        self._keyevent_api.scenario(self.enter,1)
        time.sleep(2)

        # highlights the resolution to change to
        self.__highlight_resolution()
        time.sleep(2)

        # Confirm the select resolution
        self._keyevent_api.scenario(self.enter,1)
        time.sleep(5)

    # To get back to the 1080p need to move up 29 times.
    def __up_29_times(self):
        for i in range(0,29):
            time.sleep(.5)
            self._keyevent_api.scenario(self.up,0.1)

    # Sets the HDMI resolution to 1080p before the test starts
    def __set_1080(self):
        # Launch HDMI settings menu
        self._app_api.launch_app(intent=self.PackageActivity)
        time.sleep(5)

        # Press up to select the Modes option
        self._keyevent_api.scenario(self.up,1)
        time.sleep(2)
        # Press Enter to open the mode select window
        self._keyevent_api.scenario(self.enter,1)
        time.sleep(2)

        # Change HDMI resolution
        self.__up_29_times()

        # Select 1080p
        self._keyevent_api.scenario(self.enter,1)

        # Close the hdmi settings menu and main settings menu
        self._system_api.stop_app(self.hdmi_package)

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """

        DeviceTestStepBase.run(self, context)
        self._logger.debug(self._pars.id+" : Run")

        # Start the HDMI mode switch control loop.
        end_time = float(time.time()) + (float(self._pars.duration)) * 60
        while (time.time() < end_time):
            # Launch HDMI settings menu
            self._app_api.launch_app(intent=self.PackageActivity)
            time.sleep(5)

            # Change HDMI resolution
            self.__toggle_resolution()

            # Close the hdmi settings menu and main settings menu
            self._system_api.stop_app(self.hdmi_package)
        # Check for system UI
        systemui_ok =self.phonesystem_api.check_process("systemui")
        if systemui_ok :
            self._logger.debug(self._pars.id+" : Systemui is active")

        else:
            self._logger.error(self._pars.id+" : Systemui is not active.")
            raise DeviceException(DeviceException.OPERATION_FAILED, self._pars.id+" : Systemui is not active.")

        # Attempt to return the resolution back to 1080p before concluding the test
        self.__set_1080()
        self._logger.debug(self._pars.id+" : Setting HDMI Resolution to 1080p")
        self._logger.debug(self._pars.id+" is completed. ")