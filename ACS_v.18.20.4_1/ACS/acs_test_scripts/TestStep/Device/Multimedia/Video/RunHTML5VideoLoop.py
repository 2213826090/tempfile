"""
@summary:  This file uses a Test Step which does open a browser and play three videos
of different resolutions for the specified duration.
PREREQUISITE : Internet connection(Wifi) is required.
@since 8 October 2014
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
import datetime

class RunHTML5VideoLoop(DeviceTestStepBase):

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._phone_system_api = self._device.get_uecmd("PhoneSystem")
        self._network_api = self._device.get_uecmd("Networking")
        self.system_api = self._device.get_uecmd("System")
        self._keyevent_api = self._device.get_uecmd("KeyEvent")
        self.browser_package = "com.android.browser"
        __disp_coordinate_dict ={
        "7x12":{"vid1":[175,350],"vid2":[485,350],"vid3":[175,575]},
        "10x19":{"vid1":[268,450],"vid2":[740,450],"vid3":[268,783]},
        "25x16":{"vid1":[613,949],"vid2":[1749,949],"vid3":[613,802]}
        }
        # Get the display resolution
        disp_width_height = self._phone_system_api.get_screen_resolution()
        disp_width = disp_width_height.split("x")[0]
        if disp_width == "720":
            self.disp_res = "7x12"
        elif disp_width == "1080":
            self.disp_res = "10x19"
        elif disp_width == "2560":
            self.disp_res = "25x16"

        # Set x coordinates based on display resolution
        self.vid1_x = __disp_coordinate_dict[self.disp_res]["vid1"][0]
        self.vid2_x = __disp_coordinate_dict[self.disp_res]["vid2"][0]
        self.vid3_x =   __disp_coordinate_dict[self.disp_res]["vid3"][0]
        # Set y coordinates based on display resolution
        self.vid1_y =   __disp_coordinate_dict[self.disp_res]["vid1"][1]
        self.vid2_y =   __disp_coordinate_dict[self.disp_res]["vid2"][1]
        self.vid3_y =   __disp_coordinate_dict[self.disp_res]["vid3"][1]


    def start_video(self,vid_num):

        # tap the video location
        if vid_num == 0:
            # If this is the 25x16 display, issue a page up command first
            # to properly position the web browser
            if self.disp_res == "25x16":
                self.up= ["DPAD_UP"]
                self._keyevent_api.scenario(self.up,1)
            # Start the video
            self._keyevent_api.tap_on_screen(self.vid1_x,self.vid1_y)

        elif vid_num == 1:
            # If this is the 25x16 display, issue a page up command first
            # to properly position the web browser
            if self.disp_res == "25x16":
                self._keyevent_api.scenario(self.up,1)
            # Start the video
            self._keyevent_api.tap_on_screen(self.vid2_x,self.vid2_y)
        elif vid_num == 2:
            # If this is the 25x16 display, issue a page down command first
            # to properly position the web browser
            if self.disp_res == "25x16":
                self._keyevent_api.scenario(["DPAD_DOWN"],1)
            # Start the video
            self._keyevent_api.tap_on_screen(self.vid3_x,self.vid3_y)
        else:
            self._keyevent_api.tap_on_screen(self.vid1_x,self.vid1_y)

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        time.sleep(5)
        self._logger.debug(self._pars.id +" : Run")

        end_time = float(time.time()) + (float(self._pars.duration)) * 60
        while (time.time() < end_time):
            #Open the browser.
            self._network_api.open_web_browser(self._pars.video_url,"native",10)
            time.sleep(10)

            # Start the Video playing loop
            for x in range(0, 3):
                # The video is 34 seconds, but more time is given for
                #playback due to buffering
                self.start_video(x)
                self._logger.debug(str(datetime.datetime.now()) + " " +self._pars.id+
                     ": Waiting for video to play (45s)")
                time.sleep(45)

                # Making sure that the browser is closed ontime before verify script starts
                if(end_time < time.time()):
                    break

            # Check that the browser is still active.
            browser_ok = self._phone_system_api.check_process(self.browser_package)
            if browser_ok :
                self._logger.debug(self._pars.id +"test : Browser OK")
            else:
                self._logger.error(self._pars.id +"test : Browser playing HTML5 video closed unexpectedly.")
                raise DeviceException(DeviceException.OPERATION_FAILED, self._pars.id +" test : Browser playing HTML5 video closed unexpectedly.")

            # Check for system UI
            systemui_ok =self._phone_system_api.check_process("systemui")
            if systemui_ok :
                self._logger.debug(self._pars.id +" test : Systemui is active")

            else:
                self._logger.error(self._pars.id +"  test : Systemui is not active.")
                raise DeviceException(DeviceException.OPERATION_FAILED,self._pars.id +" test : Systemui is not active.")
        self.system_api.stop_app(self.browser_package)
        time.sleep(5.5)
        self._logger.debug(self._pars.id +" test is completed.")