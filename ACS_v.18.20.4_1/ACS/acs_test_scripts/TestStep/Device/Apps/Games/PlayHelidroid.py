"""
@summary: This file implements a Test Step to run a game called Hellipad 3D.
Prerequisites: the following binary should be installed.
    Helidroid_3D.apk
This file can be found in Artifactory: acs_test_artifacts/CONCURRENCY/TESTS/Helidroid_3D/Helidroid_3D.apk.
Android only -- I don't know of a similar executable for Windows.
@since 29 January 2015
@author: Jongyoon Choi
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
import time
import random
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException

class PlayHelidroid(DeviceTestStepBase):
    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        self._logger.info("PlayHelidroid: Run")

        # Get parameters
        self._execution_seconds = self._pars.duration * 50

       # Initialize the touch control class
        self._gCtrl = TouchCtl(self._device)

        displaychecks_module = self._device.get_device_module("DeviceChecksModule")
        phonesystem_api = self._device.get_uecmd("PhoneSystem")

        # Run ACS pretest checks.
        # See the osv_media_debug python script to see what this checks.
        pretest_check_results = displaychecks_module.enable_tearing_effect_detection()

        if pretest_check_results == "FAIL":
            error_msg = "Pretest failures found. "
            self._logger.debug(error_msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, error_msg)

        # Launch the game using the adb am command
        cmd="adb shell am start -a android.intent.action.MAIN -n {0}".format(self._pars.game_package+"/."+self._pars.game_activity)
        self._device.run_cmd(cmd, 10)
        time.sleep(10)

        # Check if the resolution has been set properly.
        # Else don't run the test and issue a fail verdict.
        if self._gCtrl.buttonFwdX != -1:
            # Start the mission and fly up
            self._gCtrl.mission_1()
            time.sleep(3)
            self._gCtrl.go_joy()
            time.sleep(6)
            self._gCtrl.btn1()
            time.sleep(3)
            self._gCtrl.btn2()
            time.sleep(3)
            self._gCtrl.fly_up()

            # Start test control loop
            test_start_time = time.time()
            test_end_time = test_start_time + float(self._execution_seconds)

            while (test_end_time > time.time()):
                # This loop make the helicopter move around.
                for x in range (1,10):
                    r = random.randint(1,3)
                    self._gCtrl.fly_left()
                    time.sleep(r)
                    self._gCtrl.fly_forward()
                    time.sleep(r)

                    # Making sure that the game is closed ontime before verify script starts
                    if(test_end_time < time.time()):
                        break
                # Check that a game failure wasn't detected
                # Check that the system UI is alive
                # I don't do anything with return code since the return string
                # will detect any error.
                systemui_ok = phonesystem_api.check_process("systemui")
                if systemui_ok == False:
                    err_msg = "Systemui closed unexpectedly"
                    raise DeviceException(DeviceException.OPERATION_FAILED, err_msg)

                # Check that the game is alive
                # I don't do anything with return code since the return string
                # will detect any error.
                game_state = phonesystem_api.check_process("com.heuer.helidroid_full")
                if game_state == False:
                    err_msg = "Game app closed unexpetedly"
                    raise DeviceException(DeviceException.OPERATION_FAILED, err_msg)

            # Close out the helidroid application
            cmd="adb shell am force-stop {0}".format(self._pars.game_package)
            self._device.run_cmd(cmd, 10)
        else:
            error_msg = "Display resolution is not supported. "
            self._logger.debug(error_msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, error_msg)

        self._logger.info("PlayHelidroid: Done")

#####################################
####  HELPER CLASSES     ############
#####################################
# This class will set up the right corrdinates for the touch events
# that need to be sent during the execution of the test
class TouchCtl:

    # Select the correct set of key locations based on display resolution
    # If a new display resolution is added, this is the only section
    # of the script that needs to be modified.
    def __init__(self, device):

        self._device = device
        phone = self._device.get_uecmd("PhoneSystem")
        # Get the display resolution
        disp_width_height = phone.get_screen_resolution()
        disp_width = disp_width_height.split("x")[0]
        if disp_width == "720":
            self.disp_res = "7x12"
        elif disp_width == "1080":
            self.disp_res = "10x19"
        elif disp_width == "2560":
            self.disp_res = "25x16"

        # 720p display
        if (self.disp_res == "7x12"):
            self.buttonFwdX = 998
            self.buttonFwdY = 429
            self.buttonLeftX = 909
            self.buttonLeftY = 528
            self.flyUpX = 93
            self.flyUpY = 419
            self.missionOneX = 314
            self.missionOneY = 641
            self.joyStickX = 767
            self.joyStickY = 579
            self.goBtn1X = 1091
            self.goBtn1Y = 594
            self.goBtn2X = 750
            self.goBtn2Y = 508
            self.radPlayX = 200
            self.radPlayY = 1050
        # 25x16 display
        elif (self.disp_res == "25x16"):
            self.buttonFwdX = 2200
            self.buttonFwdY = 977
            self.buttonLeftX = 2020
            self.buttonLeftY = 1140
            self.flyUpX = 190
            self.flyUpY = 760
            self.missionOneX = 588
            self.missionOneY = 1334
            self.joyStickX = 1600
            self.joyStickY = 1170
            self.goBtn1X = 2300
            self.goBtn1Y = 1260
            self.goBtn2X = 1600
            self.goBtn2Y = 1050
            self.radPlayX = 1118
            self.radPlayY = 1286
        # 10x19 display
        elif (self.disp_res == "10x19"):
            self.buttonFwdX = 1573
            self.buttonFwdY = 699
            self.buttonLeftX = 1447
            self.buttonLeftY = 810
            self.flyUpX = 178
            self.flyUpY = 558
            self.missionOneX = 330
            self.missionOneY = 950
            self.joyStickX = 1200
            self.joyStickY = 845
            self.goBtn1X = 1707
            self.goBtn1Y = 951
            self.goBtn2X = 1115
            self.goBtn2Y = 760
            self.radPlayX = 373
            self.radPlayY = 1600
        else:
            self.buttonFwdX = -1
            self.buttonFwdY = -1
            self.buttonLeftX = -1
            self.buttonLeftY = -1
            self.flyUpX = -1
            self.flyUpY = -1
            self.missionOneX = -1
            self.missionOneY = -1
            self.joyStickX = -1
            self.joyStickY = -1
            self.goBtn1X = -1
            self.goBtn1Y = -1
            self.goBtn2X = -1
            self.goBtn2Y = -1
            self.radPlayX = -1
            self.radPlayY = -1

    # Selects the mission 1 button in the game
    def mission_1(self):
        # The parameters can only take up to 5 arguments
        # I'm just building the whole string to pass
        # as a single parameter.
        cmd="adb shell input tap {0} {1}".format(self.missionOneX, self.missionOneY)
        self._device.run_cmd(cmd, 5)

    # Selects the joystick button in the game
    def go_joy(self):
        # The parameters can only take up to 5 arguments
        # I'm just building the whole string to pass
        # as a single parameter.
        cmd="adb shell input tap {0} {1}".format(self.joyStickX, self.joyStickY)
        self._device.run_cmd(cmd, 5)

    # Selects the first go button in the game
    def btn1(self):
        # The parameters can only take up to 5 arguments
        # I'm just building the whole string to pass
        # as a single parameter.
        cmd="adb shell input tap {0} {1}".format(self.goBtn1X, self.goBtn1Y)
        self._device.run_cmd(cmd, 5)

    # Selects the second go button in the game
    def btn2(self):
        # The parameters can only take up to 5 arguments
        # I'm just building the whole string to pass
        # as a single parameter.
        cmd="adb shell input tap {0} {1}".format(self.goBtn2X, self.goBtn2Y)
        self._device.run_cmd(cmd, 5)

    # Makes the helicopter fly up
    def fly_up(self):
        # The parameters can only take up to 5 arguments
        # I'm just building the whole string to pass
        # as a single parameter.
        cmd="adb shell input swipe {0} {1} {2} {3} 1700".format(self.flyUpX, self.flyUpY, self.flyUpX, self.flyUpY)
        self._device.run_cmd(cmd, 5)

    # Makes the helicopter turn to the left
    def fly_left(self):
        # The parameters can only take up to 5 arguments
        # I'm just building the whole string to pass
        # as a single parameter.
        cmd="adb shell input swipe {0} {1} {2} {3} 1700".format(self.buttonLeftX, self.buttonLeftY, self.buttonLeftX, self.buttonLeftY)
        self._device.run_cmd(cmd, 5)

    # Makes the helicopter go forward
    def fly_forward(self):
        # The parameters can only take up to 5 arguments
        # I'm just building the whole string to pass
        # as a single parameter.
        cmd="adb shell input swipe {0} {1} {2} {3} 1700".format(self.buttonFwdX, self.buttonFwdY, self.buttonFwdX, self.buttonFwdY)
        self._device.run_cmd(cmd, 5)

    # Makes the helicopter go forward
    def intRadPlay(self):
        # The parameters can only take up to 5 arguments
        # I'm just building the whole string to pass
        # as a single parameter.
        cmd="adb shell input tap {0} {1}".format(self.radPlayX, self.radPlayY)
        self._device.run_cmd(cmd, 5)