"""

:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

:organization: INTEL PEG SVE DSV
:summary: This file implements a Test Step to run Skype video call on two devices
:since: 07/01/2014
:author: jongyoon
"""

import time
from Device.DeviceManager import DeviceManager
from UtilitiesFWK.Utilities import Global
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException

class RunVideoCallSkype(DeviceTestStepBase):

    """
    Run Skype video call
    """

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        DeviceTestStepBase.run(self, context)

        self._logger.debug("START OF RUN_VIDEOCALL_SKYPE")

        # Get device manager object
        self._device_manager = DeviceManager()

        self._caller_device =  self._device_manager.get_device(self._pars.caller_device)
        self._receiver_device =  self._device

        self._caller_device_logger =  self._caller_device.get_device_logger()
        self._receiver_device_logger =  self._receiver_device.get_device_logger()

        self._caller_skype_api = self._caller_device.get_uecmd("VideoCallSkype")
        self._receiver_skype_api = self._receiver_device.get_uecmd("VideoCallSkype")

        #Create  a wake lock to prevent device from sleep mode. If a device in sleep mode, the Skype video call will fail
        adbCmd = "adb shell echo 'skype' >> /sys/power/wake_lock"
        self._caller_device.run_cmd(cmd=adbCmd, timeout=120)
        self._receiver_device.run_cmd(cmd=adbCmd, timeout=120)

        # Set up logcat triggering for each platform
        self.triglogStartMsg   = self._caller_skype_api.get_logcat_trigger_message('Start')
        self.triglogChatMsg    = self._caller_skype_api.get_logcat_trigger_message('Chat')
        self.triglogPrecallMsg = self._caller_skype_api.get_logcat_trigger_message('Precall')
        self.triglogCallMsg    = self._caller_skype_api.get_logcat_trigger_message('Call')
        self.triglogReleaseMsg = self._caller_skype_api.get_logcat_trigger_message('Release')

        # Add triglogs for Skype start conditions
        self._caller_device_logger.add_trigger_message(self.triglogStartMsg)
        self._receiver_device_logger.add_trigger_message(self.triglogStartMsg)

        self._caller_device_logger.add_trigger_message(self.triglogChatMsg)
        self._receiver_device_logger.add_trigger_message(self.triglogChatMsg)

        self._caller_device_logger.add_trigger_message(self.triglogPrecallMsg)
        self._receiver_device_logger.add_trigger_message(self.triglogPrecallMsg)

        self._caller_device_logger.add_trigger_message(self.triglogCallMsg)
        self._receiver_device_logger.add_trigger_message(self.triglogCallMsg)

        self._caller_device_logger.add_trigger_message(self.triglogReleaseMsg)
        self._receiver_device_logger.add_trigger_message(self.triglogReleaseMsg)

        iteration_number = 0
        result = 0

        chat_duration = self._pars.chat_duration * 60
        time_between_chats = self._pars.time_between_chats * 60
        disconnect_threshold = self._pars.disconnect_threshold

        # Get Parameters
        test_duration = self._pars.duration * 60
        start_time = time.time()

        if test_duration < chat_duration + 10:
            error_msg = "Test duration {0} seconds is set to smaller than chat duration {1} seconds".format(str(test_duration), str(chat_duration))
            self._logger.error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        else:
            self._logger.debug("Test duration is {0} seconds".format(str(test_duration)))

        #Start checking loop
        while time.time()-start_time < test_duration:
            iteration_number += 1
            self._logger.info("[Skype Iteration {0}] Started".format(iteration_number))

            if result > disconnect_threshold:
                break

            self._caller_skype_api.stop_skype_app()
            self._receiver_skype_api.stop_skype_app()
            time.sleep(5)

            (verdict, msg) = self.start_skype_app(self._caller_device)
            if verdict == Global.FAILURE:
                self._logger.error("[Skype Iteration {0} Failure] {1}".format(iteration_number, msg))
                result += 1
                continue
            time.sleep(5)

            (verdict, msg) = self.start_skype_app(self._receiver_device)
            if verdict == Global.FAILURE:
                self._logger.error("[Skype Iteration {0} Failure] {1}".format(iteration_number, msg))
                result += 1
                continue
            time.sleep(5)

            (verdict, msg) = self.start_videocall_session()
            if verdict == Global.FAILURE:
                self._logger.error("[Skype Iteration {0} Failure] {1}".format(iteration_number, msg))
                result += 1
                continue

            self._logger.info("[Skype Iteration {0}] Skype video chat session started. Wait for chat duration: {1} seconds".format(iteration_number, chat_duration))
            time.sleep(chat_duration)

            (verdict, msg) = self.stop_videocall_session(self._caller_device, self._caller_device_logger)
            self._logger.info("[Skype Iteration {0}] Passed {1} seconds (Total {2} seconds)".format(iteration_number, time.time()-start_time, test_duration))
            self._logger.info("[Skype Iteration {0}] {1} disconnections so far. TC threshold is {2}".format(iteration_number, result, disconnect_threshold))
            if verdict == Global.FAILURE:
                self._logger.error("[Skype Iteration {0} Failure] {1}".format(iteration_number, msg))
                result += 1
                continue
            else:
                self._logger.info("[Skype Iteration {0} Success] Skype session was finished, VERDICT={1}".format(iteration_number, verdict))

            #wait to start the next call
            estimated_time_after_next_iteration = (time.time() - start_time) + time_between_chats + chat_duration + 30
            if estimated_time_after_next_iteration < test_duration :
                self._logger.debug("Wait for next chat Time: {0} seconds".format(time_between_chats))
                time.sleep(time_between_chats)
            else:
                time_to_stay = test_duration - (time.time() - start_time) + 10
                self._logger.debug("Wait for {0} seconds for test completion".format(time_to_stay))
                time.sleep(time_to_stay)

        self._logger.info("Skype video chat completed {0} sessions in {1} seconds".format(iteration_number, test_duration))

        #Remove the wake lock before exit.
        adbCmd = "adb shell echo 'skype' >> /sys/power/wake_unlock"
        self._caller_device.run_cmd(cmd=adbCmd, timeout=120)
        self._receiver_device.run_cmd(cmd=adbCmd, timeout=120)

        if result > disconnect_threshold:
            msg = "Video chat FAILED"
            self._logger.info(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        else:
            msg = "Video chat PASSED"
            self._logger.info(msg)

    def start_skype_app(self, device_obj):
        """
        Dials a voice call.

        :type device_obj: str
        :param device_obj: number to call (MSISDN)

        :return: None
        """

        device_obj_skype_api = device_obj.get_uecmd("VideoCallSkype")
        device_obj_logger = device_obj.get_device_logger()

        # Kick start Google Skype on a phone
        device_obj_skype_api.start_skype_app()
        time.sleep(10)

        startMsg = device_obj_logger.get_message_triggered_status(self.triglogStartMsg)
        if len(startMsg) == 0:
            verdict = Global.FAILURE
            msg = "Skype app failed to start properly on " + device_obj._device_name
        else:
            verdict = Global.SUCCESS
            msg = "Skype app started on " + device_obj._device_name

        device_obj_logger.reset_trigger_message(self.triglogStartMsg)

        return (verdict, msg)

    def start_videocall_session(self):
        """
        Start a video call skype session.

        :return: verdict and message
        """

        # Select Contact on Phone 1: Assume commercial banner on screen
        self._caller_skype_api.select_1st_contact_banner()
        time.sleep(10)

        messages = self._caller_device_logger.get_message_triggered_status(self.triglogChatMsg)
        if len(messages) == 0:
            # Select Contact on Phone 1: No commercial banner on screen
            self._caller_skype_api.select_1st_contact_nobanner()

        # Wait until PHONE1 has a chat screen
        chat_screen = 0
        while chat_screen == 0:
            messages = self._caller_device_logger.get_message_triggered_status(self.triglogChatMsg)
            if len(messages) > 0:
                self._logger.debug(self._caller_device._device_name + " has a chat screen")
                chat_screen = 1
        self._caller_device_logger.reset_trigger_message(self.triglogChatMsg)

        # Start Video Chat between Phone 1 and Phone 2
        self._caller_skype_api.start_call()

        # Wait until PHONE2 receive a incoming call
        incoming_call = 0
        while incoming_call == 0:
            messages = self._receiver_device_logger.get_message_triggered_status(self.triglogPrecallMsg)
            if len(messages) > 0:
                self._logger.debug(self._receiver_device._device_name + " has a incoming Skype video call")
                incoming_call = 1
        self._receiver_device_logger.reset_trigger_message(self.triglogPrecallMsg)

        time.sleep(5)

        # Have Phone 2 accept video chat between Phone 1 and Phone 2 by swiping Phone 2 from center to right
        self._receiver_skype_api.accept_call()
        time.sleep(5)

        messages2 = self._receiver_device_logger.get_message_triggered_status(self.triglogCallMsg)

        if len(messages2) == 0:
            verdict = Global.FAILURE
            msg = "Failed to start Skype session between " + self._caller_device._device_name + " and " + self._receiver_device._device_name + "\n"
        else:
            verdict = Global.SUCCESS
            msg = "Started Skype session between " + self._caller_device._device_name + " and " + self._receiver_device._device_name + "\n"

        self._logger.debug(msg)
        self._receiver_device_logger.reset_trigger_message(self.triglogCallMsg)

        return (verdict, msg)

    def stop_videocall_session(self, device_obj, phone_logger):
        """
        Stop a video call skype session.

        :type device_obj: Device
        :param device_obj: Device to use

        :type phone_logger: logger
        :param phone_logger: Logger to use

        :return: verdict and message
        """

        self._caller_skype_api.reveal_buttons()
        time.sleep(2)

        self._caller_skype_api.disconnect_call()
        time.sleep(3)

        releaseMsg = phone_logger.get_message_triggered_status(self.triglogReleaseMsg)

        if len(releaseMsg) == 0:
            verdict = Global.FAILURE
            msg = "Skype stopped on " + device_obj._device_name + "\n"
        else:
            verdict = Global.SUCCESS
            msg = "Skype stopped on " + device_obj._device_name + "\n"

        self._logger.debug(msg)
        phone_logger.reset_trigger_message(self.triglogReleaseMsg)

        return (verdict, msg)


