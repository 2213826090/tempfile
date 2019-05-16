"""
@summary: This test step will receive incoming call on device. It will
possibly answer for some amount of seconds.

@since 1 July 2014
@author: Stephen Smith
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
from UtilitiesFWK.Utilities import Global
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException

class ReceiveVoiceCallLoop(DeviceTestStepBase):

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Constructor
        """
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """

        DeviceTestStepBase.run(self, context)
        self._logger.debug("START OF RECEIVE_VOICE_CALL_LOOP")
        answer_call = self._pars.answer_call
        test_duration = self._pars.test_duration * 60
        disconnection_allowance = self._pars.disconnection_allowance
        if hasattr(self._pars, 'calls_received'):
            verification = True
        else:
            verification = False
        number_of_calls_received = 0
        num_consecutive_disconnect = 0
        total_disconnect = 0
        voice_call_state = ""

        # Get UECmdLayer
        voice_call_api = self._device.get_uecmd("VoiceCall")
        self._logger.debug("Test duration of RECEIVE_VOICE_CALL_LOOP is set to {0} minutes".format(str(self._pars.test_duration)))
        start_time = time.time()

        # Release any previous call
        try:
            voice_call_api.release()
        except:
            self.ts_verdict_msg = "RECEIVE_VOICE_CALL_LOOP failed to release previous call from {0} device.".format(str(self._config.device_name))
            raise DeviceException(DeviceException.OPERATION_FAILED, "RECEIVE_VOICE_CALL_LOOP:  Failed to release previous call")
        verdict = Global.SUCCESS
        # Start Voice call loop
        while time.time() - start_time < test_duration:
            try:
                # Wait for an incoming call.
                while voice_call_state != str(voice_call_api._vc_state.get("ringing")) and time.time() - start_time < test_duration:
                    # Don't want to poll too quickly and 3 seconds shouldn't cause us to miss an incoming call.
                    time.sleep(3)
                    voice_call_state = str(voice_call_api.get_state())
                self._logger.debug("RECEIVE_VOICE_CALL_LOOP:  {0} device is ringing.".format(str(self._config.device_name)))
                # Do we answer the call or just let it ring?
                if answer_call:
                    voice_call_api.answer()
                    voice_call_api.wait_for_state(voice_call_api._vc_state.get("active"), 60)
                    self._logger.debug("RECEIVE_VOICE_CALL_LOOP:  {0} device is in ACTIVE call state.".format(str(self._config.device_name)))
                # Wait for the incoming or active call to end.
                while voice_call_state != str(voice_call_api._vc_state.get("no_call")) and time.time() - start_time < test_duration:
                    time.sleep(5)
                    voice_call_state = str(voice_call_api.get_state())
                self._logger.debug("RECEIVE_VOICE_CALL_LOOP:  {0} device is back to NOCALL state.".format(str(self._config.device_name)))
            except DeviceException as de:
                # Increase number of consecutive disconnects.
                num_consecutive_disconnect += 1
                total_disconnect += 1
                self._logger.debug("RECEIVE_VOICE_CALL_LOOP:  DeviceException while answering phone call: {0}".format(de.get_error_message()))
                self._logger.debug("RECEIVE_VOICE_CALL_LOOP:  {0} consecutive Device Exception occurred ({1} allowed) trying to answer a call.".format(num_consecutive_disconnect, disconnection_allowance))
            except:
                import traceback
                self._logger.error("RECEIVE_VOICE_CALL_LOOP:  Unexpected Exception being raised.")
                self.ts_verdict_msg = "RECEIVE_VOICE_CALL_LOOP: Unexpected Exception being raised"
                self._logger.error(traceback.format_exc())
                raise
            else:
                num_consecutive_disconnect = 0
                number_of_calls_received += 1
            finally:
                # Not sure what last voice_call_state was.  Just set it back to an empty string.
                voice_call_state = ""
                # Exit loop to finish the test step if too many disconnections.
                if num_consecutive_disconnect > disconnection_allowance:
                    self._logger.error("RECEIVE_VOICE_CALL_LOOP failed due to excessive consecutive disconnections")
                    self.ts_verdict_msg = "RECEIVE_VOICE_CALL_LOOP failed due to {0} excessive consecutive disconnections on {1} device.".format(num_consecutive_disconnect, str(self._config.device_name))
                    verdict = Global.FAILURE
                    break

        self._logger.debug("RECEIVE_VOICE_CALL_LOOP received {0} calls.  There were {1} total disconnects.".format(str(number_of_calls_received), str(total_disconnect)))
        self._logger.debug("END OF RECEIVE_VOICE_CALL_LOOP")
        if (verdict != Global.SUCCESS):
            raise DeviceException(DeviceException.OPERATION_FAILED, "RECEIVE_VOICE_CALL_LOOP TestStep failed.")
        else:
            if verification:
                context.set_info(self._pars.calls_received, str(number_of_calls_received))

