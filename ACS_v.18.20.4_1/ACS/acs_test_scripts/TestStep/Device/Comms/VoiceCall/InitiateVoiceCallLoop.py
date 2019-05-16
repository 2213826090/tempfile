"""
@summary: This test step will initiate phone calls to another phone for
test duration.  How often the call is made is determined by call interval.

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
import acs_test_scripts.Utilities.OSBVUtilities as osbv_utils

class InitiateVoiceCallLoop(DeviceTestStepBase):

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

        self._logger.debug("START OF INITIATE_VOICE_CALL_LOOP")
        phone_number = self._pars.phone_number_to_call
        test_duration = self._pars.test_duration * 60
        call_interval = self._pars.call_interval
        disconnection_allowance = self._pars.disconnection_allowance
        if self._pars.call_duration is not None:
            call_duration = self._pars.call_duration
        else:
            call_duration = 0
        if self._pars.calls_made is not None:
            verification = True
        else:
            verification = False
        number_of_initiated_calls = 0
        num_consecutive_disconnect = 0
        total_disconnect = 0
        # Timeout value to use that determines amount of time to wait on gui focus lock file to be removed.
        self.gui_lock_wait_time = self._pars.gui_lock_wait_time
        # App signature to use with gui focus lock file
        appSignature = 'initiate_voice_call'
        try:
            # Delete any focus-lock file that may not have been released during the previous test run.
            osbv_utils.cleanup_focus_lock(appSignature)
        except:
            raise DeviceException(DeviceException.OPERATION_FAILED, self._pars.id + ": Issue trying to remove previous focus lock file.")

        # Get UECmdLayer
        voice_call_api = self._device.get_uecmd("VoiceCall")
        self._logger.debug("Test duration for INITIATE_VOICE_CALL_LOOP is set to {0} minutes".format(str(self._pars.test_duration)))
        start_time = time.time()

        # Release any previous call
        try:
            voice_call_api.release()
        except:
            self.ts_verdict_msg = "INITIATE_VOICE_CALL_LOOP failed to release previous call on {0} device.".format(str(self._config.device_name))
            raise DeviceException(DeviceException.OPERATION_FAILED, "INITIATE_VOICE_CALL_LOOP:  Failed to release previous call")
        verdict = Global.SUCCESS
        # Start Voice call loop
        while time.time() - start_time < test_duration:
            try:
                self._logger.debug("INITIATE_VOICE_CALL_LOOP:  Calling from {0} device.".format(str(self._config.device_name)))
                if not osbv_utils.set_focus_lock(appSignature, timeout_sec=self.gui_lock_wait_time):
                    # Could not set the focus-lock
                    raise DeviceException(DeviceException.OPERATION_FAILED, self._pars.id + ": Failed to set focus lock.")
                voice_call_api.dial(phone_number)
                self._logger.debug("INITIATE_VOICE_CALL_LOOP:  Sleep for call duration of {0} seconds.".format(str(call_duration)))
                # Call duration of 0 would let us just make calls and not worry about being answered.  When forked with voice_call_answer, call duration should account for time
                # it takes from when call is ACTIVE on caller until time call can be recognized as INCOMING (ringing) on the other device.
                time.sleep(call_duration)
                # Hang up on caller side should release this call and if in forked with voice_call_answer, should release that call as well.
                voice_call_api.release()
                if not osbv_utils.release_focus_lock(appSignature):
                    # Could not release the focus-lock
                    raise DeviceException(DeviceException.OPERATION_FAILED, self._pars.id + ": Failed to release focus lock.")
                self._logger.debug("INITIATE_VOICE_CALL_LOOP:  Call was released on {0} device.".format(str(self._config.device_name)))
            except DeviceException as de:
                # Increase number of consecutive disconnects.
                num_consecutive_disconnect += 1
                total_disconnect += 1
                self._logger.debug("INITIATE_VOICE_CALL_LOOP:  DeviceException while initiating phone call: {0}".format(de.get_error_message()))
                self._logger.debug("INITIATE_VOICE_CALL_LOOP:  {0} consecutive Device Exception occurred ({1} allowed) trying to initiate a call.".format(num_consecutive_disconnect, disconnection_allowance))
            except:
                import traceback
                self._logger.error("INITIATE_VOICE_CALL_LOOP:  Unexpected Exception being raised.")
                self.ts_verdict_msg = "INITIATE_VOICE_CALL_LOOP: Unexpected Exception being raised."
                self._logger.error(traceback.format_exc())
                raise
            else:
                num_consecutive_disconnect = 0
                number_of_initiated_calls += 1
            finally:
                # Check if focus lock still exists with appSignature and remove if so.
                if appSignature == osbv_utils.get_lock_signature():
                    if not osbv_utils.release_focus_lock(appSignature):
                        # Unable to release the focus-lock
                        raise DeviceException(DeviceException.OPERATION_FAILED, self._pars.id + ": Failed to release focus lock.")
                # Exit loop to finish the test step if too many disconnections.
                if num_consecutive_disconnect > disconnection_allowance:
                    self._logger.error("INITIATE_VOICE_CALL_LOOP failed due to excessive consecutive disconnections")
                    self.ts_verdict_msg = "INITIATE_VOICE_CALL_LOOP failed due to {0} excessive consecutive disconnections on {1} device.".format(num_consecutive_disconnect, str(self._config.device_name))
                    verdict = Global.FAILURE
                    break
            self._logger.debug("INITIATE_VOICE_CALL_LOOP:  Sleeping for {0} seconds before next call is initiated.".format(str(call_interval)))
            time.sleep(call_interval)

        self._logger.debug("INITIATE_VOICE_CALL_LOOP initiated {0} calls.  There were {1} total disconnects.".format(str(number_of_initiated_calls), str(total_disconnect)))
        self._logger.debug("END of INITIATE_VOICE_CALL_LOOP")
        if (verdict != Global.SUCCESS):
            raise DeviceException(DeviceException.OPERATION_FAILED, "INITIATE_VOICE_CALL_LOOP TestStep failed.")
        else:
            if (verification):
                context.set_info(self._pars.calls_made, str(number_of_initiated_calls))

