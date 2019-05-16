"""

:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

:organization: INTEL MCG PSI
:summary: VoiceMail SMS Indicator UseCase.
:since: 21/05/2013
:author: rbertolx
"""

import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.AcsConfigException import AcsConfigException

class LiveVoicemailIndicator(UseCaseBase):

    """
    Class implementing a VoiceMail SMS Indicator UseCase.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        UseCaseBase.__init__(self, tc_name, global_config)

        # Read callSetupTimeout from Phone_Catalog.xml
        self._call_setup_time = \
            int(self._dut_config.get("callSetupTimeout"))

        # Initialize test case parameters
        self._numtocall = None
        self._callduration = None
        self._msg_waiting_time = None

        # Get Test Cases Parameters
        self._numtocall = self._tc_parameters.get_param_value("PHONE_NUMBER")
        self._callduration = \
            self._tc_parameters.get_param_value("CALL_DURATION", 0)
        self._msg_waiting_time = \
            self._tc_parameters.get_param_value("MSG_WAITING_TIME", 0)

        # Get UECmdLayer
        self._vc_api = self._device.get_uecmd("VoiceCall")
        # Getting the Networking API.
        self._networking_api = self._device.get_uecmd("Networking")
        # Getting the audio API.
        self._audio_api = self._device.get_uecmd("AudioHost")
        # Getting the messaging API.
        self._msg_api = self._device.get_uecmd("SmsMessaging")

#-----------------------------------------------------------------------------

    def set_up(self):
        """
        Setting up the test
            Setup:
            Checking the phone number to call is valid
            Checking the call duration is valid
            Checking the message waiting time is valid
        """

        UseCaseBase.set_up(self)

        # Testing if all needed parameters are valid.
        if self._numtocall :
            self._numtocall = str(self._numtocall)
        else:
            message = "Invalid parameter value: %s for parameter '%s'." % (
                str(self._numtocall),
                "PHONE_NUMBER")
            self._logger.error(message)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, message)

        if self._callduration :
            self._callduration = int(self._callduration)
        else:
            message = "Invalid parameter value: %d for parameter '%s'." % (
                int(self._callduration),
                "CALL_DURATION")
            self._logger.error(message)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, message)

        if self._msg_waiting_time :
            self._msg_waiting_time = int(self._msg_waiting_time)
        else:
            message = "Invalid parameter value: %d for parameter '%s'." % (
                int(self._msg_waiting_time),
                "MSG_WAITING_TIME")
            self._logger.error(message)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, message)

        return Global.SUCCESS, "No error"

#-----------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
            Run test:
            Deleting all SMS/MMS on the phone.
            Dial the number of the SIM in the Bench to get to the VoiceMail.
            Start Looking for incoming SMS.
            Checks if a SMS is received and give the verdict.
        """
        UseCaseBase.run_test(self)

        # Initialize return codes.
        result_code = Global.FAILURE
        result_msg = "Error"

        # Delete all messages on the phone.
        self._msg_api.delete_all_sms()

        # Disable flight mode
        self._networking_api.set_flight_mode("off")
        # Release any previous call (Robustness)
        self._vc_api.release()

        # Dial
        self._vc_api.dial(self._numtocall)

        # Wait for the call to reach ACTIVE state
        self._vc_api.wait_for_state(self._uecmd_types.VOICE_CALL_STATE.ACTIVE,
                                    self._call_setup_time)

        self._logger.info("Wait for call duration: " +
                          str(self._callduration) + "s...")

        # Wait for near the end of the call.
        time.sleep(self._callduration)

        # At the end of the call.
        self._vc_api.release()

        # Wait for incoming SMS.
        self._logger.info("Wait for the voice mail SMS indicator...")
        sms = self._msg_api.wait_for_incoming_sms(self._msg_waiting_time)

        # If SMS exists in DUT SMS folder the test case is successful
        if sms is None:
            result_code = Global.FAILURE
            result_msg = "VoiceMail SMS indicator no received"
        else :
            result_code = Global.SUCCESS
            result_msg = "VoiceMail SMS indicator successfully received."

        return result_code, result_msg

#-----------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        UseCaseBase.tear_down(self)
        # Release any previous call (Robustness)
        self._vc_api.release()
        return Global.SUCCESS, "No errors"
