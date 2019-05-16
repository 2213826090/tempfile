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
:summary: GSM Mobile Terminated Cell BroadCast SMS over CS during voice call on network simulator
:author: fbelvezx
Created on 12 juil. 2013
"""

from UtilitiesFWK.Utilities import Global
from LAB_GSM_SMS_CS_BASE import LabGsmSmsCsBase
from ErrorHandling.AcsBaseException import AcsBaseException
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException


class LabGsmCbSmsVc(LabGsmSmsCsBase):

    """
    GSM Mobile Terminated cell broadcast SMS over CS during voice call on network simulator class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabGsmSmsCsBase.__init__(self, tc_name, global_config)
        # Set dummy phoneNumber
        self._sms_cb_destination_number = "123456789"

        # Set remote party phoneNumber
        self._distant_number = self._tc_parameters.get_param_value(
            "PHONE_NUMBER")

        # Set cell broadcast message 1 as the sms cb which will be used
        self._cell_broadcast_message = 1
        #
        self._cell_broadcast_message_identifier = int(self._tc_parameters.
                                                      get_param_value("CELL_BROADCAST_MESSAGE_IDENTIFIER"))

        self._repetition_period = int(self._tc_parameters.get_param_value(
                                      "REPETITION_PERIOD"))

        self._update_number = int(self._tc_parameters.get_param_value(
                                  "UPDATE_NUMBER"))

        self._custom_cell_broadcast_text_message = self._sms_text

        self._cb_messaging_api = self._device.get_uecmd("CellBroadcastMessaging")

        self._vc_2g = self._ns_cell_2g.get_voice_call()

        self._voicecall_api = self._device.get_uecmd("VoiceCall")

        # Read callSetupTimeout from Device_Catalog.xml
        self._call_setup_time = \
            int(self._dut_config.get("callSetupTimeout"))

        # Read VC_TYPE from test case xml file
        self._vc_type = str(self._tc_parameters.get_param_value("VC_TYPE"))

# ------------------------------------------------------------------------------
    def set_up(self):
        """
        Set up the test configuration
        """
        # Call the LAB_GSM_SMS_CS_BASE Setup function
        LabGsmSmsCsBase.set_up(self)

        # Set sender address using DESTINATION_NUMBER on Network simulator
        self._ns_messaging_2g.set_sms_sender_address(self._sms_cb_destination_number)

        self._ns_messaging_2g.set_cell_broadcast_message_identifier(
            self._cell_broadcast_message_identifier,
            self._cell_broadcast_message)

        self._ns_messaging_2g.set_cell_broadcast_message_repetition_period(
            self._repetition_period)

        self._ns_messaging_2g.select_cell_broadcast_message_content(
            self._content_type,
            self._cell_broadcast_message)

        self._ns_messaging_2g.set_cell_broadcast_message_update_number(
            self._update_number,
            self._cell_broadcast_message)

        if self._content_type == "CTEX":
            self._ns_messaging_2g.set_custom_cell_broadcast_text_message(
                self._custom_cell_broadcast_text_message,
                self._cell_broadcast_message)
        elif self._content_type == "CDAT":
            self._ns_messaging_2g.set_custom_cell_broadcast_data_string(
                self._custom_cell_broadcast_text_message,
                self._cell_broadcast_message)

        if self._vc_type not in ("MT", "MO"):
            raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG, "Invalid voice call type")

        # Release any previous call (Robustness)
        self._voicecall_api.release()

        # Establish voice call
        if self._vc_type == "MO":

            # Dial using a dummy hard-coded phone number
            self._logger.info("Calling distant party...")
            self._voicecall_api.dial(self._distant_number)

        elif self._vc_type == "MT":
            # Initiate VoiceCall to CDK
            self._vc_2g.mt_originate_call()
            # pylint: disable=E1101
            # Check call status is incoming before callSetupTimeout
            self._voicecall_api.wait_for_state(self._uecmd_types.VOICE_CALL_STATE.INCOMING,
                                               self._call_setup_time)
            # Answer incoming call
            self._voicecall_api.answer()

        # Check call status before callSetupTimeout (NS)
        self._vc_2g.check_call_connected(self._call_setup_time)

        # Check call status before callSetupTimeout (CDK)
        self._voicecall_api.wait_for_state(self._uecmd_types.VOICE_CALL_STATE.ACTIVE,  # pylint: disable=E1101
                                           self._call_setup_time)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """

        LabGsmSmsCsBase.run_test(self)

        # Start SMS cell broadcast
        self._ns_messaging_2g.start_cell_broadcast()
        self._logger.info("Start Cell Broadcast")

        received_cb_sms = self._cb_messaging_api.\
            wait_for_incoming_cell_broadcast_sms(self._sms_transfer_timeout)

        self._ns_messaging_2g.stop_cell_broadcast()

        try:
            received_cb_sms_text = received_cb_sms["sms_cb_text"]
            received_cb_sms_category = received_cb_sms["sms_cb_category"]
        except KeyError:
            self._error.Msg = "received_cb_sms doesn't have all the information" \
                "text, category and serial_number :: %s " % str(received_cb_sms)
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                   self._error.Msg)

        if received_cb_sms_text != self._custom_cell_broadcast_text_message or \
                int(received_cb_sms_category) != self._cell_broadcast_message_identifier:
            test_result = Global.FAILURE
            self._error.Msg = "Sent CB SMS isn't the same as received one"\
                "sent cb sms text : %s  received_cb_sms_text : %s " \
                "sent cb sms category : %d  received_cb_sms_category: %d" \
                % (self._custom_cell_broadcast_text_message, received_cb_sms_text,
                   self._cell_broadcast_message_identifier, int(received_cb_sms_category))
        else:
            test_result = Global.SUCCESS
            self._error.Msg = "Match between sent and received SMS CB regarding text, category and serial number"

        return test_result, self._error.Msg

#------------------------------------------------------------------------------
    def tear_down(self):
        """
        End and dispose the test
        """

        # Release the call
        self._vc_2g.voice_call_network_release()

        try:
            # Check call is released (NS)
            self._vc_2g.check_call_idle(self._registration_timeout,
                                        blocking=False)

            # Check call is released (CDK)
            self._voicecall_api.wait_for_state(self._uecmd_types.VOICE_CALL_STATE.NOCALL,  # pylint: disable=E1101
                                               self._call_setup_time)

        except AcsBaseException as acs_exception:
            self._logger.warning("Call release fail:" + str(acs_exception))

        # Call use case base tear_down function
        LabGsmSmsCsBase.tear_down(self)

        # Clear just received cell broadcast sms
        self._cb_messaging_api.clear_all_cell_broadcast_sms()

        return Global.SUCCESS, "No errors"
