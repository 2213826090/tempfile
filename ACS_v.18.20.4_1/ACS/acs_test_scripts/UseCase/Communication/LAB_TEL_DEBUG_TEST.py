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
self._modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual
property right is granted to or conferred upon you by disclosure or delivery
of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

:organization: INTEL MCG PSI
:summary: TEL - this UseCase test all UECmd used by all TEL UseCases
:author: lvacheyx
:since: 19/03/2013
"""

from acs_test_scripts.Utilities.SmsUtilities import compute_sms_segments, SmsMessage, \
    compute_sms_equals
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.Communication.LAB_TEL_DEBUG_BASE import LabTelDebugBase
from acs_test_scripts.UseCase.Misc.UECMD_TEST_TOOL import UECmdTestTool

import acs_test_scripts.Device.UECmd.UECmdTypes as UECmdTypes
import time


class LabTelDebugTest(LabTelDebugBase, UECmdTestTool):

    """
    Lab Telephony Debug Test class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LabTelDebugBase base Init function
        LabTelDebugBase.__init__(self, tc_name, global_config)
        UECmdTestTool.__init__(self, self.get_name(),
                               self._logger, self._device)

        self._test_list = "MODEM,VOICECALL,MESSAGING, NETWORKING"

        self._test_type = self._tc_parameters.\
            get_param_value("UECMD_TYPE_LIST", self._test_list).upper()

        # If TC parameter not defined, then run all tests
        if self._test_type == "":
            self._test_type = self._test_list

        self._fcts2test_vc = list()
        self._fcts2test_mod = list()
        self._fcts2test_mess = list()
        self._fcts2test_net = list()

        self._vca = self._voicecall_api
        self._mod = self._modem_api
        self._mess = self._messaging_api
        self._net = self._networking_api

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Set up the test configuration
        """
        # Call Tel Debug Base Setup function
        LabTelDebugBase.set_up(self)
        UECmdTestTool.set_up(self, self.tc_order)

        """
        List of tests to perform. Each test is described as followed:
        {Label to print in the secondary report,
         UECmd to test,
         parameter(s) for the UECmd to test,
         Depends on the test names in the list}
        """

        if "VOICECALL" in self._test_type:
            self._fcts2test_vc = [
                {self._FCT: self._vca.set_ringtone, self._PARMS: ["Cairo"]},
                {self._FCT: self._vca.dial, self._PARMS: ["123"]},
                {self._FCT: self._vca.get_state},
                {self._NAME: "wait_for_state call state ACTIVE",
                 self._FCT: self._vca.wait_for_state,
                 self._PARMS: [UECmdTypes.VOICE_CALL_STATE.ACTIVE, 120]},
                {self._NAME: "voice call release", self._FCT: self._vca.release},
                {self._NAME: "wait_for_state call state No Call",
                 self._FCT: self._vca.wait_for_state,
                 self._PARMS: [UECmdTypes.VOICE_CALL_STATE.NOCALL, 120]},
                {self._NAME: "MT voice Call", self._FCT: self.__mt_voice_call}]

        if "MODEM" in self._test_type:
            self._fcts2test_mod = [
                {self._FCT: self._mod.get_imsi, self._PARMS: [10]},
                {self._FCT: self._mod.set_modem_online, self._PARMS: ["1"]},
                {self._FCT: self._mod.get_modem_online_status},
                {self._FCT: self._mod.get_lac},
                {self._NAME: "dut_registration", self._FCT: self.__dut_registration},
                {self._FCT: self._mod.get_modem_power_status},
                {self._FCT: self._mod.get_sim_operator_info},
                {self._FCT: self._mod.register_to_network},
                {self._FCT: self._mod.get_cellular_operator_info},
                {self._FCT: self._mod.set_preferred_radio, self._PARMS: ["any"]}]

        if "MESSAGING" in self._test_type:
            self._fcts2test_mess = [
                {self._FCT: self._mess.delete_all_sms},
                {self._FCT: self._mess.set_service_center_address,
                 self._PARMS: ["0689004000"]},
                {self._NAME: "send a SMS", self._FCT: self.__send_a_sms,
                 self._EXP_RES: Global.SUCCESS},
                {self._NAME: "receive a SMS", self._FCT: self.__receive_a_sms,
                 self._EXP_RES: Global.SUCCESS}]

        if "NETWORKING" in self._test_type:
            self._fcts2test_net = [
                {self._FCT: self._net.set_flight_mode, self._PARMS: ["0"]},
                {self._FCT: self._net.get_flight_mode},
                {self._FCT: self._net.activate_pdp_context, self._PARMS: [None, False]},
                {self._FCT: self._net.deactivate_pdp_context, self._PARMS: [None, False]},
                {self._FCT: self._net._get_pdp_context_status},
                {self._FCT: self._net.set_roaming_mode, self._PARMS: ["off"]},
                {self._FCT: self._net.get_roaming_mode},
                {self._FCT: self._net.set_apn},
                {self._FCT: self._net.usb_tether, self._PARMS: [1, 0, 0]}]

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        # Call the Tel Debug Base Run_test function
        LabTelDebugBase.run_test(self)

        # Check every EUCmd
        for fct2test in self._fcts2test_vc \
            + self._fcts2test_mod \
            + self._fcts2test_mess \
                + self._fcts2test_net:
            self._check_uecmd(fct2test)
            time.sleep(self._wait_btwn_cmd)

        # Raise an Exception in case of all tests do not pass
        self._compute_general_verdict()

        return Global.SUCCESS, "All UECmds OK"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        # Call the Tel Debug Base tear_down function
        LabTelDebugBase.tear_down(self)

        # PDP context deactivation
        self._networking_api.deactivate_pdp_context()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def __mt_voice_call(self):
        """
        Perform MT voice call test
        """
        # Release any previous call (Robustness)
        self._vca.release()
        # Mobile Terminated originate call
        self._ns_voice_call_3g.mt_originate_call()
        # test wait for State INCOMING validity
        self._vca.wait_for_state(UECmdTypes.VOICE_CALL_STATE.INCOMING, 20)
        # test Answer voice call validity
        self._vca.answer()
        # test wait for State ACTIVE validity
        self._vca.wait_for_state(UECmdTypes.VOICE_CALL_STATE.ACTIVE, 20)
        # Release voice call
        self._vca.release()

#------------------------------------------------------------------------------

    def __dut_registration(self):
        """
        Perform DUT registration test
        """
        # Get RAT from Equipment
        network_type = self._ns_data_3g.get_network_type()

        # Check that DUT is registered on the good RAT
        self._mod.check_network_type_before_timeout(network_type,
                                                    self._registration_timeout)

#---------------------------------------------------------------------------

    def __send_a_sms(self):
        """
        test all UECmd linked to send SMS feature
        """
        # Calculate how many SMS will be sent to equipment
        time.sleep(self._wait_btwn_cmd)
        nb_segments = compute_sms_segments(
            self._sms_text, self._nb_bits_per_char)

        if nb_segments > 1:
            # Enable message queuing
            self._ns_messaging_3g.set_sms_message_queuing_state("ON")

        # Send SMS to equipment using SMS parameters :
        # - SMS_TEXT
        # - DESTINATION_NUMBER
        time.sleep(self._wait_btwn_cmd)
        self._mess.send_sms(self._destination_number,
                            self._sms_text)
        sms_sent = SmsMessage(self._sms_text, self._destination_number, "CSD")

        time.sleep(self._wait_btwn_cmd)

        # Check SMS delivery status OK before timeout using
        # SMS_TRANSFER_TIMEOUT value and number of SMS to be received
        (result_verdict, result_message) = \
            self._ns_messaging_3g.check_sms_delivery_state(sms_sent, nb_segments,
                                                           self._sms_transfer_timeout)

        self._logger.info(result_message)

        return result_verdict

#---------------------------------------------------------------------------

    def __receive_a_sms(self):
        """
        test all EUCmd linked to SMS reception feature
        """
        # Calculate how many SMS will be sent to equipment
        time.sleep(self._wait_btwn_cmd)
        nb_segments = compute_sms_segments(
            self._sms_text, self._nb_bits_per_char)

        sms_sent = SmsMessage(self._sms_text, self._destination_number, "CSD")

        # register on intent to receive incoming sms
        self._mess.register_for_sms_reception()

        if nb_segments > 1:
            # [SEND MO SMS PROCESS]

            # Activate loopback on equipment
            self._ns_messaging_3g.set_sms_mo_loopback_state("ON")

            # Enable message queuing
            self._ns_messaging_3g.set_sms_message_queuing_state("ON")

            # Send SMS to equipment using SMS parameters :
            # - SMS_TEXT
            # - DESTINATION_NUMBER
            # Also ask the UE Command to use synchronization
            self._mess.send_sms(
                self._destination_number,
                self._sms_text,
                True)

            # Wait all incoming sms from DUT to Network Simulator
            self._ns_messaging_3g.check_sms_delivery_state(sms_sent, nb_segments,
                                                           self._sms_transfer_timeout)

            # Read all messages in the queue
            self._ns_messaging_3g.read_all_received_sms()

        else:
            # [SEND SMS DIRECTLY BY EQUIPMENT]

            # Configure the type of the message to send CUSTOM TEXT.
            self._ns_messaging_3g.select_sms_content(self._content_type)

            # Set the custom text message to send, using SMS_TEXT parameter
            if self._content_type == "CTEX":
                self._ns_messaging_3g.set_custom_sms_text(self._sms_text)
            elif self._content_type == "CDAT":
                self._ns_messaging_3g.set_custom_sms_data(self._sms_text)

            # Send SMS to CDK using SMS parameters :
            # - SMS_TEXT
            self._ns_messaging_3g.send_sms()

            # Check sms acknowledged by network simulator
            self._ns_messaging_3g.check_sms_state(
                'ACK', self._sms_transfer_timeout)

        # get sms received
        sms_received = self._mess.wait_for_incoming_sms(self._sms_transfer_timeout)

        # Compare sent and received SMS (Text,
        # Destination number)
        (result_verdict, result_message) = \
            compute_sms_equals(sms_sent, sms_received)

        self._logger.info(result_message)

        return result_verdict
