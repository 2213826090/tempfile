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
:summary: Common messaging(2G, 3G & 4G) implementation for Agilent 8960 Visa
:since: 13/03/2015
:author: gcharlex
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException
import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Interface.IMessaging import IMessaging
from acs_test_scripts.TestStep.Utilities.Visa import VisaObject
from acs_test_scripts.Utilities.SmsUtilities import SmsMessage, compute_sms_equals


class MessagingVisa(IMessaging, VisaObject):

    """
    Common messaging(2G, 3G) implementation for Agilent 8960 Visa
    """

    def __init__(self, visa):
        """
        Constructor
        :type visa: VisaInterface
        :param visa: the PyVisa connection
        """
        VisaObject.__init__(self, visa)

    def __error_check(self, err, msg):
        """
        Error checking and warning reporting
        :raise TestEquipmentException: if err < 0
        """
        if err < 0:
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, msg)
        elif err > 0:
            self.get_logger().warning(msg)

    def start_cell_broadcast(self):
        """
        Starts the cell broadcast service.
        """
        gpib_command = "CALL:SMService:CBRoadcast:STARt"
        self._visa.write(gpib_command)

    def stop_cell_broadcast(self):
        """
        Stops the cell broadcast service.
        """
        gpib_command = "CALL:SMService:CBRoadcast:STOP"
        self._visa.write(gpib_command)

    def set_cell_broadcast_message_identifier(self,
                                              message_identifier,
                                              cell_broadcast_message_index=1):
        """
        Set cell broadcast message identifier to identify the message topic.
        :type message_identifier: int
        :param message_identifier: The parameter is a header number identifying
         the message topic  (such as 'Weather Report' or 'Traffic Information').
        :type cell_broadcast_message_index: int
        :param cell_broadcast_message_index:cell broadcast message number
        Possible values for Agilent 8960 :1, 2 or 3
        """

        if cell_broadcast_message_index not in (1, 2, 3):
            msg = "cell broadcast message number possible values for" \
                "Agilent 8960 :1, 2 or 3"
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        self.get_logger().info("Set cell broadcast message_identifier %d for "
                               " Cell Broadcast Message Index  %d",
                               int(message_identifier), cell_broadcast_message_index)
        gpib_command = "CALL:SMService:CBRoadcast:MESSage%d:IDENtifier %d" % \
            (cell_broadcast_message_index, int(message_identifier))
        self._visa.write(gpib_command)

    def set_custom_cell_broadcast_text_message(self,
                                               text,
                                               cell_broadcast_message_index=1):
        """
        Sets the user defined text message in the cell broadcast message
        :type text: str
        :param text: user defined text message
        :type cell_broadcast_message_index: int
        :param cell_broadcast_message_index:cell broadcast message number
        Possible values for Agilent 8960 :1, 2 or 3
        """

        if cell_broadcast_message_index not in (1, 2, 3):
            error_msg = "cell broadcast message number possible values for" \
                "Agilent 8960 :1, 2 or 3"
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, error_msg)

        self.get_logger().info("Set custom cell broadcast text message %s for "
                               " Cell Broadcast Message %d",
                               text, cell_broadcast_message_index)
        gpib_command = "CALL:SMService:CBRoadcast:MESSage%d:CTEXt '%s'" % \
            (cell_broadcast_message_index, text)
        self._visa.write(gpib_command)

    def set_cell_broadcast_message_repetition_period(self, repetition_period):
        """
        Sets the user defined text message in the cell broadcast message
        :type repetition_period: int
        :param repetition_period: the repetition period in seconds for sending
                                  the cell broadcast message, range: 1 to 1800
                                  default value is 30
        """
        if 0 < repetition_period < 1800:
            self.get_logger().info("Set custom cell broadcast message repetition"
                                   "period to %d seconds" % repetition_period)
        else:
            error_msg = "repetition_period should be an integer between 1 and 1800"
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, error_msg)

        gpib_command = "CALL:SMService:CBRoadcast:REPetition %d" % int(repetition_period)
        self._visa.write(gpib_command)

    def select_cell_broadcast_message_content(self,
                                              cell_broadcast_message_content,
                                              cell_broadcast_message_index=1):
        """
        Selects SMS content.
        :type cell_broadcast_message_content: str
        :param cell_broadcast_message_content: the cell broadcast sms
               content to select. Possible values:
                - "TXT1"
                - "TXT2"
                - "CTEX"
                - "CDAT"
        :type cell_broadcast_message_index: int
        :param cell_broadcast_message_index:cell broadcast message number
        Possible values for Agilent 8960 :1, 2 or 3
        """

        if cell_broadcast_message_index not in (1, 2, 3):
            error_msg = "cell broadcast message number possible values for" \
                "Agilent 8960 :1, 2 or 3"
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, error_msg)

        if cell_broadcast_message_index not in (1, 2, 3):
            error_msg = "cell broadcast message number possible values for" \
                "Agilent 8960 :1, 2 or 3"
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, error_msg)

        gpib_command = "CALL:SMService:CBRoadcast:MESSage%d:CONT %s" % \
            (cell_broadcast_message_index, cell_broadcast_message_content)
        self._visa.write(gpib_command)

    def set_custom_cell_broadcast_data(self, data_string, cell_broadcast_message_index):
        """
        Sets the user defined data str in the cell broadcast message
        :type data_string: str
        :param data_string: user defined text message
        :type cell_broadcast_message_index: int
        :param cell_broadcast_message_index:cell broadcast message number
        Possible values for Agilent 8960 :1, 2 or 3
        """

        if cell_broadcast_message_index not in (1, 2, 3):
            error_msg = "cell broadcast message number possible values for" \
                "Agilent 8960 :1, 2 or 3"
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, error_msg)

        self.get_logger().info("Set custom cell broadcast data %s for "
                               " Cell Broadcast Message %d",
                               data_string, cell_broadcast_message_index)
        gpib_command = "CALL:SMService:CBRoadcast:MESSage%d:CDATa '%s'" % \
            (cell_broadcast_message_index, data_string)
        self._visa.write(gpib_command)

    def set_cell_broadcast_message_update_number(self,
                                                 update_number,
                                                 cell_broadcast_message_index):
        """
        Sets the number to identify a particular version of the message
        :type update_number: int
        :param update_number: the number to identify the message version
        Possible values: 0-15
        :type cell_broadcast_message_index: int
        :param cell_broadcast_message_index:cell broadcast message number
        Possible values for Agilent 8960 :1, 2 or 3
        """

        if cell_broadcast_message_index not in (1, 2, 3):
            error_msg = "cell broadcast message number possible values for" \
                "Agilent 8960 :1, 2 or 3"
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, error_msg)
        elif update_number not in range(16):
            error_msg = "cell broadcast message update number should be" \
                "between 0-15"
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, error_msg)

        self.get_logger().info("Set custom cell broadcast update number %d for "
                               " Cell Broadcast Message %d",
                               update_number, cell_broadcast_message_index)
        gpib_command = "CALL:SMService:CBRoadcast:MESSage%d:UPDate %d" % \
            (cell_broadcast_message_index, update_number)
        self._visa.write(gpib_command)

    def check_sms_delivery_state(self, sms_sent, nb_sms, timeout_per_sms=0):
        """
        Check if all SMS sent have been received before timeout_per_sms * nb_sms
        seconds.
        :type sms_sent: SmsMessage object
        :param sms_sent: sent SMS to be checked
        :type nb_sms: integer
        :param nb_sms: number of SMS to be delivered
        :type timeout_per_sms: integer
        :param timeout_per_sms: timeout per SMS. If timeout is 0, only one test
        is performed
        :raise TestEquipmentException: if one or more expected SMS have not been received
        before timeout*nb_sms seconds.
        """
        log = True
        all_received = False

        timeout = timeout_per_sms * nb_sms

        # Check if the expected number of SMS is reached
        while timeout > 0:
            if int(self._visa.query("CALL:SMService:PTPoint:MORiginated:COUNt?", log)) == int(nb_sms):
                all_received = True
                break
            time.sleep(1)
            timeout -= 1

        if not all_received:
            error_msg = "SMS not received : waited %d seconds for %d SMS." % \
                (timeout, nb_sms)
            self._logger.error(error_msg)
            raise TestEquipmentException(TestEquipmentException.SMS_EXCEPTION, error_msg)

        # Check if the content of the received SMS
        sms_message = ""
        sms_destination = ""
        sms_transportation = ""

        # Get SMS from Equipment
        nb_sms = int(self._visa.query("CALL:SMService:PTPoint:MORiginated:COUNt?", log))
        for _ in range(nb_sms):
            message_tmp = str(self._visa.query("CALL:SMService:PTPoint:MORiginated:TEXT?", log)).replace("\"", "")
            sms_destination = str(self._visa.query("CALL:SMService:PTPoint:MORiginated:DESTination?", log)).replace("\"", "")
            sms_transportation = self._visa.query("CALL:SMService:PTPoint:MORiginated:TRANsport?", log)
            sms_message += message_tmp
            self._visa.write("CALL:SMService:PTPoint:MORiginated:QUEue:NEXT")

        sms_received = SmsMessage(sms_message, sms_destination, sms_transportation)
        (result_verdict, result_message) = \
            compute_sms_equals(sms_sent, sms_received, check_transportation=True)

        self._logger.info(result_message)
        if result_verdict == Global.FAILURE:
            raise TestEquipmentException(TestEquipmentException.SMS_EXCEPTION, result_message)

        return result_verdict, result_message

    def select_sms_transportation(self, sms_trans):
        """
        Selects SMS transportation
        :type sms_trans: str
        :param sms_trans: the SMS transportation mechanism to select.
            Possible values:
                - "GSM"
                - "GPRS"
        """
        gpib_command = "CALL:SMS:PTP:TRAN %s" % (sms_trans)
        self._visa.write(gpib_command)

    def set_sms_data_coding_scheme(self, scheme):
        """
        Sets SMS data coding scheme.
        :type scheme: integer
        :param scheme: data coding scheme to set (0 to 255).
        """
        gpib_command = "CALL:SMS:PTP:DCSC %s" % (scheme)
        self._visa.write(gpib_command)

    def set_sms_sender_address(self, address):
        """
        Sets SMS sender address.
        :type address: str
        :param address: address to be set. Range: 2 to 20 characters.
        SMS address to set. ASCII str of BCD digits 0-9, the symbols
        * and #, and the lower case characters a, b, c, and f.
        """
        gpib_command = "CALL:SMS:PTP:OADD \"%s\"" % (address)
        self._visa.write(gpib_command)

    def set_sms_message_queuing_state(self, state):
        """
        Sets SMS message queuing state.
        :type state: str
        :param state: desired state:
            - "ON"
            - "OFF"
        """
        gpib_command = "CALL:SMS:PTP:MOR:QUE %s" % (state)
        self._visa.write(gpib_command)

    def set_sms_mo_loopback_state(self, state):
        """
        Sets SMS loopback state.
        :type state: str
        :param state: the desired state. Possible values:
            - "ON"
            - "OFF"
        """
        gpib_command = "CALL:SMS:PTP:MOR:LOOP %s" % (state)
        self._visa.write(gpib_command)

    def read_all_received_sms(self):
        """
        Reads all SMS from the SMS queue.
        """
        nb_rcv_sms = self._visa.query("CALL:SMService:PTPoint:MORiginated:COUNt?", True)
        while nb_rcv_sms > 0:
            self.move_next_sms()
            time.sleep(1)
            nb_rcv_sms -= 1

    def select_sms_content(self, sms_content):
        """
        Selects SMS content.
        :type sms_content: str
        :param sms_content: the SMS content to select. Possible values:
                - "TXT1"
                - "TXT2"
                - "CTEX"
                - "CDAT"
        """
        gpib_command = "CALL:SMS:PTP:CONT %s" % (sms_content)
        self._visa.write(gpib_command)

    def set_custom_sms_text(self, sms_text):
        """
        Sets SMS custom text.
        :type sms_text: str
        :param sms_text: the text of the SMS to send.
        """
        gpib_command = "CALL:SMS:PTP:TEXT:CUST \"%s\"" % (sms_text)
        self._visa.write(gpib_command)

    def send_sms(self):
        """
        Sends a SMS.
        """
        gpib_command = "CALL:SMS:PTP:SEND"
        self._visa.write(gpib_command)

    def check_sms_state(self, state, timeout=0):
        """
        Check that SMS state is reached before timeout seconds.
        If timeout <= 0, only one test is performed.
        :type state: str
        :param state: expected SMS state
        :type timeout: integer
        :param timeout: allowed time to reach expected state
        """
        self._logger.info(
            "Check SMS state is %s before %d seconds",
            state,
            timeout)

        timer = timeout
        sms_state = self.get_sms_send_state()
        state_reached = (sms_state == state)

        while (timer > 0) and not state_reached:
            time.sleep(1)
            timer -= 1
            state_reached = (self.get_sms_send_state() == state)

        if state_reached:
            self._logger.info("SMS state is %s!", state)
        else:  # expected state not reached
            msg = "Check SMS state to %s timeout !" % state
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.SMS_EXCEPTION, msg)

    def get_sms_send_state(self):
        """
        Gets SMS send state.
        :rtype: str
        :return: the str representation of the SMS send state.
            Possible returned values:
                - "IDLE"
                - "SEND"
                - "ACK"
                - "NACK"
                - "REJ"
                - "FAIL"
        """
        state = self._visa.query("CALL:SMService:PTPoint:SEND:STATe?", True)
        return state
