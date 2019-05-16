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
:summary: messaging 2G implementation for Agilent 8960 cellular network
simulator
:since: 08/03/2011
:author: ymorel
"""

import time
from acs_test_scripts.Utilities.SmsUtilities import SmsMessage, compute_sms_equals
from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Interface.IMessaging2G import IMessaging2G
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Agilent8960.Common.Messaging import Messaging
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Agilent8960.Wrapper.Tech2G import WMessaging2G as W


class Messaging2G(IMessaging2G, Messaging):

    """
    2G messaging  implementation for Agilent 8960
    """

    def __init__(self, root):
        """
        Constructor
        :type root: weakref
        :param root: a weak reference on the root class (Agilent8960)
        """
        self.__root = root
        Messaging.__init__(self, root)

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

    def get_root(self):
        """
        Gets the root of the equipment
        :rtype: Agilent8960
        :return: the root of the equipment
        """
        return self.__root

    def get_logger(self):
        """
        gets the logger
        """
        return self.get_root().get_logger()

    def clear_message_data(self):
        """
        Clears all messages.
        """
        (err, msg) = W.ClearMessageData(self.get_root())
        self.__error_check(err, msg)

    def set_sms_message_queuing_state(self, state):
        """
        Sets SMS message queuing state.
        :type state: str
        :param state: desired state:
            - "ON"
            - "OFF"
        """
        (err, msg) = W.SetSmsMessageQueuingState(self.get_root(), state)
        self.__error_check(err, msg)

    def get_last_sms(self):
        """
        Gets last SMS.
        :rtype: str
        :return: the text of the last SMS.
        """
        (err, sms, msg) = W.GetLastSMS(self.get_root())
        self.__error_check(err, msg)
        return sms

    def get_last_sms_length(self):
        """
        Gets last SMS length.
        :rtype: integer
        :return: the length of the last SMS.
        """
        (err, length, msg) = W.GetLastSMSLength(self.get_root())
        self.__error_check(err, msg)
        return length

    def get_nb_received_sms(self):
        """
        Gets the number of received SMS.
        :rtype: long
        :return: the number of SMS received by the equipment.
        """
        (err, nb_received, msg) = W.GetNbReceivedSMS(self.get_root())
        self.__error_check(err, msg)
        return nb_received

    def move_next_sms(self):
        """
        Moves to next SMS.
        """
        (err, msg) = W.MoveNextSMS(self.get_root())
        self.__error_check(err, msg)

    def set_sms_data_coding_scheme(self, scheme):
        """
        Sets SMS data coding scheme.
        :type scheme: integer
        :param scheme: data coding scheme to set (0 to 255).
        """
        (err, msg) = W.SetSMSDataCodingScheme(self.get_root(), scheme)
        self.__error_check(err, msg)

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
        (err, msg) = W.SelectSMSContent(self.get_root(), sms_content)
        self.__error_check(err, msg)

    def select_sms_transportation(self, sms_trans):
        """
        Selects SMS transportation
        :type sms_trans: str
        :param sms_trans: the SMS transportation mechanism to select.
            Possible values:
                - "GSM"
                - "GPRS"
        """
        (err, msg) = W.SelectSMSTransportation(self.get_root(), sms_trans)
        self.__error_check(err, msg)

    def set_custom_sms_text(self, sms_text):
        """
        Sets SMS custom text.
        :type sms_text: str
        :param sms_text: the text of the SMS to send.
        """
        (err, msg) = W.SetCustomSMSText(self.get_root(), sms_text)
        self.__error_check(err, msg)

    def send_sms(self):
        """
        Sends a SMS.
        """
        (err, msg) = W.SendSMS(self.get_root())
        self.__error_check(err, msg)

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
        (err, state, msg) = W.GetSMSSendState(self.get_root())
        self.__error_check(err, msg)
        return state

    def set_service_center_address(self, service_center):
        """
        Sets service center address.
        :type service_center: str
        :param service_center: the address of the SMS service center to set
        """
        (err, msg) = W.SetServiceCenterAddress(self.get_root(), service_center)
        self.__error_check(err, msg)

    def set_sms_mo_loopback_state(self, state):
        """
        Sets SMS loopback state.
        :type state: str
        :param state: the desired state. Possible values:
            - "ON"
            - "OFF"
        """
        (err, msg) = W.SetSmsMoLoopbackState(self.get_root(), state)
        self.__error_check(err, msg)

    def get_sms_destination(self):
        """
        gets SMS destination.
        :rtype: str
        :return: the destination of the last received SMS.
        """
        (err, dest, msg) = W.GetSmsDestination(self.get_root())
        self.__error_check(err, msg)
        return dest

    def get_sms_transportation(self):
        """
        gets SMS transportation.
        :rtype: str
        :return: the the transportation of the last received SMS.
        """
        (err, sms_transportation, msg) = \
            W.GetLastSMSTransportation(self.get_root())
        self.__error_check(err, msg)
        return sms_transportation

    def set_custom_sms_data(self, sms_data):
        """
        Sets custom SMS data.
        :type sms_data: str
        :param sms_data: the data of the SMS to send. Size of the data
        to set must be a pair number between 0 and 280. String of hexadecimal
        digits (0-9; a-f; A-F).
        """
        (err, msg) = W.SetCustomSMSData(self.get_root(), sms_data)
        self.__error_check(err, msg)

    def set_sms_sender_address(self, address):
        """
        Sets SMS sender address.
        :type address: str
        :param address: address to be set. Range: 2 to 20 characters.
        SMS address to set. ASCII str of BCD digits 0-9, the symbols
        * and #, and the lower case characters a, b, c, and f.
        """
        (err, msg) = W.SetSmsSenderAdress(self.get_root(), address)
        self.__error_check(err, msg)

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
        timeout = timeout_per_sms * nb_sms
        all_received = (self.get_nb_received_sms() == nb_sms)

        # Check if the expected number of SMS is reached
        while (timeout > 0) and not all_received:
            time.sleep(1)
            all_received = (self.get_nb_received_sms() == nb_sms)
            timeout -= 1

        if not all_received:
            error_msg = "SMS not received : waited %d seconds for %d SMS." % \
                (timeout, nb_sms)
            self.get_logger().error(error_msg)
            raise TestEquipmentException(TestEquipmentException.SMS_EXCEPTION, error_msg)

        # Check if the content of the received SMS
        sms_received = self.assemble_received_sms()

        (result_verdict, result_message) = sms_sent.compare(sms_received, check_transportation=True)

        self.get_logger().info(result_message)

        return result_verdict, result_message

    def check_sms_transportation(self, transportation):
        """
        Check that SMS transportation is equal to expected value.
        :type transportation: str
        :param transportation: expected transportation.
            Possible values:
                - "GSM"
                - "GPRS"
        """
        self.get_logger().info(
            "Check that received SMS transportation is %s",
            transportation)

        sms_transportation = self.get_sms_transportation()

        if sms_transportation != transportation:
            error_msg = "Error SMS transportation is %s !" % \
                        sms_transportation
            self.get_logger().error(error_msg)
            raise TestEquipmentException(TestEquipmentException.SMS_EXCEPTION, error_msg)

    def assemble_received_sms(self):
        """
        Gets all SMS received from the DUT, and re-form the original message.
        :rtype: (str, str)
        :return: couple of message and destination number of all SMS received.
        """
        sms_message = ""
        sms_destination = ""
        sms_transportation = ""

        # Get SMS from Equipment
        nb_sms = self.get_nb_received_sms()
        while nb_sms > 0:
            message_tmp = self.get_last_sms()
            sms_destination = self.get_sms_destination()
            sms_transportation = self.get_sms_transportation()
            sms_message += message_tmp
            self.move_next_sms()
            nb_sms -= 1

        return SmsMessage(sms_message, sms_destination, sms_transportation)

    def read_all_received_sms(self):
        """
        Reads all SMS from the SMS queue.
        """
        nb_rcv_sms = self.get_nb_received_sms()
        while nb_rcv_sms > 0:
            self.move_next_sms()
            time.sleep(1)
            nb_rcv_sms -= 1

    def check_sms_state(self, state, timeout=0):
        """
        Check that SMS state is reached before timeout seconds.
        If timeout <= 0, only one test is performed.
        :type state: str
        :param state: expected SMS state
        :type timeout: integer
        :param timeout: allowed time to reach expected state
        """
        self.get_logger().info(
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
            self.get_logger().info("SMS state is %s!", state)
        else:  # expected state not reached
            msg = "Check SMS state to %s timeout !" % state
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.SMS_EXCEPTION, msg)

    def start_cell_broadcast(self):
        """
        Starts the cell broadcast service.
        """
        Messaging.start_cell_broadcast(self)

    def stop_cell_broadcast(self):
        """
        Stops the cell broadcast service.
        """
        Messaging.stop_cell_broadcast(self)

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
        Messaging.set_cell_broadcast_message_identifier(self,
                                                        message_identifier,
                                                        cell_broadcast_message_index)

    def set_custom_cell_broadcast_text_message(self,
                                               text,
                                               cell_broadcast_message_index=1):
        """
        Sets the user defined text message in the cell broadcast message
        :type text: str
        :param text: user defined text message
        :type cell_broadcast_message_index: int
        :param cell_broadcast_message:cell broadcast message number
        Possible values for Agilent 8960 :1, 2 or 3
        """
        Messaging.set_custom_cell_broadcast_text_message(self,
                                                         text,
                                                         cell_broadcast_message_index)

    def set_cell_broadcast_message_repetition_period(self, repetition_period):
        """
        Sets the user defined text message in the cell broadcast message
        :type repetition_period: int
        :param repetition_period: the repetition period in seconds for sending
                                  the cell broadcast message, range: 1 to 1800
                                  default value is 30
        """
        Messaging.set_cell_broadcast_message_repetition_period(self,
                                                               repetition_period)

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
        :param cell_broadcast_message:cell broadcast message number
        Possible values for Agilent 8960 :1, 2 or 3
        """
        Messaging.select_cell_broadcast_message_content(self,
                                                        cell_broadcast_message_content,
                                                        cell_broadcast_message_index)

    def set_custom_cell_broadcast_data(self, data_string, cell_broadcast_message_index):
        """
        Sets the user defined data str in the cell broadcast message
        :type data_string: str
        :param data_string: user defined text message
        :type cell_broadcast_message_index: int
        :param cell_broadcast_message_index:cell broadcast message number
        Possible values for Agilent 8960 :1, 2 or 3
        """
        Messaging.set_custom_cell_broadcast_data(self,
                                                 data_string,
                                                 cell_broadcast_message_index)

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
        Messaging.set_cell_broadcast_message_update_number(self,
                                                           update_number,
                                                           cell_broadcast_message_index)
