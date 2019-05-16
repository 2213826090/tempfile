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
:summary: virtual interface of  messaging functionalities for cellular network
simulators
:since: 10/02/2011
:author: ymorel
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException

# pylint: disable=W0613


class IMessaging(object):

    """
    IMessaging class: virtual interface of messaging functionalities
    for cellular network simulators.
    """

    def clear_message_data(self):
        """
        Clears all messages.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_sms_message_queuing_state(self, state):
        """
        Sets SMS message queuing state.
        :type state: str
        :param state: the state to set. Possible values:
            - "ON"
            - "OFF"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_last_sms(self):
        """
        Gets last SMS.
        :rtype: str
        :return: the text of the last SMS.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_last_sms_length(self):
        """
        Gets last SMS length.
        :rtype: integer
        :return: the length of the last SMS.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_nb_received_sms(self):
        """
        Gets the number of received SMS.
        :rtype: long
        :return: the number of SMS received by the equipment.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def move_next_sms(self):
        """
        Moves to next SMS.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_sms_data_coding_scheme(self, scheme):
        """
        Sets SMS data coding scheme.
        :type scheme: integer
        :param scheme: data coding scheme to set (0 to 255)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def select_sms_transportation(self, sms_trans):
        """
        Selects SMS transportation.
        :type sms_trans: str
        :param sms_trans: the SMS transportation mechanism to select. Possible values:
            - "GSM"
            - "GPRS"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_custom_sms_text(self, sms_text):
        """
        Sets custom SMS text.
        :type sms_text: str
        :param sms_text: the text of the SMS to send.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def send_sms(self):
        """
        Sends a SMS.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_service_center_address(self, service_center):
        """
        Sets service center address.
        :type service_center: str
        :param service_center: the address of the SMS service center to set
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_sms_mo_loopback_state(self, state):
        """
        Sets SMS loopback state.
        :type state: str
        :param state: the desired state. Possible values:
            - "ON"
            - "OFF"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_sms_destination(self):
        """
        Gets SMS destination.
        :rtype: str
        :return: the destination of the last received SMS.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_sms_transportation(self):
        """
        Gets SMS transportation.
        :rtype: str
        :return: the transportation of the last received SMS.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_custom_sms_data(self, sms_data):
        """
        Sets SMS custom data.
        :type sms_data: str
        :param sms_data: the data of the SMS to send. Size of the data
        to set must be a pair number between 0 and 280. String of hexadecimal
        digits (0-9; a-f; A-F).
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_sms_sender_address(self, address):
        """
        Sets SMS sender address.
        :type address: str
        :param address: address to be set. Range: 2 to 20 characters.
        SMS address to set. ASCII str of BCD digits 0-9, the symbols
        * and #, and the lower case characters a, b, c, and f.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def check_sms_transportation(self, transportation):
        """
        Check that SMS transportation is equal to expected value.
        :type transportation: str
        :param transportation: expected transportation. Possible values:
            .. todo:: to complete with possible transportation values
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def assemble_received_sms(self):
        """
        Gets all SMS received from the DUT, and re-form the original message.
        :rtype: (str, str)
        :return: couple of message and destination number of all SMS received.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def read_all_received_sms(self):
        """
        Reads all SMS from the SMS queue.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def check_sms_state(self, state, timeout=0):
        """
        Check that SMS state is reached before timeout seconds.
        If timeout <= 0, only one test is performed.
        :type state: str
        :param state: expected SMS state
        :type timeout: integer
        :param timeout: allowed time to reach expected state
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_cell_broadcast_message_identifier(self,
                                              message_identifier,
                                              cell_broadcast_message):
        """
        Set cell broadcast message identifier to identify the message topic.
        :type message_identifier: int
        :param message_identifier: The parameter is a header number identifying
         the message topic  (such as 'Weather Report' or 'Traffic Information').
        :type cell_broadcast_message: int
        :param cell_broadcast_message:cell broadcast message number
        Possible values for Agilent 8960 :1, 2 or 3
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def start_cell_broadcast(self):
        """
        Starts the cell broadcast service.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def stop_cell_broadcast(self):
        """
        Stops the cell broadcast service.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_custom_cell_broadcast_text_message(self, text, cell_broadcast_message):
        """
        Sets the user defined text message in the cell broadcast message
        :type text: str
        :param text: user defined text message
        :type cell_broadcast_message: int
        :param cell_broadcast_message:cell broadcast message number
        Possible values for Agilent 8960 :1, 2 or 3
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_cell_broadcast_message_repetition_period(self, repetition_period):
        """
        Sets the user defined text message in the cell broadcast message
        :type repetition_period: int
        :param repetition_period: the repetition period in seconds for sending
                                  the cell broadcast message.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def select_cell_broadcast_message_content(self,
                                              cell_broadcast_message_content,
                                              cell_broadcast_message_index):
        """
        Selects SMS content.
        :type cell_broadcast_message_content: str
        :param cell_broadcast_message_content: the cell broadcast sms content
                to select. Possible values:
            - "TXT1"
            - "TXT2"
            - "CTEX"
            - "CDAT"
        :type cell_broadcast_message_index: int
        :param cell_broadcast_message:cell broadcast message number
        Possible values for Agilent 8960 :1, 2 or 3
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_custom_cell_broadcast_data(self, data_string, cell_broadcast_message):
        """
        Sets the user defined data str in the cell broadcast message
        :type data_string: str
        :param data_string: user defined text message
        :type cell_broadcast_message: int
        :param cell_broadcast_message:cell broadcast message number
        Possible values for Agilent 8960 :1, 2 or 3
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_cell_broadcast_message_update_number(self,
                                                 update_number,
                                                 cell_broadcast_message_index):
        """
        Sets the number to identify a particular version of the message
        :type update_number: int
        :param update_number: the number to identify the message version
        :type cell_broadcast_message_index: int
        :param cell_broadcast_message_index:cell broadcast message number
        Possible values for Agilent 8960 :1, 2 or 3
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)
