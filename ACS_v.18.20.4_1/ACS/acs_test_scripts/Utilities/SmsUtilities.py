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
:summary: Utilities class for SMS implementation
:author: dgongalez
:since: 19/08/2010
"""

import math
from UtilitiesFWK.Utilities import Global


class SmsMessage:

    """
    Structure that represent SMS message for UECmd
    """

    def __init__(self, message="", sender="", transportation="", ns_messaging=None, messaging_api=None, data_coding_scheme=None, nb_bits_per_char=7, sms_transfer_timeout=None, content_type=None, direction="MO", sms_service_center_address=None):
        """
        :type ns_messaging: object
        :param ns_messaging: the object representing the equipment messaging api

        :type messaging_api: object
        :param messaging_api: the object representing the DUT messaging api

        :type destination_number: str
        :param destination_number: SMS destination number

        :type transportation: str
        :param transportation: SMS transportation (GSM|GPRS|CSD|PSD)

        :type data_coding_scheme: str
        :param data_coding_scheme: data coding scheme

        :type nb_bits_per_char: Integer
        :param nb_bits_per_char: nb bits by sms character

        :type sms_transfer_timeout: Integer
        :param sms_transfer_timeout: SMS transfer timeout

        :type content_type: str
        :param content_type: content type

        :type direction: str
        :param direction: SMS direction (MO|MT)

        :type sms_service_center_address: str
        :param sms_service_center_address: SMS service center address
        """
        self.message = message
        self.sender = sender
        self.transportation = transportation
        self.ns_messaging = ns_messaging
        self.messaging_api = messaging_api
        self.data_coding_scheme = data_coding_scheme
        self.nb_bits_per_char = nb_bits_per_char
        self.sms_transfer_timeout = sms_transfer_timeout
        self.content_type = content_type
        self.direction = direction
        self.nb_segments = self._compute_sms_segments()
        self._sms_service_center_address = sms_service_center_address

    def __str__(self):
        return "message: " + str(self.message) + ", Destination Number: " + str(self.sender)
        # TODO: need to clarify the phone number displayed above: either Destination or Sender !!!

    def configure_sms(self):
        """
        Configure equipment and DUT for SMS.
        """
        if self.direction == "MO":
            self._configure_mo_sms()
        else:
            self._configure_mt_sms()

    def send_sms(self):
        """
        Send MO or MT SMS.
        """
        if self.direction == "MO":
            self._configure_mo_sms()
            self._send_mo_sms()
        else:
            self._configure_mt_sms()
            self._send_mt_sms()

    def get_sms(self):
        """
        Get MO or MT SMS.

        :rtype: Boolean
        :return: True if sms have the same message text and the same sender,
        return false otherwise
        """
        if self.direction == "MO":
            return self._get_mo_sms()
        else:
            return self._get_mt_sms()

    def _configure_mt_sms(self):
        """
        Configure equipment for MT SMS.
        """
        # Set the preferred connection type
        self.ns_messaging.select_sms_transportation(self.transportation)

        # Set the Data coding scheme using DATA_CODING_SCHEME value
        self.ns_messaging.set_sms_data_coding_scheme(int(self.data_coding_scheme, 16))

    def _configure_mo_sms(self):
        """
        Configure equipment and DUT for MO SMS.
        """
        self.messaging_api.set_service_center_address(self._sms_service_center_address)

        if self.nb_segments > 1:
            # Enable message queuing
            self.ns_messaging.set_sms_message_queuing_state("ON")

    def compare(self, sms_to_compare, check_transportation=False):
        """
        Compare two sms and return True if they are equals, else return False.

        :param sms_to_compare: Instance of SmsMessage to compare with
        :param check_transportation: Check transportation or not
        This option is used only for comparison from Network simulators.

        :rtype: Boolean
        :return: True if sms have the same message text and the same sender,
        return false otherwise
        """
        sms_equals = Global.SUCCESS
        msg = "Sms sent and sms received are equal"
        if self.message != sms_to_compare.message:
            msg = "Sms text received and sms text sent aren't equal"
            sms_equals = Global.FAILURE
        elif self.sender != sms_to_compare.sender:
            msg = "Sms destination received and sms destination sent aren't equal"
            sms_equals = Global.FAILURE

        if check_transportation:
            if self.transportation != sms_to_compare.transportation:
                msg = "Sms transportation received and sms transportation sent aren't equal"
                msg += " (sent:(%s), received:(%s))" % (str(self.transportation), str(sms_to_compare.transportation))
                sms_equals = Global.FAILURE

        if sms_equals == Global.FAILURE:
            msg += " (sent:{%s}, received:{%s})." % (str(self), str(sms_to_compare))
        return sms_equals, msg

    def _send_mt_sms(self):
        """
        Send a MT SMS from equipment (8960).
        """
        # register on intent to receive incoming sms
        self.messaging_api.register_for_sms_reception()

        # Set sender address using DESTINATION_NUMBER on Network simulator
        self.ns_messaging.set_sms_sender_address(self.sender)

        if self.nb_segments > 1:
            # [SEND MO SMS PROCESS]

            # Activate loopback on equipment
            self.ns_messaging.set_sms_mo_loopback_state("ON")

            # Enable message queuing
            self.ns_messaging.set_sms_message_queuing_state("ON")

            # Send SMS to equipment using SMS parameters :
            # - SMS_TEXT
            # - DESTINATION_NUMBER
            # Also ask the UE Command to use synchronization
            self.messaging_api.send_sms(self.sender,
                                        self.message,
                                        True)

            # Wait all incoming sms from DUT to Network Simulator
            self.ns_messaging.check_sms_delivery_state(self, self.nb_segments, self.sms_transfer_timeout)

        else:
            # [SEND SMS DIRECTLY BY EQUIPMENT]

            # Configure the type of the message to send CUSTOM TEXT.
            self.ns_messaging.select_sms_content(self.content_type)

            # Set the custom text message to send, using SMS_TEXT parameter
            if self.content_type == "CTEX":
                self.ns_messaging.set_custom_sms_text(self.message)
            elif self.content_type == "CDAT":
                self.ns_messaging.set_custom_sms_data(self.message)

            # Send SMS to CDK using SMS parameters :
            # - SMS_TEXT
            self.ns_messaging.send_sms()

            # Check sms acknowledged by network simulator
            self.ns_messaging.check_sms_state('ACK', self.sms_transfer_timeout)

    def _send_mo_sms(self):
        """
        Send a MO SMS to equipment (8960).
        """
        # Send SMS to equipment using SMS parameters :
        # - SMS_TEXT
        # - DESTINATION_NUMBER
        self.messaging_api.send_sms(self.sender,
                                    self.message)

    def _get_mt_sms(self):
        """
        Wait for MT SMS. and compare it to SMS sent

        :rtype: Boolean
        :return: True if sms have the same message text and the same sender,
        return false otherwise
        """
        # get sms received
        sms_received = self.messaging_api.wait_for_incoming_sms(self.sms_transfer_timeout)
        # Compare sent and received SMS (Text,
        # Destination number)
        return self.compare(sms_received)

    def _get_mo_sms(self):
        """
        Wait for MO SMS. and compare it to SMS sent

        :rtype: Boolean
        :return: True if sms have the same message text and the same sender,
        return false otherwise
        """

        # Check SMS delivery status OK before timeout using
        # SMS_TRANSFER_TIMEOUT value and number of SMS to be received
        return self.ns_messaging.check_sms_delivery_state(self,
                                                          self.nb_segments,
                                                          self.sms_transfer_timeout)

    def _compute_sms_segments(self):
        """
        Return the segment numbers of a given sms for 8 bits or 7 bits encoding.

        Return the segment numbers of a given sms by performing the following operations:
            2. check the sms length
            3. calculate the number of segments for the sms

        :rtype: int
        :return: Segment numbers
        """
        segments = 0
        message_lenth = len(self.message)

        result = ((float(self.nb_bits_per_char) * float(message_lenth)) / float(8 * 140))

        if result == 0:
            segments = 1
        else:
            segments = int(math.ceil(result))

        return segments


def compute_sms_equals(sms_base, sms_to_compare, check_transportation=False):
    """
    Compare two sms and return True if they are equals, else return False.

    :param sms_base: Instance of SmsMessage representing a SMS.
    :param sms_to_compare: Instance of SmsMessage to compare with
    :param check_transportation: Check transportation or not
    This option is used only for comparison from Network simulators.

    :rtype: Boolean
    :return: True if sms have the same message text and the same sender,
    return false otherwise
    """
    sms_equals = Global.SUCCESS
    msg = "Sms sent and sms received are equal"
    if sms_base.message != sms_to_compare.message:
        msg = "Sms text received and sms text sent aren't equal"
        sms_equals = Global.FAILURE
    elif sms_base.sender != sms_to_compare.sender:
        msg = "Sms destination received and sms destination sent aren't equal"
        sms_equals = Global.FAILURE

    if check_transportation:
        if sms_base.transportation != sms_to_compare.transportation:
            msg = "Sms transportation received and sms transportation sent aren't equal"
            msg += " (sent:(%s), received:(%s))" % (str(sms_base.transportation), str(sms_to_compare.transportation))
            sms_equals = Global.FAILURE

    if sms_equals == Global.FAILURE:
        msg += " (sent:{%s}, received:{%s})." % (str(sms_base), str(sms_to_compare))
    return sms_equals, msg


def compute_sms_equals_dual_phone (sms_base, sms_to_compare, check_transportation=False):
    """
    Compare two sms and return True if they are equals, else return False.

    :param sms_base: Instance of SmsMessage representing a SMS.
    :param sms_to_compare: Instance of SmsMessage to compare with
    :param check_transportation: Check transportation or not
    This option is used only for comparison from Network simulators.

    :rtype: Boolean
    :return: True if sms have the same message text,
    return false otherwise
    """
    sms_equals = Global.SUCCESS
    msg = "Sms sent and sms received are equal"
    if sms_base.message != sms_to_compare.message:
        msg = "Sms text received and sms text sent aren't equal"
        sms_equals = Global.FAILURE

    if check_transportation:
        if sms_base.transportation != sms_to_compare.transportation:
            msg = "Sms transportation received and sms transportation sent aren't equal"
            msg += " (sent:(%s), received:(%s))" % (str(sms_base.transportation), str(sms_to_compare.transportation))
            sms_equals = Global.FAILURE

    if sms_equals == Global.FAILURE:
        msg += " (sent:{%s}, received:{%s})." % (str(sms_base), str(sms_to_compare))
    return sms_equals, msg


def compute_sms_segments(messagebody, encoding):
    """
    Return the segment numbers of a given sms for 8 bits or 7 bits encoding.

    Return the segment numbers of a given sms by performing the following operations:
        2. check the sms length
        3. calculate the number of segments for the sms

    :rtype: int
    :return: Segment numbers
    """
    segments = 0
    message_lenth = len(messagebody)

    result = ((float(encoding) * float(message_lenth)) / float(8 * 140))

    if result == 0:
        segments = 1
    else:
        segments = int(math.ceil(result))

    return segments


class DataCodingScheme:

    """
    Represents a data coding scheme
    """

    def __init__(self, DCS):
        """
        Constructor
        :type DCS: str
        :param DCS: str representing Data Coding Scheme value (in hexadecimal)
        """
        self._character_set = None
        self._message_class = None
        self._compression = None

        self.__dcs = int(DCS)

    def decode(self):
        """
         Decode the data coding scheme
        """

        mask_message_class = int("00000011", 2)
        mask_character_set = int("00001100", 2)
        mask_compression = int("00100000", 2)
        character_set = self.__dcs & mask_character_set
        character_set >>= 2
        self._character_set = {
            0: "7BITS",
            1: "8BITS DATA",
            2: "UCS2",
            3: "Reserved"
        }[character_set]
        compression_value = self.__dcs & mask_compression
        if compression_value == 0:
            self._compression = False
        else:
            self._compression = True
        self._message_class = self.__dcs & mask_message_class

    def encode(self):
        """
        Encecode the data coding scheme
        returning data coding scheme (integer)
        """
        return self.__dcs

    def get_character_set(self):
        """
        Getter: return character set if data coding scheme
        has been decoded, else return None

        :rtype: str or None
        :return: "7BITS", "8BITS DATA", "UCS2", "Reserved" or None
        """
        return self._character_set

    def get_message_class(self):
        """
        Getter: return message class if data coding scheme
        has been decoded, else return None

        :rtype: int or None
        :return: 0, 1, 2 or 3
        """
        return self._message_class

    def get_compression(self):
        """
        Getter: return true or false if data coding scheme
        has been decoded, else return None

        :rtype: boolean
        :return:
            - C{True} if text is compressed.
            - C{False} if text is uncompressed.
        """
        return self._compression

    def set_character_set(self, character_set):
        """
        Setter: set the character set

        :type character_set: str
        :param character_set: "7BITS", "8BITS DATA", "UCS2" or "Reserved"
        """
        self._character_set = character_set

        char_set = {
            "7BITS": 0,
            "8BITS DATA": 1,
            "UCS2": 2,
            "Reserved": 3
        }[self._character_set]
        remove_mask = int("11110011", 2)
        self.__dcs &= remove_mask
        self.__dcs += char_set << 2

    def set_message_class(self, message_class):
        """
        Setter :  set the message class

        :type message_class: integer 0, 1, 2 or 3
        :param message_class: message class to set.
        """
        self._message_class = message_class
        remove_mask = int("11111100", 2)
        self.__dcs &= remove_mask
        self.__dcs += self._message_class

    def set_compression(self, compression):
        """
        Setter : set the compression

        :type compression: boolean
        :param compression: set text compression:
            - C{True} text is compressed.
            - C{False} text is uncompressed.
        """
        self._compression = compression
        compression_value = 0
        if self._compression:
            compression_value = 1
        remove_mask = int("11011111", 2)
        self.__dcs &= remove_mask
        self.__dcs += compression_value << 5

    def compute_character_size(self):
        """
        Compute the number of bit used per character, or None if not defined.

        :rtype: integer
        :return: number of bit per character.
        """
        size = {
            "7BITS": 7,
            "8BITS DATA": 8,
            "UCS2": 16,
            "Reserved": None
        }[self._character_set]
        return size
    

