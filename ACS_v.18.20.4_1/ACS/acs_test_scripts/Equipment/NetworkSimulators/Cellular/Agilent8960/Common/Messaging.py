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
:summary: Common messaging(2G, 3G & 4G) implementation for Agilent 8960
:since: 14/03/2013
:author: hbian
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException


class Messaging(object):

    """
    Common messaging(2G, 3G) implementation for Agilent 8960
    """

    def __init__(self, root):  # pylint: disable=W0231
        """
        Constructor
        :type root: weakref
        :param root: a weak reference on the root class (Agilent8960)
        """
        self.__root = root

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
        Gets the logger
        """
        return self.get_root().get_logger()

    def start_cell_broadcast(self):
        """
        Starts the cell broadcast service.
        """
        gpib_command = "CALL:SMService:CBRoadcast:STARt"
        self.__root.send_command(gpib_command)

    def stop_cell_broadcast(self):
        """
        Stops the cell broadcast service.
        """
        gpib_command = "CALL:SMService:CBRoadcast:STOP"
        self.__root.send_command(gpib_command)

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
        self.__root.send_command(gpib_command)

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

        if cell_broadcast_message_index not in (1, 2, 3):
            error_msg = "cell broadcast message number possible values for" \
                "Agilent 8960 :1, 2 or 3"
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, error_msg)

        self.get_logger().info("Set custom cell broadcast text message %s for "
                               " Cell Broadcast Message %d",
                               text, cell_broadcast_message_index)
        gpib_command = "CALL:SMService:CBRoadcast:MESSage%d:CTEXt '%s'" % \
            (cell_broadcast_message_index, text)
        self.__root.send_command(gpib_command)

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
        self.__root.send_command(gpib_command)

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
        self.__root.send_command(gpib_command)

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
        self.__root.send_command(gpib_command)

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
        self.__root.send_command(gpib_command)
