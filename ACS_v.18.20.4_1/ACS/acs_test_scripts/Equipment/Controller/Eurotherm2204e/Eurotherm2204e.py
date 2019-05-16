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
:summary: Implementation for Eurotherm2204e Controller.
:since: 14/03/2012
:author: vgombert
"""

from acs_test_scripts.Equipment.IEquipment import EquipmentBase
from acs_test_scripts.Equipment.Controller.Interface.IController import IController
from ErrorHandling.TestEquipmentException import TestEquipmentException
import serial
import time


class Eurotherm2204e(EquipmentBase, IController):

    """
    Class that implements Eurotherm2204e controller.
    """

    SERIAL_TRANSPORT = "SERIAL"

    def __init__(self, name, model, eqt_params, bench_params):
        """
        Constructor
        :type name: str
        :param name: the bench configuration name of the equipment
        :type model: str
        :param model: the model of the equipment
        :type eqt_params: dict
        :param eqt_params: the dictionary containing equipment parameters
        :type bench_params: dict
        :param bench_params: the dictionary containing equipment bench parameters
        """
        EquipmentBase.__init__(self, name, model, eqt_params)
        IController.__init__(self)
        self.__bench_params = bench_params
        self.__handle = None
        self._reply_timeout = 120

    def __del__(self):
        """
        Destructor: release automatically the connection
        """
        self.get_logger().debug("Delete %s", str(self.get_name()))
        self.release()

    def __connect_via_SERIAL(self):
        """
        Connect to equipment via SERIAL
        """
        # Initialize local variables
        handle = None
        try_index = 0

        # Get Serial parameters for connection
        # on widows , serial port pass by value is incremented by 1 by serial.Serial, so we remove 1 to target exactly match the right com port
        serial_com_port = max(int(self.__bench_params.get_param_value("ComPort")) - 1, 0)
        serial_baud_rate = int(self.__bench_params.get_param_value("BaudRate"))
        retry_nb = int(self.__bench_params.get_param_value("ConnectionRetry"))

        # Access to serial port is not reliable
        # Try several times
        while try_index < retry_nb:
            try:
                handle = serial.Serial(
                    port=serial_com_port,
                    baudrate=serial_baud_rate,
                    timeout=1
                )

                break
            except serial.SerialException as error:
                if try_index < retry_nb:
                    self.get_logger().error(str(error))
                    self.get_logger().info("Retry the connection...")
                    try_index += 1
                    time.sleep(1)
                else:
                    raise error

        if handle is None:
            raise TestEquipmentException(TestEquipmentException.CONNECTION_ERROR,
                                "Serial port connection failed.")

        # Update handle value
        self.__set_handle(handle)

    def __connect_via_transport(self, transport):
        """
        Connect to equipment using parameter transport

        :type transport: String
        :param transport: The transport use to connect to the equipment
        """
        if transport == Eurotherm2204e.SERIAL_TRANSPORT:
            self.__connect_via_SERIAL()
        else:
            error_msg = "Unsupported transport mode '%s'." % str(transport)
            self.get_logger().error(error_msg)
            raise TestEquipmentException(TestEquipmentException.READ_PARAMETER_ERROR, error_msg)

    def __set_handle(self, handle):
        """
        Sets the connection handle
        :type handle: unsigned integer
        :param handle: the new connection handle
        """
        self.__handle = handle

    def __get_handle(self):
        """
        Gets the connection handle
        :rtype: unsigned long
        :return: the handle of connection with the equipment, None if no
        equipment is connected
        """
        return self.__handle

    def __encode(self, message):
        """
        Format the given message to be sent to the controller.

        :param message: message to send to the controller,
                        each element of the message should be separated in a list
                        like [0xFF, 0x01,0x00]
        :type message: list

        :return: data frame transformed into ASCII chain
        :rtype: str
        """
        # Compute CRC LSB and MSB
        crc = 0xFFFF
        for element in message:
            crc = crc ^ element
            iteration = 0
            while iteration <= 7:
                report = crc & 0b1
                crc >>= 1
                if report:
                    crc ^= 0xA001
                iteration += 1

        crc_msb = int(crc % 256)
        crc_lsb = int(crc / 256)
        # complete the frame with CRC
        message.append(crc_msb)
        message.append(crc_lsb)

        # encode the frame into a ASCII character
        encoded_data_frame = ""
        for element in message:
            encoded_data_frame += chr(element)

        return encoded_data_frame

    def __decode(self, message):
        """
        decode the controller reply

        :param message: reply from controller
        :type message: depending of controller message type

        :rtype: list
        :return: list containing the controller reply
        """
        # convert reply in integer
        decoded_message = []
        for char in message:
            decoded_message.append(ord(char))

        # Check the CRC of returned response
        max_len = len(decoded_message) - 2
        if len(decoded_message) > 2:
            crc = 0xFFFF
            i = 0
            while i < max_len:
                crc = crc ^ decoded_message[i]
                iteration = 0
                while iteration <= 7:
                    report = crc & 0b1
                    crc >>= 1
                    if report:
                        crc ^= 0xA001
                    iteration += 1
                i += 1

            crc_msb = int(crc % 256)
            crc_lsb = int(crc / 256)

            # compare the frame CRC and the computed one
            if decoded_message[-1] != crc_lsb or \
                    decoded_message[-2] != crc_msb:
                msg = "bad CRC found in returned response"
                self.get_logger().error(msg)
                raise TestEquipmentException(TestEquipmentException.CRC_ERROR, msg)

        return decoded_message

    def init(self):
        """
        Initializes the equipment. Final state: the equipment is ready to use.
        """
        self.get_logger().info("Initialization")

        if self.__get_handle() is not None:
            return

        # Get transport mode and try to connect to equipment
        transport = str(self.__bench_params.get_param_value("Transport"))

        # Check if transport is supported
        transport_catalog = self.get_eqt_dict()[self.get_model()]["Transports"]
        if transport not in transport_catalog:
            msg = "Unsupported transport %s" % (str(transport))
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.TRANSPORT_ERROR, msg)

        # Check if selected transport is enabled
        if transport_catalog[transport] == "enable":
            # Trying to connect to the equipment
            self.__connect_via_transport(transport)
        else:
            msg = "%s transport is disabled" % (str(transport))
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.TRANSPORT_ERROR, msg)

    def release(self):
        """
        Releases the connection with the equipment and all resources allocated
        to its use.
        """
        self.get_logger().info("Release")
        if self.__get_handle() is not None:
            self.__get_handle().close()
            # Update handle value
            self.__set_handle(None)

    def send(self, message):
        """
        send message to the controller.

        :param message: message to send to the controller,
                        each element of the message should be separated in a list
                        like [0xFF,0x01,0x00]
        :type message: list

        :rtype: list
        :return: reply from controller
        """
        result = []
        if self.is_connected():
            data_frame = self.__encode(message)
            self.__get_handle().write(data_frame)
            response = ""
            next_response = self.__get_handle().readline()
            start_time = time.time()

            while next_response != "":
                response += next_response
                next_response = self.__get_handle().readline()
                if start_time - time.time() > self._reply_timeout:
                    msg = "%s timeout exceed for reply" % self._reply_timeout
                    self.get_logger().error(msg)
                    raise TestEquipmentException(TestEquipmentException.CONNECTION_ERROR, msg)
            result = self.__decode(response)

        return result

    def receive(self):
        """
        Listen to the controller output.

        :rtype: list
        :return: list containing the controller reply
        """
        result = []
        if self.is_connected():
            response = self.__get_handle().readline()
            result = self.__decode(response)

        return result

    def is_connected(self):
        """
        check connection with the controller.

        :rtype: boolean
        :return: true if connection is established, false otherwise
        """
        return self.__get_handle() is not None
