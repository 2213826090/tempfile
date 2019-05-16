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
:summary: implementation of Spirent GSS6700 gps network simulator
:since:18/01/2012
:author: ssavrimoutou
"""

import time
import serial
from serial.serialutil import SerialException
from xml.dom.minidom import parseString

from acs_test_scripts.Equipment.IEquipment import EquipmentBase
from acs_test_scripts.Equipment.NetworkSimulators.GPS.Interface.IGPSNetSim import IGPSNetSim

from ErrorHandling.TestEquipmentException import TestEquipmentException
from UtilitiesFWK.Utilities import Global


class SpirentGSS6700(EquipmentBase, IGPSNetSim):

    """
    Implementation of Spirent GSS6700 equipment
    """

    SERIAL_TRANSPORT = "SERIAL"
    TCPIP_TRANSPORT = "TCPIP"
    """
    Global variable used for transport connection
    """

    class EQUIPMENT_STATE(object):

        """
        Define list of supported status of the equipment
        """
        UNKNOWN = "NOT DEFINED"
        READY = "READY"
        LOADING = "LOADING"
        ARMING = "ARMING"
        ARMED = "ARMED"
        RUNNING = "RUNNING"
        PAUSED = "PAUSED"
        ENDED = "ENDED"
        IDLE = "IDLE"

    def __init__(self, name, model, eqt_params, bench_params):
        """
        Constructor
        :type name: str
        :param name: the bench name of the equipment
        :type model: str
        :param model: the model of the equipment
        :type eqt_params: dict
        :param eqt_params: the dictionary containing equipment catalog parameters
        :type bench_params: dict
        :param bench_params: the dictionary containing bench parameters of the equipment
        """
        IGPSNetSim.__init__(self)
        EquipmentBase.__init__(self, name, model, eqt_params)
        self.__bench_params = bench_params
        self.__handle = None

    def __del__(self):
        """
        Destructor: releases all allocated resources.
        """
        self.release()

    def __connect_via_SERIAL(self):
        """
        Connect to equipment via SERIAL
        """

        # Initialize local variables
        handle = None
        try_index = 0

        # Get Serial parameters for connection
        serial_com_port = str(self.__bench_params.get_param_value("ComPort"))
        serial_baud_rate = int(self.__bench_params.get_param_value("BaudRate"))
        retry_nb = int(self.__bench_params.get_param_value("ConnectionRetry"))

        # Access to serial port is not reliable
        # Try several times
        while try_index < retry_nb:
            try:
                handle = serial.Serial(
                    port=serial_com_port,
                    baudrate=serial_baud_rate,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS
                )

                break
            except SerialException as error:
                if try_index < retry_nb:
                    self.get_logger().error(str(error))
                    self.get_logger().info("Retry the connection...")
                    try_index += 1
                    time.sleep(1)
                else:
                    raise error

        if handle is None:
            raise TestEquipmentException(TestEquipmentException.CONNECTION_ERROR, "Serial port connection failed.")

        # Update handle value
        self._set_handle(handle)

    def connect_via_transport(self, transport):
        """
        Connect to equipment using parameter transport

        :type transport: String
        :param transport: The transport use to connect to the equipment
        """

        if transport == SpirentGSS6700.SERIAL_TRANSPORT:
            self.__connect_via_SERIAL()
        else:
            error_msg = "Unsupported transport mode '%s'." % str(transport)
            self.get_logger().error(error_msg)
            raise TestEquipmentException(TestEquipmentException.READ_PARAMETER_ERROR, error_msg)

    def __write(self, message, delay):
        """
        Write message to the equipment

        :type command: str
        :param command: String representing a well formed command.

        :type delay: float
        :param delay: Delay to wait before reading the response
                      Default is 5 seconds
        """
        self.get_logger().debug("Write data to the equipment : %s" % str(message))

        self.get_handle().flushInput()
        self.get_handle().flushOutput()
        self.get_handle().write(message + '\r\n')

        time.sleep(float(delay))

    def __read(self):
        """
        Write message to the equipment

        :rtype: str
        :return: Data received
        """

        data_received = ""
        # Read data received from the equipment
        while self.get_handle().inWaiting() > 0:
            data_received += self.get_handle().read(1)

        self.get_logger().debug("Read data from equipment : %s" % str(data_received))
        return data_received

    def __parse_data(self, data_received):
        """
        Parse received data from equipment

        :rtype: str
        :return: output status
        """

        if data_received != "":
            try:
                dom = parseString(data_received)
                StatusValue = dom.getElementsByTagName('status')
            except:  # pylint: disable=W0702
                return SpirentGSS6700.EQUIPMENT_STATE.UNKNOWN

            _n_node = StatusValue[0]
            _child = _n_node.firstChild
            _text = _child.data

            if _text.find('2') > 0:
                return SpirentGSS6700.EQUIPMENT_STATE.READY
            elif _text.find('1') > 0:
                return SpirentGSS6700.EQUIPMENT_STATE.LOADING
            elif _text.find('3') > 0:
                return SpirentGSS6700.EQUIPMENT_STATE.ARMING
            elif _text.find('4') > 0:
                return SpirentGSS6700.EQUIPMENT_STATE.ARMED
            elif _text.find('5') > 0:
                return SpirentGSS6700.EQUIPMENT_STATE.RUNNING
            elif _text.find('6') > 0:
                return SpirentGSS6700.EQUIPMENT_STATE.PAUSED
            elif _text.find('7') > 0:
                return SpirentGSS6700.EQUIPMENT_STATE.ENDED
            elif _text.find('0') > 0:
                return SpirentGSS6700.EQUIPMENT_STATE.IDLE
        else:
            return SpirentGSS6700.EQUIPMENT_STATE.UNKNOWN

    def get_handle(self):
        """
        Gets the connection handle
        :rtype: unsigned long
        :return: the handle of connection with the equipment, None if no
        equipment is connected
        """
        return self.__handle

    def _set_handle(self, handle):
        """
        Sets the connection handle
        :type handle: unsigned integer
        :param handle: the new connection handle
        """
        self.__handle = handle

    def init(self):
        """
        Initializes the equipment and establishes the connection.
        """
        self.get_logger().info("Initialization")

        if self.get_handle() is not None:
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
            self.connect_via_transport(transport)
        else:
            msg = "%s transport is disabled" % (str(transport))
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.TRANSPORT_ERROR, msg)

    def release(self):
        """
        Releases equipment resources and close connection.
        """
        self.get_logger().info("Release")

        if self.get_handle() is not None:
            self.get_handle().close()
            # Update handle value
            self._set_handle(None)

    def get_status(self):
        """
        Get the status of the equipment

        :rtype: str
        :return: Status returned by the equipment
        """

        # Send a NULL commmand with 2 seconds of delay
        self.__write("NULL", 2)

        # Read received data
        data_received = self.__read()

        # Parse received data to extract the equipment status
        return_status = self.__parse_data(data_received)
        self.get_logger().debug("Equipment status is %s" % str(return_status))

        return return_status

    def send_command(self, command, check_status=None, max_retry=10, delay=5.0):
        """
        Send the command to the spirent. If a status is awaited and not received, try to send again the command.
        10 times failed command sending is failed.

        :type command: str
        :param command: String representing a well formed command.

        :type check_status: List of str
        :param check_status: String or a list of str representing the status to check once the command is sent

        :type max_retry: integer
        :param max_retry: Maximum retries before returning Failure status.
                          Default is 10

        :type delay: float
        :param delay: Delay to wait before reading the response
                      Default is 5 seconds

        :rtype: int
        :return: return Global.SUCCESS or Global.FAILURE if an error occurred
        """
        self.get_logger().debug("Send command '%s' to the equipment" % str(command))

        nb_try = 0

        while nb_try <= max_retry:
            nb_try += 1

            # Send command with the delay
            self.__write(command, delay)

            # Read received data
            data_received = self.__read()

            # Parse received data to extract the equipment status
            return_status = self.__parse_data(data_received)
            # Check the returned status of the equipment
            if check_status not in (None, ""):
                # Convert status parameter into list if a str
                if not isinstance(check_status, list):
                    check_status = [check_status]
                if return_status in check_status:
                    return Global.SUCCESS
            else:
                return Global.SUCCESS

        # Max retry has been reached return failure
        return Global.FAILURE

    def set_rf_power_level(self, power_level):
        """
        Set the RF power level

        :type power_level: integer
        :param power_level: the value RF power level to set
        """

        self.get_logger().info("Set RF power level")

        # Send the command to set the RF power level
        rf_value = int(power_level) + 130
        rf_command = "-,POW_LEV,v1_a1," + str(rf_value) + ",GPS,1,0,0,1,1,1,1"
        self.send_command(rf_command)

        # Check that equipment is ready
        return_status = self.get_status()

        if return_status != SpirentGSS6700.EQUIPMENT_STATE.READY:
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, "Failed to set RF power level.")

    def load_configuration_file(self, filename):
        """
        Loads equipment configuration/scenario from a filename.

        :type filename: str
        :param filename: the configuration file to load
        """

        self.get_logger().info("Load scenario '%s'" % str(filename))

        # Now Spirent must be stopped, we can load the scenario
        # Release all input and output buffer, send the load scenario command: SC,<path of scenario>s
        self.send_command("SC," + str(filename))

        # Check that equipment is ready
        return_status = self.get_status()

        if return_status != SpirentGSS6700.EQUIPMENT_STATE.READY:
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, "Failed to load scenario.")

    def start_test(self):
        """
        Start test on equipment using loaded file scenario.
        """

        self.get_logger().info("Start scenario")

        # Start scenario
        self.send_command("RU")

        # Check that scenario is running
        return_status = self.get_status()

        if return_status != SpirentGSS6700.EQUIPMENT_STATE.RUNNING:
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, "Failed to start scenario.")

    def stop_test(self):
        """
        Stop test on equipment using loaded file scenario.
        """
        self.get_logger().info("Stop scenario")

        # Stop scenario
        self.send_command("-,EN,1")

        # Check that equipment is idle or ready
        return_status = self.get_status()
        if return_status != SpirentGSS6700.EQUIPMENT_STATE.IDLE and \
                return_status != SpirentGSS6700.EQUIPMENT_STATE.READY:
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, "Failed to stop scenario.")
