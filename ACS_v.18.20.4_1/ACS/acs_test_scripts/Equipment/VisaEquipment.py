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
:summary: Equipment interface implementation to call Pyvisa library
:since: 13/01/2012
:author: ssavrimoutou
"""

from Lib.PyVisa import visa as Visa
from Lib.PyVisa import visa_exceptions as VisaException
from acs_test_scripts.Equipment.IEquipment import EquipmentBase
from ErrorHandling.TestEquipmentException import TestEquipmentException


class VisaEquipment(EquipmentBase):

    """
    Visa interface to control equipment in full python
    """

    SUPPORTED_TRANSPORT = ["GPIB", "SERIAL", "TCPIP"]
    """
    List of supported transports to connect to the equipment
    """

    GPIB_TRANSPORT = "GPIB"
    SERIAL_TRANSPORT = "SERIAL"
    TCPIP_TRANSPORT = "TCPIP"
    """
    Global variable used for transport connection
    """

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
        # Initialize attributes
        self.__bench_params = bench_params
        self.__handle = None

    def get_handle(self):
        """
        Gets the connection handle
        :rtype: unsigned long
        :return: the handle of connection with the equipment, None if no
        equipment is connected
        """
        return self.__handle

    def check_handle(self):
        """
        Check if the handle for equipment is created

        :return: None
        """
        # Check if equipment is connected first
        if self.get_handle() is None:
            error_msg = "Connection is not initialized with the equipment."
            self.get_logger().error(error_msg)
            raise TestEquipmentException(TestEquipmentException.INSTANTIATION_ERROR, error_msg)

    def _set_handle(self, handle):
        """
        Sets the connection handle of the equipment
        """
        self.__handle = handle

    def __connect_via_GPIB(self):
        """
        Connect to equipment via GPIB
        """
        board_id = int(self.__bench_params.get_param_value("GPIBBoardId"))
        gpib_addr = int(self.__bench_params.get_param_value("GPIBAddress"))

        try:
            # Initialize Gpib connection to the equipment
            handle = Visa.GpibInstrument(gpib_addr, board_id)
            # Update handle value
            self._set_handle(handle)
        except (VisaException.VisaIOError,
                VisaException.VisaIOWarning,
                VisaException.VisaTypeError) as ex:
            msg = "Connection via Gpib failed. "
            msg += str(ex)
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.VISA_LIBRARY_ERROR, msg)

    def __connect_via_SERIAL(self):
        """
        Connect to equipment via SERIAL
        """
        com_port = str(self.__bench_params.get_param_value("ComPort"))

        try:
            # Initialize Serial connection to the equipment
            handle = Visa.SerialInstrument(com_port)
            # Update handle value
            self._set_handle(handle)
        except (VisaException.VisaIOError,
                VisaException.VisaIOWarning,
                VisaException.VisaTypeError) as ex:
            msg = "Connection via Serial failed. "
            msg += str(ex)
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.VISA_LIBRARY_ERROR, msg)

    def __connect_via_TCPIP(self):
        """
        Connect to equipment via TCPIP
        """
        ip_addr = str(self.__bench_params.get_param_value("TcpIpAddress"))
        board_id = str(self._bench_params.get_param_value("TcpIpBoardId", ""))
        device_name = str(self._bench_params.get_param_value("TcpIpDeviceName", "INST0"))

        try:
            # Initialize TCP/IP connection to the equipment
            resource_name = "TCPIP%s::%s::%s::INSTR" % (board_id, ip_addr, device_name)
            handle = Visa.Instrument(resource_name)
            # Update handle value
            self._set_handle(handle)
        except (VisaException.VisaIOError,
                VisaException.VisaIOWarning,
                VisaException.VisaTypeError) as ex:
            msg = "Connection via TCP/IP failed. "
            msg += str(ex)
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.VISA_LIBRARY_ERROR, msg)

    def connect_via_transport(self, transport):
        """
        Connect to equipment using parameter transport

        :type transport: String
        :param transport: The transport use to connect to the equipment
        """

        if transport == VisaEquipment.GPIB_TRANSPORT:
            self.__connect_via_GPIB()
        elif transport == VisaEquipment.SERIAL_TRANSPORT:
            self.__connect_via_SERIAL()
        elif transport == VisaEquipment.TCPIP_TRANSPORT:
            self.__connect_via_TCPIP()
        else:
            error_msg = "Unsupported transport mode '%s'." % str(transport)
            self.get_logger().error(error_msg)
            raise TestEquipmentException(TestEquipmentException.READ_PARAMETER_ERROR, error_msg)

    def connect(self):
        """
        Initialize connection to the equipment
        """
        self.get_logger().debug("Connection")

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

    def disconnect(self):
        """
        Disconnect from the equipment and all associated resources
        """

        try:
            self.get_logger().debug("Disconnection")

            if self.get_handle() is not None:
                self.get_handle().clear()
                self.get_handle().close()
                # Update handle value
                self._set_handle(None)

        except (VisaException.VisaIOError,
                VisaException.VisaIOWarning,
                VisaException.VisaTypeError) as ex:
            msg = "Fail to disconnect from the equipment. "
            msg += str(ex)
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.VISA_LIBRARY_ERROR, msg)

    def write(self, command):
        """
        Send message to the equipment

        :type command: str
        :param command: write command to send
        """

        try:
            # Check handle of the equipment
            self.check_handle()

            # Send write command on equipment
            self.get_handle().write(command)

        except (VisaException.VisaIOError,
                VisaException.VisaIOWarning,
                VisaException.VisaTypeError) as ex:
            msg = "Write command failed. "
            msg += str(ex)
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.VISA_LIBRARY_ERROR, msg)

    def query(self, command):
        """
        Send query message to the equipment

        :type command: str
        :param command: query command to send

        :rtype: str
        :return: the message returned by the equipment
        """

        try:
            # Check handle of the equipment
            self.check_handle()

            # Send query command to the equipment
            return self.get_handle().ask(command)

        except (VisaException.VisaIOError,
                VisaException.VisaIOWarning,
                VisaException.VisaTypeError) as ex:
            msg = "Query command failed. "
            msg += str(ex)
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.VISA_LIBRARY_ERROR, msg)

    def read(self):
        """
        Read str message from the equipment

        :rtype: str
        :return: read command

        :raise: TestEquipmentException

        """

        try:
            # Check handle of the equipment
            self.check_handle()

            # Read message from equipment
            return self.get_handle().read()

        except (VisaException.VisaIOError,
                VisaException.VisaIOWarning,
                VisaException.VisaTypeError) as ex:
            msg = "Read command failed. "
            msg += str(ex)
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.VISA_LIBRARY_ERROR, msg)
