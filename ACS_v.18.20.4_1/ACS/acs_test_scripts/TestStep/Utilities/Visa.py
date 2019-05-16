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
:since: 12/09/2014
:author: jduran4x
"""
from acs_test_scripts.Equipment.IEquipment import EquipmentBase
from Lib.PyVisa import visa as pyvisa
from Lib.PyVisa import visa_exceptions as visaexception
from Core.Report.ACSLogging import ACS_LOGGER_NAME, EQT_LOGGER_NAME
from ErrorHandling.AcsBaseException import AcsBaseException
from ErrorHandling.TestEquipmentException import TestEquipmentException
from Core.Factory import Factory
import os
from lxml import etree


class VisaEquipmentBase(EquipmentBase):
    """
    Equipment Base implementation with a visa instrument declared
    it also implies
    - database file usage (database path defined in DB_FILE)
    - quick configurations given in an XML file ( CONFIG_FILE_PATH)
    """
    CONFIG_FILE_PATH = None
    DB_FILE = None

    def __init__(self, name, model, eqt_params, bench_params):
        """
        Constructor
        :type name: str
        :param name: the bench configuration name of the equipment
        :type model: str
        :param model: the model of the equipment
        :type eqt_params: dict
        :param eqt_params: the dictionary containing equipment parameters
        :type bench_params: BenchConfigParameters
        :param bench_params: the dictionary containing equipment bench parameters
        """
        EquipmentBase.__init__(self, name, model, eqt_params)

        self._bench_params = bench_params
        self._visa = None
        # configuration file:
        # keep only the root node of the xml tree
        self._root_node = None
        self._loaded_conf = []

        transport = self._bench_params.get_param_value("Transport", default_value="GPIB")
        if transport == VisaInterface.GPIB_TRANSPORT:
            kwargs = {"GPIBBoardId": self._bench_params.get_param_value("GPIBBoardId"),
                      "GPIBAddress": self._bench_params.get_param_value("GPIBAddress")}
        elif transport == VisaInterface.TCPIP_TRANSPORT:
            kwargs = {"TcpIpAddress": self._bench_params.get_param_value("TcpIpAddress"),
                      "TcpIpBoardId": self._bench_params.get_param_value("TcpIpBoardId", ""),
                      "TcpIpDeviceName": self._bench_params.get_param_value("TcpIpDeviceName", "INST0")
                      }
        elif transport == VisaInterface.SERIAL_TRANSPORT:
            kwargs = {"ComPort": self._bench_params.get_param_value("ComPort")}

        self.create_visa_interface(transport, kwargs)

    def reset(self):
        """
        reset equipment object when a reset is physically done on equipment
        """
        self._loaded_conf = []

    def load_configuration_file(self, filename):
        """
        Loads equipment parameters from a file.
        :type filename: str
        :param filename: the configuration file to load with his path
        """
        # reset previous config
        self._root_node = None
        if not filename.endswith(".xml"):
            raise TestEquipmentException(TestEquipmentException.PROHIBITIVE_BEHAVIOR,
                                         "Network config file must be a xml file")
        return self.set_configuration_file(filename)

    def get_interface(self):
        return self._visa

    def create_visa_interface(self, transport, kwargs):
        """
        Create visa interface
        :type transport: str
        :param transport: the bench configuration name of the equipment
        :type kwargs: dict
        :param kwargs: various needed parameters depending on the transport
        """
        self._visa = VisaInterface(transport, **kwargs)

    def set_configuration_file(self, name):
        if os.path.exists(name):
            self._root_node = etree.parse(name)
        else:
            self.get_logger().warning("configuration file %s not found" % name)

    def load_configuration(self, config, cell_number=None):
        """
        This method executes the GPIB commands set identified by 'config'
        in configuration file
        :type config: str
        :param config: the name of the configuration to load

        :type cell_number: int
        :param cell_number: the number of the cell to configure

        :rtype list
        :returns for each matching config, the list attributes as dictionaries
        """
        # ensure the config file object exists
        if self._root_node is None:
            self.set_configuration_file(self.CONFIG_FILE_PATH)

        # get all matching configurations
        elems = self._root_node.findall(".//Config[@name=\"%s\"]" % config)
        if elems is None or elems == []:
            raise Exception("Configuration not found")
        elem = None
        apply_list = None
        # retrieve the apply_to configurations
        # if desired configuration appears several times in config file
        # use the apply_to attribute to determine the relevant one to apply
        for elem in elems:
            if "apply_to" in elem.attrib.keys():
                apply_list = elem.attrib["apply_to"].split(";")
            else:
                apply_list = []
            # intersect is not empty in 2 cases
            # # if elems has just one element, the needed conf is already loaded
            # # if elems has several elements, apply_to is used for filtering only
            if len(elems) > 1:
                intersect = set(apply_list).intersection(set(self._loaded_conf))
                if len(intersect) > 0:
                    del apply_list
                    apply_list = []
                    break
                else:
                    # cannot choose the good configuration to load
                    raise Exception("Missing loaded configuration prior to %s" % config)

        # apply_to attribute can be used to reduce the XML file length
        # so if 1 conf in apply_to is not loaded, load it prior to the current
        # (take the same order as the list)
        for ap in apply_list:
            if ap not in self._loaded_conf:
                self.load_configuration(ap)
        # configuration element found. Now extract all the GPIB commands and apply them
        if config not in self._loaded_conf:
            self.get_logger().info("Load configuration '%s'" % config)
            commands = elem.xpath("./GPIB/@command")
            for gpib in commands:
                self._visa.send_command(gpib, cell_number)
        # append loaded config
        self._loaded_conf.append(config)

    def send_command(self, command):
        """
        Send a GPIB Command.

        :type command: str
        :param command: String representing a well formed GPIB command.
        """
        # Send command to the equipment
        self._visa.send_command(command)


class VisaObject(object):
    """
    Cell object having PyVisa interface and logger attributes
    """

    def __init__(self, visa):
        """
        Constructor
        :type visa: VisaInterface
        :param visa: the PyVisa connection
        """
        self._logger = Factory().create_logger("%s.%s.%s" % (ACS_LOGGER_NAME, EQT_LOGGER_NAME, self.__class__.__name__))
        self._visa = visa

    def __del__(self):
        """
        Destructor
        """
        del self._logger
        del self._visa

    def get_logger(self):
        """
        Gets the logger of the equipment, i.e root's one
        """
        return self._logger


class VisaInterface(object):

    """
    Visa interface to control equipment in full python
    supported transports are "GPIB", "SERIAL", "TCPIP"
    each transport needs specific additional arguments
    "GPIB": "GPIBBoardId", "GPIBAddress"
    "SERIAL": "ComPort"
    "TCPIP": "TcpIpAddress"
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

    def __init__(self, transport, **kwargs):
        """
        Constructor
        :type transport: str
        :param transport: the bench configuration name of the equipment
        :type kwargs: dict
        :param kwargs: various needed parameters depending on the transport
        """

        # Initialize attributes
        self.__logger = Factory().create_logger("%s.%s.VISA" % (ACS_LOGGER_NAME, EQT_LOGGER_NAME))
        self.__args = dict(kwargs)
        self.connect_via_transport(transport)

    def get_logger(self):
        """
        Gets the internal logger of the equipment
        """
        return self.__logger

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

    def __connect_via_gpib(self):
        """
        Connect to equipment via GPIB
        """
        if "GPIBBoardId" in self.__args.keys():
            board_id = int(self.__args["GPIBBoardId"])
        else:
            raise AcsBaseException(AcsBaseException.INVALID_PARAMETER,
                                   "The parameter 'GPIBBoardId' is required for configuring GPIB connection")
        if "GPIBAddress" in self.__args.keys():
            gpib_addr = int(self.__args["GPIBAddress"])
        else:
            raise AcsBaseException(AcsBaseException.INVALID_PARAMETER,
                                   "The parameter 'GPIBAddress' is required for configuring GPIB connection")
        try:
            # Initialize GPIB connection to the equipment
            handle = pyvisa.GpibInstrument(gpib_addr, board_id)
            # Update handle value
            self._set_handle(handle)
        except (visaexception.VisaIOError,
                visaexception.VisaIOWarning,
                visaexception.VisaTypeError) as ex:
            msg = "Connection via GPIB failed. "
            msg += str(ex)
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.VISA_LIBRARY_ERROR, msg)

    def __connect_via_serial(self):
        """
        Connect to equipment via SERIAL
        """
        if "ComPort" in self.__args.keys():
            com_port = str(self.__args["ComPort"])
        else:
            raise AcsBaseException(AcsBaseException.INVALID_PARAMETER,
                                   "The parameter 'ComPort' is required for configuring serial connection")

        try:
            # Initialize Serial connection to the equipment
            handle = pyvisa.SerialInstrument(com_port)
            # Update handle value
            self._set_handle(handle)
        except (visaexception.VisaIOError,
                visaexception.VisaIOWarning,
                visaexception.VisaTypeError) as ex:
            msg = "Connection via Serial failed. "
            msg += str(ex)
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.VISA_LIBRARY_ERROR, msg)

    def __connect_via_tcpip(self):
        """
        Connect to equipment via TCPIP
        """
        if "TcpIpAddress" in self.__args.keys():
            ip_addr = str(self.__args["TcpIpAddress"])
        else:
            raise AcsBaseException(AcsBaseException.INVALID_PARAMETER,
                                   "The parameter 'TcpIpAddress' is required for configuring TCP/IP connection")
        board_id = ""
        if "TcpIpBoardId" in self.__args.keys():
            board_id = str(self.__args["TcpIpBoardId"])
        device_name = "INST0"
        if "TcpIpDeviceName" in self.__args.keys():
            device_name = str(self.__args["TcpIpDeviceName"])

        try:
            # Initialize TCP/IP connection to the equipment
            resource_name = "TCPIP%s::%s::%s::INSTR" % (board_id, ip_addr, device_name)
            self.get_logger().debug("TCPIP ressource name: %s" % resource_name)
            handle = pyvisa.Instrument(resource_name)
            handle.timeout = 10000

            # Update handle value
            self._set_handle(handle)
        except (visaexception.VisaIOError,
                visaexception.VisaIOWarning,
                visaexception.VisaTypeError) as ex:
            msg = "Connection via TCP/IP failed. "
            msg += str(ex)
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.VISA_LIBRARY_ERROR, msg)

    def connect_via_transport(self, transport):
        """
        Connect to equipment using parameter transport

        :type transport: str
        :param transport: The transport use to connect to the equipment
        """

        if transport == self.GPIB_TRANSPORT:
            self.__connect_via_gpib()
        elif transport == self.SERIAL_TRANSPORT:
            self.__connect_via_serial()
        elif transport == self.TCPIP_TRANSPORT:
            self.__connect_via_tcpip()
        else:
            error_msg = "Unsupported transport mode '%s'." % str(transport)
            self.get_logger().error(error_msg)
            raise TestEquipmentException(TestEquipmentException.READ_PARAMETER_ERROR, error_msg)

    def disconnect(self):
        """
        Disconnect from the equipment and all associated resources
        """

        try:
            self.get_logger().info("Disconnection")

            if self.get_handle() is not None:
                self.get_handle().clear()
                self.get_handle().close()
                # Update handle value
                self._set_handle(None)

        except (visaexception.VisaIOError,
                visaexception.VisaIOWarning,
                visaexception.VisaTypeError) as ex:
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

            self.__logger.debug("sending write command '%s'" % command)
            # Send write command on equipment
            self.get_handle().write(command)

        except (visaexception.VisaIOError,
                visaexception.VisaIOWarning,
                visaexception.VisaTypeError) as ex:
            msg = "Write command failed. "
            msg += str(ex)
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.VISA_LIBRARY_ERROR, msg)

    def query(self, command, log=True):
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
            if log:
                self.__logger.debug("sending query command '%s'" % command)
            # Send query command to the equipment
            result = self.get_handle().ask(command)
            if log:
                self.__logger.debug("got result '%s'" % result)
            return result

        except (visaexception.VisaIOError,
                visaexception.VisaIOWarning,
                visaexception.VisaTypeError) as ex:
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

        except (visaexception.VisaIOError,
                visaexception.VisaIOWarning,
                visaexception.VisaTypeError) as ex:
            msg = "Read command failed. "
            msg += str(ex)
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.VISA_LIBRARY_ERROR, msg)

    def send_command(self, command, bts=None):
        """
        Send a GPIB Command.

        :type command: str
        :param command: String representing a well formed GPIB command.

        :type bts: str
        :param bts: the BTS to apply the command
        """
        # Clear Status command. Clears the whole status structure
        self.write(command)
