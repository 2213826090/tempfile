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
:summary: COM port XML RPC server.
:since: 14/10/2011
:author: asebbane
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException
from SimpleXMLRPCServer import SimpleXMLRPCRequestHandler, SimpleXMLRPCServer
from serial.serialutil import SerialException
from threading import Thread, Lock
from weakref import proxy
import serial
import time


class USBRlyRpcServer(SimpleXMLRPCServer):

    """
    An XML RPC server that handle request address to COM port
    for C{USBRly08}.
    """

    __CMDS = {
        "get_software_version": 90,
        "get_relay_states": 91,
        "set_relay_states": 92,
        "set_all_relay_on": 100,
        "set_all_relay_off": 110
    }

    def __init__(self, server_ip, server_port, com_port, wiring_table, logger):
        """
        Initializes this instance.

        :type server_ip: str
        :param server_ip: the IP address to be used by this instance

        :type server_port: int
        :param server_port: this server instance port number

        :type com_port: int
        :param com_port: the number of the COM port that this server
            will use to write and write commands.

        :type wiring_table: int
        :param wiring_table: the wiring table
        """
        # Call superclass initialization
        SimpleXMLRPCServer.__init__(
            self,
            (server_ip, server_port),
            requestHandler=USBRlyRpcServer._RequestHandler,
            logRequests=False)

        # The logger instance to use
        self.__logger = logger

        # The wiring table
        self.__wiring_table = wiring_table

        # Register an instance offering the required methods
        self.register_instance(USBRlyRpcServer.__ServerMethods(self))

        # The com port to use
        self.__com_port = com_port

        # The serial.Serial instance
        self.__serial = None

        # A boolean indicating whether this server is currently
        # responding to XML RPC request
        self.__is_running = False

        # Lock object to protect from concurent serial com post access
        self.__serial_lock = Lock()

        # parameters table
        self.__params = {}

    def get_logger(self):
        """
        Returns the logger to use.
        """
        return self.__logger

    def start(self):
        """
        Starts this server instance.
        """
        self.__is_running = True
        while self.__is_running:
            self.handle_request()

    def stop(self):
        """
        Stops this server instance.
        """
        self.__is_running = False
        self.server_close()
        if self.__serial is not None:
            self.__serial.close()
            self.__serial = None

    def set_param(self, name, value):
        self.__params[name] = value
        return True

    def get_param(self, name, default=None):
        return self.__params.get(name, default)

    def get_serial(self):
        """
        Returns the serial port instance to use.

        :rtype: serial.Serial
        :return: the serial port
        """
        if not self.__serial:
            try_index = 0
            retry_nb = 3

            # Access to serial port is not reliable
            # Try several times
            while try_index < retry_nb:
                try:
                    self.__serial = serial.Serial(
                        str(self.__com_port),
                        19200,
                        serial.EIGHTBITS,
                        serial.PARITY_NONE,
                        serial.STOPBITS_TWO,
                        None)
                    break
                except SerialException as error:
                    if try_index < retry_nb:
                        self.get_logger().error(str(error))
                        self.get_logger().info("Retry the connection...")
                        try_index += 1
                        time.sleep(1)
                    else:
                        raise error

        return self.__serial

    def dispose_serial(self):
        """
        Disposes of this object's current I{serial} instance if any.
        :rtype: None
        """
        if str.lower(self.get_param('DisposeDisabled', 'false')) == 'true':
            # dispose disabled, just return
            return
        if self.__serial:
            self.__serial.close()
            self.__serial = None

    def get_relay_states(self):
        """
        Sends a single byte back to the controller, bit high meaning
        the corresponding relay is powered.
        :rtype: integer
        :return: an integer representing the states of the relays
        An integer from 0 to 255
        """
        self.get_logger().info("Get relay states")
        cmd = chr(USBRlyRpcServer.__CMDS["get_relay_states"])
        # get serial before read/write to avoid a connection closed
        serial_port = self.get_serial()
        self.write(cmd, serial_port)
        relay_states = self.read(1, serial_port)
        if relay_states is None:
            msg = "Failed to read relay states"
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.READ_PARAMETER_ERROR, msg)
        elif len(relay_states) == 0:
            msg = "Failed to read relay states"
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.READ_PARAMETER_ERROR, msg)
        else:
            self.get_logger().info("Relays states: %s", str(ord(relay_states)))
        return ord(relay_states)

    def set_relay_states(self, states):
        """
        Sends a single byte that represents the desired relay states.
        All on: 255, all off: 0.
        :type states: integer
        :param states: the desired states far all relays. An integer
        from 0 to 255.
        """
        if states not in range(0, 256):
            msg = "Invalid relay states (%s)" % (str(states))
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)
        cmd = chr(USBRlyRpcServer.__CMDS["set_relay_states"]) + \
            chr(states ^ self.__wiring_table)
        self.write(cmd)

    def set_all_relay_on(self):
        """
        Sets all relay ON.
        """
        self.get_logger().info("Set all relay to ON")
        self.set_relay_states(255)

    def set_all_relay_off(self):
        """
        Sets all relay OFF.
        """
        self.get_logger().info("Set all relay to OFF")
        self.set_relay_states(0)

    def set_relay_on(self, relay_id, serial_port=None):
        """
        Enables relay of relay_id.
        :type relay_id: integer
        :param relay_id: the number of relay
        """
        relay_ids = str(relay_id)
        cmd = list()
        for relay in relay_ids:
            if int(relay) not in range(1, 9):
                error_msg = "Relay number %s is not supported" % relay
                self.get_logger().error(error_msg)
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, error_msg)
            if (pow(2, int(relay) - 1) & self.__wiring_table) != 0:
                # Relay is normally closed
                self.get_logger().debug("Set relay %s to OFF", str(relay_id))
                cmd.append(chr(USBRlyRpcServer.__CMDS["set_all_relay_off"] + int(relay)))
            else:  # Relay is normally opened
                self.get_logger().debug("Set relay %s to ON", str(relay_id))
                cmd.append(chr(USBRlyRpcServer.__CMDS["set_all_relay_on"] + int(relay)))
        self.write(cmd, serial_port)

    def set_relay_off(self, relay_id, serial_port=None):
        """
        Disables relay of relay_id.
        :type relay_id: integer
        :param relay_id: the number of relay
        """
        relay_ids = str(relay_id)
        cmd = list()
        for relay in relay_ids:
            if int(relay) not in range(1, 9):
                error_msg = "Relay number %s is not supported" % relay
                self.get_logger().error(error_msg)
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, error_msg)
            if (pow(2, int(relay) - 1) & self.__wiring_table) != 0:
                # Relay is normally closed
                self.get_logger().debug("Set relay %s to ON", str(relay_id))
                cmd.append(chr(USBRlyRpcServer.__CMDS["set_all_relay_on"] + int(relay)))
            else:  # Relay is normally opened
                self.get_logger().debug("Set relay %s to OFF", str(relay_id))
                cmd.append(chr(USBRlyRpcServer.__CMDS["set_all_relay_off"] + int(relay)))
        self.write(cmd, serial_port)

    def press_relay(self, relay_id, delay):
        """
        Press (enable/disable) relay of relay_id
        during a delay.
        :type relay_id: integer
        :param relay_id: the number of relay
        :type delay: integer
        :param delay: delay
        """
        self.get_logger().info("Press button %d during %3.2f second(s)" % (relay_id, delay))

        serial_port = self.get_serial()

        self.set_relay_on(relay_id, serial_port)

        while delay > 0:
            t0 = time.time()
            time.sleep(0.2)
            t1 = time.time()
            delay -= (t1 - t0)

        self.set_relay_off(relay_id, serial_port)

        self.dispose_serial()

    def write(self, cmd, serial_port=None):
        """
        Writes the given command to the configured COM port.
        :type cmd: str
        :param cmd: the command to write
        """
        result = ""
        do_dispose = False
        with self.__serial_lock:
            if serial_port is None:
                serial_port = self.get_serial()
                do_dispose = True

            if serial_port is not None:
                if type(cmd) == list:
                    for single in cmd:
                        result += str(serial_port.write(single))
                else:
                    result = serial_port.write(cmd)
            else:
                raise TestEquipmentException(TestEquipmentException.CONNECTIVITY_ERROR,
                                    "Serial Port is not initialized, check USBRly08 connection")

            if do_dispose:
                self.dispose_serial()
        return result

    def read(self, byte_count, serial_port=None):
        """
        Read the given number of bytes from the configured COM port.
        :type byte_count: int
        :param byte_count: the number of bytes to read

        :rtype: str
        :return: a one character-long str
        """
        result = ""
        do_dispose = False
        with self.__serial_lock:
            if serial_port is None:
                serial_port = self.get_serial()
                do_dispose = True

            if serial_port is not None:
                result = serial_port.read(byte_count)
            else:
                raise TestEquipmentException(TestEquipmentException.CONNECTIVITY_ERROR,
                                    "Serial Port is not initialized, check USBRly08 connection")

            if do_dispose:
                self.dispose_serial()

        return result

    # Restrict to a particular path.
    class _RequestHandler(SimpleXMLRPCRequestHandler):

        """
        Inner class used to restrict XML RPC request
        to allowed path only.
        """
        rpc_paths = ('/RPC2',)

    class __ServerMethods(object):

        """
        This class implements the server methods.
        Used to avoid recursion when registering an instance.
        """

        def __init__(self, calling_server):
            """
            Initializes this instance.

            :type calling_server: USBRlyRpc
            :param calling_server: the XML RPC instance that
                will use this object's methods
            """
            self.__calling_server = proxy(calling_server)

        def format_string(self, a_string):
            """
            Returns a copy of the given str stripped of all
            undesired characters.

            :type a_string: str
            :param a_string: the str to format

            :rtype: str
            :return: the formatted str
            """
            formatted_string = a_string.decode("ascii", "ignore")
            return formatted_string

        def ping(self):
            """
            A simple method used to ping this instance.

            :rtype: str
            :return: a fixed str "output:SUCCESS"
            """
            return "output:SUCCESS"

        def set_param(self, name, value):
            """
            Set optional parameter to serial
            :rtype: str
            :param: name: parameter name
            :rtype: str
            :param: value: parameter value

            :rtype: bool
            :return: True if success
            """
            result = self.__calling_server.set_param(name, value)
            return result

        def write(self, cmd):
            """
            Writes the given command to the configured COM port.
            :type cmd: str
            :param cmd: the command to write
            """
            if self.__calling_server is not None:
                result = self.__calling_server.write(cmd)
            else:
                raise TestEquipmentException(TestEquipmentException.PROPERTY_ERROR, "COM Port not configured")
            return result

        def read(self, byte_count):
            """
            Read the given number of bytes from the configured COM port.
            :type byte_count: int
            :param byte_count: the number of bytes to read

            :rtype: str
            :return: a one character-long str
            """
            result = self.__calling_server.read(byte_count)
            return result

        def stop(self):
            """
            Stops the calling server.
            """
            msg = "output:SUCCESS"
            try:
                self.__calling_server.stop()
            except Exception as exc:  # pylint: disable=W0703
                exc_as_string = self.format_string(str(exc))
                msg = "output:FAILURE (%s)" % exc_as_string
            return msg

        def set_all_relay_off(self):
            """
            Sets all relay OFF.
            """
            msg = "output:SUCCESS"
            try:
                self.__calling_server.set_all_relay_off()
            except Exception as exc:  # pylint: disable=W0703
                exc_as_string = self.format_string(str(exc))
                msg = "output:FAILURE (%s)" % exc_as_string
            return msg

        def set_all_relay_on(self):
            """
            Sets all relay ON.
            """
            msg = "output:SUCCESS"
            try:
                self.__calling_server.set_all_relay_on()
            except Exception as exc:  # pylint: disable=W0703
                exc_as_string = self.format_string(str(exc))
                msg = "output:FAILURE (%s)" % exc_as_string
            return msg

        def set_relay_on(self, relay_id):
            """
            Enables relay of relay_id.
            :type relay_id: integer
            :param relay_id: the number of relay
            """
            msg = "output:SUCCESS"
            try:
                self.__calling_server.set_relay_on(relay_id)
            except Exception as exc:  # pylint: disable=W0703
                exc_as_string = self.format_string(str(exc))
                msg = "output:FAILURE (%s)" % exc_as_string
            return msg

        def set_relay_off(self, relay_id):
            """
            Disables relay of relay_id.
            :type relay_id: integer
            :param relay_id: the number of relay
            """
            msg = "output:SUCCESS"
            try:
                self.__calling_server.set_relay_off(relay_id)
            except Exception as exc:  # pylint: disable=W0703
                exc_as_string = self.format_string(str(exc))
                msg = "output:FAILURE (%s)" % exc_as_string
            return msg

        def press_relay(self, relay_id, delay):
            """
            Press (enable/disable) relay of relay_id
            during a delay.
            :type relay_id: integer
            :param relay_id: the number of relay
            :type delay: integer
            :param delay: delay
            """
            msg = "output:SUCCESS"
            try:
                self.__calling_server.press_relay(relay_id, delay)
            except Exception as exc:  # pylint: disable=W0703
                exc_as_string = self.format_string(str(exc))
                msg = "output:FAILURE (%s)" % exc_as_string
            return msg

        def set_relay_states(self, states):
            """
            Sends a single byte that represents the desired relay states.
            All on: 255, all off: 0.
            :type states: integer
            :param states: the desired states far all relays. An integer
            from 0 to 255.
            """
            msg = "output:SUCCESS"
            try:
                self.__calling_server.set_relay_states(states)
            except Exception as exc:  # pylint: disable=W0703
                exc_as_string = self.format_string(str(exc))
                msg = "output:FAILURE (%s)" % exc_as_string
            return msg


class USBRlyRpcThread(Thread):

    """
    This class is intended to run a RPC USB relay server.
    Once this thread has been started and then stopped it
    cannot be restarted. A new instance has to be created
    in order to obtain a working XML RPC server.
    """

    def __init__(self, server=None):
        """
        Initializes this instance.

        :type server: USBRlyRpc
        :param server: the XML RPC server handling USB relay requests.
        """
        Thread.__init__(self)
        self.daemon = True
        self.__server = server

    def set_rpc_server(self, server):
        """
        Sets the XML RPC server to use to the given instance.

        :type server: USBRlyRpc
        :param server: the XML RPC server handling USB relay requests.
        """
        self.__server = server

    def run(self):
        """
        Starts this thread's activity.
        """
        self.__server.start()

    def stop(self):
        """
        Stops this thread.
        """
        self.__server.stop()
