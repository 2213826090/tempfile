"""
:copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
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

:organization: INTEL OTC Android
:summary: Configure python RPC Server and commands for the SUBRELAY32 Hardware
          Equipment
:since: 7/17/15
:author: mmaraci
"""
from SimpleXMLRPCServer import SimpleXMLRPCServer, SimpleXMLRPCRequestHandler
from threading import Thread, Lock
import time
from weakref import proxy

import serial
from serial.serialutil import SerialException

from ErrorHandling.TestEquipmentException import TestEquipmentException


class USBRELAY32RpcServer(SimpleXMLRPCServer):
    """
    Configure python RPC Server and commands for the SUBRELAY32 Hardware
          Equipment
    """
    __CMDS = {"ON": "on", "OFF": "off"}

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
            requestHandler=USBRELAY32RpcServer._RequestHandler,
            logRequests=False)

        # The logger instance to use
        self.__logger = logger

        # The wiring table
        self.__wiring_table = wiring_table

        # Register an instance offering the required methods
        self.register_instance(USBRELAY32RpcServer.__ServerMethods(self))

        # The com port to use
        self.__com_port = com_port

        # The serial.Serial instance
        self.__serial = None

        # A boolean indicating whether this server is currently
        # responding to XML RPC request
        self.__is_running = False

        # Lock object to protect from concurent serial com post access
        self.__serial_lock = Lock()


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
        if self.__serial:
            self.__serial.close()
            self.__serial = None

    def set_relay_on(self, relay_id, serial_port=None):
        """
        Enables relay of relay_id.
        :type relay_id: integer
        :param relay_id: the number of relay
        """
        self.get_logger().info("Set relay to ON: " + str(relay_id))
        self.set_relay_one_state(relay_id, "ON")

    def set_relay_off(self, relay_id, serial_port=None):
        """
        Enables relay of relay_id.
        :type relay_id: integer
        :param relay_id: the number of relay
        """
        self.get_logger().info("Set relay to OFF: " + str(relay_id))
        self.set_relay_one_state(relay_id, "OFF")

    def set_relay_one_state(self, relay_port, state):
        cmd = list()
        if int(relay_port) not in range(0, 31):
            error_msg = "Relay number %s is not supported" % relay_port
            self.get_logger().error(error_msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, error_msg)

        relay_id = str(relay_port) if int(relay_port) < 10 else chr(55 + int(relay_port))

        cmd.append("relay " + str(USBRELAY32RpcServer.__CMDS[state]) + " " + relay_id + "\n\r")
        self.write(cmd)

    def set_relay_states(self, states):
        """
        Sends a single byte that represents the desired relay states.
        All on: ON, all off: OFF.
        :type states: integer
        :param states: the desired states far all relays. A String ON or OFF.
        """
        if states not in USBRELAY32RpcServer.__CMDS:
            msg = "Invalid relay states (%s)" % (str(states))
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)
        for relay_id in range(0, 31):
            self.set_relay_one_state(relay_id, states)

    def set_all_relay_on(self):
        """
        Sets all relay ON.
        """
        self.get_logger().info("Set all relays to ON")
        self.set_relay_states("ON")

    def set_all_relay_off(self):
        """
        Sets all relay OFF.
        """
        self.get_logger().info("Set all relays to OFF")
        self.set_relay_states("OFF")

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
                                             "Serial Port is not initialized, check USBRELAY32 connection")

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
                                             "Serial Port is not initialized, check USBRELAY32 connection")

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
