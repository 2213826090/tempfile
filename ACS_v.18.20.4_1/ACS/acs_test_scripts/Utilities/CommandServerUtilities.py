"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

:organization: INTEL NDG SW DEV
:summary: This file implements the command server API
:since: 05 December 2014
:author: ymorel
"""
import socket
import json
import time

from ErrorHandling.DeviceException import DeviceException
from Core.Report.ACSLogging import LOGGER_FWK

class CommandServerApi(object):
    """
    API to communicate with embedded command server
    Supports SocketProtocol
    """

    # Server command status possible values
    SRV_CMD_SUCCESS = "success"
    SRV_CMD_FAILURE = "failure"
    SRV_CMD_UNKNWON = "unknown"
    SRV_CMD_DEFERRED = "deferred"

    def __init__(self, proto=None):
        """
        Constructor
        :type proto: IProtocol object
        :param proto: protocol to use for communication with embedded server
        """
        # Check proto is an instance of IProtocol
        if not isinstance(proto, IProtocol):
            raise

        # Create Protocol factory
        self._proto_factory = ProtocolFactory(proto)

    def _check_cmd_status(self, status, result):
        """
        Check server command status:
        """
        if status == CommandServerApi.SRV_CMD_UNKNWON:
            msg = "Wrong embedded agent output"
            LOGGER_FWK.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        elif status == CommandServerApi.SRV_CMD_FAILURE:
            # Just log the error now, let UEcmd decide of what to do in case of failure
            # In this case, result contains the error message to log
            LOGGER_FWK.error(result)

    def send_cmd(self, cmd, args={}):
        """
        Send a command to embedded server via socket
        :type cmd: string
        :param cmd: command name
        :type args: dict
        :param args: command arguments
        :rtype: tuple
        :return: the status and the result of the command
        """
        request = json.dumps([cmd, [], args])
        LOGGER_FWK.info("Sending command %s %s" % (cmd, str(args)))
        try:
            conn = self._proto_factory.create()
            conn.connect()
            conn.send(request)
            status, result = json.loads(conn.receive())
            self._check_cmd_status(status, result)
        except:
            msg = "Failed to communicate with embedded command server"
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        finally:
            conn.disconnect()

        if status == CommandServerApi.SRV_CMD_DEFERRED:
            LOGGER_FWK.debug("Command ID: %d" % result)

        return status, result

    def get_result(self, cmd_id):
        """
        Get result of deferred command
        :type cmd_id: int
        :param cmd_id: ID of the command
        :rtype: tuple
        :return: the status and the result of the command
        """
        request = json.dumps(["join", [cmd_id], {}])
        LOGGER_FWK.debug("Joining command server %d" % cmd_id)

        try:
            conn = self._proto_factory.create()
            conn.connect()
            conn.send(request)
            status, result = json.loads(conn.receive())
            self._check_cmd_status(status, result)
        except socket.error as E:
            err, err_msg = E
            msg = "Socket error (%d): %s" % (err, err_msg)
            LOGGER_FWK.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        except:
            msg = "Failed to communicate with embedded command server"
            LOGGER_FWK.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        finally:
            conn.disconnect()

        return status, result

class IProtocol():
    """
    Interface to realize to implement a protocol of communication with embedded command server
    """

    def __init__(self, dest, port):
        """
        Constructor
        :type dest: object
        :param dest: commands destination (eg, an IP address in case of SocketUtilities)
        :type port: object
        :param port: communication port
        """
        self._dest = dest
        self._port = port

    def init(self):
        """
        Initialization function. It is called once when Protocol object is created
        """
        pass

    def get_dest(self):
        """
        Get destination
        :rtype: object
        :return: the command destination
        """
        return self._dest

    def get_port(self):
        """
        Get communication port
        :rtype: object
        :return: the communication port
        """
        return self._port

    def connect(self):
        """
        Connect to destination
        """
        raise

    def disconnect(self):
        """
        Disconnect from destination
        """
        raise

    def send(self, request):
        """
        Send command request
        :type request: object
        :param request: command request
        """
        raise

    def receive(self):
        """
        Receive answer of command request
        :rtype: object
        :return: answer of command request
        """
        raise

class ProtocolFactory():
    """
    Factory of IProtocol objects
    """

    def __init__(self, proto):
        """
        Constructor
        :type proto: IProtocol object
        :param proto: IProtocol model for objects creation
        """
        self._dest = proto.get_dest()
        self._port = proto.get_port()
        self._class = proto.__class__

    def create(self):
        """
        Create an IProtocol object from IProtocol model
        :rtype: IProtocol object
        :return: an new instance of IProtocol model
        """
        # Create Protocol object
        conn = self._class(self._dest, self._port)
        # Initialize Protocol object
        conn.init()
        # Return created Protocol object
        return conn

class SocketProtocol(IProtocol):
    """
    IProtocol interface realization with use of socket
    """

    # Default communication with server server parameters
    DFLT_SOCK_TIMEOUT = 10
    DFLT_SRV_PORT = 8888
    CHUNK_SIZE = 2048
    READ_TIME_INTERVAL = 0.1

    def __init__(self, ip, port=DFLT_SRV_PORT):
        """
        Constructor
        :type ip: str
        :param ip: command server IP adress
        :type port: int
        :param port: command server port
        """
        IProtocol.__init__(self, ip, port)
        self._conn = None

    def connect(self):
        """
        Connect to command server
        """
        # Create socket
        self._conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Set socket timeout
        self._conn.settimeout(SocketProtocol.DFLT_SOCK_TIMEOUT)
        # Connect to command server
        self._conn.connect((self.get_dest(), self.get_port()))

    def disconnect(self):
        """
        Disconnect from command server
        """
        # Close connected socket
        self._conn.close()

    def send(self, request):
        """
        Send command request to command server
        :type request: object
        :param request: command request
        """
        # Just send the request to the command server
        self._conn.sendall(request)

    def receive(self):
        """
        Receive data from command server
        """
        # Just read data with default timeout
        return self._recv_timeout()

    def _recv_timeout(self, timeout=0.1):
        """
        Read data from socket with timeout
        :type conn: socket
        :param conn: socket to communicate with embedded command server
        :type timeout: float
        :param timeout: reading timeout in seconds
        :rtype: string
        :return: data from embedded command server
        """
        # Make non blocking socket
        self._conn.setblocking(0)

        chunks = []
        chunk = ""

        # Beginning time
        begin = time.time()

        while True:
            # If something has been read, break after timeout
            if chunks != [] and (time.time() - begin > timeout):
                break

            # Read something
            try:
                chunk = self._conn.recv(SocketProtocol.CHUNK_SIZE)
                if chunk:
                    chunks.append(chunk)
                    # Update the beginning time for measurement
                    begin = time.time()
                else:
                    # Sleep for sometime before reading again
                    time.sleep(SocketProtocol.READ_TIME_INTERVAL)
            except:
                pass

        # Join all chunks to make final string
        return "".join(chunks)
