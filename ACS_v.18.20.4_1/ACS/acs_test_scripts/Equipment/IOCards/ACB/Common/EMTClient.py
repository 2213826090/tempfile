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

:organization: INTEL MCG PSI
:summary: Client implementation for EMT equipment
:author: vgomberx
:since: 15/01/2014
"""
from ErrorHandling.TestEquipmentException import TestEquipmentException
from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT
import xmlrpclib
import time
import socket
import subprocess
import os


class EMTClient():
    """
    Client base that connect or create the server.
    """

    def __init__(self):
        # load server configuration
        # address and other things
        # card where the usb connection is declared
        self._client_id = None
        self.__server_proxy = None
        self.__com_port = None
        self.__serial_baud_rate = None
        self.__retry_nb = None
        self.__card_conf = None

    def init(self, card_configuration, server_ip, server_port, com_port, serial_baud_rate, retry_nb):
        """
        connect sever
        """
        # check if a server exist
        # create one if necessary
        # send io card configuration to server
        # create a proxy to access to server
        # check if server exist
        self.__com_port = com_port
        self.__serial_baud_rate = serial_baud_rate
        self.__retry_nb = retry_nb
        self.__card_conf = card_configuration
        self.__server_ip = server_ip
        self.__server_port = server_port
        if not self.__connect_server():
            LOGGER_TEST_SCRIPT.info("not server running, trying to start one")
            self.__start_server()

    def __start_server(self):
        """
        start server
        """
        # create the server
        # Run the instance in a thread
        server_name = os.path.join(os.path.dirname(__file__), "EMTServer.py")
        cmd = "python %s %s %s" % (server_name, self.__server_ip, self.__server_port)
        subprocess.Popen(cmd)
        # Wait some time in order to give a chance
        # to the server to complete its starting procedure
        time.sleep(1)
        msg = ""
        if self.__server_proxy is None:
            self.__server_proxy = xmlrpclib.ServerProxy("http://" + str(self.__server_ip) + ":" + str(self.__server_port))

        if not self.__is_server_running():
        # if server still can be run ,raised an error
            msg = "cannot establish a connection or create a server"
            LOGGER_TEST_SCRIPT.error(msg)
            raise TestEquipmentException(
                TestEquipmentException.OPERATION_FAILED, msg)

        LOGGER_TEST_SCRIPT.info("connection establish with server")
        LOGGER_TEST_SCRIPT.info("trying to establish server to equipment connection")
        # reset your client id if you have started a new server
        self._client_id = None
        self.__ask_server_to_connect_with_eq(self.__com_port, self.__serial_baud_rate, self.__retry_nb)
        LOGGER_TEST_SCRIPT.info("trying to check that your bench configuration is compatible with server")
        self.__add_me_as_client(self.__card_conf)

    def __connect_server(self):
        """
        check if there is an available server
        if not create one.
        """
        msg = "already connected to server"
        if self.__server_proxy is None:
            self.__server_proxy = xmlrpclib.ServerProxy("http://" + str(self.__server_ip) + ":" + str(self.__server_port))
            msg = "connection establish with server"

        if not self.__is_server_running():
            self.__server_proxy = None
            msg = "fail to connect to server"
            LOGGER_TEST_SCRIPT.error(msg)
            return False

        LOGGER_TEST_SCRIPT.info(msg)
        # if we are connected check that we are connected to the right one
        result, msg = self.__server_proxy.am_i_the_right_server(self.__com_port, self.__serial_baud_rate)
        if not result:
            msg = "the running server does not match with your bench configuration: \n" + msg
            LOGGER_TEST_SCRIPT.error(msg)
            raise TestEquipmentException(
                TestEquipmentException.OPERATION_FAILED, msg)
        self.__add_me_as_client(self.__card_conf)
        return True

    def __ask_server_to_connect_with_eq(self, com_port, serial_baud_rate, retry_nb):
        """
        ask server to establish a connection with the equipment

        :type com_port: int
        :param com_port: com port number to connect with
        :type serial_baud_rate: int
        :param serial_baud_rate: baud rate for the serial connection
        :type retry_nb: int
        :param retry_nb: Number of serial connection attempt
        """
        result = self.__server_proxy.start_serial_connection(com_port, serial_baud_rate, retry_nb)
        if not result:
            msg = "fail to connect server to serial equipment"
            LOGGER_TEST_SCRIPT.error(msg)
            raise TestEquipmentException(
                TestEquipmentException.OPERATION_FAILED, msg)

    def __is_server_running(self):
        result = False
        if self.__server_proxy is None:
            LOGGER_TEST_SCRIPT.error("no connection to server established")
        else:
            try:
                self.__server_proxy.is_alive()  # Call a fake method.
                result = True
            except socket.error as e:
                # Not connected ; socket error mean that the service is unreachable.
                LOGGER_TEST_SCRIPT.warning("socket error : %s" + str(e))
            except Exception as e:
                # connected to the server and the method doesn't exist which is expected.
                LOGGER_TEST_SCRIPT.warning("server cannot be reached for unknown reason : %s" + str(e))

        return result

    def is_communication_up(self):
        """
        common function with other communication object
        """
        return self.__is_server_running()

    def communicate(self, cmd):
        """
        send command to server
        """
        result = None
        if self.__is_server_running():
            result = self.__server_proxy.communicate(self._client_id, cmd)
        return result

    def open_communication(self):
        """
        common function with other communication object
        """
        return self.__start_server()

    def __add_me_as_client(self, card_configuration):
        """
        send your io card configuration from your benchconfig
        to the server.
        """
        if self._client_id is not None:
            # test if server know you
            if not self.__server_proxy.am_i_client(self._client_id):
                msg = "failed to be recognized by server"
                LOGGER_TEST_SCRIPT.error(msg)
                raise TestEquipmentException(
                    TestEquipmentException.OPERATION_FAILED, msg)

        else:
            error, msg = self.__server_proxy.add_client(card_configuration)
            # in case of error, msg is all compatibility error found on your code
            if error:
                msg = "failed to add new client to server:\n" + str(msg)
                LOGGER_TEST_SCRIPT.error(msg)
                raise TestEquipmentException(
                    TestEquipmentException.OPERATION_FAILED, msg)
            # in case of success, msg is the client id
            else:
                self._client_id = msg

    def release(self):
        """
        stop the client and stop the server if
        there is no more client connect
        """
        LOGGER_TEST_SCRIPT.error("Try to release CLIENT %s" % self._client_id)
        try:
            if self.__is_server_running():
                # client list empty
                if self._client_id is not None:
                    self.__server_proxy.remove_client(self._client_id)

        except Exception as exc:  # pylint: disable=W0703
            # Initialize log message
            LOGGER_TEST_SCRIPT.error("error during client %s deletion: %s" % (self._client_id, exc))
        finally:
            if self.__is_server_running():
                # ask the server to stop if there is no more client
                if self.__server_proxy.stop_if_empty():
                    self.__server_proxy = None

    def __del__(self):
        """
        Destructor: release automatically the connection
        """
        LOGGER_TEST_SCRIPT.error("Deleting CLIENT %s" % self._client_id)
        try:
            if self.__is_server_running():
                # client list empty
                if self._client_id is not None:
                    self.__server_proxy.remove_client(self._client_id)

        except Exception as exc:  # pylint: disable=W0703
            # Initialize log message
            LOGGER_TEST_SCRIPT.error("error during client %s deletion: %s" % (self._client_id, exc))
        finally:
            if self.__is_server_running():
                # ask the server to stop if there is no more client
                if self.__server_proxy.stop_if_empty():
                    self.__server_proxy = None
