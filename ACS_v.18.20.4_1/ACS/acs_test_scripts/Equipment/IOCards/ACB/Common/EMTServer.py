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

:organization: SII on behalf INTEL MCG PSI
:summary: Server implementation for EMT350 equipment
:author: vgomberx
:since: 15/01/2014
"""
from EMT350Cards import SUPPORTED_DAUGHTER_CARD
from SerialComms import SerialComms
from SimpleXMLRPCServer import SimpleXMLRPCRequestHandler, SimpleXMLRPCServer
from threading import Thread
from random import randint
import time
import sys


class EMTServer(SimpleXMLRPCServer):

    # Restrict to a particular path.
    class _RequestHandler(SimpleXMLRPCRequestHandler):
        """
        Inner class used to restrict XML RPC request
        to allowed path only.
        """
        rpc_paths = ('/RPC2',)

    def __init__(self, server_ip, server_port):
        """
        instanciate the server with a given address and port

        :type server_ip: str
        :param server_ip: address of the server
        :type server_port: int
        :param server_port: server port
        """
        # first check if com port is available and connect to it
        self.__is_running = False
        self.__com_object = SerialComms()
        self.__server_ip = server_ip
        self.__server_port = int(server_port)

        # then start server
        SimpleXMLRPCServer.__init__(
            self,
            (self.__server_ip, self.__server_port),
            requestHandler=EMTServer._RequestHandler,
            logRequests=False, allow_none=True)

        # then register function to share with client
        self.register_function(self.communicate)
        self.register_function(self.add_client)
        self.register_function(self.is_alive)
        self.register_function(self.stop)
        self.register_function(self.remove_client)
        self.register_function(self.stop_if_empty)
        self.register_function(self.start_serial_connection)
        self.register_function(self.am_i_the_right_server)
        self.register_function(self.am_i_client)

        # client configuration is described by a client number as dict key and is configuration as value
        self.__client_configuration = {}
        # initialize the serial part
        self.__log("server started")

    def __log(self, msg):
        """
        format msg before print them
        """
        formated_msg = time.strftime("%Y-%m-%d_%Hh%M.%S") + "\tEMT_SERVER: \t" + msg
        print formated_msg

    def start_serial_connection(self, com_port, baud_rate, retry_nb):
        """
        establish the server to equipment connection

        :type com_port: int
        :param com_port: com port number to connect with
        :type baud_rate: int
        :param baud_rate: baud rate for the serial connection
        :type retry_nb: int
        :param retry_nb: Number of serial connection attempt

        :rtype: boolean
        :return: True if serial object has been well instanciate and connected, False otherwise
        """
        return self.__com_object.init(com_port, baud_rate, retry_nb)

    def am_i_the_right_server(self, com_port, baud_rate):
        """
        check if we are contacting the right server by comparing COM port and baud rate

        :type com_port: int
        :param com_port: com port number to connect with
        :type baud_rate: int
        :param baud_rate: baud rate for the serial connection

        :rtype: tuple
        :return: (True if we target the right server, error msg)
        """

        msg = ""
        result, msg_s = self.__com_object.is_info_correct(com_port, baud_rate)
        if not result:
            msg += msg_s
            result = False
        return result, msg

    def communicate(self, client_id, cmd):
        """
        get entry from client and send it to serial object

        :rtype: boolean
        :return: True if serial object has been well instanciate and connected, False otherwise
        """
        # TODO: to be implemented
        self.__log("received communication from client %s : '%s'" % (client_id, cmd))
        self.__log("Equipment response: '" + self.__com_object.communicate(cmd) + "'")
        return True

    def start(self):
        """
        Starts this server instance.
        """
        self.__is_running = True
        time_to_see_one_client = 120
        start_time = time.time()
        # change the request timeout to a lower value
        self.timeout = 1

        while self.__is_running:
            self.handle_request()

            # as long as there is client , re init server
            if len(self.__client_configuration) > 0:
                start_time = time.time()

            # time to see one client
            if  ((time.time() - start_time) > time_to_see_one_client) and len(self.__client_configuration) <= 0:
                self.__log("not client seen after %ss, stopping" % str(time_to_see_one_client))
                self.__is_running = False
                break

        self.__log("main thread die")

    def stop(self):
        """
        Stops this server instance.
        """
        self.__log("killing server")
        self.__is_running = False
        self.__com_object.release()
        self.server_close()

    def __del__(self):
        """"
        ensure that we release the serial connection if destroy
        """
        if self.__com_object is not None:
            self.__com_object.release()

    def is_alive(self):
        """
        function for client to test is server is here.

        :rtype: boolean
        :return: always return True
        """
        return True

    def is_running(self):
        """
        function for client to test is server is here.

        :rtype: boolean
        :return: always return True
        """
        return self.__is_running

    def add_client(self, card_configuration):
        """
        first check that the client configuration is not in conflict
        with current client setting then add given client to the client pool

        :type card_configuration: dict of dict
        :param card_configuration: card configuration found on your benchconfig
               and cleaned after comparison with device info

        :rtype: tuple
        :return: (True if we target the right server, error msg)
        """
        super_error_msg = ""
        error_happen = False

        for client in self.__client_configuration.keys():
            server_card_conf = self.__client_configuration[client]

            for card_slot in server_card_conf.keys():
                # check that if the client used the same slot and same card
                error = False
                error_msg = ""
                if card_configuration.get(card_slot) is not None:
                    # check if the type of the card match first
                    if card_configuration[card_slot]["TYPE"] != server_card_conf[card_slot]["TYPE"]:
                        raise
                        continue
                    # if yes let the card implementation decide if the client configuration is valid with
                    # current server configuration
                    error, error_msg = SUPPORTED_DAUGHTER_CARD[card_configuration[card_slot]["TYPE"]]().validate_multi_use(card_configuration.get(card_slot),
                                                                                                server_card_conf[card_slot])
                if error:
                    error_happen = True
                    super_error_msg += error_msg

        #  leave here if crash happened
        if error_happen:
            return error_happen, super_error_msg

        id_is_right = False
        client_id = 0000
        while not id_is_right:
            client_id = randint(1000, 9999)
            if client_id not in self.__client_configuration.keys():
                id_is_right = True
        self.__client_configuration[client_id] = card_configuration

        return  error_happen, client_id

    def am_i_client(self, client_id):
        """
        return if you are known by the server

        :rtype: boolean
        :return: True if server know you
        """
        return client_id in self.__client_configuration.keys()

    def remove_client(self, client_id):
        """
        ask server to remove a client from the list.
        """
        if client_id  in self.__client_configuration.keys():
                self.__client_configuration.pop(client_id)

    def stop_if_empty(self):
        """
        ask server to stop if there is no more client connected to it

        :rtype: boolean
        :return: True if server will stop
        """
        result = False
        if len(self.__client_configuration) <= 0:
                self.__log("No more client detected, stopping server")
                self.stop()
                result = True
        return result


class EMTThread(Thread):

    """
    This class is intended to run a RPC USB relay server.
    Once this thread has been started and then stopped it
    cannot be restarted. A new instance has to be created
    in order to obtain a working XML RPC server.
    """

    def __init__(self, server_ip, server_port):
        """
        Initializes this instance.

        :type server_ip: str
        :param server_ip: address of the server
        :type server_port: int
        :param server_port: server port
        """
        Thread.__init__(self)
        self.daemon = True
        self.__server = EMTServer(server_ip, server_port)

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

    def is_running(self):
        result = False
        if self.__server is not None and self.__server.is_running():
            result = True
        return result

if __name__ == "__main__":

    params = sys.argv
    if len(params) == 3:
        address = params[1]
        port = params[2]
        # create the server
        # Run the instance in a thread
        xml_rpc_thread = EMTThread(address, port)
        xml_rpc_thread.name = "USBRlyRpcThread"
        xml_rpc_thread.start()

        print time.strftime("%Y-%m-%d_%Hh%M.%S") + "\tEMT_SERVER_MAIN: \t" + "start to loop until server is stopped"
        while xml_rpc_thread.is_running():
            time.sleep(10)

        # Wait some time in order to give a chance
        # to the server to complete its starting procedure
        print time.strftime("%Y-%m-%d_%Hh%M.%S") + "\tEMT_SERVER_MAIN: \t" + "server stopped"
    else:
        print time.strftime("%Y-%m-%d_%Hh%M.%S") + "\tEMT_SERVER_MAIN: \t" + "missing parameter in call, you need to do like this: python EMTServer.py server_address ip"
