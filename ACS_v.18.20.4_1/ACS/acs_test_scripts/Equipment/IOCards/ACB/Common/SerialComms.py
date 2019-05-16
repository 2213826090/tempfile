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
:summary: Serial communication implementation for EMT350
:author: vgomberx
:since: 15/01/2014
"""
import serial
import time


class SerialComms():

    def __init__(self):
        """
        init the var
        """
        # Initialize local variables
        self.__handle = None
        self.__com_port = None
        self.__baudrate = None
        self.__retry = None
        self.__reply_timeout = 120

    def init(self, com_port, baud_rate, retry_nb):
        """
        try to establish the serial connection

        :type com_port: int
        :param com_port: com port number to connect with
        :type baud_rate: int
        :param baud_rate: baud rate for the serial connection
        :type retry_nb: int
        :param retry_nb: Number of serial connection attempt
        """
        # Initialize local variables
        self.__retry = retry_nb
        self.__baudrate = baud_rate
        self.__com_port = com_port
        return self.open_communication()

    def is_info_correct(self, com_port, baud_rate):
        """
        check that info is correct when you use
        server client system

        :type com_port: int
        :param com_port: com port number to connect with
        :type baud_rate: int
        :param baud_rate: baud rate for the serial connection

        :rtype: boolean
        :return: True if server will stop
        """
        result = True
        msg = ""
        if self.__com_port != com_port:
            msg += "Serial communication does not match: server:[%s] - you:[%s]\n" % (self.__com_port, com_port)
            result = False
        if self.__baudrate != baud_rate:
            msg += "baud rate does not match: server:[%s] - you:[%s]\n" % (self.__baudrate, baud_rate)
            result = False
        return result, msg

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
        return message

    def __decode(self, message):
        """
        decode the controller reply

        :param message: reply from controller
        :type message: depending of controller message type

        :rtype: list
        :return: list containing the controller reply
        """
        return message

    def release(self):
        """
        Releases the connection with the equipment and all resources allocated
        to its use.
        """
        if self.__handle is not None:
            print "Release serial connection"
            self.__handle.close()
            # Update handle value
            self.__handle = None

    def receive(self):
        """
        Listen to the controller output.

        :rtype: list
        :return: list containing the controller reply
        """
        response = self.__handle.readline()
        return self.__decode(response)

    def is_communication_up(self):
        """
        common function with other communication object
        """
        result = False
        if self.__handle is not None:
            result = True
        return result

    def communicate(self, message):
        """
        send command to com port

        :type message: list
        :param message: message to send to the equipment,

        :rtype: list
        :return: reply from com port
        """
        data_frame = self.__encode(message)
        self.__handle.write(data_frame)
        response = ""
        next_response = self.__handle.readline()
        start_time = time.time()

        while next_response != "":
            response += next_response
            next_response = self.__handle.readline()
            if start_time - time.time() > self.__reply_timeout:
                msg = "%s timeout exceed for reply" % self.__reply_timeout
                print msg
                raise

        return self.__decode(response)

    def open_communication(self):
        """
        common function with other communication object
        """
        handle = None
        try_index = 0
        result = False
        # Access to serial port is not reliable
        # Try several times
        while try_index < self.__retry:
            try:
                handle = serial.Serial(
                    port=self.__com_port,
                    baudrate=self.__baudrate,
                    timeout=1
                )
                break
            except serial.SerialException as error:
                try_index += 1
                if try_index < self.__retry:
                    print str(error)
                    print "Retry the connection..."
                    time.sleep(1)
                else:
                    raise error

        if handle is None:
            print "Serial port connection failed."
        else:
            # Update handle value
            self.__handle = handle
            result = True
        return result

    def __del__(self):
        """"
        ensure that we release the serial connection if destroy
        """
        self.release()
