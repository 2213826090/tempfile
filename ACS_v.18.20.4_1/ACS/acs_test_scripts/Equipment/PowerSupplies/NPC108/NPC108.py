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
:summary: implementation of NPC108 power supplies
:since: 29/05/2011
:author: xzhao23
"""
import socket
from time import sleep
from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT
from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Equipment.PowerSupplies.Interface.IPowerSupply import IPowerSupply

# pylint: disable=W0613


class NPC108(IPowerSupply):

    """
    Implementation class of NPC108 power supply
    """

    def __init__(self, name, model, eqt_params, bench_params):  # pylint: disable=W0231
        """
        Constructor
        :type ip: str
        :param ip: the NPC108 device's ip used to create socket connection
        :type port: int
        :param port: its is the communication port need when creating the socket connection with NPC
        :type portnum: int
        :param portnum: tell how many available power ports in NPC108
        """
        # Initialize attributes
        self.__logger = LOGGER_TEST_SCRIPT
        self.__model = model
        self.__name = name
        self.__bench_params = bench_params
        # Initialize attributes
        self.__ip = None
        self.__port = None
        self.__portnum = None

        ip = str(self.__bench_params.get_param_value("IP"))
        socket_port = int(self.__bench_params.get_param_value("SocketPort"))
        total_outputs = int(self.__bench_params.get_param_value("TotalOutputs"))

        self.general_init(ip, socket_port, total_outputs)

        self.__current_output = int(self.__bench_params.get_param_value("CurrentOutput"))
        self.__delay = int(self.__bench_params.get_param_value("Delay"))

    def init(self):
        pass

    def general_init(self, ip, port=8887, portnum=8):
        """
        Constructor
        :type ip: str
        :param ip: the NPC108 device's ip used to create socket connection
        :type port: int
        :param port: its is the communication port need when creating the socket connection with NPC
        :type portnum: int
        :param portnum: tell how many available power ports in NPC108
        """
        # Initialize attributes
        self.__ip = ip
        # the port is specified for NPC108
        self.__port = port
        self.__portnum = portnum

    def __del__(self):
        """
        Destructor
        """
        self.release()

    def release(self):
        pass

    def perform_full_preset(self):
        # for NPC108, no parameter reset needed
        pass

    def get_eq_id(self):
        """
        Gets the ID of the equipment
        :rtype: str
        :return: the identification str of the equipment
        """
        return "NPC108"

    def configure_basic_parameters(self):
        """
        Configuration of basic parameters of the power supply
        Nothing to do for NPC108
        """
        pass

    def power_on(self):
        self._power_on(self.__current_output, self.__delay)

    def _power_on(self, port, delay=2):
        """
        Turns on the power supply, the "port" parameter can be one of value that is smaller or equal with self.__portnum
        each number represents a port in NPC108 device
        """
        if (not isinstance(port, type(0))) or (port <= 0 or port >= self.__portnum):
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, "Port should be digit, should be one of\
            value that is smaller or equal with %s" % self.__portnum)

        conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            conn.connect((self.__ip, self.__port))
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            raise TestEquipmentException(TestEquipmentException.CONNECTION_ERROR,
                                         "[Errno 10061] No connection could be made because the target machine actively refused it")
        self.__logger.info("Power on specified port, via NPC108")
        conn.send('OPEN 0%s\0\r\n' % str(port))
        conn.close()
        sleep(delay)

    def power_off(self):
        self._power_off(self.__current_output, self.__delay)

    def _power_off(self, port, delay=2):
        """
        Turns off the power supply, the "port" parameter can be one of value that is smaller or equal with self.__portnum
        each number represents a port in NPC108 device
        """
        if (not isinstance(port, type(0))) or (port <= 0 or port >= self.__portnum):
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, "Port should be digit, should be one of\
            value that is smaller or equal with %s" % self.__portnum)

        conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            conn.connect((self.__ip, self.__port))
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            raise TestEquipmentException(TestEquipmentException.CONNECTION_ERROR,
                                         "[Errno 10061] No connection could be made because the target machine actively refused it")
        self.__logger.info("Power off specified port, via NPC108")
        # sockobj.send('CLOSE 0%s\0\r\n'%str(a))
        conn.send('CLOSE 0%s\0\r\n' % str(port))
        conn.close()
        sleep(delay)

    def power_cycle(self):
        self._power_off(self.__current_output, self.__delay)
        self._power_on(self.__current_output, self.__delay)

    def _power_cycle(self, port, delay=2):
        """
        Cycle the specified port, the "port" parameter can be one of value that is smaller or equal with self.__portnum
        each number represents a port in NPC108 device
        :type port: int
        :param port: specify which power port
        """
        if (not isinstance(port, type(0))) or (port <= 0 or port >= self.__portnum):
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, "Port should be digit, should be one of\
            value that is smaller or equal with %s" % self.__portnum)

        conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            conn.connect((self.__ip, self.__port))
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            raise TestEquipmentException(TestEquipmentException.CONNECTION_ERROR,
                                         "[Errno 10061] No connection could be made because the target machine actively refused it")
        self.__logger.info("Power Cycle on specified port on NPC108")
        # sockobj.send('CLOSE 0%s\0\r\n'%str(a))
        conn.send('REBOOT 0%s\0\r\n' % str(port))
        conn.close()
        sleep(delay)

    def reset_device(self, delay=10):
        """
        Reboot the NPC108
        :type delay: str
        :param delay:after doing a action on NPC, the NPC need some delay time to finish the action
       """
        conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            conn.connect((self.__ip, self.__port))
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            raise TestEquipmentException(TestEquipmentException.CONNECTION_ERROR,
                                         "[Errno 10061] No connection could be made because the target machine actively refused it")

        conn.send('REBOOT NOW\0\r\n')
        conn.close()
        sleep(delay)
