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
:summary: temperature chamber implementation for TMT80.
:since: 14/03/2012
:author: vgombert
"""

from acs_test_scripts.Equipment.IEquipment import EquipmentBase
from acs_test_scripts.Equipment.TemperatureChamber.Interface.ITemperatureChamber import ITemperatureChamber
from ErrorHandling.TestEquipmentException import TestEquipmentException

import time


class TMT80(EquipmentBase, ITemperatureChamber):

    """
    Class that implements TMT80.
    """

    # time taken in second by the temperature to increase/decrease 1 degree Celsius
    TEMPERATURE_CHANGE_STEP = 90
    UNIT = "DegreeCelsius"

    def __init__(self, name, model, eqt_params, bench_params, controller):
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
        ITemperatureChamber.__init__(self)
        self.__bench_params = bench_params
        self.__equipement_address = 1
        self.__controller = controller

    def __del__(self):
        """
        Destructor: release automatically the connection
        """
        self.get_logger().debug("Delete %s", str(self.get_name()))
        self.release()

    def __compute_temperature(self, message):
        """
        decode equipment reply and return a readable temperature.

        :param message: reply from the equipment
        :type message: list

        :rtype: float or int
        :return: temperature value in Celsius
        """
        if message is not None and message != []:
            index = 0
            for element in message:

                if index == 1:
                    if element != 0x3:
                        msg = "Cant read temperature value"
                        self.get_logger().error(msg)
                        raise TestEquipmentException(
                            TestEquipmentException.OPERATION_FAILED, msg)
                if index == 3:
                    temp_high = element
                if index == 4:
                    temp_low = element
                index += 1

            if index >= 4:
                # compute temperature value
                temperature = int(temp_high * 256.0 + temp_low)
                if temperature > 32767:
                    temperature = ((temperature ^ 0xffff) + 1) & 0xffff
                    temperature = temperature * (-1) / 10.0
                else:
                    temperature /= 10.0
                return temperature
            else:
                msg = "Incomplete temperature frame returned"
                self.get_logger().error(msg)
                raise TestEquipmentException(
                    TestEquipmentException.OPERATION_FAILED, msg)
        else:
            msg = "Cant read temperature value"
            self.get_logger().error(msg)
            raise TestEquipmentException(
                TestEquipmentException.OPERATION_FAILED, msg)

    def init(self):
        """
        Initializes the equipment. Final state: the equipment is ready to use.
        """
        self.get_logger().info("Initialization")

        # Get supported controller
        controllers_catalog = \
            self.get_eqt_dict()[self.get_model()]["Controllers"]

        # get controller instance
        if self.__controller is None:
            msg = "Missing controller declaration in bench config"
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.PROPERTY_ERROR, msg)

        # get the model of declared control on bench configuration
        controller_model = self.__controller.get_model()

        if controller_model not in controllers_catalog or \
                controllers_catalog[controller_model] != "supported":
            msg = "Unsupported controller %s" % (str(controller_model))
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.CONTROLLER_ERROR, msg)

        # get the equipment address
        address = self.__bench_params.get_param_value("Address")
        if address is not None:
            self.__equipement_address = int(address)
        else:
            msg = "Missing 'Address' key on your bench configuration"
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.PROPERTY_ERROR, msg)

        # init the controller
        self.__controller.init()

        # check connection
        if not self.is_connected():
            msg = "Cant read reply from temperature chamber equipment, check your connection and equipment state"
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.CONNECTION_ERROR, msg)

    def release(self):
        """
        Releases the connection with the equipment and all resources allocated
        to its use.
        """
        self.get_logger().info("Release")
        self.set_regulation(False)
        # release the controller
        self.__controller.release()

    def set_regulation(self, state):
        """
        toggle the regulation state, allowed the chamber
        to adjust and keep it's temperature to the wanted one.

        :param state: False to stop to regulate the temperature
                      True to start to regulate the temperature
        :type state: boolean
        """
        self.get_logger().info("toggle regulation to " + str(state))
        if state:
            data_frame = [self.__equipement_address,
                          0x06, 0x01, 0x11, 0x00, 0x00]
            delay = 10

        elif not state:
            data_frame = [self.__equipement_address,
                          0x06, 0x01, 0x11, 0x00, 0x01]
            delay = 2
        else:
            raise TestEquipmentException(TestEquipmentException.PROPERTY_ERROR)

        # send message
        first_response = self.__controller.send(data_frame)

        # leave some time to the equipment to initialize or release
        time.sleep(delay)
        return first_response

    def get_regulation(self):
        """
        Check the regulation state.

        :rtype: boolean
        :return: true if regulation is running, false otherwise
        """
        self.get_logger().info("get regulation")
        data_frame = [self.__equipement_address,
                      0x03, 0x01, 0x11, 0x00, 0x01]

        # send message
        first_response = self.__controller.send(data_frame)

        if not first_response:
            msg = "error during regulation reading, equipment returned empty response"
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.MEASUREMENT_ERROR, msg)

        regulation = False
        if first_response[4] == 0:
            regulation = True

        return regulation

    def set_temperature(self, value):
        """
        Set the temperature.

        :param value: temperature value in Celsius.
        :type value: float or int
        """
        self.get_logger().info("set temperature to %s Degree Celsius" % str(value))
        value *= 10
        if value >= 0:
            temperature_low = value % 256
            temperature_high = value / 256
        else:
            temperature = ((abs(value) ^ 0xffff) + 1) & 0xffff
            temperature_low = temperature % 256
            temperature_high = temperature / 256

        data_frame = [self.__equipement_address, 0x06, 0x00, 0x02,
                      temperature_high, temperature_low]
        # send message
        self.__controller.send(data_frame)

    def get_temperature(self):
        """
        Get the temperature.

        :rtype: float or int
        :return: temperature value in Celsius
        """
        data_frame = [self.__equipement_address,
                      0x03, 0x00, 0x01, 0x00, 0x02]
        # send message
        first_response = self.__controller.send(data_frame)

        if not first_response:
            msg = "error during temperature reading, equipment returned empty response"
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.MEASUREMENT_ERROR, msg)

        temperature = self.__compute_temperature(first_response)

        self.get_logger().info("get temperature returned %s Degree Celsius" % temperature)
        return temperature

    def wait_for_temperature(self, expected_temperature, timeout=None, margin=0):
        """
        wait until temperature is reached.

        :param value: temperature value in Celsius.
        :type value: float or int

        :param timeout: timeout for temperature waiting in seconds
        :type timeout: int

        :type margin: int
        :param margin: will adjust temp value by considering chamber temp +/- the margin value

        :rtype: boolean
        :return: true if temperature is reached , false otherwise
        """

        min_temp = expected_temperature - margin
        max_temp = expected_temperature + margin

        temperature = self.get_temperature()
        if expected_temperature == temperature:
            self.get_logger().debug("temperature already at %s degree Celsius" % temperature)
            return True
        # case where magin is > than 0
        elif  min_temp <= temperature <= max_temp:
            self.get_logger().debug("temperature %s already in between [%s;%s] degree Celsius" % (temperature, min_temp, max_temp))
            return True

        if timeout is None:
            timeout = abs(expected_temperature - temperature) * self.TEMPERATURE_CHANGE_STEP

        txt = ""
        if margin != 0:
            txt = "wait at most %s second(s) for temperature changing from %s to a value in [%s;%s] degree Celsius" % (timeout, temperature, min_temp, max_temp)
        else:
            txt = "wait at most %s second(s) for temperature changing from %s to %s degree Celsius" % (timeout, temperature, expected_temperature)

        self.get_logger().info(txt)

        start_time = time.time()
        while start_time - time.time() < timeout:
            temperature = self.get_temperature()
            if expected_temperature == temperature:
                self.get_logger().debug("temperature at %s degree Celsius reached!" % temperature)
                return True
            elif margin != 0 and  min_temp <= temperature <= max_temp:
                self.get_logger().debug("temperature %s in between [%s;%s] degree Celsius" % (temperature, min_temp, max_temp))
                return True

        self.get_logger().debug("temperature at %s degree Celsius not reached before %s second(s)!" % (expected_temperature, timeout))
        return False

    def is_connected(self):
        """
        check connection with the temperature chamber.

        :rtype: boolean
        :return: true if connection is established, false otherwise
        """
        self.get_logger().info("check if connection is established")
        connection = False
        # check  chamber is connected.
        if self.__controller is not None and self.__controller.is_connected():
            data_frame = [self.__equipement_address,
                          0x08, 0x00, 0x00, 0x12, 0x34]
            reply = self.__controller.send(data_frame)

            if reply:
                connection = True

        self.get_logger().info("connection state is " + str(connection))
        return connection

    def get_measurement_unit(self):
        """
        Return the measurement unit used currently by this equipment.

        :rtype: Str
        :return: measurement unit
        """
        return self.UNIT
