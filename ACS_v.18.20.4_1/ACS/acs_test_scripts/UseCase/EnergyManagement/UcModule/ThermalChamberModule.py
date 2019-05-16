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
:summary: This module represent thermal module allowing you to interact with temperature chamber.
:author: vgomberx
:since: July 29 2013
"""
import time
from acs_test_scripts.Utilities.EMUtilities import EMConstant
from acs_test_scripts.UseCase.EnergyManagement.UcModule.OverMind import OverMind
from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT


class ThermalChamberModule():
    """
    init.
    """
    __LOG_TAG = "[THERMAL_CHAMBER_MODULE]\t"

    def __init__(self, execution_temp, simulator_name="TEMPERATURE_CHAMBER1"):
        """
        describe basic parameters to set up temperature chamber
        """
        overmind = OverMind()
        self.__logger = LOGGER_TEST_SCRIPT
        self.__logger.info(self.__LOG_TAG + "INIT")
        device = overmind.get_instance(overmind.DEVICE)
        eq_manager = overmind.get_instance(overmind.EQUIPMENT_MANAGER)
        # DECLARE OPTIONAL GLOBAL PARAMETER HERE
        self.__tct = execution_temp
        # get temperature chamber & init var
        self.__logger.info(self.__LOG_TAG + "Used of temperature chamber requested")
        self.__temperature_chamber = eq_manager.get_temperature_chamber(simulator_name)
        # Set thermal test to DEFAULT else it will be SPECIFIC to your test
        self.__thermal_test = "DEFAULT"

        self.__chamber_tag = EMConstant.CHAMBER_TAG
        if self.__temperature_chamber is not None:
            self.__chamber_tag = self.__chamber_tag + "_" + \
                self.__temperature_chamber.get_measurement_unit()

        em_params = device.get_em_parameters()

        self.__low_charging_lim = em_params["BATTERY"]["THERMAL_CHARGING_LOW_LIMIT"]
        self.__high_charging_lim = em_params["BATTERY"]["THERMAL_CHARGING_HIGH_LIMIT"]
        self.__ambient_temperature = 25

    def is_default_test(self):
        """
        check if the setting for this thermal module is declared as a 'default' test
        default test is manage by the EM usecase base itself.

        :rtype: boolean
        :return: true if the test is set as default test
        """
        result = False
        if self.__thermal_test == "DEFAULT":
            result = True

        return result

    def set_test_type(self, test_type):
        """
        set the type of thermal test to change the module behavior
        """
        self.__thermal_test = test_type

    def release_eq(self, temp_margin=10):
        """
        Release equipment used.
        """
        # release equipment at ambient temperature
        if self.__temperature_chamber.is_connected():
            self.__temperature_chamber.set_regulation(True)
            self.__temperature_chamber.set_temperature(self.__ambient_temperature)
            self.__temperature_chamber.wait_for_temperature(self.__ambient_temperature, margin=temp_margin)
            self.__logger.info(self.__LOG_TAG + "Temperature chamber released")
            self.__temperature_chamber.release()

#----------------------------------------------------------------------
    def set_up_eq(self,):
        """
        initialize equipment.
        Does not turn on regulation
        """
        self.__temperature_chamber.init()

    def adjust_temp_according_to_test_value(self, consider_limit=True, consider_margin=0):
        """
        set equipment to the test temperature according to charging limit
        if you are outside the limit, ambient temperature will be chosen.
        test temperature are defined during this module initialization.
        This is designed to be used on usecase set_up.

        :type consider_limit: boolean
        :param consider_limit: True to evaluate your device charging limit and adapt the
                               temperature change to it.

        :type consider_margin: int
        :param consider_margin: will adjust temp value by considering chamber temp +/- the margin value
        :rtype: boolean
        :return: True if setup is at test temperature, False for ambient temperature
        """
        self.__temperature_chamber.set_regulation(True)
        result = False
        # get device temperature limits
        if consider_limit and (self.__tct < self.__low_charging_lim or self.__tct > self.__high_charging_lim):
            self.__temperature_chamber.set_temperature(self.__ambient_temperature)
            msg = "temperature chamber %s Degree Celsius is outside battery thermal charging limits [%s,%s]" % (self.__tct,
                                                                                                               self.__low_charging_lim,
                                                                                                        self.__high_charging_lim)
            self.__logger.warning(self.__LOG_TAG + msg)
            self.__logger.warning(self.__LOG_TAG + "setup will be done at ambient temperature")

        else:
            self.__temperature_chamber.set_temperature(self.__tct)
            self.__temperature_chamber.wait_for_temperature(self.__tct, margin=consider_margin)
            result = True

        return result

    def adjust_temp_according_to_current_value(self):
        """
        set the test temperature according to its current value return by equipment.
        if test temperature was outside the charging range it will be set at this time.

        This is designed to be used on usecase run_test.
        """
        # get device temperature limits
        # set the temperature to wanted one
        if self.__temperature_chamber.is_connected() and self.__tct < self.__low_charging_lim or self.__tct > self.__high_charging_lim:
            temperature = self.__temperature_chamber.get_temperature()

            if temperature not in range(self.__tct - 1, self.__tct + 1):
                self.__logger.info(self.__LOG_TAG + "Temperature chamber configured")
                self.__temperature_chamber.set_temperature(self.__tct)
                self.__temperature_chamber.wait_for_temperature(self.__tct)
                self.__logger.info(self.__LOG_TAG + "wait 120 sec to let the board adapt to this temperature")
                time.sleep(120)

#----------------------------------------------------------------------

    def get_eq(self):
        """
        return the instance of temperature chamber.
        Allowing you to interact directly with it.

        :rtype: ITemperatureChamber
        :return: object that represent your temperature chamber
        """
        return self.__temperature_chamber

    def get_chamber_tag(self):
        """
        return chamber tag name for specific used on usecase

        :rtype: str
        :return: text containing a tag that represent your temperature chamber
        """
        return self.__chamber_tag

    def feed_meas_report(self):
        """
        return a tuple for the purpose of measurement reports
        :rtype: tuple (str, int)
        :return: (chamber tag, current temperature)
        """
        return self.get_chamber_tag(), self.__temperature_chamber.get_temperature()
