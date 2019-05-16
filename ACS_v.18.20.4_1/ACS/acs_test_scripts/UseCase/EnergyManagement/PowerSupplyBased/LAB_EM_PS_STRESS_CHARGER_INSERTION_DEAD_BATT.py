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
:summary: STress Test for Charger Insertion with Dead battery
:author: jvauchex
:since: 1/08/2014
"""
import time
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import Global


class LabEmPsStressChargerInsertionDeadBatt(EmUsecaseBase):

    """
    Lab Energy Management class.
    """
    DEDICATED_BENCH = "POWER_SUPPLY_BENCH"

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LAB_EM_BASE Init function
        EmUsecaseBase.__init__(self, tc_name, global_config)

        self.__charger_type = self._tc_parameters.get_param_value("CHARGER_TYPE", "DCP")
        self.__start_battery_voltage = float(self._tc_parameters.get_param_value("START_BATTERY_VOLTAGE", 2.6))
        self.__end_battery_voltage = float(self._tc_parameters.get_param_value("END_BATTERY_VOLTAGE", 2.3))
        self.__battery_step = float(self._tc_parameters.get_param_value("BATTERY_STEP", 0.025))
        self.__expected_charge_current = float(self._tc_parameters.get_param_value("EXPECTED_CHARGE_CURRENT", 0.40))
        self.__wait_time = int(self._tc_parameters.get_param_value("WAIT_TIME", 120))

        # Redefine initial value for setting USBDIO:
        # - Platform
        self.em_core_module.io_card_init_state["Platform"] = "OFF"

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # Call the LabEmBasePS Setup function
        EmUsecaseBase.set_up(self)

        if ((self.__end_battery_voltage > self.__start_battery_voltage) or
                    ((self.__start_battery_voltage - self.__end_battery_voltage) < self.__battery_step)):
            txt = "End battery voltage is higher than start battery voltage or we cant done any measure => Quit the test"
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, txt)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        """

        # Cpt Initializa
        cpt = 1
        new_voltage = self.__start_battery_voltage

        # Call LAB_EM_BASE_PS Run function
        EmUsecaseBase.run_test_body(self)

        while new_voltage >= self.__end_battery_voltage:

            # set the vbatt voltage
            self.em_core_module.get_eq_emulated_battery().set_current_voltage(new_voltage,
                            self.em_core_module.ps_properties["BATT"]["PortNumber"])

            # Plug the right USB Charger
            self._io_card.simulate_insertion(self.__charger_type)

            # Wait during wait_time value
            self._logger.info("Wait %d seconds..." % self.__wait_time)
            time.sleep(self.__wait_time)

            # Measure current from Vbatt
            meas_ibatt = self.em_core_module.get_battery_current()

            # Disconnect USB
            self.em_core_module.plug_charger("None")

            if float(meas_ibatt[0]) > self.__expected_charge_current:
                txt = "Current value %f is out of range for voltage : %f V => Wrong charge current" \
                      % (float(meas_ibatt[0]), new_voltage)
                self._logger.error(txt)
                raise DeviceException(DeviceException.OPERATION_FAILED, txt)
            else:
                txt = "Current value %f A is good" % meas_ibatt[0]
                self._logger.info(txt)

            # Set a the voltage value
            cpt += 1
            new_voltage = self.__start_battery_voltage - (float(cpt) * self.__battery_step)
            self._logger.info("New voltage value : %f V" % new_voltage)
            time.sleep(5)

        return Global.SUCCESS, "no error"
