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
:summary: Charger Insertion with Dead battery
:author: jvauchex
:since: 30/07/2014
"""
import time
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import Global


class LabEmPsChargerInsertionDeadBatt(EmUsecaseBase):

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
        self.__charger_type = self._tc_parameters.get_param_value("CHARGER_TYPE", "SDP")
        self.__battery_type = self._tc_parameters.get_param_value("BATTERY_TYPE", self.phone_info["BATTERY"]["BATTID_TYPE"])
        self.__battery_voltage = int(self._tc_parameters.get_param_value("BATTERY_VOLTAGE", 2.6))
        self.__expected_charge_current = float(self._tc_parameters.get_param_value("EXPECTED_CHARGE_CURRENT", 0.45))
        self.__expected_current_ratio = int(self._tc_parameters.get_param_value("EXPECTED_CURRENT_RATIO", 90))
        self.__measure_type = self._tc_parameters.get_param_value("MEASURE_TYPE", "CURRENT")
        self.__wait_time = int(self._tc_parameters.get_param_value("WAIT_TIME", 120))
        self.__current_range = float(self._tc_parameters.get_param_value("CURRENT_RANGE", 0.02))

        # Redefine initial value for setting USBDIO:
        # - Platform
        self.em_core_module.io_card_init_state["Platform"] = "OFF"

        # Set initial value for setting Power Supply VBATT:
        # - VoltageLevel = VBATT
        self.em_core_module.eqp_init_state["BATT"]["VoltageLevel"] = self.__battery_voltage
        # Set initial value for setting Power Supply VUSB:
        # - VoltageLevel = VUSB
        self.em_core_module.eqp_init_state["USB"]["VoltageLevel"] = self.em_core_module.vusb

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        """
        # Call LAB_EM_BASE_PS Run function
        EmUsecaseBase.run_test_body(self)

        # Plug the right USB Charger
        self._io_card.simulate_insertion(self.__charger_type)

        # Wait during wait_time value
        self._logger.info("Wait %d seconds..." % self.__wait_time)
        time.sleep(self.__wait_time)

        # Measure the current
        if (self.__measure_type.upper() == "CURRENT"):

            # Measure current from Vbatt
            meas_ibatt = self.em_core_module.get_battery_current()

            # Check the current
            high_current_range = -(self.__expected_charge_current + self.__current_range)
            low_current_range = -(self.__expected_charge_current - self.__current_range)
            if (low_current_range < float(meas_ibatt[0]) < high_current_range):
                txt = "Current value %f is out of range => Wrong charge current" % meas_ibatt[0]
                self._logger.error(txt)
                raise DeviceException(DeviceException.OPERATION_FAILED, txt)
            else:
                txt = "Current value %f is good" % meas_ibatt[0]
                self._logger.info(txt)

        elif (self.__measure_type == "RATIO"):

            # Measure current from Vusb
            meas_icharger = self.em_core_module.get_charger_current(self.__charger_type)

            # Measure current from Vbatt
            meas_ibatt = self.em_core_module.get_battery_current()

            if ((float(meas_ibatt[0]) < 0) and (float(meas_icharger[0]) > 0)):
                ratio = abs((float(meas_ibatt[0]) / float(meas_icharger[0])) * 100)
            else:
                txt = "Battery (%f) and Charger (%f) current is not OK (Maybe current is not in the range and the operation will not possible) => Quit the test" % (float(meas_ibatt[0]), float(meas_icharger[0]))
                raise AcsConfigException(AcsConfigException.OPERATION_FAILED, txt)

            self._logger.info("Ratio value is equal to %s" % str(ratio))

            if (ratio < self.__expected_current_ratio):
                txt = "Ratio current %s is below ratio theshold" % str(ratio)
                self._logger.error(txt)
                raise DeviceException(DeviceException.OPERATION_FAILED, txt)
        else:
            txt = "Measure type %s is not correct => Quit the test" % self.__measure_type
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, txt)

        return Global.SUCCESS, "no error"
