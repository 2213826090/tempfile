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
:summary: Remove USB Charger when board is in COS
:author: jvauchex
:since: 07/16/2014
"""
import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import str_to_bool


class LabEmPsCosDeadBattUsbPlugged(EmUsecaseBase):

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
        self.__cos_boot_timeout_value = int(self._tc_parameters.get_param_value("COS_BOOT_TIMEOUT_VALUE", 120))
        self.__hardware_boot_timeout_value = int(self._tc_parameters.get_param_value("HARDWARE_BOOT_TIMEOUT_VALUE", 60))
        self.__power_button_press_activation = str_to_bool(self._tc_parameters.get_param_value("POWER_BUTTON_PRESS_ACTIVATION", False))
        self.__charging_os_voltage = float(self._tc_parameters.get_param_value("CHARGING_OS_VOLTAGE", 3.2))
        self.__hardware_charging_voltage = float(self._tc_parameters.get_param_value("HARDWARE_CHARGING_VOLTAGE", 2.5))
        self.__hardware_charge_current = float(self._tc_parameters.get_param_value("HARDWARE_CHARGE_CURRENT", 0.4))

        # Redefine initial value for setting USBDIO:
        # - Platform
        self.em_core_module.io_card_init_state["Platform"] = "OFF"

        # Set initial value for setting Power Supply VBATT:
        # - VoltageLevel = VBATT
        self.em_core_module.eqp_init_state["BATT"]["VoltageLevel"] = self.__hardware_charging_voltage

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        EmUsecaseBase.set_up(self)
        # plug the charger type
        self._io_card.simulate_insertion(self.__charger_type)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        """
        # Call LAB_EM_BASE_PS Run function
        EmUsecaseBase.run_test_body(self)

        # Wait the Hardware charging Setup
        self._logger.info("Wait %ss for hardware charging mode" % self.__hardware_boot_timeout_value)
        time.sleep(self.__hardware_boot_timeout_value)

        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()

        # Check if the hardware charging is ok
        current_threshold = -(self.__hardware_charge_current)
        if (meas_ibatt[0] > current_threshold):
            txt = "Current value %s is below Hardware current Threshold %s => Hardware Charging Failed" \
                  % (str(meas_ibatt[0]), str(current_threshold))
            self._logger.error(txt)
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, txt)

        # Second Part => Test the Charging OS
        # set the vbatt voltage to the Charging OS threshold
        self.em_core_module.get_eq_emulated_battery().set_current_voltage(self.__charging_os_voltage,
                                             self.em_core_module.ps_properties["BATT"]["PortNumber"])

        # Check the power button activation
        if (self.__power_button_press_activation == True):
            self._io_card.press_power_button(3)

        # Wait the Boot of the board => Charging OS Expected
        self._logger.info("Wait %ss for booting board" % self.__cos_boot_timeout_value)
        mode = self._device.get_boot_mode()
        end_time = time.time() + self.__cos_boot_timeout_value
        while (time.time() < end_time) and (mode == "UNKNOWN"):
            mode = self._device.get_boot_mode()
            time.sleep(1)

        # Check if the board boots in COS
        if mode != "UNKNOWN":

            if mode != "COS":
                txt = "Board does not boot in COS before the COS Timeout. boot %s" % mode
                self._logger.error(txt)
                raise DeviceException(DeviceException.OPERATION_FAILED, txt)
            else:
                txt = "Board boot mode is COS => This test is PASS"
                self._logger.info(txt)

        else:
            txt = "Board boot mode is UNKNOWN => Problem during the boot"
            self._logger.error(txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, txt)

        return self._error.Code, self._error.Msg
