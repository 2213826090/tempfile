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
:summary: Energy Management usb over voltage Use case
:author: dbatutx
:since: 09/23/2011
"""
import time
from UtilitiesFWK.Utilities import Global, str_to_bool
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase


class LabEmUsbTestVoltage(EmUsecaseBase):

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

        # Read CHARGER_TYPE from TC parameters
        self._charger_type = self._tc_parameters.get_param_value("CHARGER_TYPE", "SDP")

        # Read idle state during the over or under voltage condition from TC parameters
        self._idle_state_during_test = str_to_bool(self._tc_parameters.get_param_value("IDLE_STATE"))
        # Read idle duration from TC parameters
        self._idle_duration = int(self._tc_parameters.get_param_value("IDLE_DURATION"))
        # Read over voltage duration from TC parameters
        self._test_voltage_duration = int(self._tc_parameters.get_param_value("TEST_VOLTAGE_DURATION"))

        # Read Vusb value from ConfigFile parameters
        self._vusb = float(self._tc_parameters.get_param_value("VUSB_NORMAL"))
        self._vusb_test_voltage = float(self._tc_parameters.get_param_value("VUSB_TEST"))

        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_USB_TEST_VOLTAGE", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets, self.tcd_to_test)

        # Set initial value for setting Power Supply VBATT:
        # - VoltageLevel = VBATT
        self.em_core_module.eqp_init_state["BATT"]["VoltageLevel"] = self.em_core_module.vbatt
        # - VoltageLevel = VUSB
        if self._vusb_test_voltage > self._vusb:
            self.em_core_module.get_eq_emulated_charger().set_voltage_protection_level(self._vusb_test_voltage + 4)
        else:
            self.em_core_module.get_eq_emulated_charger().set_voltage_protection_level(self._vusb + 4)
        self.em_core_module.eqp_init_state["USB"]["VoltageLevel"] = self.em_core_module.vusb

        # - Delay to wait for scheduled commands
        if self._idle_state_during_test:
            self._scheduled_timer_1 = self._test_voltage_duration + self._idle_duration
        else:
            self._scheduled_timer_1 = self._test_voltage_duration
        self._scheduled_timer_2 = self._scheduled_timer_1 + self._test_voltage_duration

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test:
        """
        EmUsecaseBase.set_up(self)

        # Set the brightness mode to manual
        self.phonesystem_api.set_brightness_mode("manual")
        time.sleep(5)

        # set brightness to 100%
        self.phonesystem_api.set_display_brightness(100)
        time.sleep(5)

        # Set screen timeout to 30 minutes
        self.phonesystem_api.set_screen_timeout(60 * 60)
        time.sleep(20)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        """
        # Call LAB_EM_BASE Run function
        EmUsecaseBase.run_test_body(self)

        self._logger.debug("        run API get_misc_register")

        # Set API to start in x seconds to read and store all MSIC registers
        time1_before = time.time()
        pid1 = self.em_api.get_msic_registers("scheduled", self._scheduled_timer_1)
        time1_after = time.time()

        # Set API to start in x seconds to read and store all MSIC registers
        time2_before = time.time()
        pid2 = self.em_api.get_msic_registers("scheduled", self._scheduled_timer_2)
        time2_after = time.time()

        # disconnect usb and close connection
        self._io_card.usb_connector(False)
        self._device.disconnect_board()

        # Select usb device DCP and connect usb
        self._io_card.simulate_insertion(self._charger_type)

        # Change the charger voltage to the test voltage condition
        self.em_core_module.get_eq_emulated_charger().set_current_voltage(
            self._vusb_test_voltage, self.em_core_module.ps_properties["USB"]["PortNumber"])
        self._logger.debug("        set VUSB to %s" % self._vusb_test_voltage)
        time.sleep(1)

        # wake up or put the the board in idle upon test configuration
        if self._idle_state_during_test:
                self._io_card.press_power_button(0.3)

        # Wait time between command
        if (time.time() - time1_before) > self._scheduled_timer_1:
            return Global.FAILURE, "Test implementation incorrect (increase MSIC_REGISTER1 / scheduled_time constant)"
        self._wait_smart_time(time1_after, self._scheduled_timer_1)
        time.sleep(3)

        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()

        # Measure current from Vusb
        meas_iusb = self.em_core_module.get_charger_current(self._charger_type)

        self._logger.info(
            "Charger %s plugged : IBatt = %s %s , IUsb = %s %s " %
            (self._charger_type, meas_ibatt[0], meas_ibatt[1], meas_iusb[0], meas_iusb[1]))

        # Compare ibatt value with limit parameters
        self._meas_list.add("IBATT1", meas_ibatt)

        # Compare iusb value with limit parameters
        self._meas_list.add("IUSB1", meas_iusb)

        # Change back the charger voltage to the normal voltage condition
        self.em_core_module.get_eq_emulated_charger().set_current_voltage(
            self._vusb, self.em_core_module.ps_properties["USB"]["PortNumber"])
        self._logger.debug("        set VUSB to %s" % self._vusb)

        # Wait time between command
        if (time.time() - time2_before) > self._scheduled_timer_2:
            return Global.FAILURE, "Test implementation incorrect (increase MSIC_REGISTER2 / scheduled_time constant)"
        self._wait_smart_time(time2_after, self._scheduled_timer_2)

        # wake up the screen
        self._io_card.press_power_button(0.3)
        time.sleep(2)

        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()

        # Measure current from Vusb
        meas_iusb = self.em_core_module.get_charger_current(self._charger_type)

        self._logger.info(
            "Charger %s plugged : IBatt = %s %s , IUsb = %s %s " %
            (self._charger_type, meas_ibatt[0], meas_ibatt[1], meas_iusb[0], meas_iusb[1]))

        # Compare ibatt value with limit parameters
        self._meas_list.add("IBATT2", meas_ibatt)

        # Compare iusb value with limit parameters
        self._meas_list.add("IUSB2", meas_iusb)

        # Connect USB PC/Host and DUT
        self.em_core_module.check_board_connection(only_reconnect=True)

        self._logger.debug("        get misc register")

        # Read Platform OS and compare MSIC registers with expected values
        msic_registers = self.em_api.get_msic_registers("read", pid1)
        self._meas_list.add_dict("MSIC_REGISTER1", msic_registers,
                                 msic_registers["TIME_STAMP"][0])

        # Read Platform OS and compare MSIC registers with expected values
        msic_registers = self.em_api.get_msic_registers("read", pid2)
        self._meas_list.add_dict("MSIC_REGISTER2", msic_registers,
                                 msic_registers["TIME_STAMP"][0])

        # generate em verdict
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
        self._em_meas_verdict.judge()
        self._meas_list.clean()

        return self._em_meas_verdict.get_current_result_v2()
