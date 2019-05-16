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
:since: 01/20/2012
"""
import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase


class LabEmUsbHighVoltage(EmUsecaseBase):

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

        # Read VBATT from TC parameters
        self._batt_temp = int(self._tc_parameters.get_param_value("BATTERY_TEMPERATURE"))

        # Read Vusb value from ConfigFile parameters
        self._vusb_normal_voltage = float(self._tc_parameters.get_param_value(
            "VUSB_NORMAL"))
        self._vusb_high_voltage = float(self._tc_parameters.get_param_value(
            "VUSB_HIGH"))

        # Call ConfigsParser to parse Energy_Management
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_USB_HIGH_VOLTAGE", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

        # Redefine initial value for setting USBDIO:
        # - BatteryType = ANALOG
        self.em_core_module.io_card_init_state["BatteryType"] = self.phone_info["BATTERY"]["BATTID_TYPE"]
        # - Battery  = True (inserted)
        self.em_core_module.io_card_init_state["Battery"] = True
        # - Platform = True (ON)
        self.em_core_module.io_card_init_state["Platform"] = "ON"
        # - USBChargerType = USB_HOST_PC
        self.em_core_module.io_card_init_state["USBChargerType"] = "USB_HOST_PC"
        # - USBCharger = False (removed)
        self.em_core_module.io_card_init_state["USBCharger"] = True
        # - BatteryTemperature = BATTERY_TEMPERATURE
        self.em_core_module.io_card_init_state["BatteryTemperature"] = self._batt_temp

        # VBATT and VUSB power supplies
        self.em_core_module.pwrs_vbatt = self._em.get_power_supply("BATT")
        self.em_core_module.pwrs_vusb = self._em.get_power_supply("USB")

        # Set initial value for setting Power Supply VBATT:
        # - VoltageLevel = VBATT
        self.em_core_module.eqp_init_state["BATT"]["VoltageLevel"] = 3.8
        # - VoltageLevel = VUSB
        if self._vusb_high_voltage > self._vusb_normal_voltage:
            self.em_core_module.pwrs_vusb.set_voltage_protection_level(self._vusb_high_voltage + 6)
        else:
            self.em_core_module.pwrs_vusb.set_voltage_protection_level(self._vusb_normal_voltage + 6)
        self.em_core_module.eqp_init_state["USB"]["VoltageLevel"] = self._vusb_normal_voltage

        # - Delay to wait for scheduled commands
        self._scheduled_timer_1 = \
            int(self._em_targets["MSIC_REGISTER1"]["scheduled_time"])
        self._scheduled_timer_2 = \
            int(self._em_targets["MSIC_REGISTER2"]["scheduled_time"])

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

        # Close connection
        self._device.disconnect_board()

        # Select usb device DCP and connect usb
        self._io_card.simulate_insertion(self._io_card.DCP)

        # Wait time between command
        if (time.time() - time1_before) > self._scheduled_timer_1:
            return Global.FAILURE, "Test implementation incorrect (increase MSIC_REGISTER1 / scheduled_time constant)"
        self._wait_smart_time(time1_after, self._scheduled_timer_1)
        time.sleep(3)

        # Measure current from Vbatt
        meas_ibatt = (self.em_core_module.pwrs_vbatt.get_current_meas(
            self.em_core_module.ps_properties["BATT"]["PortNumber"], "DC"), "A")

        # Measure current from Vusb
        meas_iusb = (self.em_core_module.pwrs_vusb.get_current_meas(
            self.em_core_module.ps_properties["USB"]["PortNumber"], "DC"), "A")

        self._logger.info(
            "Charger DCP plugged : IBatt = %s %s , IUsb = %s %s " %
            (meas_ibatt[0], meas_ibatt[1], meas_iusb[0], meas_iusb[1]))

        # Compare ibatt value with limit parameters
        self._meas_list.add("IBATT1", meas_ibatt)

        # Compare iusb value with limit parameters
        self._meas_list.add("IUSB1", meas_iusb)

        # Change the power supply voltage to the second value
        self.em_core_module.pwrs_vusb.set_current_voltage(
            self._vusb_high_voltage, self.em_core_module.ps_properties["USB"]["PortNumber"])

        self._logger.debug("        set VUSB to %s" % self._vusb_high_voltage)

        # Wait time between command
        if (time.time() - time2_before) > self._scheduled_timer_2:
            return Global.FAILURE, "Test implementation incorrect (increase MSIC_REGISTER2 / scheduled_time constant)"
        self._wait_smart_time(time2_after, self._scheduled_timer_2)

        # Measure current from Vbatt
        meas_ibatt = (self.em_core_module.pwrs_vbatt.get_current_meas(
            self.em_core_module.ps_properties["BATT"]["PortNumber"], "DC"), "A")

        # Measure current from Vusb
        meas_iusb = (self.em_core_module.pwrs_vusb.get_current_meas(
            self.em_core_module.ps_properties["USB"]["PortNumber"], "DC"), "A")

        self._logger.info(
            "Charger DCP plugged : IBatt = %s %s , IUsb = %s %s " %
            (meas_ibatt[0], meas_ibatt[1], meas_iusb[0], meas_iusb[1]))

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
        self._meas_list.clean()

        # Save data report in xml file
        self._error.Code = self._em_meas_verdict.get_current_result()
        self._error.Msg = self._em_meas_verdict.save_data_report_file()

        return self._error.Code, self._error.Msg
