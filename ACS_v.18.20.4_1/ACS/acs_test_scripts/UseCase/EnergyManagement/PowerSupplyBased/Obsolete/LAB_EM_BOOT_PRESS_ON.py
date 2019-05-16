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
:summary: Energy Management battery discharge monitoring Use case
:author: dbatutx
:since: 09/30/2011
"""
import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase


class LabEmBootPressOn(EmUsecaseBase):

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

        # Read BATTERY_TEMPERATURE from TC parameters
        self._vcritical = float(self._tc_parameters.get_param_value("VBATT_CRITICAL"))

        # Call ConfigsParser to parse Energy_Management
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_BOOT_PRESS_ON", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

        # Redefine initial value for setting USBDIO:
        # - Battery  = True (inserted)
        self.em_core_module.io_card_init_state["Battery"] = True
        # - Platform = True (ON)
        self.em_core_module.io_card_init_state["Platform"] = "ON"

        # Set initial value for setting Power Supply VBATT:
        # - VoltageLevel = VBATT
        self.em_core_module.eqp_init_state["BATT"]["VoltageLevel"] = self.em_core_module.vbatt

        # Retrieve the power supply equipment
        self.em_core_module.pwrs_vbatt = self._em.get_power_supply("BATT")

#------------------------------------------------------------------------------
    def run_test_body(self):
        """
        Execute the test
        """
        # Call LAB_EM_BASE Run function
        EmUsecaseBase.run_test_body(self)

        # Run the sub test about power on with power button
        status = self._boot_on_power_button()
        if status != Global.SUCCESS:
            self._logger.error("Phone has can't start on power button press. "
                               + "Test STOPS!")
            # Save data report in xml file
            self._error.Code = self._em_meas_verdict.get_global_result()
            self._error.Msg = self._em_meas_verdict.save_data_report_file()

            return self._error.Code, self._error.Msg

        # Run the sub test about User Alarm
        self._no_boot_on_alarm()

        # Run the sub test about Not booting under critical Voltage
        self._no_boot_under_critical_voltage()

        # generate em verdict
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
        self._meas_list.clean()

        self._error.Code = self._em_meas_verdict.get_current_result()
        self._error.Msg = self._em_meas_verdict.save_data_report_file()

        return self._error.Code, self._error.Msg

    def _boot_on_power_button(self):

        # press on power ON 8s to perform a hard reset and switch off the phone
        self._io_card.press_power_button(8)
        time.sleep(15)

        # Check that the power supply doesn't deliver power (phone should be off)
        meas_ibatt = (self.em_core_module.pwrs_vbatt.get_current_meas(
            self.em_core_module.ps_properties["BATT"]["PortNumber"], "DC"), "A")
        self._logger.info("Phone should be OFF : IBatt = %s %s" %
                          (meas_ibatt[0], meas_ibatt[1]))
        self._meas_list.add("IBATT1_OFF", meas_ibatt)

        # press on power ON to start the platform
        self._io_card.press_power_button(self.pwr_btn_boot)
        time.sleep(30)

        # Measure current from Vbatt
        meas_ibatt = (self.em_core_module.pwrs_vbatt.get_current_meas(
            self.em_core_module.ps_properties["BATT"]["PortNumber"], "DC"), "A")

        self._logger.info("After power button key press : IBatt = %s %s" %
                          (meas_ibatt[0], meas_ibatt[1]))

        # Compare ibatt value with limit parameters
        self._meas_list.add("IBATT1_ON1", meas_ibatt)
        self._meas_list.add("IBATT1_ON2", meas_ibatt)
        self._meas_list.add("IBATT1_ON3", meas_ibatt)

        if float(self._em_targets["IBATT1_OFF"]["hi_lim"]) > meas_ibatt[0] > float(self._em_targets["IBATT1_OFF"]["lo_lim"]):
            return Global.FAILURE

        return Global.SUCCESS

    def _no_boot_on_alarm(self):
        min_offset = 2

        # Set user alarm in 2 minutes
        self.em_core_module.check_board_connection(only_reconnect=True)
        self.phonesystem_api.set_user_alarm_in_x_min(min_offset)
        self._logger.info("User alarm should be set")
        self._device.disconnect_board()
        self._io_card.usb_connector(False)

        # Switch off the platform
        self._logger.info("Switching off the phone")
        self._io_card.press_power_button(10)
        time.sleep(20)

        # Control that the board is well OFF
        meas_ibatt = (self.em_core_module.pwrs_vbatt.get_current_meas(
            self.em_core_module.ps_properties["BATT"]["PortNumber"], "DC"), "A")
        self._logger.info("Phone should be OFF : IBatt = %s %s" %
                          (meas_ibatt[0], meas_ibatt[1]))
        self._meas_list.add("IBATT2_OFF", meas_ibatt)

        # Wait for the User alarm time to expire
        self._logger.info("Wait for the user alarm")
        time.sleep(min_offset * 60)
        meas_ibatt = (self.em_core_module.pwrs_vbatt.get_current_meas(
            self.em_core_module.ps_properties["BATT"]["PortNumber"], "DC"), "A")
        self._logger.info("Phone should still be OFF : IBatt = %s %s" %
                          (meas_ibatt[0], meas_ibatt[1]))
        self._meas_list.add("IBATT2_STILL_OFF", meas_ibatt)

    def _no_boot_under_critical_voltage(self):

        # Battery insertion at a level lower to critical voltage
        self._io_card.battery_connector(False)
        critical_voltage = self._vcritical - 0.1
        self.em_core_module.pwrs_vbatt.set_current_voltage(critical_voltage,
                                             self.em_core_module.ps_properties['BATT']['PortNumber'])
        time.sleep(2)
        self._io_card.battery_connector(True)

        self._logger.info("wait 20 sec")
        time.sleep(20)

        # 1st check of the power consumption
        meas_ibatt = (self.em_core_module.pwrs_vbatt.get_current_meas(
            self.em_core_module.ps_properties["BATT"]["PortNumber"], "DC"), "A")
        self._logger.info("Battery critical : IBatt = %s %s" %
                          (meas_ibatt[0], meas_ibatt[1]))
        self._meas_list.add("IBATT3_OFF", meas_ibatt)

        # Press power button
        self._io_card.press_power_button(self.pwr_btn_boot)

        self._logger.info("wait 30 sec")
        time.sleep(30)

        # 1st check of the power consumption
        meas_ibatt = (self.em_core_module.pwrs_vbatt.get_current_meas(
            self.em_core_module.ps_properties["BATT"]["PortNumber"], "DC"), "A")
        self._logger.info("Battery critical after power button press : "
                          + "IBatt = %s %s" % (meas_ibatt[0], meas_ibatt[1]))
        self._meas_list.add("IBATT3_STILL_OFF", meas_ibatt)
