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
:summary: Energy Management Battery Monitor - Low VBatt policy management
:author: arnaudx.paire
:since: 11/18/2011
"""
import time
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from UtilitiesFWK.Utilities import Global


class LabEmLowBattPolicyManagement(EmUsecaseBase):

    """
    Lab Energy Management Low Battery Policy Management class.
    """
    DEDICATED_BENCH = "POWER_SUPPLY_BENCH"

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LAB_EM_BASE Init function
        EmUsecaseBase.__init__(self, tc_name, global_config)

        # Read the TC parameters
        self._test_type = self._tc_parameters.get_param_value("TEST_TYPE")
        self._mos_th_wo = float(self._tc_parameters.get_param_value("MOS_WO_CHARGER_THRESHOLD"))
        self._mos_th_shutdown = float(self._tc_parameters.get_param_value("MOS_THRESHOLD_SHUTDOWN"))

        # Call ConfigsParser to parse Energy_Management
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_LOW_BATT_POLICY_MANAGEMENT", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

        # Redefine initial value for setting USBDIO:
        # - BatteryType = ANALOG
        self.em_core_module.io_card_init_state["BatteryType"] = self.phone_info["BATTERY"]["BATTID_TYPE"]
        # - Battery  = True (inserted)
        self.em_core_module.io_card_init_state["Battery"] = True
        # - Platform = True (ON)
        self.em_core_module.io_card_init_state["Platform"] = "OFF"
        # - USBChargerType = USB_HOST_PC
        self.em_core_module.io_card_init_state["USBChargerType"] = self._io_card.WALL_CHARGER
        # - USBCharger = False (removed)
        self.em_core_module.io_card_init_state["USBCharger"] = True
        self.em_core_module.io_card_init_state["ExtPwrSupply"] = True

        # Set initial value for setting Power Supply VBATT:
        # - VoltageLevel = VBATT
        self.em_core_module.eqp_init_state["BATT"]["VoltageLevel"] = self._mos_th_wo

    def set_up(self):
        """
        Initialize the test:
        boot boot in low vbatt case
        """
        EmUsecaseBase.set_up(self)

        # boot the board
        self._io_card.press_power_button(self.pwr_btn_boot)
        time.sleep(self._device.get_boot_timeout())

        # plug wall charger
        self.em_core_module.plug_charger(self._io_card.WALL_CHARGER, ext_ps=True)

        return Global.SUCCESS, "No errors"

    def run_test_body(self):
        """
        Execute the test
        """
        # Call LAB_EM_BASE Run function
        EmUsecaseBase.run_test_body(self)

        if self._test_type == "CRITICAL_BATT" or self._test_type == "BOTH":
            self._run_power_down_upon_charger_removal()

        if self._test_type == "BELOW_CRITICAL" or self._test_type == "BOTH":
            self._run_no_poweron_on_critical_vbatt()

        # generate em verdict
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
        self._meas_list.clean()

        # Save data report in xml file
        self._error.Code = self._em_meas_verdict.get_current_result()
        self._error.Msg = self._em_meas_verdict.save_data_report_file()

        return self._error.Code, self._error.Msg

    def _check_ibatt_icharger(self, target):
        """
        Read the current from Vbatt and Vcharger and check if the values
        correspond to the excepted ones

        :type target: str
        :param target: parameter suffix name in the limit file
        """
        # wait for a stable current
        time.sleep(5)

        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()

        # Measure current from Vusb
        meas_charger = self.em_core_module.get_charger_current(self._io_card.WALL_CHARGER)

        # Compare Vbatt value with limit parameters
        self._meas_list.add("IBATT_" + target, meas_ibatt)

        # Compare Vusb value with limit parameters
        self._meas_list.add("ICHARGER_" + target, meas_charger)

        self._logger.info("IBatt = %s %s , Icharger = %s %s " %
                          (meas_ibatt[0], meas_ibatt[1],
                           meas_charger[0], meas_charger[1]))

    def _check_ibatt(self, target):
        """
        Read the current from Vbatt and check if the value
        correspond to the excepted one

        :type target: str
        :param target: parameter suffix name in the limit file
        """
        # wait for a stable current
        time.sleep(5)

        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()

        # Compare Vbatt value with limit parameters
        self._meas_list.add("IBATT_" + target, meas_ibatt)

        self._logger.info("IBatt = %s %s." % (meas_ibatt[0], meas_ibatt[1]))

    def _check_power_off(self, max_wait_time, target):
        """
        Read the current from Vbatt and check if the value
        correspond to the power off
        Make this during  x time until the phone is switch off

        :type max_wait_time: int
        :param max_wait_time: max time to wait
        """
        end_time = time.time() + max_wait_time
        criteria = float(self._em_targets[target]["hi_lim"])
        while time.time() < end_time:
            # Measure current from Vbatt
            meas_ibatt = self.em_core_module.get_battery_current()
            self._logger.info("IBatt = %s %s." % (meas_ibatt[0], meas_ibatt[1]))
            if meas_ibatt[0] < criteria:
                self._logger.info("Phone is off : IBatt =" +
                                  " %s %s." % (meas_ibatt[0], meas_ibatt[1]))
                break
            self._logger.info("wait 2 min")
            time.sleep(self.em_core_module.wait_voltage_time)

    def _run_power_down_upon_charger_removal(self):
        """
        Run "CRITICAL_BATT" test
        """

        # Check the initial state
        self._check_ibatt_icharger("1_INIT")

        # Set the battery voltage under critical level
        self.em_core_module.pwrs_vbatt.set_current_voltage(self._mos_th_shutdown,
                                             self.em_core_module.ps_properties["BATT"]["PortNumber"])

        # Check the current before unplug
        self._check_ibatt_icharger("1_BEFORE_UNPLUG")

        # Unplug the charger
        self._logger.info("Unplug the Wall charger charger")
        self.em_core_module.unplug_charger(self._io_card.WALL_CHARGER)
        self._logger.info("Waiting  power off")
        self._check_power_off(90 * 60, "ICHARGER_1_AFTER_UNPLUG")

        # Check the current after unplug
        self._check_ibatt_icharger("1_AFTER_UNPLUG")

    def _run_no_poweron_on_critical_vbatt(self):
        """
        Run "BELOW_CRITICAL" test
        """

        # Battery insertion
        self._io_card.battery_connector(False)
        time.sleep(5)
        self._io_card.battery_connector(True)
        time.sleep(5)

        # Check that the charger is well unplugged
        self.em_core_module.unplug_charger(self._io_card.WALL_CHARGER)
        self._logger.info("Waiting 5 seconds")
        time.sleep(5)

        # Set Vbatt to critical
        self.em_core_module.pwrs_vbatt.set_current_voltage(self._mos_th_shutdown,
                                             self.em_core_module.ps_properties["BATT"]["PortNumber"])

        # Check the phone is switched off
        self._check_ibatt("2_INIT")

        # Press power button
        self._io_card.press_power_button(self.pwr_btn_boot)
        time.sleep(15)

        # Check the phone is still switched off
        self._check_ibatt("2_STILL_OFF")

        # Set Vbatt to MOS available
        self.em_core_module.pwrs_vbatt.set_current_voltage(self._mos_th_wo,
                                             self.em_core_module.ps_properties["BATT"]["PortNumber"])

        # Battery insertion
        self._io_card.battery_connector(False)
        time.sleep(5)
        self._io_card.battery_connector(True)
        time.sleep(5)

        # Press power button
        self._io_card.press_power_button(self.pwr_btn_boot)
        time.sleep(15)

        # Check the phone is still switched off
        self._check_ibatt("2_AFTER_POWER_BUTTON")
