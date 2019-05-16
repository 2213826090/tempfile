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
:summary: Energy Management hardware charging Use case
:author: apairex
:since: 11/22/2011
"""
import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from ErrorHandling.AcsConfigException import AcsConfigException


class LabEmMonitorBattChargeRate(EmUsecaseBase):

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

        self._vbatt_mos = float(self._tc_parameters.get_param_value(
            "VBATT_START_MOS"))
        self._vbatt_init = float(self._tc_parameters.get_param_value(
            "VBATT_INIT"))
        self._vbatt_list = self._tc_parameters.get_param_value("VBATT_LIST").split(";")

        # Call ConfigsParser to parse Energy_Management
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_MONITOR_BATT_CHARGE_RATE", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

        # Redefine initial value for setting USBDIO:
        # - BatteryType = ANALOG
        self.em_core_module.io_card_init_state["BatteryType"] = self.phone_info["BATTERY"]["BATTID_TYPE"]
        # - Battery  = False (removed)
        self.em_core_module.io_card_init_state["Battery"] = True
        # - Platform = False (OFF)
        self.em_core_module.io_card_init_state["Platform"] = "ON"
        # - USBChargerType = DCP
        self.em_core_module.io_card_init_state["USBChargerType"] = "SDP"
        # - USBCharger = True (inserted)
        self.em_core_module.io_card_init_state["USBCharger"] = True
        # - BatteryTemperature = 25
        self.em_core_module.io_card_init_state["BatteryTemperature"] = 25

        # VBATT and VUSB power supplies
        self.em_core_module.eqp_init_state["BATT"]["VoltageLevel"] = self._vbatt_init

        self._charger_type = self._io_card.WALL_CHARGER

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

        # wake up the board if necessary
        if not self.phonesystem_api.get_screen_status():
            self._io_card.press_power_button(0.3)

        # unplug the SDP
        self.em_core_module.unplug_charger("SDP")

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        """
        EmUsecaseBase.run_test_body(self)

        self._run_charging_sequence_control()
        self._run_cc_cv_charge_mode()

        # generate em verdict
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
        self._meas_list.clean()

        # Save data report in xml file
        self._error.Code = self._em_meas_verdict.get_current_result()
        self._error.Msg = self._em_meas_verdict.save_data_report_file()

        return self._error.Code, self._error.Msg

    def _run_cc_cv_charge_mode(self):
        """
        Platform shall support a CC-CV charge mode
        """
        prev_current = 0

        # Call Control battery capacity to avoid OCV effect
        self.em_core_module.control_battery_capacity(30, 2000)

        # Connect the WALL_CHARGER charger
        self._device.disconnect_board()
        self.em_core_module.plug_charger(self._charger_type, ext_ps=True)
        if self._vbatt_list != ['']:
            for vbatt in self._vbatt_list:

                self._logger.info("Starting %s V test" % vbatt)

                # Control that the limit file contains the datas for this vbatt val
                key = "MSIC_REGISTER_" + vbatt
                if key not in self._em_targets:
                    self._logger.error("NO data in limit file "
                                       + "_Configs/EnergyManagement.xml "
                                       + "for %s Vbatt" % vbatt)
                    continue

                # Then we can start the phone in MOS
                self.em_core_module.pwrs_vbatt.set_current_voltage(float(vbatt),
                                                     self.em_core_module.ps_properties["BATT"]["PortNumber"])
                # disconnect battery
                self._io_card.battery_connector(False)
                time.sleep(30)
                # connect battery
                self._io_card.battery_connector(True)
                time.sleep(0.3)
                # press power on button during 3 seconds
                self._io_card.press_power_button(self.pwr_btn_boot)
                # wait & check current
                time.sleep(30)

                # Plug USB Host cable
                result = self.em_core_module.check_board_connection(use_exception=False, only_reconnect=True)

                if not result:
                    self.em_core_module.unplug_charger("SDP")
                    time.sleep(20)
                    self._logger.info("the board failed to boot at vbatt = %s" % vbatt)
                    continue

                # Set a schedule MSIC reading
                schedule_time = int(self._em_targets[key]["scheduled_time"])
                time_before = time.time()
                pid = self.em_api.get_msic_registers("scheduled", "%d"
                                                      % schedule_time)
                time_after = time.time()

                # wake up the board if necessary
                if not self.phonesystem_api.get_screen_status():
                    self._io_card.press_power_button(0.3)

                # Connect the DCP charger
                self._device.disconnect_board()
                self.em_core_module.plug_charger(self._charger_type, ext_ps=True)

                # Wait for the scheduled MSIC register read to occur
                if (time.time() - time_before) > schedule_time:
                    tmp_txt = "Test implementation incorrect (increase " \
                        + key + " / scheduled_time constant)"
                    self._logger.error(tmp_txt)
                    raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, tmp_txt)

                # Check initial value of the current during the waiting time
                time.sleep(5)
                self.__check_ibatt_icharger(vbatt)

                self._wait_smart_time(time_after, schedule_time)

                # Reconnect SDP HOST
                self.em_core_module.check_board_connection(only_reconnect=True)

                # Retrieve MSIC information
                msic_registers = self.em_api.get_msic_registers("read", pid)
                msic_registers["BATTERY"]["CURRENT_DELTA"] = \
                    (msic_registers["BATTERY"]["CURRENT_NOW"][0] - prev_current,
                     'none')
                self._meas_list.add_dict(key, msic_registers,
                                         msic_registers["TIME_STAMP"][0])
                prev_current = msic_registers["BATTERY"]["CURRENT_NOW"][0]

                # Disconnect phone
                self._device.disconnect_board()
                self.em_core_module.unplug_charger(self._charger_type)

    def _change_vbatt(self, old, new):
        """
        Progressive modification of the Battery Voltage
        simulated by the power supply

        :type old: float
        :param old: Current value of the voltage set to the power supply
        :type new: float
        :param new: Value to set the voltage to the power supply
        """
        now = old

        if old < new:
            delta = 0.005
        else:
            delta = -0.005

        while abs(now - new) > 0.005:
            now += delta
            self.em_core_module.pwrs_vbatt.set_current_voltage(now,
                                                 self.em_core_module.ps_properties["BATT"]["PortNumber"])
            time.sleep(60)

        self.em_core_module.pwrs_vbatt.set_current_voltage(new,
                                             self.em_core_module.ps_properties["BATT"]["PortNumber"])

    def _run_charging_sequence_control(self):
        """
        - A battery charging sequence is automatically initiated upon
        insertion of a charger (except in failure cases)
        - Platform shall indicate to the user that a charging
        sequence is in progress
        """
        # Time to set the scheduled MSIC register read
        schedule_time = int(self._em_targets["MSIC_REGISTER"]["scheduled_time"])

        # Check initial value of the current
        self.__check_ibatt_icharger("INIT")

        # plug SDP
        self.em_core_module.plug_charger("SDP")

        # Plug USB Host cable
        self.em_core_module.check_board_connection(only_reconnect=True)

        # put the board in idle if necessary
        if self.phonesystem_api.get_screen_status():
            self._io_card.press_power_button(0.3)

        # Set a schedule MSIC reading
        interrupt_ref = self.em_api.get_proc_interrupt()
        time_before = time.time()
        pid1 = self.em_api.get_msic_registers("scheduled", "%d" % schedule_time)
        pid2 = self.em_api.get_proc_interrupt("scheduled", "%d" % schedule_time)
        time_after = time.time()

        # wake up the board if necessary
        if not self.phonesystem_api.get_screen_status():
            self._io_card.press_power_button(0.3)

        # Connect the DCP charger
        self._device.disconnect_board()
        self.em_core_module.plug_charger(self._charger_type, ext_ps=True)

        # Wait for the scheduled MSIC register read to occur
        if (time.time() - time_before) > schedule_time:
            tmp_txt = "Test implementation incorrect (increase " \
                + "MSIC_REGISTER1 / scheduled_time constant)"
            self._logger.error(tmp_txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, tmp_txt)

        # Check initial value of the current during the waiting time
        time.sleep(5)
        self.__check_ibatt_icharger("DCP_PLUG")

        self._wait_smart_time(time_after, schedule_time)

        # Reconnect SDC HOST
        self.em_core_module.check_board_connection(only_reconnect=True)

        # Retrieve MSIC information
        msic_registers = self.em_api.get_msic_registers("read", pid1)
        self._meas_list.add_dict("MSIC_REGISTER", msic_registers,
                                 msic_registers["TIME_STAMP"][0])
        interrupts = self.em_api.get_proc_interrupt("read", pid2)
        int_meas = (interrupts - interrupt_ref, "none")
        self._meas_list.add("INTERRUPTS", int_meas)

        # Disconnect phone
        self._device.disconnect_board()
        self.em_core_module.unplug_charger()

    def __check_ibatt_icharger(self, target):
        """
        Read the current from Vbatt and Vcharger and check if the values
        correspond to the excepted ones

        :type target: str
        :param target: Suffix of the target in the XML limit file
        """

        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()

        # Measure current from Vcharger
        meas_icharger = self.em_core_module.get_charger_current(self._charger_type)

        # Compare Vbatt value with limit parameters
        self._meas_list.add("IBATT_" + target, meas_ibatt)

        # Compare Vcharger value with limit parameters
        if meas_icharger[0] != 'None':
            self._meas_list.add("ICHARGER_" + target, meas_icharger)

        self._logger.info("IBatt = %s %s , ICharger = %s %s " %
                          (meas_ibatt[0], meas_ibatt[1],
                           meas_icharger[0], meas_icharger[1]))
