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
:summary: Energy Management press shutdown while phone is on Use case
:author: dbatutx
:since: 05/14/2012
"""
import time
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase


class LabEmPsForcedShutdown(EmUsecaseBase):

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
        self._batt_temp = \
            int(self._tc_parameters.get_param_value("BATTERY_TEMPERATURE"))

        # Read power button time to press in a HardWare shutdown
        self._HW_shutdown_time = \
            float(self._tc_parameters.get_param_value("HW_SHUTDOWN_PRESS_TIME"))

        # Call ConfigsParser to parse Energy_Management
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_PS_FORCED_SHUTDOWN", self._tc_parameters.get_params_as_dict(),
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
        self.em_core_module.io_card_init_state["USBChargerType"] = self._io_card.SDP
        # - USBCharger = False (removed)
        self.em_core_module.io_card_init_state["USBCharger"] = True
        # - BatteryTemperature = BATTERY_TEMPERATURE
        self.em_core_module.io_card_init_state["BatteryTemperature"] = self._batt_temp

        # Set initial value for setting Power Supply VBATT:
        # - VoltageLevel = VBATT
        self.em_core_module.eqp_init_state["BATT"]["VoltageLevel"] = self.em_core_module.vbatt

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        """
        # Call LAB_EM_BASE Run function
        EmUsecaseBase.run_test_body(self)

        # wake up the board if necessary
        if not self.phonesystem_api.get_screen_status():
            # unplug SDP
            self.em_core_module.unplug_charger(self._io_card.SDP)
            self._device.disconnect_board()
            self._io_card.press_power_button(0.3)
        else:
            # unplug SDP
            self.em_core_module.unplug_charger(self._io_card.SDP)
            self._device.disconnect_board()

        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()
        # Compare ibatt value with limit parameters
        self._meas_list.add("IBATT1", meas_ibatt)

        #
        # Hardware shutdown sequence
        #
        # PUSH power button for a hardware shutdown
        # freeze the board before in order to avoid a graceful shutdown betwwen 4s and 7s
        self.em_core_module.plug_charger("SDP")
        self._device.connect_board()
        self.phonesystem_api.freeze_board("SOFT")
        time.sleep(5)
        self._device.disconnect_board()
        self.em_core_module.unplug_charger("SDP")
        # do the Hardware shutdown
        before_shutdown_time = time.time()
        self._logger.info("        press on %s" % self._HW_shutdown_time)
        self._io_card.press_power_button(self._HW_shutdown_time)
        time.sleep(10)
        # Measure current from Vbatt
        meas_ibatt = (self.em_core_module.pwrs_vbatt.get_current_meas(
            self.em_core_module.ps_properties["BATT"]["PortNumber"], "DC"), "A")
        self._logger.info("        state after 'HW shutdown' : IBatt2 = %s %s " %
                         (meas_ibatt[0], meas_ibatt[1]))
        # Compare ibatt value with limit parameters
        self._meas_list.add("IBATT2", meas_ibatt)
        result = self._em_meas_verdict.test_value(self._meas_list,
                                                  self._em_targets["IBATT2"])
        if not result:
            # disconnect battery
            self._io_card.battery_connector(False)
            time.sleep(3)
            # connect battery
            self._io_card.battery_connector(True)
            time.sleep(3)

        #
        # Power On sequence
        #
        # PUSH ON button 3s
        self._io_card.press_power_button(self.pwr_btn_boot)
        time.sleep(self._device.get_boot_timeout())
        # connect SDP
        self.em_core_module.plug_charger(self._io_card.SDP)
        self.em_core_module.check_board_connection(use_exception=False, only_reconnect=True)
        after_shutdown_time = time.time()
        # Measure current from Vbatt
        self.em_core_module.unplug_charger(self._io_card.SDP)
        meas_ibatt = self.em_core_module.get_battery_current()
        self._logger.info("        state should be on : IBatt3 = %s %s " %
                         (meas_ibatt[0], meas_ibatt[1]))
        # Compare ibatt value with limit parameters
        self._meas_list.add("IBATT3", meas_ibatt)
        result = self._em_meas_verdict.test_value(self._meas_list,
                                                  self._em_targets["IBATT3"])
        # boot the board if the board is not on
        if not result:
            self.em_core_module.reboot_board()
        #  connect USB SDP
        self.em_core_module.plug_charger(self._io_card.SDP)
        self.em_core_module.check_board_connection(only_reconnect=True)
        time.sleep(5)
        # check shutdown confirmation
        confirm = self.phonesystem_api.check_message_in_log("SHUTDOWN_REASON",
                                                             before_shutdown_time, after_shutdown_time, True)
        # Compare shutdown confirmation value with limit parameters
        self._meas_list.add("HARDWARE_SHUTDOWN", (confirm[0], "none"))
        # check for android boot wake src
        confirm = self.phonesystem_api.check_message_in_log("BOOT_REASON",
                                                             before_shutdown_time, after_shutdown_time, True)
        # Compare Vbatt value with limit parameters
        self._meas_list.add("BOOT_REASON", (confirm[0], "none"))
        # check for android boot mode
        confirm = self.phonesystem_api.check_message_in_log("BOOT_MODE",
                                                             before_shutdown_time, after_shutdown_time, True)
        # Compare boot mode value with limit parameters
        self._meas_list.add("BOOT_MODE", (confirm[0], "none"))

        # generate em verdict
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
        self._meas_list.clean()

        # Save data report in xml file
        self._error.Code = self._em_meas_verdict.get_current_result()
        self._error.Msg = self._em_meas_verdict.save_data_report_file()

        return self._error.Code, self._error.Msg
