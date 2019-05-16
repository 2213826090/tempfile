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
:since: 11/18/2011
"""
import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase


class LabEmPressShutdown(EmUsecaseBase):

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

        # Call ConfigsParser to parse Energy_Management
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_PRESS_SHUTDOWN", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

        # Redefine initial value for setting USBDIO:
        # - BatteryType
        self.em_core_module.io_card_init_state["BatteryType"] = self.phone_info["BATTERY"]["BATTID_TYPE"]
        # - Battery
        self.em_core_module.io_card_init_state["Battery"] = True
        # - Platform
        self.em_core_module.io_card_init_state["Platform"] = "ON"
        # - USBChargerType
        self.em_core_module.io_card_init_state["USBChargerType"] = "USB_HOST_PC"
        # - USBCharger
        self.em_core_module.io_card_init_state["USBCharger"] = True
        # - BatteryTemperature = BATTERY_TEMPERATURE
        self.em_core_module.io_card_init_state["BatteryTemperature"] = self._batt_temp

        # Set initial value for setting Power Supply VBATT:
        # - VoltageLevel = VBATT
        self.em_core_module.eqp_init_state["BATT"]["VoltageLevel"] = self.em_core_module.vbatt

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

        # wake up the board if necessary
        if not self.phonesystem_api.get_screen_status():
            self._io_card.press_power_button(0.3)
        # unplug USB SDP
        self._io_card.usb_connector(False)
        self._device.disconnect_board()
        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()
        self._logger.info("        initial state : IBatt1 = %s %s " %
                         (meas_ibatt[0], meas_ibatt[1]))
        # Compare ibatt value with limit parameters
        self._meas_list.add("IBATT1", meas_ibatt)

        #
        # NO WINDOW sequence
        #
        self.em_core_module.check_board_connection(use_exception=False, only_reconnect=True)
        # wake up the board if necessary
        if not self.phonesystem_api.get_screen_status():
            self._io_card.press_power_button(0.3)
        time.sleep(2)
        # record time before activate window
        before_window_time = time.time()
        time.sleep(5)
        # PUSH ON button 1s
        self._logger.info("        press on 0.45s")
        self._io_card.press_power_button(0.45)
        # check for window
        time.sleep(5)
        window = None
        # try to check menu from UI
        if self.phonesystem_api.is_uiautomator_present():
            window = str(self.phonesystem_api.is_ui_element_viewable("POWER_OFF_MENU")).upper()
        # else check from logs
        else:
            window = self.phonesystem_api.check_message_in_log("WINDOW",
                                                                before_window_time)[0]
        # Compare window presence with limit parameters
        self._meas_list.add("WINDOW1", (window, "none"))
        # put the board in idle mode
        self.phonesystem_api.set_phone_lock(1)
        time.sleep(5)

        #
        # WINDOW + CANCEL sequence
        #
        # wake up the board if necessary
        window = None
        if not self.phonesystem_api.get_screen_status():
            self._io_card.press_power_button(0.3)
        # record time before activate window
        before_window_time = time.time()
        time.sleep(2)
        # PUSH ON button 3
        self._logger.info("        press on 0.8s")
        self._io_card.press_power_button(0.8)
        # record time for check_mesage function
        before_cancel_time = time.time()
        # select cancel in power off menu
        time.sleep(5)
        # try to check menu from UI
        if self.phonesystem_api.is_uiautomator_present():
            window = str(self.phonesystem_api.is_ui_element_viewable("POWER_OFF_MENU")).upper()

        self.phonesystem_api.select_pwoff_menu("cancel")
        time.sleep(2)
        after_cancel_time = time.time()
        # else check from logs
        if window is None:
            window = self.phonesystem_api.check_message_in_log("WINDOW",
                                                                before_window_time)[0]
        # unplug USB SDP
        self._io_card.usb_connector(False)
        self._device.disconnect_board()
        # Measure current from Vbatt
        time.sleep(10)
        meas_ibatt = self.em_core_module.get_battery_current()
        self._logger.info("        state after 'cancel' : IBatt2 = %s %s " %
                         (meas_ibatt[0], meas_ibatt[1]))
        # connect SDP
        self.em_core_module.check_board_connection(use_exception=False, only_reconnect=True)
        # check for shutdown confirm "no"
        time.sleep(2)
        confirm = self.phonesystem_api.check_message_in_log("SHT_CONFIRM",
                                                             before_cancel_time, after_cancel_time, True)
        # Compare shutdown confirmation value with limit parameters
        self._meas_list.add("SHUTDOWN_CONFIM_NO", (confirm[0], "none"))
        # Compare window presence with limit parameters
        self._meas_list.add("WINDOW2", (window, "none"))
        # Compare ibatt value with limit parameters
        self._meas_list.add("IBATT2", meas_ibatt)
        result = self._em_meas_verdict.test_value(self._meas_list,
                                                  self._em_targets["IBATT2"])
        # put the board in idle mode
        self.phonesystem_api.set_phone_lock(1)
        time.sleep(5)
        if not result:
            self.em_core_module.reboot_board()

        #
        # Select 'OK'+ shutdown sequence
        #
        # wake up the board if necessary
        if not self.phonesystem_api.get_screen_status():
            self._io_card.press_power_button(0.3)
        time.sleep(2)
        # PUSH ON button 3s
        self._logger.info("        press on 3s")
        self._io_card.press_power_button(3)
        # record time for shutdown time
        before_shutdown_time = time.time()
        time.sleep(5)
        # select "ok" in power off menu
        self.phonesystem_api.select_pwoff_menu("ok")
        # unplug USB SDP
        self._io_card.usb_connector(False)
        self._device.disconnect_board()
        # wait for the power off
        pw_off_time = None
        wait_pw_off = time.time() + 120
        while wait_pw_off > time.time():
            time.sleep(1)
            meas_ibatt = self.em_core_module.get_battery_current()
            if meas_ibatt[0] < self._em_targets["IBATT3"]["hi_lim"]:
                pw_off_time = time.time()
                self._logger.info("The board shutdown at : %s " % pw_off_time)
                break
        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()
        self._logger.info("        state after 'ok' : IBatt3 = %s %s " %
                         (meas_ibatt[0], meas_ibatt[1]))
        # Compare ibatt value with limit parameters
        self._meas_list.add("IBATT3", meas_ibatt)
        result = self._em_meas_verdict.test_value(self._meas_list,
                                                  self._em_targets["IBATT3"])
        # switch off the board if the board is on
        if not result:
            self._io_card.press_power_button(10)
            time.sleep(10)

        #
        # boot board sequence + get result
        #
        # PUSH ON button 1.1s
        self._io_card.press_power_button(1.1)
        time.sleep(10)
        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()
        self._logger.info("        state should be off : IBatt4 = %s %s " %
                         (meas_ibatt[0], meas_ibatt[1]))
        # Compare ibatt value with limit parameters
        self._meas_list.add("IBATT4", meas_ibatt)
        result = self._em_meas_verdict.test_value(self._meas_list,
                                                  self._em_targets["IBATT4"])
        # switch off the board if the board is on
        if not result:
            self._io_card.press_power_button(10)
            time.sleep(10)
        # PUSH ON button 2.1s
        before_power_on_time = time.time()
        self._io_card.press_power_button(2.1)
        time.sleep(self._device.get_boot_timeout())
        after_power_on_time = time.time()
        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()

        self._logger.info("        state should be on : IBatt5 = %s %s " %
                         (meas_ibatt[0], meas_ibatt[1]))
        # Compare ibatt value with limit parameters
        self._meas_list.add("IBATT5", meas_ibatt)
        result = self._em_meas_verdict.test_value(self._meas_list,
                                                  self._em_targets["IBATT5"])
        # boot the board if the board is not on
        if not result:
            self.em_core_module.reboot_board()

        # connect SDP
        self.em_core_module.check_board_connection(use_exception=False, only_reconnect=True)
        # check shutdown confirmation
        confirm = self.phonesystem_api.check_message_in_log("SHT_CONFIRM",
                                                             before_shutdown_time, pw_off_time, True)
        # Compare shutdown confirmation value with limit parameters
        self._meas_list.add("SHUTDOWN_CONFIM_YES", (confirm[0], "none"))
        # check for android boot wake src
        confirm = self.phonesystem_api.check_message_in_log("BOOT_REASON",
                                                             before_power_on_time, after_power_on_time, True)
        # Compare Vbatt value with limit parameters
        self._meas_list.add("BOOT_REASON", (confirm[0], "none"))
        # check for android boot mode
        confirm = self.phonesystem_api.check_message_in_log("BOOT_MODE",
                                                             before_power_on_time, after_power_on_time, True)
        # Compare boot mode value with limit parameters
        self._meas_list.add("BOOT_MODE", (confirm[0], "none"))
        # check for modem_off time
        time_modem = self.phonesystem_api.check_message_in_log("MODEM_OFF",
                                                                before_shutdown_time, pw_off_time, True)
        if pw_off_time is None or time_modem[0] == "FALSE":
            # Compare time_modem_off value with limit parameters
            self._meas_list.add("TIME_MODEM_TO_OFF", (-1, ""))
        else:
            # Compare time_modem_off value with limit parameters
            self._meas_list.add("TIME_MODEM_TO_OFF", (pw_off_time - time_modem[1], ""))

        #
        # switch off sequence
        #
        # wake up the board if necessary
        if not self.phonesystem_api.get_screen_status():
            self._io_card.press_power_button(0.3)
        time.sleep(2)
        # PUSH ON button 3s
        self._logger.info("        press on 3s")
        self._io_card.press_power_button(3)
        time.sleep(2)
        # select "ok" in power off menu
        self.phonesystem_api.select_pwoff_menu("ok")
        # unplug USB SDP
        self._device.disconnect_board()
        self._io_card.usb_host_pc_connector(False)
        # wait for the power off
        time.sleep(30)
        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()
        self._logger.info("        state should be off : IBatt6 = %s %s " %
                         (meas_ibatt[0], meas_ibatt[1]))
        # Compare ibatt value with limit parameters
        self._meas_list.add("IBATT6", meas_ibatt)
        result = self._em_meas_verdict.test_value(self._meas_list,
                                                  self._em_targets["IBATT6"])
        # switch off the board if the board is on
        if not result:
            self._io_card.press_power_button(10)
            time.sleep(10)

        #
        # push on 9s sequence
        #
        # PUSH ON button 9s
        self._io_card.press_power_button(9)
        time.sleep(40)
        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()
        self._logger.info("        state should be on : IBatt7 = %s %s " %
                         (meas_ibatt[0], meas_ibatt[1]))
        # Compare ibatt value with limit parameters
        self._meas_list.add("IBATT7", meas_ibatt)

        # generate em verdict
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
        self._meas_list.clean()

        # Save data report in xml file
        self._error.Code = self._em_meas_verdict.get_current_result()
        self._error.Msg = self._em_meas_verdict.save_data_report_file()

        return self._error.Code, self._error.Msg
