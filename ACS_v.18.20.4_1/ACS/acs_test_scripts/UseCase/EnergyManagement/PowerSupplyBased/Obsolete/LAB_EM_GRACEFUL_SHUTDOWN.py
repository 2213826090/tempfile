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
:since: 30/12/2011
"""
import time
from UtilitiesFWK.Utilities import str_to_bool, Global
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase


class LabGracefulShutdown(EmUsecaseBase):

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

        # Initialize variables
        self._after_shutdown_time = 0
        self._before_shutdown_time = 0

        # Read battery type from TC parameters
        self._battery_type = self._tc_parameters.get_param_value("BATTERY_TYPE", self.phone_info["BATTERY"]["BATTID_TYPE"])

        # Read shutdown type from TC parameters
        self._shutdown_type = self._tc_parameters.get_param_value("SHUTDOWN_TYPE")

        # Read UNPLUG_CHARGER_WHILE_SHUTDOWN from TC parameters
        self._unplug_charger_while_shutdown = str_to_bool(
            self._tc_parameters.get_param_value("UNPLUG_CHARGER_WHILE_SHUTDOWN"))

        # Read PRESS_PWR_BUTTON_TIME from TC parameters
        self._press_pwr_button_time = float(self._tc_parameters.get_param_value("PRESS_PWR_BUTTON_TIME"))

        # Read CHARGER_TYPE from TC parameters
        self._charger_type = \
            str(self._tc_parameters.get_param_value("CHARGER_TYPE"))

        # Call ConfigsParser to parse Energy_Management
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_GRACEFUL_SHUTDOWN", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

        # Redefine initial value for setting USBDIO:
        # - BatteryType = ANALOG
        self.em_core_module.io_card_init_state["BatteryType"] = self._battery_type
        # - Battery  = True (inserted)
        self.em_core_module.io_card_init_state["Battery"] = True
        # - Platform = True (ON)
        self.em_core_module.io_card_init_state["Platform"] = "ON"
        # - USBChargerType = USB_HOST_PC
        self.em_core_module.io_card_init_state["USBChargerType"] = self._io_card.SDP
        # - USBCharger = False (removed)
        self.em_core_module.io_card_init_state["USBCharger"] = True
        # - BatteryTemperature = BATTERY_TEMPERATURE
        self.em_core_module.io_card_init_state["BatteryTemperature"] = 25

        # Set initial value for setting Power Supply VBATT:
        # - VoltageLevel = VBATT
        self.em_core_module.eqp_init_state["BATT"]["VoltageLevel"] = self.em_core_module.vbatt

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Initialize the test:
        """
        EmUsecaseBase.set_up(self)

        # reboot if battery type is not analog
        if self._battery_type == "BATTERY_EMULATOR":
            self.em_core_module.reboot_board()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        """
        # Call LAB_EM_BASE Run function
        EmUsecaseBase.run_test_body(self)

        # get ibatt
        meas_ibatt = self.em_core_module.get_battery_current()
        # Compare ibatt value with limit parameters
        self._meas_list.add("IBATT1", meas_ibatt)
        # wake up the screen
        self.phonesystem_api.wake_screen()
        time.sleep(5)

        #
        # GRACEFUL shutdown
        #
        # PUSH ON button xs
        if self._shutdown_type == "HOLD_BUTTON":
            self._device.disconnect_board()
            # the case where the charger has to be unplug
            if self._unplug_charger_while_shutdown:
                self.em_core_module.unplug_charger(self._io_card.SDP)
        self._logger.info("        press on %s" % str(self._press_pwr_button_time))
        self._io_card.press_power_button(self._press_pwr_button_time)
        # record time for shutdown time
        self._before_shutdown_time = time.time()
        time.sleep(5)

        # If the power button is hold
        if self._shutdown_type != "HOLD_BUTTON":
            # select "ok" in power off menu
            self._logger.info("UI is activated !")
            self.phonesystem_api.select_pwoff_menu("ok")
            self._device.disconnect_board()
            # the case where the charger has to be unplug
            if self._unplug_charger_while_shutdown:
                self.em_core_module.unplug_charger(self._io_card.SDP)
        # wait 60s
        self._logger.info("Waiting 60 seconds")
        time.sleep(60)
        self._after_shutdown_time = time.time()
        if self._device.get_boot_mode() in ["COS", "ROS", "POS"]:
            # boot the board
            self.em_core_module.reboot_board()
        elif self._device.get_boot_mode() == "UNKNOWN":
            # boot the board
            self.em_core_module.reboot_board()
            self._after_shutdown_time = time.time()
        # check shutdown confirmation
        result = self.phonesystem_api.check_message_in_log("SHUTDOWN_REASON",
                                                            self._before_shutdown_time, self._after_shutdown_time, True)
        # Compare shutdown confirmation value with limit parameters
        self._meas_list.add("SHUTDOWN_REASON", (result[0], "none"))
        # check os
        result = self.phonesystem_api.check_message_in_log("COS_MODE",
                                                            self._before_shutdown_time, self._after_shutdown_time)
        if result[0] == "TRUE":
            os = "COS"
        else:
            os = "UNKNOWN"
        # Compare boot mode mos value with limit parameters
        self._meas_list.add("STATE_OS", os, "none")
        # get ibatt
        meas_ibatt = self.em_core_module.get_battery_current()
        # Compare ibatt value with limit parameters
        self._meas_list.add("IBATT2", meas_ibatt)
        # generate em verdict
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
        self._meas_list.clean()

        # Save data report in xml file
        self._error.Code = self._em_meas_verdict.get_current_result()
        self._error.Msg = self._em_meas_verdict.save_data_report_file()

        return self._error.Code, self._error.Msg

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        EmUsecaseBase.tear_down(self)

        # reboot in normal mode for next test
        if self._battery_type == "BATTERY_EMULATOR":
            # Set Battery type using BATTERY_TYPE parameter
            self._io_card.set_battery_type("DEFAULT")
            self.em_core_module.reboot_board()

        return Global.SUCCESS, "No errors"
