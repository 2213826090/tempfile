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
:summary: Energy Management system boot usecase
:author: apairex
:since: 12/05/2011 (dec the 5th)
"""
import time
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase


class LabEmSystemBoot(EmUsecaseBase):

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

        # Read parameters from TC parameters
        self._watchdog_timeout = \
            int(self._tc_parameters.get_param_value("WATCHDOG_TIMEOUT"))

        # Call ConfigsParser to parse Energy_Management
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_SYSTEM_BOOT", self._tc_parameters.get_params_as_dict(),
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
        # - USBCharger = False (removed)
        self.em_core_module.io_card_init_state["USBChargerType"] = "USB_HOST_PC"
        self.em_core_module.io_card_init_state["USBCharger"] = True

        # Set initial value for setting Power Supply VBATT:
        # - VoltageLevel = VBATT
        self.em_core_module.eqp_init_state["BATT"]["VoltageLevel"] = self.em_core_module.vbatt
        self.em_core_module.pwrs_vbatt = self._em.get_power_supply("BATT")

        # Initialize variable for cos time
        self._before_cos_time = time.time()

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        """
        # Call LAB_EM_BASE Run function
        EmUsecaseBase.run_test_body(self)

        # Run Watchdog test
        self._run_watchdog_test()

        # Run cold reset test
        self._run_cold_reset_test()

        # Run Transition OS test
        self._run_transition_os_test()

        # Run Start MOS test
        self._run_start_mos_test()

        # generate em verdict
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
        self._meas_list.clean()

        # Save data report in xml file
        self._error.Code = self._em_meas_verdict.get_current_result()
        self._error.Msg = self._em_meas_verdict.save_data_report_file()

        return self._error.Code, self._error.Msg

    def _run_watchdog_test(self):
        """
        Run a command that makes the phone crash
        and check that the watchdog reboots the board

        Prerequisite : Phone should be connected through USB
        """
        self._logger.info("Entering WATCHDOG TEST")

        # Generate AP crash
        self.phonesystem_api.generate_ap_crash()
        time.sleep(5)

        # Check that adb connection does not respond
        state = self._device.get_state()
        self._meas_list.add("PHONE_STATE_WATCHDOG", (state.upper(), "none"))

        # Disconnect USB
        # self._device.disconnect_board()
        # self._io_card.usb_connector(False)

        self._logger.info(("Phone should have crashed. Waiting %d sec for " +
                          "the Watchdog to reboot") % self._watchdog_timeout)
        time.sleep(self._watchdog_timeout)

        # connect board
        self.em_core_module.check_board_connection(only_reconnect=True)

        # Compare boot mode value with limit parameters
        self._meas_list.add("BOOT_REASON_WATCHDOG",
                           (self._device.get_boot_mode(), "none"))

        # wake up the board
        self.phonesystem_api.wake_screen()
        time.sleep(2)

        # unplug SDP
        self.em_core_module.unplug_charger("SDP")

    def _run_cold_reset_test(self):
        """
        Send a "reboot" command to perform a cold reset
        and control that the board reboots
        """
        self._logger.info("Entering COLD RESET TEST")

        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()
        self._logger.info("Phone should be ON: IBatt = %s %s"
                          % (meas_ibatt[0], meas_ibatt[1]))
        self._meas_list.add("IBATT_COLD_RESET", meas_ibatt)

        # Perform a cold reset
        self.em_core_module.check_board_connection(only_reconnect=True)
        self.em_core_module.reboot_board()
        self._device.disconnect_board()
        self.em_core_module.unplug_charger(self._io_card.SDP)
        self._logger.info("Phone should cold reboot. Waiting 40 sec")
        time.sleep(40)

        # Check the adb connection
        self.em_core_module.plug_charger(self._io_card.SDP)
        time.sleep(7)
        self._device.connect_board()

        # Compare boot mode value with limit parameters
        self._meas_list.add("BOOT_REASON_COLD_RESET",
                            (self._device.get_boot_mode(), "none"))

        # get phone state
        state = self._device.get_state()
        self._meas_list.add("PHONE_STATE_COLD_RESET", (state.upper(), "none"))

        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()
        self._logger.info("After cold reset: IBatt = %s %s"
                          % (meas_ibatt[0], meas_ibatt[1]))
        self._meas_list.add("IBATT_AFTER_REBOOT", meas_ibatt)

        self._device.disconnect_board()
        self.em_core_module.unplug_charger(self._io_card.SDP)

    def _run_transition_os_test(self):
        """
        Run transition OS tests:
        - Perfom a hard reset (10s Power Button key press)
        - Start the phone (2s Power button key press)
        - Plug SDP charger
        - Perform a graceful shutdown (2s Power button + user OK on UI)
        """
        self._logger.info("Entering Transition OS test")

        # Hard reset
        self._io_card.press_power_button(8)
        time.sleep(5)

        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()
        self._logger.info("Phone should be OFF: IBatt = %s %s"
                          % (meas_ibatt[0], meas_ibatt[1]))
        self._meas_list.add("IBATT_HARD_RESET", meas_ibatt)
        self._em_meas_verdict.test_value(self._meas_list,
                                         self._em_targets["IBATT_HARD_RESET"])

        # Start the phone
        self._io_card.press_power_button(self.pwr_btn_boot)
        self._logger.info("Starting the phone. Waiting 40s")
        time.sleep(40)

        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()
        self._logger.info("Before charger plug: IBatt = %s %s"
                          % (meas_ibatt[0], meas_ibatt[1]))
        self._meas_list.add("IBATT_BEFORE", meas_ibatt)

        # Plug USB SDP charger and connect the phone
        self.em_core_module.plug_charger("DCP", ext_ps=True)
        time.sleep(20)

        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()
        self._logger.info("After charger plug: IBatt = %s %s"
                          % (meas_ibatt[0], meas_ibatt[1]))
        self._meas_list.add("IBATT_AFTER", meas_ibatt)

        # plug the SDP charger
        self.em_core_module.plug_charger("SDP")
        self.em_core_module.check_board_connection(only_reconnect=True)

        # Perform a graceful shutdown
        self._before_cos_time = time.time()
        self._device.soft_shutdown_cmd()
        self._logger.info("Graceful shutdown... rebooting in COS. Wait 60s")
        time.sleep(60)

        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()
        self._logger.info("After soft shutdown: IBatt = %s %s"
                          % (meas_ibatt[0], meas_ibatt[1]))
        self._meas_list.add("IBATT_AFTER_SOFT", meas_ibatt)

    def _run_start_mos_test(self):
        """
        Press Power button during, at least 3s, to boot MOS from COS
        """
        self._logger.info("Entering run start MOS test")

        # wake up the board
        self._io_card.press_power_button(1)
        time.sleep(2)

        # boot in MOS from COS
        after_cos_time = time.time()
        self._io_card.press_power_button(4)
        self._logger.info("The boad should boot in MOS. Waiting 30sec")
        time.sleep(30)

        self.em_core_module.check_board_connection(only_reconnect=True)

        # check for android boot mode cos
        result = self.phonesystem_api.check_message_in_log("COS_MODE",
                                                            self._before_cos_time, after_cos_time, False)
        # Compare cos boot mode value with limit parameters
        self._meas_list.add("BOOT_REASON_GRACEFUL_SHUTDOWN", (result[0], "none"))

        state = self._device.get_state()
        self._meas_list.add("PHONE_STATE_START_MOS", (state.upper(), "none"))
