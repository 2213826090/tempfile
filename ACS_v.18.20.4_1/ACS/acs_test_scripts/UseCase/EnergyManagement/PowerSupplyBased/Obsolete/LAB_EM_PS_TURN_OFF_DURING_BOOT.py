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
:summary: Energy Management Turn off during the boot test.
          The test turn on the board and try to switch off the board during the board.
          Several ways are used to turn off the board :
             -battery removal,
             -hardware_shutdown,
             -emergency_shutdown
          The test do it in a loop, at different timing.
          Then it try to boot the board in Main Os and verify everything is OK.
:author: dbatutx
:since: 27/03/2013
"""

import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from ErrorHandling.AcsConfigException import AcsConfigException


class LabEmPsTurnOffDuringBoot(EmUsecaseBase):

    """
    Lab Energy Management class.
    """
    DEDICATED_BENCH = "POWER_SUPPLY_BENCH"

    def __init__(self, tc_name, global_config):

        # Call LAB_EM_BASE Init function
        EmUsecaseBase.__init__(self, tc_name, global_config)

        # Get the normal battery voltage
        self.batt_voltage = float(self._tc_parameters.get_param_value(
            "BATT_VOLTAGE", "3.8"))

        # Get the emergency battery voltage
        self._emergency_batt_voltage = float(self._tc_parameters.get_param_value(
            "EMERGENCY_BATT_VOLTAGE", "2.7"))

        # Get the power_button press to boot the board
        self._pwr_button_press_time = float(self._tc_parameters.get_param_value(
            "POWER_BUTTON_PRESS_TIME", "2.1"))

        # get the number of loop
        self._loop_number = int(self._tc_parameters.get_param_value(
            "LOOP_NUMBER", 10))

        # get the first time from boot to try to turn off the board
        self._first_turn_off_timing = int(self._tc_parameters.get_param_value(
            "FIRST_TURN_OFF_TIMING", 4))

        # get the turn off step for the loop use
        self._turn_off_step = int(self._tc_parameters.get_param_value(
            "TURN_OFF_TIMING_STEP", 4))

        # get the turn off way to use in the test
        self._turn_off_way = self._tc_parameters.get_param_value(
            "TURN_OFF_WAY", "HARDWARE_SHUTDOWN")

        # Redefine initial value for setting USBDIO:
        # - BatteryType = ANALOG
        self.em_core_module.io_card_init_state["BatteryType"] = self.phone_info["BATTERY"]["BATTID_TYPE"]
        # - Battery
        self.em_core_module.io_card_init_state["Battery"] = True
        # - Platform
        self.em_core_module.io_card_init_state["Platform"] = "OFF"
        # - USBChargerType
        self.em_core_module.io_card_init_state["USBChargerType"] = "USB_HOST_PC"
        # - USBCharger
        self.em_core_module.io_card_init_state["USBCharger"] = False
        # - BatteryTemperature
        self.em_core_module.io_card_init_state["BatteryTemperature"] = 25

        # Set initial value for setting Power Supply VBATT:
        # - VoltageLevel
        self.em_core_module.eqp_init_state["BATT"]["VoltageLevel"] = self.batt_voltage

        self._boot_timeout = self._device.get_boot_timeout()

    def set_up(self):
        """
        Initialize the test:
        """
        EmUsecaseBase.set_up(self)

        # Call ConfigsParser to parse Energy_Management
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_PS_TURN_OFF_DURING_BOOT", self._tc_parameters.get_params_as_dict(), self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

        # check parameter turn off way
        if self._turn_off_way not in ["HARDWARE_SHUTDOWN",
                                      "EMERGENCY_SHUTDOWN", "BATTERY_REMOVAL"]:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                   "The parameter TURN_OFF_WAY '%s' is not a valid way to turn off the board"
                                   % self._turn_off_way)
        # check parameter first turn off timing
        if self._first_turn_off_timing > 45:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                   "The parameter FIRST_TURN_OFF_TIMING '%s'" % self._first_turn_off_timing
                                   + " shall be under 45s or the test will be done after booting")

        return Global.SUCCESS, "No errors"

    def run_test_body(self):
        """
        Execute the test
        """
        # Call LAB_EM_BASE Init function
        EmUsecaseBase.run_test_body(self)

        # do the test for the loop
        for i in range(self._loop_number):
            self._logger.info("-*-*Test Iteration %s*-*-" % str(i + 1))
            # compute the time to wait after we start to boot
            timing_after_turn_on = self._first_turn_off_timing + i * self._turn_off_step
            # begin to boot the board and return after pressing the power button
            self._begin_boot_board(self._pwr_button_press_time)
            # wait the good time
            self._logger.info("***Wait %s second before initiating a shutdown***"
                              % str(timing_after_turn_on))
            time.sleep(timing_after_turn_on)
            # Turn off the board
            self._turn_off_board(self._turn_off_way)
            # check the board status
            self._check_board(self._pwr_button_press_time)
            # add measure to the verdict
            self._em_meas_verdict.compare_list(self._meas_list, self._em_targets, True)
            self._em_meas_verdict.judge(ignore_blocked_tc=True)

        # Save data report in xml file
        self._error.Code = self._em_meas_verdict.get_current_result()
        self._error.Msg += self._em_meas_verdict.save_data_report_file()
        return self._error.Code, self._error.Msg

    def _begin_boot_board(self, button_press_time):
        """
        unplug/plug the battery,
        then press the power button in order to switch on the board

        :type button_press_time: int
        :param button_press_time: the press duration on the power button
                                 in order to switch on the board
        """
        self._logger.info("***Boot The board***")
        # disconnect battery
        self._io_card.battery_connector(False)
        time.sleep(2)
        # connect battery
        self._io_card.battery_connector(True)
        # press the power button
        self._io_card.press_power_button(button_press_time)

    def _turn_off_board(self, turn_off_way):
        """
        Turn off the board while the board is booting.

        :type turn_off_way: str
        :param turn_off_way: the way to use to turn off the board
        """
        self._logger.info("***Switch off the board by %s***" % turn_off_way)
        if turn_off_way == "HARDWARE_SHUTDOWN":
            # press the power button 8s to initiate a hardware shutdown
            self._io_card.press_power_button(8)
            time.sleep(2)
        elif turn_off_way == "EMERGENCY_SHUTDOWN":
            # set the battery emergency shutdown level
            self.em_core_module.pwrs_vbatt.set_current_voltage(self._emergency_batt_voltage,
                                                 self.em_core_module.ps_properties["BATT"]["PortNumber"])
            # wait for the board extinction
            time.sleep(20)
            # set a normal battery voltage
            self.em_core_module.pwrs_vbatt.set_current_voltage(self.batt_voltage,
                                                 self.em_core_module.ps_properties["BATT"]["PortNumber"])
            time.sleep(2)
        elif turn_off_way == "BATTERY_REMOVAL":
            # disconnect battery
            self._io_card.battery_connector(False)
            time.sleep(2)
            # connect battery
            self._io_card.battery_connector(True)
            time.sleep(2)

    def _check_board(self, button_press_time):
        """
        Check the board is well booting.
        Then  check shutdown logs

        :type button_press_time: int
        :param button_press_time: the press duration on the power button
                                 in order to switch on the board
        """
        self._logger.info("***Check the board state***")
        # switch on the board
        self._io_card.press_power_button(button_press_time)
        # record time after booting
        start_time = time.time()
        # connect SDP
        time.sleep(1)
        self.em_core_module.plug_charger("SDP")
        is_board_booted = False
        timer = time.time() - start_time
        while timer < self._boot_timeout * 2:
            boot_mode = self._device.get_boot_mode()
            if boot_mode == "MOS":
                is_board_booted = True
                self._logger.info("board has booted in %s seconds in MOS" %
                                  str((time.time() - start_time)))
                # connect board
                self._device.connect_board()
                # wait some sec to settle down
                time.sleep(5)
                # add boot mode result
                self._meas_list.add("BOOT_MODE", boot_mode, "none")
                # add boot timer result
                self._meas_list.add("BOOT_TIMER", timer, "second")
                # check shutdown confirmation
                shutdown_reason = self.phonesystem_api.check_message_in_log("SHUTDOWN_REASON",
                                                                             0, time.time(), True)
                # add shutdown reason
                self._meas_list.add("SHUTDOWN_REASON", shutdown_reason[0], "none")
                # check shutdown2 confirmation
                shutdown_reason2 = self.phonesystem_api.check_message_in_log("SHUTDOWN_REASON2",
                                                                              0, time.time(), True)
                # add shutdown reason
                self._meas_list.add("SHUTDOWN_REASON_2", shutdown_reason2[0], "none")
                break
            elif boot_mode in ["COS", "ROS", "POS"]:
                is_board_booted = True
                # reboot in MOS
                self._device.reboot("MOS")
                self._logger.info("board has booted in %s seconds in MOS" %
                                  str((time.time() - start_time)))
                # add boot mode result
                self._meas_list.add("BOOT_MODE", boot_mode, "none")
                # add boot timer result
                self._meas_list.add("BOOT_TIMER", timer, "second")
                # check shutdown confirmation
                shutdown_reason = self.phonesystem_api.check_message_in_log("SHUTDOWN_REASON",
                                                                             0, time.time(), True)
                # add shutdown reason
                self._meas_list.add("SHUTDOWN_REASON", shutdown_reason[0], "none")
                # check shutdown2 confirmation
                shutdown_reason2 = self.phonesystem_api.check_message_in_log("SHUTDOWN_REASON2",
                                                                              0, time.time(), True)
                # add shutdown reason
                self._meas_list.add("SHUTDOWN_REASON_2", shutdown_reason2[0], "none")
                break
            # increase the timer
            timer = time.time() - start_time

        if not is_board_booted:
            self._logger.info("board hasn't booted...")
            # add boot mode result
            self._meas_list.add("BOOT_MODE", boot_mode, "none")
            # add boot timer result
            self._meas_list.add("BOOT_TIMER", timer, "second")
            # add shutdown reason
            self._meas_list.add("SHUTDOWN_REASON", "DEVICE NOT BOOTED", "none")
