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
:summary: EM - Platform shall terminate the main battery charging in case of charger timer expiration.
Test the end of the main battery charging after timer expiration (8 hours for CTP, 13 hours for MFLD)
Every xx minutes, we check the capacity level and the charging status. When the charging status is "not charging" the test is stopped.
:author: jvauchex
:since: 03/03/2013
"""
import time
import os
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.EMUtilities import update_conf, XMLMeasurementFile
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from ErrorHandling.DeviceException import DeviceException


class LabEmBattChargerTimerExpiration(EmUsecaseBase):

    """
    Live Energy Management class.
    """
    DEDICATED_BENCH = "BATTERY_BENCH"

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LAB_EM_BASE Init function
        EmUsecaseBase.__init__(self, tc_name, global_config)

        # Read parameters from TC parameters
        self.__boot_mode = \
            self._tc_parameters.get_param_value("BOOT_MODE", "MOS")
        self.__video_path = \
            self._tc_parameters.get_param_value("VIDEO_PATH", "")
        self.__timer_expiration_value = \
            int(self._tc_parameters.get_param_value("TIMER_EXPIRATION_VALUE", "28800"))
        self.__check_time_value = \
            int(self._tc_parameters.get_param_value("CHECK_VALUE_TIME", "600"))
        self.__cos_capacity_start = \
            int(self._tc_parameters.get_param_value("COS_CAPACITY_START", "10"))

        self._logger.info("Type of OS : %s" % self.__boot_mode)
        self._logger.info("Path of the video : %s" % self.__video_path)
        self._logger.info("Value of timer expiration : %d" % self.__timer_expiration_value)
        self._logger.info("Value of check time : %d" % self.__check_time_value)

        # Call ConfigsParser to parse Energy_Management
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_BATT_CHARGER_TIMER_EXPIRATION", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

        # Update the target xml file
        update_conf(self._em_targets["VALUE_CHARGE_TIMER"],
                    "lo_lim", self.__timer_expiration_value - 600, "=")
        update_conf(self._em_targets["VALUE_CHARGE_TIMER"],
                    "hi_lim", self.__timer_expiration_value + 600, "=")

        # measurement file
        meas_file_name = os.path.join(self._saving_directory,
                                      "EM_meas_report.xml")
        self.__em_meas_tab = XMLMeasurementFile(meas_file_name)
        # enable Global Measurement file
        name = os.path.join(self._campaign_folder,
                            self._em_cst.GLOBAL_MEAS_FILE)
        self.__em_meas_tab.enable_global_meas(name, self._name)

        self._system_api = self._device.get_uecmd("System")
        self._video_api = self._device.get_uecmd("Video")
        self._multimedia_path = self._device.multimedia_path

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # Call the UseCaseBase Setup function
        EmUsecaseBase.set_up(self)

        # Disable lock screen
        self.phonesystem_api.disable_lockscreen()

        # Config the brightness
        if self.__boot_mode == "MOS":
            # set the screen on and brightness to 100%
            self.phonesystem_api.set_screen_timeout(3600 * 4)
            # deactivate set auto brightness
            self.phonesystem_api.set_brightness_mode("manual")
            # set display brightness to max value
            self.phonesystem_api.set_display_brightness(100)
            # Wake up the screen
            self.phonesystem_api.wake_screen()

        # Check the battery capacity
        msic_result = self.em_api.get_msic_registers()
        self.batt_capacity = msic_result["BATTERY"]["CAPACITY"][0]

        # If the test is on COS, discharge the board and launch the test
        if self.__boot_mode == "COS" and self.batt_capacity > self.__cos_capacity_start:
            self._device.reboot("MOS")
            self._system_api.adjust_specified_stream_volume("Media", 100)
            self._video_api.play(self._multimedia_path + self.__video_path, loop=True)
            # Discharge battery
            self.em_core_module.monitor_discharging(self.__cos_capacity_start,
                                      self.em_core_module.discharge_time, self.__em_meas_tab)
        self.em_api.set_usb_charging("on")

        # Check the type of Boot Mode
        boot_mode = self._device.get_boot_mode()

        # Verify if the boot mode is the right else => reboot with the correct boot mode
        if boot_mode != self.__boot_mode:
            self._logger.info("Board isn't in %s : Reboot Board" % self.__boot_mode)
            result = self._device.reboot(self.__boot_mode)
            if not result:
                txt = "Boot Mode isn't correct : %s ! " % self.__boot_mode
                self._logger.error(txt)
                raise DeviceException(DeviceException.INVALID_DEVICE_STATE, txt)

        # disconnect-connect SDP in order to clear the counter
        self._io_card.usb_connector(False)
        self._io_card.simulate_insertion("SDP")

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        Check the end of the main battery charge
        """
        # Call LAB_EM_BASE Run function
        EmUsecaseBase.run_test_body(self)

        # Initialize the variables
        cpt_charge_timer = 0
        cycle_time = 0

        # check the battery capacity
        msic_result = self.em_api.get_msic_registers()
        self.batt_capacity = msic_result["BATTERY"]["CAPACITY"][0]

        # Initialize the control loop for charge or discharge
        if self.batt_capacity > 20:
            control_high_low_level = True
        else:
            control_high_low_level = False

        # Loop in order to check the charge timer expiration
        while cpt_charge_timer < self.__timer_expiration_value:

            # Check the value of status and capacity (xx minutes)
            self._device.connect_board()
            msic_result = self.em_api.get_msic_registers()
            charge_status = msic_result["BATTERY"]["STATUS"][0]
            self.batt_capacity = msic_result["BATTERY"]["CAPACITY"][0]
            self.batt_voltage = msic_result["BATTERY"]["VOLTAGE"][0]
            self._logger.info("Time : %d " % cpt_charge_timer)
            self.__em_meas_tab.add_dict_measurement(msic_result)
            self.__em_meas_tab.add_measurement(
                [self.get_time_tuple(),
                 (self._em_cst.COMMENTS, "RUNTEST")])
            # switch to next meas
            self.__em_meas_tab.switch_to_next_meas()

            # If the capacity is too low or high
            if self.batt_capacity > 80 and control_high_low_level:
                # Set an heavy load on the phone in order to discharge
                self.em_api.set_usb_charging("low")
                value = self.em_api.get_charger_level()
                self._logger.info("The value of charger level is equal to %s" % value)
                # Launch vibra, torchlight, and video player
                self.phonesystem_api.set_vibration("on")
                if self.__boot_mode == "MOS":
                    self.phonesystem_api.set_torchlight("on")
                    self._system_api.adjust_specified_stream_volume("Media", 100)
                    self._video_api.play(self._multimedia_path + self.__video_path, loop=True)
                control_high_low_level = False

            elif self.batt_capacity < 20 and control_high_low_level is False:
                # Remove heavy load in order to charge
                value = self.em_api.get_charger_level()
                self._logger.info("The value of charger level is equal to %s" % value)
                if self.__boot_mode == "MOS":
                    self.em_api.set_usb_charging("on")
                    # Stop vibra, torchlight and video player
                    self.phonesystem_api.set_vibration("off")
                    self.phonesystem_api.set_torchlight("off")
                    self._video_api.stop()
                    control_high_low_level = True

                # Just put vibration to on when "COS"
                elif self.__boot_mode == "COS":
                    self.em_api.set_usb_charging("low")
                    value = self.em_api.get_charger_level()
                    self._logger.info("The value of charger level is equal to %s" % value)
                    self.phonesystem_api.set_vibration("on")
                    control_high_low_level = True

            # If the battery is not charging => end of loop
            if charge_status != "Charging":
                break
            self._device.disconnect_board()

            # Wait xx minutes
            internal_temp_cpt = 0
            while cpt_charge_timer < (cycle_time + self.__check_time_value):
                time.sleep(1)
                if self.__boot_mode == "COS" and internal_temp_cpt >= 8:
                    self._io_card.press_power_button(0.3)
                    internal_temp_cpt = 0
                internal_temp_cpt += 1
                cpt_charge_timer += 1
            cycle_time += self.__check_time_value

        # Connect the board and stop vibra, tochlight and player
        self._device.connect_board()
        self.phonesystem_api.set_vibration("off")
        if self.__boot_mode == "MOS":
            self.phonesystem_api.set_torchlight("off")
            self._video_api.stop()

        # Check the battery status after the charge timer expiration
        msic_result = self.em_api.get_msic_registers()
        status_end_charge_timer = msic_result["BATTERY"]["STATUS"][0]

        # Add the value
        self._meas_list.add("VALUE_CHARGE_TIMER", cpt_charge_timer, "Seconds")
        self._meas_list.add("STATUS_END_CHARGE_TIMER", status_end_charge_timer, "none")

        # compare values with targets
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets, clean_meas_list=True)
        self._em_meas_verdict.judge()

        # Save data report in xml file
        self._error.Code = self._em_meas_verdict.get_current_result()
        self._error.Msg = self._em_meas_verdict.save_data_report_file()

        return self._error.Code, self._error.Msg

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        # call tear down after some operations
        EmUsecaseBase.tear_down(self)

        # retrieve measurement from test
        self.__em_meas_tab.generate_global_file()
        # as measurement is kept over B2B iteration,
        # just reset global file to its init state
        self.__em_meas_tab.reset_global_file()

        # activate set auto brightness
        self.phonesystem_api.set_brightness_mode("automatic")

        # Re-activate the USB Charging
        self.em_api.set_usb_charging("on")

        # clean the board state and retrieve logs
        self.em_core_module.clean_up()

        return Global.SUCCESS, "No errors"
