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
:summary: test that vbatt threshold at shutdown evolve with the running load.
:author: vgomberx
:since: 22/11/2013
"""
import time
import os
from acs_test_scripts.Utilities.EMUtilities import XMLMeasurementFile
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from ErrorHandling.AcsBaseException import AcsBaseException
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.UseCase.EnergyManagement.UcModule.LoadModule import LoadModule


class LabEmBattCritThresholdAdapted(EmUsecaseBase):

    """
    Lab Energy Management base class.
    """
    DEDICATED_BENCH = "BATTERY_BENCH"

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base Init function
        EmUsecaseBase.__init__(self, tc_name, global_config)
        # init fuel gauging parameters
        self.em_core_module.init_fg_param()

        # Read the battery capacity expected for the init of the test
        self.__high_load_timeout = self._tc_parameters.get_param_value("HIGH_LOAD_DISCHARGE_TIME", default_cast_type=int)
        self.__light_load_timeout = self._tc_parameters.get_param_value("LIGHT_LOAD_DISCHARGE_TIME", default_cast_type=int)
        self.__light_load = self._tc_parameters.get_param_value("LIGHT_LOAD", "")
        self.__high_load = self._tc_parameters.get_param_value("HIGH_LOAD")
        self.__load_module = LoadModule()
        self.__load_module.add_load(self.__light_load)
        self.__load_module.add_load(self.__high_load)
        #-----------------------------------------------------------------------

        # Call ConfigsParser to parse Energy_Management
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_BATT_CRIT_THRESHOLD_ADAPTED", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)
        # Initialize EM  xml object
        # measurement file
        meas_file_name = os.path.join(self._saving_directory, "EM_meas_report.xml")
        self.__em_meas_tab = XMLMeasurementFile(meas_file_name)

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # Call the UseCaseBase Setup function
        EmUsecaseBase.set_up(self)
        # init capacity
        self.update_battery_info()

        # discharge battery
        if str(self.em_core_module.batt_start_capacity).isdigit() and \
            (self.batt_capacity > self.em_core_module.batt_start_capacity):
            # add video to the setup discharge load

            self.em_core_module.monitor_discharging(self.em_core_module.batt_start_capacity,
                                     self.em_core_module.discharge_time,
                                     self.__em_meas_tab, self.__load_module)

        # help board to discharge faster

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        here we discharge twice to compare the voltage at shutdown level.
        Shutdown level is at 0% of capacity on android system.
        it may change on other
        """
        EmUsecaseBase.run_test_body(self)
        msic = self.update_battery_info()
        # get voltage unit
        unit = msic["BATTERY"]["VOLTAGE"][1]

        def __discharge_part(load, load_timeout, text):
            self.__load_module.stop_load()
            self.__load_module.start_load(load)
            self.__discharge_to_off_without_data(load_timeout)
            error_text = ""

            # plug a data cable to see if the board boot in COS or MOS
            self.__retrieve_the_board_to_mos()

            if self.is_board_and_acs_ok():
                voltage = self.__retrieve_autolog_result(text)
                # exit here if we fail to get the voltage at 0%
                if voltage is None:
                    error_text = "load %s never reach 0%% of capacity" % str(load)

            if error_text != "":
                # leave here as test is KO if 0% not seen
                self._meas_list.add("DELTA_BETWEEN_HIGH_AND_LIGHT_LOAD", "CAPACITY_AT_ZERO_NEVER_REACHED", "")
                self._logger.error(error_text)
                raise DeviceException(DeviceException.OPERATION_FAILED, error_text)

            return voltage

        # First discharge with the load that take more time to discharge
        light_voltage = __discharge_part(self.__light_load, self.__light_load_timeout, "light_load")
        if self.is_board_and_acs_ok() and light_voltage is not None:
            # if board capacity is too low then charge a little bit
            if self.batt_capacity + 1 < int(self.em_core_module.batt_start_capacity):
                self.em_core_module.monitor_charging(self.em_core_module.batt_start_capacity, 120, self.__em_meas_tab)

            # Discharge with high load
            high_voltage = __discharge_part(self.__high_load, self.__high_load_timeout, "high_load")

            if high_voltage is not None:
                # generate em verdict
                self._meas_list.add("DELTA_BETWEEN_HIGH_AND_LIGHT_LOAD", high_voltage - light_voltage, unit)

        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets, True)
        self._em_meas_verdict.judge(ignore_blocked_tc=True)
        # Save data report in xml file
        return(self._em_meas_verdict.get_current_result(),
               self._em_meas_verdict.save_data_report_file())

    def tear_down(self):
        """
        End and dispose the test
        """
        # call tear down after some operations
        EmUsecaseBase.tear_down(self)
        # check the board health, get the aplog, and charge again if necessary
        load_stopped = False
        if self.is_board_and_acs_ok():
            # restore setting that does not persist on reboot
            self.__load_module.stop_load(raise_error=False)
            load_stopped = True

        # clean the board state and retrieve logs
        self.em_core_module.clean_up()
        if not load_stopped:
            self.__load_module.stop_load(raise_error=False)

        return Global.SUCCESS, "No errors"

    def __discharge_to_off_without_data(self, timeout):
        """
        discharge board until shutdown
        designed for a short discharge period(like discharging from 20% to 0).
        If discharge fail to reach shutdown , it will discharge for the all of the shutdown
        timeout set
        """
        # start  non persistent autolog with a short period of data polling
        # choose function to put in logger
        self.em_api.clean_autolog()
        self.em_api.add_fct_to_auto_logger(self.em_api.AUTOLOG_THERMAL, "sequenced")
        self.em_api.add_fct_to_auto_logger(self.em_api.AUTOLOG_UEVENT, "sequenced")
        self.em_api.start_auto_logger(60, 1, "sequenced")
        # disconnect board and plug the wanted cable
        self._device.disconnect_board()
        self._io_card.ac_charger_connector(False)
        self._io_card.usb_connector(False)

        # wait for 0% of battery capacity or connection lost
        self._logger.info("Waiting %ss to let the board discharge from %s%% to off" % (timeout, self.batt_capacity))
        time.sleep(timeout)
        self._io_card.usb_host_pc_connector(True)
        time.sleep(self.usb_sleep)
        mode = self._device.get_boot_mode()
        if mode == "MOS":
            self._device.connect_board()
            self.update_battery_info()
            self.__load_module.restart_load(consider_only_checkable_load=True)
            self._logger.info("board not yet turn off, Waiting %ss to let the board discharge from %s%% to off" % (
            timeout / 2, self.batt_capacity))
            self._device.disconnect_board()
            self._io_card.usb_host_pc_connector(False)

    def __retrieve_the_board_to_mos(self):
        """
        retrieve the board.
        """
        # plug a data cable to see if the board boot in COS or MOS
        self._io_card.usb_host_pc_connector(True)
        time.sleep(self.usb_sleep)
        mode = self._device.get_boot_mode()
        if mode != "MOS":
            # wait for a while to let the board boot
            if self.phone_info.get("DATA_WHILE_CHARGING") is True:
                self._logger.info("Waiting 5 minutes for the board to boot")
                time.sleep(300)
                self._logger.info("Waiting at most 15 minutes to see the boot mode")
                end_time = time.time() + 900
                while time.time() < end_time:
                    mode = self._device.get_boot_mode()
                    if mode in ["COS", "MOS"]:
                        self._logger.info("Board seen booted in %s after %ss" % (mode, abs((time.time() - end_time) + 900)))
                        break

            if mode != "MOS":
                # plug a charge and charge it for a while
                self._io_card.wall_charger_connector(True)
                self._logger.info("Waiting 20 minutes to charge the board with a wall charger")
                time.sleep(1200)
                # plug a data cable to see if the board boot is still in COS
                self._io_card.wall_charger_connector(False)
                self._io_card.usb_host_pc_connector(True)
                mode = self._device.get_boot_mode()
                self._logger.info("Waiting at most 10 minutes to detect the boot mode")
                end_time = time.time() + 600
                while time.time() < end_time:
                    mode = self._device.get_boot_mode()
                    if mode != "UNKNOWN":
                        self._logger.info("Board seen booted in %s after %ss" % (mode, abs((time.time() - end_time) + 600)))
                        break

                # if the boot mode is not unknown , try to reboot the board
                if mode == "UNKNOWN":
                    self.em_core_module.reboot_board("HARD")
                # if in MOS connect board
                elif mode != "MOS":
                    self._device.reboot(skip_failure=True)

                # after all of this , try to connect ACS
                mode = self._device.get_boot_mode()

        if mode == "MOS":
            # else we are in MOS thus connecting ACS
            self._device.connect_board()

        # finally do action here only if board is well booted
        if not self.is_board_and_acs_ok():
            tmp_txt = "failed to retrieve the board in MAIN OS and connect ACS after shutdown link to load testing"
            self._logger.error(tmp_txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, tmp_txt)

    def __retrieve_autolog_result(self, text):
        """
        retrieve data from autolog.
        return voltage when capacity reach the first 0%.
        """
        # get info from autologger
        result = None
        self.update_battery_info()
        self.em_api.stop_auto_logger()
        msic_list = self.em_api.get_autolog_msic_registers()
        thermal_list = self.em_api.get_autolog_thermal_sensor_info()
        self.em_api.clean_autolog()

        # get the highest logs length
        log_length = max(len(thermal_list), len(msic_list))
        for i in range(log_length):
            try:
                # get battery/charger info
                if len(msic_list) > i:
                    msic_dict = msic_list[i][1]
                    if len(msic_dict) > 1:
                        # get the voltage for the first 0% reported
                        # FIXME: maybe check the 0 from aplog to catch  exactly what board see
                        if result is None and msic_dict["BATTERY"]["CAPACITY"][0] == 0:
                            result = msic_dict["BATTERY"]["VOLTAGE"][0]
                        self.__em_meas_tab.add_dict_measurement(msic_dict)

                    # get thermal info
                if len(thermal_list) > i:
                    thermal_dict = thermal_list[i][1]
                    if len(thermal_dict) > 1:
                        # store result on xml
                        self.__em_meas_tab.add_dict_measurement(thermal_dict)

                # reset error
                measurement_fail = 0
            except AcsBaseException as e:
                # try to reconnect to the board if uecmd failed
                self._logger.error("fail to get measurement: " + str(e))
                measurement_fail += 1

                # stop the usecase if measurement fail several times.
                if measurement_fail >= self._consecutive_meas_error:
                    tmp_txt = "Measurement failed after %s times, abort usecase" % \
                        self._consecutive_meas_error
                    self._logger.error(tmp_txt)
                    # leave here ,this is not used to generate this test verdict
                    # so not error raised
                    return
            finally:
                # Store various information
                self.__em_meas_tab.add_measurement([self.get_time_tuple(),
                                                  (self._em_cst.COMMENTS,
                                                   "RUNTEST : Discharge without data cable with load : %s" % text)])
                # switch meas to next meas
                self.__em_meas_tab.switch_to_next_meas()
        return result
