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
:summary: Energy Management Battery Monitor - MOS maintenance charging at specific temperature
:author: vgombert
:since: 21/03/2012 (march)
"""
import time
import os

from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from acs_test_scripts.UseCase.EnergyManagement.UcModule.ThermalChamberModule import ThermalChamberModule
from UtilitiesFWK.Utilities import Global
import acs_test_scripts.Utilities.EMUtilities as EMUtil
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class LabEmBattThermalMaintenanceCharging(EmUsecaseBase):

    """
    Lab Energy Management base class.
    """
    DEDICATED_BENCH = "BATTERY_BENCH"

    def __init__(self, tc_name, global_config):
        """
        Get parameter from xml testcase file
        """
        # Call UseCase base Init function
        EmUsecaseBase.__init__(self, tc_name, global_config)
        self.em_core_module.init_fg_param()

        # Read MAINTENANCE_CHARGING_TIMEOUT from test case xml file
        self.__maintenance_timeout = int(self._tc_parameters.get_param_value("MAINTENANCE_CHARGING_TIMEOUT"))
        # Read EXPECTED_BATTERY_STATE from test case xml file
        self.__expected_batt_state = str(self._tc_parameters.get_param_value(
                "EXPECTED_BATT_STATE")).upper()
        # Read DATA_POLLING from test case xml file
        self.__data_polling = int(self._tc_parameters.get_param_value(
                "DATA_POLLING"))
        # Read TEMPERATURE from test case xml file
        self.tc_module = None
        self.__chamber_tag = "ROOM"
        self.__temperature = str(self._tc_parameters.get_param_value("TEMPERATURE"))
        if self.__temperature != "ROOM":
            # get temperature chamber equipment if not room temperature
            self.__temperature = int(self.__temperature)
            self.tc_module = ThermalChamberModule(self.__temperature)
            # inform module that this is not a default thermal test
            self.tc_module.set_test_type("SPECIFIC")
            self.__chamber_tag = self.tc_module.get_chamber_tag()

        # Read LOAD from test case xml file
        self.__load = \
            str(self._tc_parameters.get_param_value("LOAD")).upper()

        # Initialize EM  xml object
        # measurement file
        name = os.path.join(self._saving_directory,
                            "EM_meas_report.xml")
        self.__em_meas_tab = EMUtil.XMLMeasurementFile(name)
        # enable Global Measurement file
        name = os.path.join(self._campaign_folder,
                            self._em_cst.GLOBAL_MEAS_FILE)
        self.__em_meas_tab.enable_global_meas(name, self._name)
        # hysteresis file
        name = os.path.join(self._saving_directory,
                            "EM_hysteresis_report.xml")
        self.__em_hysteresis_tab = EMUtil.XMLMeasurementFile(name)

#-----------------------------------------------------------------------

    def set_up(self):
        """
        Charge/discharge the board to match the start capacity and
        set environment temperature to the wanted one.
        """
        # Call the UseCaseBase Setup function
        EmUsecaseBase.set_up(self)

        # check battery wanted state is in expected value
        if self.__expected_batt_state not in ["CHARGING", "DISCHARGING",
                                              "NOT CHARGING", "FULL"]:
            tmp_txt = "unknown EXPECTED_BATT_STATE value : %s" % \
                str(self.__expected_batt_state)
            self._logger.error(tmp_txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, tmp_txt)

        # get EM capabilities dict
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_BATT_THERMAL_MAINTENANCE_CHARGING",
            self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())
        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

        # turn ON regulation
        if self.tc_module is not None:
            self.tc_module.set_up_eq()
            autorised_temp_to_charge = self.tc_module.adjust_temp_according_to_test_value()

        # init capacity
        self.update_battery_info()

        # Charge battery
        if (self.em_core_module.batt_max_capacity != "FULL" and self.em_core_module.batt_max_capacity.isdigit() and
                                        self.batt_capacity > int(self.em_core_module.batt_max_capacity)):

            # launch uecmd to help the discharge
            self.phonesystem_api.set_screen_timeout(self.__maintenance_timeout)
            # deactivate set auto brightness
            self.phonesystem_api.set_brightness_mode("manual")
            # set display brightness to max value
            self.phonesystem_api.set_display_brightness(100)
            self.em_core_module.monitor_discharging(self.em_core_module.batt_max_capacity,
                                     self.em_core_module.discharge_time,
                                     self.__em_meas_tab)
        else:
            self.em_core_module.monitor_charging(self.em_core_module.batt_max_capacity,
                                  self.em_core_module.charge_time,
                                  self.__em_meas_tab)
            # launch uecmd to help the discharge
            self.phonesystem_api.set_screen_timeout(self.__maintenance_timeout)
            # deactivate set auto brightness
            self.phonesystem_api.set_brightness_mode("manual")
            # set display brightness to max value
            self.phonesystem_api.set_display_brightness(100)

        if self.tc_module is not None and not autorised_temp_to_charge:
            self.tc_module.get_eq().set_temperature(self.__temperature)
            if not self.tc_module.get_eq().wait_for_temperature(self.__temperature):
                msg = "timeout exceed for changing temperature"
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # toggle techno ON depending of what was put in LOAD var
        if "WIFI" in self.__load:
            self.networking_api.set_wifi_power("on")
        if "BLUETOOTH" in self.__load:
            self.bt_api.set_bt_power("on")
        if "TORCH" in self.__load:
            self.phonesystem_api.set_torchlight("on")
        if "VIBRA" in self.__load:
            self.phonesystem_api.set_vibration("on")

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Do a maintenance charging during x seconds and then compute result.
        """
        EmUsecaseBase.run_test_body(self)

        # check charger max current
        msic_dict = self.em_api.get_msic_registers()
        self._meas_list.add_dict("MSIC_REGISTER_BEFORE_MAINTENANCE", msic_dict,
                                 msic_dict["TIME_STAMP"][0])
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
        self._meas_list.clean()

        # do the maintenance charging
        (time_before_maintenance, phone_start_time, temperature_list,
            msic_result, thermal_result) = self.__maintenance_charging()

        # clean autolog
        if self.is_board_and_acs_ok():
            self.em_api.clean_autolog()
            # toggle techno off depending of what was put in LOAD var
            if "WIFI" in self.__load:
                self.networking_api.set_wifi_power("off")
            if "BLUETOOTH" in self.__load:
                self.bt_api.set_bt_power("off")
            if "TORCH" in self.__load:
                self.phonesystem_api.set_torchlight("off")
            if "VIBRA" in self.__load:
                self.phonesystem_api.set_vibration("off")

        # prepare to compute hysteresis
        local_total_test = len(temperature_list)
        time_stamp = []
        voltage = []
        current = []
        expected_state_seen = False
        first_stamped_misc_found = False
        measurement_found = False

        for iteration in range(local_total_test):
            thermal_stamp = 0
            msic_stamp = 0
            global_stamp = 0
            special_stamp_msic = {}
            special_stamp_thermal = {}

            # store temperature
            if iteration < len(temperature_list):
                self.__em_meas_tab.add_measurement(
                    [(self.__chamber_tag, temperature_list[iteration])])

            # get msic info
            if iteration < len(msic_result):
                measurement_found = True
                msic_dict = msic_result[iteration][1]
                special_stamp_msic = msic_result[iteration][0]
                msic_stamp = special_stamp_msic[self._em_cst.AUTOLOG_TIME]
                msic_stamp = msic_stamp - phone_start_time
                self.__em_meas_tab.add_dict_measurement(msic_dict)

                self.batt_voltage = msic_dict["BATTERY"]["VOLTAGE"][0]
                time_stamp.append(float(msic_stamp) / 3600)
                voltage.append(self.batt_voltage)
                current.append(msic_dict["BATTERY"]["CURRENT_NOW"][0])

                # detect and evaluate a point in the first charging phase
                capacity = msic_dict["BATTERY"]["CAPACITY"][0]
                if not first_stamped_misc_found and (
                        (self.em_core_module.batt_max_capacity == "FULL" and self.__expected_batt_state == "FULL") or
                        (self.em_core_module.batt_max_capacity.isdigit() and capacity >= int(self.em_core_module.batt_max_capacity))):
                    self._meas_list.add_dict("MSIC_REGISTER_DURING_FIRST_CHARGE", msic_dict,
                                             msic_dict["TIME_STAMP"][0])
                    self._em_meas_verdict.compare_list(self._meas_list,
                                                       self._em_targets, True)
                    first_stamped_misc_found = True

                # start to compute verdict after the first battery wanted state is seen
                if msic_dict["BATTERY"]["STATUS"][0].upper() == self.__expected_batt_state and \
                        not expected_state_seen:
                    expected_state_seen = True

                if expected_state_seen:
                    self._meas_list.add_dict("MSIC_REGISTER_MAINTENANCE", msic_dict,
                                             msic_dict["TIME_STAMP"][0])

            # get thermal info
            if iteration < len(thermal_result):
                measurement_found = True
                thermal_dict = thermal_result[iteration][1]
                special_stamp_thermal = thermal_result[iteration][0]
                thermal_stamp = special_stamp_thermal[self._em_cst.AUTOLOG_TIME]
                thermal_stamp = thermal_stamp - phone_start_time
                self.__em_meas_tab.add_dict_measurement(thermal_dict)

            # if not measurement found for above iter on next one
            if not measurement_found:
                continue

            if expected_state_seen:
                self._em_meas_verdict.compare_list(self._meas_list, self._em_targets, True)
                self._em_meas_verdict.save_data_report_file()

            # get the measurement time
            if msic_stamp != 0 and msic_stamp != 0:
                global_stamp = \
                    time_before_maintenance + (msic_stamp + thermal_stamp) / 2
            else:
                global_stamp = \
                    time_before_maintenance + max(msic_stamp, thermal_stamp)

            # get reboot state
            reboot = False
            if self._em_cst.REBOOT in special_stamp_msic:
                reboot = special_stamp_msic[self._em_cst.REBOOT]

            if self._em_cst.REBOOT in special_stamp_thermal:
                reboot = special_stamp_thermal[self._em_cst.REBOOT] or reboot

            # Store various information
            self.__em_meas_tab.add_measurement(
                [(self._em_cst.COMMENTS, "Maintenance charging test at %s Degree Celsius" % self.__temperature),
                 (self._em_cst.REBOOT, reboot),
                 (self._em_cst.TIME_TAG, global_stamp / 3600, self._em_cst.HR)])

            # switch to next meas
            self.__em_meas_tab.switch_to_next_meas()

        # if expected not seen, take a measurement and compare it with target -------------------------------------------------------
        if not expected_state_seen:
            # compute verdict for msic
            if len(msic_dict) > 0:
                # compute first charge first
                msic_dict = msic_result[0][1]
                self._meas_list.add_dict("MSIC_REGISTER_DURING_FIRST_CHARGE", msic_dict,
                                         msic_dict["TIME_STAMP"][0])
                self._em_meas_verdict.compare_list(self._meas_list, self._em_targets, True)
                # then value during step where we should be in maintenance
                msic_dict = msic_result[int(len(msic_result) / 2)][1]
                self._meas_list.add_dict("MSIC_REGISTER_MAINTENANCE", msic_dict,
                                         msic_dict["TIME_STAMP"][0])
                self._em_meas_verdict.compare_list(self._meas_list, self._em_targets, True)

        # compute hysteresis ------------------------------------------------------------------
        self.__compute_hysteresis(current, voltage, time_stamp)

        # judge and clean
        self._em_meas_verdict.judge(ignore_blocked_tc=True)
        self._meas_list.clean()

        return(self._em_meas_verdict.get_global_result(),
               self._em_meas_verdict.save_data_report_file())

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        # call tear down after some operations
        EmUsecaseBase.tear_down(self)

        # retrieve measurement from test
        self.__em_meas_tab.generate_global_file()

        # release equipment
        if self.tc_module is not None:
            self.tc_module.release_eq()

        # clean the board state and retrieve logs
        self.em_core_module.clean_up()

        # clean autolog
        if self.is_board_and_acs_ok():
            self.em_api.clean_autolog()
            # toggle techno off depending of what was put in LOAD var
            if "WIFI" in self.__load:
                self.networking_api.set_wifi_power("off")
            if "BLUETOOTH" in self.__load:
                self.bt_api.set_bt_power("off")
            if "TORCH" in self.__load:
                self.phonesystem_api.set_torchlight("off")
            if "VIBRA" in self.__load:
                self.phonesystem_api.set_vibration("off")

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def __maintenance_charging(self):
        """
        Do the maintenance charging during

        :rtype: tuple
        :return: different value for next function
        """
        # init var
        temperature_list = []

        # clean old logs
        self.em_api.clean_autolog()

        # set log to be persistent even if the board reboot
        self.em_api.set_persistent_autolog(True)
        phone_start_time = self.get_time_from_board()

        # choose function to put in logger
        self.em_api.add_fct_to_auto_logger(self.em_api.AUTOLOG_THERMAL, "sequenced")
        self.em_api.add_fct_to_auto_logger(self.em_api.AUTOLOG_UEVENT, "sequenced")
        self.em_api.start_auto_logger(60, self.__data_polling, "sequenced")
        # start maintenance charging
        self._logger.info("Maintenance charging started")
        # disconnect board
        self._device.disconnect_board()
        # connect DCP
        self._io_card.wall_charger_connector(True)
        # wait x min
        self._logger.info("Maintenance charging during %s seconds" %
                          self.__maintenance_timeout)

        # 2 var for time stamp to cover logs checking
        host_start_time = time.time()

        time_before_maintenance = (host_start_time - self.start_time)
        while (time.time() - host_start_time) < self.__maintenance_timeout:
            # measure temperature
            time.sleep(self.__data_polling)
            if self.tc_module is not None:
                chamber_value = self.tc_module.get_eq().get_temperature()
            else:
                chamber_value = "ROOM"
            temperature_list.append(chamber_value)

        self._io_card.wall_charger_connector(False)
        # connect USB SDP
        self._io_card.usb_host_pc_connector(True)
        # start to restore ambient temperature
        if self.tc_module is not None:
            self.tc_module.get_eq().set_temperature(self._ambient_temperature)
        # wait x seconds to allow connection to be recognized
        time.sleep(self.usb_sleep)
        # connect board
        self._device.connect_board()

        # check board connection
        if not self.em_core_module.check_board_connection(tries=1,
                                           use_exception=False):
            # if board is not alive charge it for a while
            if self.tc_module is not None:
                self.tc_module.get_eq().wait_for_temperature(
                    self._ambient_temperature)
            self.em_core_module.retrieve_off_board()
            self.em_core_module.check_board_connection()

        self._logger.info("Maintenance charging stopped, compute result")
        phone_stop_time = phone_start_time + (time.time() - host_start_time)
        # stop logger
        self.em_api.stop_auto_logger("sequenced")

        # checking that maintenance started from logs
        maintenance_mode_started = self.phonesystem_api.check_message_in_log(
            "MAINTENANCE_STARTED", phone_start_time, phone_stop_time)
        self._meas_list.add("MAINTENANCE_STARTED", maintenance_mode_started[0], "NONE",
                            maintenance_mode_started[1])
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets, True)

        # get result from log
        msic_result = self.em_api.get_autolog_msic_registers()
        thermal_result = self.em_api.get_autolog_thermal_sensor_info()
        # remove parasite logs
        if len(msic_result) > 2:
            msic_result.pop(-1)
        if len(thermal_result) > 2:
            thermal_result.pop(-1)

        return (time_before_maintenance, phone_start_time,
                temperature_list, msic_result, thermal_result)

#------------------------------------------------------------------------------

    def __compute_hysteresis(self, current, voltage, time_stamp):
        """
        compute hysteresis

        :type current: dict
        :param current: Ibatt dict

        :type voltage: dict
        :param voltage: Vbatt dict

        :type time_stamp: dict
        :param time_stamp: time stamp dict
        """
        self._logger.info("compute hysteresis")
        iteration = 0
        cycle = 0
        discharge_done = False
        charge_done = False

        # get the minimal index where to start to compute
        for meas in current:
            if (meas < 0) and (iteration + 1) < len(current) and \
                    (current[iteration + 1] > 0):
                break
            else:
                iteration += 1
        # init value
        if iteration + 1 < len(current):
            iteration += 1
            vmin = voltage[iteration]
            tmin = time_stamp[iteration]
            vmax = voltage[iteration]
            tmax = time_stamp[iteration]
            tr = tmax - tmin
            hr = vmax - vmin
            tf = tmin - tmax
            hf = vmin - vmax
            imin = current[iteration]
            imax = current[iteration]
            iteration += 1

        while iteration + 1 < len(current):
            # discharge phase
            if current[iteration] > 0 > current[iteration + 1]:
                vmax = voltage[iteration + 1]
                imax = current[iteration]
                tmax = time_stamp[iteration + 1]
                tr = tmax - tmin
                hr = vmax - vmin
                self.__em_hysteresis_tab.add_measurement([
                    ("Tr", tr),
                    ("Hr", hr),
                    ("VMin", vmin),
                    ("VMax", vmax)])
                charge_done = True
            # charge phase
            elif current[iteration] < 0 < current[iteration + 1]:
                vmin = voltage[iteration + 1]
                imin = current[iteration]
                tmin = time_stamp[iteration + 1]
                tf = tmin - tmax
                hf = abs(vmin - vmax)
                self.__em_hysteresis_tab.add_measurement([
                    ("Tf", tf),
                    ("Hf", hf)])
                discharge_done = True

            if charge_done and discharge_done:
                cycle += 1
                self._logger.info("adding cycle : " + str(cycle))
                self.__em_hysteresis_tab.add_measurement([
                    ("cycle", cycle)])
                # calculate verdict
                self._meas_list.add("RISING_TIME", (tr, "hour"))
                self._meas_list.add("FALLING_TIME", (tf, "hour"))
                self._meas_list.add("BATTERY_HYSTERESIS", (hr, "V"))
                self._meas_list.add("IMIN", imin, "A")
                self._meas_list.add("IMAX", imax, "A")
                # Store test information
                self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
                self._meas_list.clean()
                self.__em_hysteresis_tab.switch_to_next_meas()
                discharge_done = False
                charge_done = False

            iteration += 1
        # save last change on the data
        if cycle == 0:
            self._logger.error("No enough exploitable measurements to compute hysteresis")
        else:
            self.__em_hysteresis_tab.switch_to_next_meas()
