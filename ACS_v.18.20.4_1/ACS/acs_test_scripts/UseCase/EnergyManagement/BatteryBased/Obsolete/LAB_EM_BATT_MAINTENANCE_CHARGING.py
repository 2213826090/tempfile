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
:summary: Energy Management Battery Monitor - MOS maintenance charging
:author: vgombert
:since: 27/01/2012 (jan)
.. warning:: WILL BE DELETED SOON
"""
import time
import os

from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase

from UtilitiesFWK.Utilities import Global
import acs_test_scripts.Utilities.EMUtilities as EMUtil
from ErrorHandling.AcsConfigException import AcsConfigException


class LabEmBattMaintenanceCharging(EmUsecaseBase):

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
        # init fuel gauging parameters
        self.em_core_module.init_fg_param()
        # Read MAINTENANCE_CHARGING_TIMEOUT from test case xml file
        self.__maintenance_timeout = \
            int(self._tc_parameters.get_param_value("MAINTENANCE_CHARGING_TIMEOUT"))
        # Read EXPECTED_BATTERY_STATE from test case xml file
        self.__expected_batt_state = \
            str(self._tc_parameters.get_param_value(
                "EXPECTED_BATT_STATE")).upper()
        # Read DATA_POLLING from test case xml file
        self.__data_polling = \
            int(self._tc_parameters.get_param_value(
                "DATA_POLLING"))
        # Read CAPABILITY_TAG from test case xml file
        self._capability_tag = \
            str(self._tc_parameters.get_param_value(
                "CAPABILITY_TAG"))
        # Read MULTIMEDIA_TYPE from test case xml file
        self.__multimedia_type = \
            str(self._tc_parameters.get_param_value("MULTIMEDIA_TYPE")).upper()
        # Get Multimedia Parameters
        self.__multimedia_file = self._tc_parameters.get_param_value("MULTIMEDIA_FILE")
        self.__volume = int(self._tc_parameters.get_param_value("VOLUME"))

        # Get path to multimedia files
        self.__multimedia_path = self._device.multimedia_path

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

        # init variables
        self.__multimedia_api = None
        self._system_api = self._device.get_uecmd("System", True)

#-----------------------------------------------------------------------

    def set_up(self):
        """
        Charge the board to match the start capacity.
        """
        # Call the UseCaseBase Setup function
        EmUsecaseBase.set_up(self)

        # check battery state is in expected value
        if self.__expected_batt_state not in ["CHARGING", "DISCHARGING",
                                              "NOT CHARGING", "FULL"]:
            tmp_txt = "unknown EXPECTED_BATT_STATE value : %s" % \
                str(self.__expected_batt_state)
            self._logger.error(tmp_txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, tmp_txt)

        if self.__multimedia_type in ["AUDIO", "VIDEO"]:
            self.__multimedia_type = self.__multimedia_type.capitalize()
            self.__multimedia_api = self._device.get_uecmd(self.__multimedia_type)
        elif self.__multimedia_type != "NONE":
            tmp_txt = "unknown multimedia type: %s" % \
                str(self.__multimedia_api)
            self._logger.error(tmp_txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, tmp_txt)

        # init capacity
        self.update_battery_info()

        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_BATT_MAINTENANCE_CHARGING", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

        # update battery temperature expected depending of TCT
        if self.tc_module is not None:
            if self._em_targets["THERMAL_MSIC_REGISTER_MAINTENANCE.BATTERY.TEMP"] is not None:
                EMUtil.update_conf(
                    self._em_targets["THERMAL_MSIC_REGISTER_MAINTENANCE.BATTERY.TEMP"],
                    ["lo_lim", "hi_lim"], self._tct, "*")

        # Charge battery
        self.em_core_module.monitor_charging(self.em_core_module.batt_max_capacity, self.em_core_module.charge_time,
                              self.__em_meas_tab)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Do a maintenance charging during x seconds and then compute result.
        """
        EmUsecaseBase.run_test_body(self)
        # do the maintenance charging ------------------------------------------------------------------
        (temperature_list, polling_counter, time_before_maintenance,
            phone_start_time, msic_result, thermal_result) = self.__maintenance_charging()

        # prepare to compute hysteresis ----------------------------------------------------------------
        time_stamp = []
        voltage = []
        current = []
        expected_state_seen = False
        last_voltage = 0

        for iteration in range(polling_counter):
            thermal_stamp = 0
            msic_stamp = 0
            global_stamp = 0
            special_stamp_msic = {}
            special_stamp_thermal = {}

            # store temperature if chamber is here
            if self.tc_module is not None:
                if iteration < len(temperature_list):
                    self.__em_meas_tab.add_measurement(
                        [(self.tc_module.get_chamber_tag(), temperature_list[iteration])])

            # get msic info
            if iteration < len(msic_result):
                msic_dict = msic_result[iteration][1]
                special_stamp_msic = msic_result[iteration][0]
                msic_stamp = special_stamp_msic[self._em_cst.AUTOLOG_TIME]
                msic_stamp = msic_stamp - phone_start_time
                self.__em_meas_tab.add_dict_measurement(msic_dict)

                self.batt_voltage = msic_dict["BATTERY"]["VOLTAGE"][0]
                time_stamp.append(float(msic_stamp) / 3600)
                voltage.append(self.batt_voltage)
                current.append(msic_dict["BATTERY"]["CURRENT_NOW"][0])

                # start to compute verdict after the first battery wanted state is seen
                if msic_dict["BATTERY"]["STATUS"][0].upper() == self.__expected_batt_state and \
                        not expected_state_seen:
                    expected_state_seen = True

                if expected_state_seen:
                    self._meas_list.add_dict("MSIC_REGISTER_MAINTENANCE", msic_dict,
                                             msic_dict["TIME_STAMP"][0])
                    if self.batt_voltage >= last_voltage:
                        self._meas_list.add_dict("MSIC_REGISTER_MAINTENANCE_RISING",
                                                 msic_dict,
                                                 msic_dict["TIME_STAMP"][0])
                    else:
                        self._meas_list.add_dict("MSIC_REGISTER_MAINTENANCE_FALLING",
                                                 msic_dict,
                                                 msic_dict["TIME_STAMP"][0])

                    if self.tc_module is not None:
                        self._meas_list.add_dict("THERMAL_MSIC_REGISTER_MAINTENANCE",
                                                 msic_dict,
                                                 msic_dict["TIME_STAMP"][0])
                last_voltage = self.batt_voltage

            # get thermal info
            if iteration < len(thermal_result):
                thermal_dict = thermal_result[iteration][1]
                special_stamp_thermal = msic_result[iteration][0]
                thermal_stamp = special_stamp_thermal[self._em_cst.AUTOLOG_TIME]
                thermal_stamp = thermal_stamp - phone_start_time
                self.__em_meas_tab.add_dict_measurement(thermal_dict)

                if expected_state_seen and self.tc_module is not None:
                    self._meas_list.add_dict("THERMAL_CONF_MAINTENANCE", thermal_dict,
                                             thermal_dict["TIME_STAMP"][0])

            if expected_state_seen:
                self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
                self._meas_list.clean()
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
                [(self._em_cst.COMMENTS, "Maintenance charging test"),
                 (self._em_cst.REBOOT, reboot),
                 (self._em_cst.TIME_TAG, global_stamp / 3600, self._em_cst.HR)])

            # switch to next meas
            self.__em_meas_tab.switch_to_next_meas()

        # if expected not seen, take a measurement and compare it with target -------------------------------------------------------
        if not expected_state_seen:
            # compute verdict for msic
            if len(msic_dict) > 0:
                msic_dict = msic_result[int(len(msic_result) / 2)][1]
                self.__em_meas_tab.add_dict_measurement(msic_dict)
                ibatt = msic_dict["BATTERY"]["CURRENT_NOW"][0]
                self._meas_list.add_dict("MSIC_REGISTER_MAINTENANCE", msic_dict,
                                         msic_dict["TIME_STAMP"][0])
                if ibatt > 0:
                    self._meas_list.add_dict("MSIC_REGISTER_MAINTENANCE_RISING",
                                             msic_dict,
                                             msic_dict["TIME_STAMP"][0])
                else:
                    self._meas_list.add_dict("MSIC_REGISTER_MAINTENANCE_FALLING",
                                             msic_dict,
                                             msic_dict["TIME_STAMP"][0])

                if self.tc_module is not None:
                    self._meas_list.add_dict("THERMAL_MSIC_REGISTER_MAINTENANCE",
                                             msic_dict,
                                             msic_dict["TIME_STAMP"][0])

            # compute verdict for thermal
            if len(thermal_result) > 0:
                thermal_dict = thermal_result[int(len(thermal_result) / 2)][1]
                self.__em_meas_tab.add_dict_measurement(thermal_dict)
                if self.tc_module is not None:
                    self._meas_list.add_dict("THERMAL_CONF_MAINTENANCE", thermal_dict,
                                             thermal_dict["TIME_STAMP"][0])

            self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
            self._meas_list.clean()

        # compute hysteresis --------------------------------------------------------------------------------------------------
        self.__compute_hysteresis(current, voltage, time_stamp)

        # finish verdict and clean
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

        # clean the board state and retrieve logs
        self.em_core_module.clean_up()

        # clean autolog
        if self.is_board_and_acs_ok():
            self.em_api.clean_autolog()
            if self.__multimedia_api is not None:
                self._multimedia_api.stop()

        return Global.SUCCESS, "No errors"

# maintenance charging ------------------------------------------------------------------------------

    def __maintenance_charging(self):
        """
        part that do the maintenance charging

        :rtype: tuple
        :return: different value for next function
        """
        # init var
        temperature_list = []
        polling_counter = 0

        # clean old logs
        self.em_api.clean_autolog()

        # launch uecmd to help the discharge
        self.phonesystem_api.set_screen_timeout(self.__maintenance_timeout)
        # deactivate set auto brightness
        self.phonesystem_api.set_brightness_mode("manual")
        # set display brightness to max value
        self.phonesystem_api.set_display_brightness(100)

        if self.__multimedia_type in ("VIDEO", "AUDIO"):
            # play music
            self._system_api.adjust_specified_stream_volume("Media", self.__volume)
            self.__multimedia_api.play(self.__multimedia_path +
                                       self.__multimedia_file, True)
        # set log to be persistent even if the board reboot
        self.em_api.set_persistent_autolog(True)

        phone_start_time = self.get_time_from_board()

        # start logger api
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
        time_before_maintenance = (time.time() - self.start_time)
        start_time = time.time()
        while (time.time() - start_time) < self.__maintenance_timeout:
            time.sleep(self.__data_polling)
            polling_counter += 1

            if self.tc_module is not None:
                # measure temperature
                temperature_list.append(
                    self.tc_module.get_eq().get_temperature())

        self._io_card.wall_charger_connector(False)
        # connect USB SDP
        self._io_card.usb_host_pc_connector(True)
        # wait x seconds
        time.sleep(self.usb_sleep)
        # connect board
        self._device.connect_board()

        # check board connection
        if not self.em_core_module.check_board_connection(tries=1,
                                           use_exception=False):
            # if board is not alive charge it for a while
            time.sleep(2)
            self.em_core_module.retrieve_off_board()
            self.em_core_module.check_board_connection()

        self._logger.info("Maintenance charging stopped, compute result")
        # stop logger
        self.em_api.stop_auto_logger("sequenced")
        # get result from log
        msic_result = self.em_api.get_autolog_msic_registers()
        thermal_result = self.em_api.get_autolog_thermal_sensor_info()
        # remove parasite logs
        if len(msic_result) > 2:
            msic_result.pop(0)
            msic_result.pop(-1)
        if len(thermal_result) > 2:
            thermal_result.pop(0)
            thermal_result.pop(-1)

        return (temperature_list, polling_counter, time_before_maintenance,
                phone_start_time, msic_result, thermal_result)

# hysteresis computing ------------------------------------------------------------------------------

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
                hf = vmin - vmax
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
                self._meas_list.add("TR", tr, "hour")
                self._meas_list.add("TF", tf, "hour")
                self._meas_list.add("HR", hr, "V")
                self._meas_list.add("HF", hf, "V")
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
