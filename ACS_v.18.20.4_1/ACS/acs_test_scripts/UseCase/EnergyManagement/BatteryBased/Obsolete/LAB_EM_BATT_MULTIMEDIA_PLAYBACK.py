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
:summary: Energy Management multimedia playback
:author: vgombert
:since: 01/13/2011 (jan)
"""
import time
import os

from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.EMUtilities import update_conf, XMLMeasurementFile, OcvComputingTool

from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.AcsBaseException import AcsBaseException
from ErrorHandling.DeviceException import DeviceException


class LabEmBattVideoPlayback(EmUsecaseBase):

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
        # Read CYCLE_BEHAVIOR from test case xml file
        self.__behavior = str(self._tc_parameters.get_param_value(
            "CYCLE_BEHAVIOR")).upper()
        # Read CHARGING_MILESTONE from test case xml file
        self.__charging_milestone = str(self._tc_parameters.get_param_value(
                                        "CHARGING_MILESTONE")).upper()
        # Read DISCHARGING_MILESTONE from test case xml file
        self.__discharging_milestone = str(self._tc_parameters.get_param_value(
            "DISCHARGING_MILESTONE")).upper()

        #-----------------------------------------------------------------------
        # Read MULTIMEDIA_TYPE from test case xml file
        self.__multimedia_type = \
            str(self._tc_parameters.get_param_value("MULTIMEDIA_TYPE"))
        # Get Multimedia Parameters
        self.__multimedia_path = self._device.multimedia_path
        self.__multimedia_file = self.__multimedia_path + self._tc_parameters.get_param_value("MULTIMEDIA_FILE")
        self.__volume = int(self._tc_parameters.get_param_value("VOLUME"))

        # init api
        self.__multimedia_api = None
        #-----------------------------------------------------------------------
        # Initialize EM  xml object
        # measurement file
        meas_file_name = os.path.join(self._saving_directory,
                                      "EM_meas_report.xml")
        self.__em_meas_tab = XMLMeasurementFile(meas_file_name)
        # enable Global Measurement file
        name = os.path.join(self._campaign_folder,
                            self._em_cst.GLOBAL_MEAS_FILE)
        # enable OCV computing
        self.__em_meas_tab.enable_global_meas(name, self._name)
        # get OCV target file
        target = self.em_core_module.ocv_target_file.parse_em_ocv_targets(
            self._device.get_phone_model(), self._tct)
        self.__ocv_tool = OcvComputingTool(target)
        # get capability targets
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_BATT_MULTIMEDIA_PLAYBACK", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)
        self._system_api = self._device.get_uecmd("System", True)

#-----------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # Call the UseCaseBase Setup function
        EmUsecaseBase.set_up(self)

        if self.__multimedia_type.upper() in ["AUDIO", "VIDEO"]:
            self.__multimedia_type = self.__multimedia_type.capitalize()
            self.__multimedia_api = self._device.get_uecmd(self.__multimedia_type)
        else:
            tmp_txt = "unknown multimedia type: %s" % \
                str(self.__multimedia_api)
            self._logger.error(tmp_txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, tmp_txt)

        # init verdict value
        self.update_battery_info()

        # Check charging milestone
        if self.__behavior in ["DISCHARGE_CHARGE", "CHARGE_DISCHARGE", "CHARGE_ONLY"]:
            for milestone in self.__charging_milestone.split(","):
                if (not milestone.strip().isdigit() and milestone != "FULL") or milestone == "DEAD":
                    txt = "invalid charging milestone '%s', please check CHARGING_MILESTONE parameter" % milestone
                    self._logger.error(txt)
                    raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, txt)

        # Check discharging milestone
        if self.__behavior in ["DISCHARGE_CHARGE", "CHARGE_DISCHARGE", "DISCHARGE_ONLY"]:
            for milestone in self.__discharging_milestone.split(","):
                if (not milestone.strip().isdigit() and milestone != "DEAD") or milestone == "FULL":
                    txt = "invalid discharging milestone '%s', please check DISCHARGING_MILESTONE parameter" % milestone
                    self._logger.error(txt)
                    raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, txt)

        # Charge/discharge battery depending of fuel gauging option used
        if self.__behavior in ["CHARGE_DISCHARGE", "CHARGE_ONLY"]:
            self.em_core_module.monitor_discharging(self.em_core_module.batt_min_capacity, self.em_core_module.discharge_time,
                                     self.__em_meas_tab)
        elif self.__behavior in ["DISCHARGE_CHARGE", "DISCHARGE_ONLY"]:
            self.em_core_module.monitor_charging(self.em_core_module.batt_max_capacity, self.em_core_module.charge_time,
                                  self.__em_meas_tab)
        else:
            txt = "bad CYCLE_BEHAVIOR parameter '%s', you should use  one of this : CHARGE_DISCHARGE, CHARGE_ONLY, DISCHARGE_CHARGE, DISCHARGE_ONLY" % self.__behavior
            self._logger.error(txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, txt)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        EmUsecaseBase.run_test_body(self)

        # order the fuel gauging cycle depending of tc parameters
        message = ""
        test = self.__behavior.split("_")

        for element in test:
            if element == "CHARGE":
                message += "CHARGING TEST\n"
                message += self.__charge_phase()
            elif element == "DISCHARGE":
                message += "DISCHARGING TEST\n"
                message += self.__discharge_no_data()

        self._logger.info(message)

        return(self._em_meas_verdict.get_current_result(),
               message)

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        # call tear down after some operations
        EmUsecaseBase.tear_down(self)
        # clean the measurement file from last ocv computing
        self.__em_meas_tab.remove_meas(
            self.__ocv_tool.get_constant_list())
        # compute ocv and store it in measurement file
        result = self.__ocv_tool.compute_as_meas()
        map(self.__em_meas_tab.add_computed_meas, result)  # pylint: disable=W0141
        # retrieve measurement from test
        self.__em_meas_tab.generate_global_file()
        # as measurement is kept over B2B iteration,
        # just reset global file to its init state
        self.__em_meas_tab.reset_global_file()

        if self.is_board_and_acs_ok():
            # stop multimedia
            if self.__multimedia_api is not None:
                self.__multimedia_api.stop()

        # clean the board state and retrieve logs
        self.em_core_module.clean_up()

        if self.is_board_and_acs_ok():
            self.phonesystem_api.set_screen_timeout(30)
            self.phonesystem_api.clean_daemon_files()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def __evaluate_capacity_time_lap(self, tmp_list, raw_capacity_to_check,
                                     start_capacity, sign_of_charge):
        """
        evaluate capacity time lapse by generating a message.

        :type tmp_list: list
        :param tmp_list: list of tuple which contains (capacity, time_stamp)
        :type capacity_to_check: str
        :param capacity_to_check: raw capacity to check in str format
        :type start_capacity:str
        :param start_capacity:capacity where to start computing time lapse , can be numeric or "DEAD" or "FULL"
        :type sign_of_charge:str
        :param sign_of_charge:change the function behavior, can take value 'charge' or 'discharge'

        :rtype: str
        :rtype: message which contains detail on time lapse
        """
        capacity_to_check = []
        list_len = len(tmp_list)
        # construct a list of capacity to check from str
        for element in raw_capacity_to_check.split(","):
            element = element.strip()
            if element != "":
                if element.isdigit():
                    element = int(element)
                capacity_to_check.append(element)

        # Start capacity can only be declared MIN or MAX capacity
        capacity_txt = str(start_capacity) + "(+/- 1)"
        if start_capacity < 0:
            capacity_txt = "DEAD"
        elif start_capacity == "FULL":
            capacity_txt = "FULL"
        message = "computing time lapse to reach following capacities %s from %s\n" % (
            str(capacity_to_check), capacity_txt)
        # check tmp_list length
        if list_len > 0:
            start_time = None
            # first find start_capacity in your measurement:
            if start_capacity in ["FULL", "DEAD"] or start_capacity < 0:
                start_time = tmp_list[0][1]
            else:
                for capacity, time_stamp, status in tmp_list:
                    # get start capacity at around +/- 1
                    if -1 <= (int(start_capacity) - capacity) <= 1:
                        start_time = time_stamp
                        break

            if start_time is not None:
                #  find target capacity then compute delta
                for target_capacity in capacity_to_check:
                    found = False
                    for capacity, time_stamp, status in tmp_list:
                        if sign_of_charge == "charge":
                            # consider FULL state
                            if target_capacity == "FULL":
                                # check below action only if target_capacity is str
                                if target_capacity == status:
                                    delta = time_stamp - start_time
                                    found = True
                                    break
                            # else consider target equal or superior
                            elif target_capacity <= capacity:
                                delta = time_stamp - start_time
                                found = True
                                break

                        elif sign_of_charge == "discharge":
                            # consider DEAD state
                            if target_capacity == "DEAD":
                                # DEAD capacity can only be the last 0 found
                                if capacity <= 0 and tmp_list.index((capacity, time_stamp, status)) == (list_len - 1):
                                    delta = time_stamp - start_time
                                    found = True
                                    break
                            # else consider target equal or inferior
                            elif target_capacity >= capacity:
                                delta = time_stamp - start_time
                                found = True
                                break
                    if found:
                        message += "- it tooks %ss to reach capacity %s from %s\n" % (delta, target_capacity,
                                                                                      capacity_txt)
                    else:
                        message += "- target %s not found in measurement \n" % target_capacity
            else:
                message = "Failed to find start capacity %s\n" % capacity_txt
        else:
            message = "Failed to compute time lapse, no measurement found\n"

        return message

    def __compute_current_now(self, duration, nb_measurement):
        """
        Read the current now delivered by the battery
        The measurement is done "nb_measurement" times during "duration" seconds
        The connection should have been established before calling this function

        :type  duration: int
        :param duration: number of seconds of measurement
        :type  nb_measurement: int
        :param nb_measurement: number of times that the measurement is done.

        :rtype: int tuple
        :return: return a tuple with following current in A value (min, max, average)
        """
        msic_registers = self.em_api.get_msic_registers()
        min_current = msic_registers["BATTERY"]["CURRENT_NOW"][0]
        max_current = msic_registers["BATTERY"]["CURRENT_NOW"][0]
        average = 0.0
        nb_meas = 0

        sleep_duration = int(duration / nb_measurement)
        if sleep_duration == 0:
            sleep_duration = 1

        while nb_meas < nb_measurement:
            msic_registers = self.em_api.get_msic_registers()
            current_now = msic_registers["BATTERY"]["CURRENT_NOW"][0]

            if min_current > current_now:
                min_current = current_now
            elif max_current < current_now:
                max_current = current_now
            average = (average * nb_meas + current_now) / (nb_meas + 1)
            nb_meas += 1
            time.sleep(sleep_duration)

        return min_current, max_current, average

#------------------------------------------------------------------------------

    def __charge_phase(self):
        """
        Charge the board with a dcp, screen off

        :rtype: str
        :rtype: message which contains detail on time lapse
        """
        # start charging through usb
        self.em_api.set_usb_charging("on")
        # init capacity
        msic = self.update_battery_info()
        battery_status = msic["BATTERY"]["STATUS"][0].upper()
        charging_capacity_list = []

        # store capacity and time for milestone computing purpose
        charging_capacity_list.append((self.batt_capacity,
                                       time.time() - self.start_time, battery_status))
        # set a fake value to capacity if status full is wanted
        max_capacity = 10000
        good_text = "FULL"
        if self.em_core_module.batt_max_capacity.isdigit():
            max_capacity = int(self.em_core_module.batt_max_capacity)
            good_text = str(max_capacity) + "%"

        # verdict message
        result = "Failed to compute time lapse, board already above or equal to %s" % good_text

        # Charge battery, if board is already full then leave
        if (self.em_core_module.batt_max_capacity == "FULL" and battery_status != self.em_core_module.batt_max_capacity) or\
                (self.em_core_module.batt_max_capacity != "FULL" and self.batt_capacity < max_capacity) or battery_status != "FULL":
            self._logger.info("Start to charge battery until %s before %s seconds" %
                              (good_text,
                               self.em_core_module.charging_time_limit))

            # case when no data allowed during charge
            if self._io_card.get_default_wall_charger() == self._io_card.DCP:
                result = self.__charge_no_data(charging_capacity_list, battery_status,
                                               max_capacity, good_text)
            # case when data is allowed during charge
            else:
                result = self.__charge_data(charging_capacity_list, battery_status,
                                            max_capacity, good_text)

        return result

    def __charge_no_data(self, charging_capacity_list, battery_status, max_capacity, good_text):
        """
        private function served to monitor charging when there is no data during charge.

        :type charging_capacity_list: list
        :param charging_capacity_list: list for computing time taken to reach given milestone

        :type battery_status: str
        :param battery_status: last battery status seen

        :type max_capacity: int
        :param max_capacity: max battery capacity reworked

        :type good_text: str
        :param good_text: str formated depending input_max_capacity value

        :rtype: str
        :rtype: message which contains detail on time lapse
        """
        # start autolog here
        self.em_api.clean_autolog()
        # need to set the polling
        self.em_api.set_persistent_autolog(True)
        self.em_api.add_fct_to_auto_logger(self.em_api.AUTOLOG_THERMAL, "sequenced")
        self.em_api.add_fct_to_auto_logger(self.em_api.AUTOLOG_UEVENT, "sequenced")
        self.em_api.start_auto_logger(0, 10, "sequenced")
        keep_looping = True
        measurement_fail = 0
        local_start_time = time.time()
        while keep_looping:
            # charge board
            self.em_core_module.charge_battery(self.em_core_module.charge_time)
            try:
                # parse autolog response
                msic_list = self.em_api.get_autolog_msic_registers()
                thermal_list = self.em_api.get_autolog_thermal_sensor_info()
                self.em_api.reset_running_log()
                # get the highest logs length
                log_length = max(len(thermal_list), len(msic_list))
                for i in range(log_length):

                    try:
                        # get battery/charger info
                        if len(msic_list) > i:
                            msic_dict = msic_list[i]
                            if len(msic_dict) > 1:
                                msic_dict = msic_dict[1]
                                msic_batt = msic_dict["BATTERY"]
                                self.batt_capacity = msic_batt["CAPACITY"][0]
                                self.batt_voltage = msic_batt["VOLTAGE"][0]
                                battery_status = msic_batt["STATUS"][0].upper()
                                # store capacity and time for milestone computing purpose
                                charging_capacity_list.append((self.batt_capacity,
                                                               time.time() - self.start_time, battery_status))
                                # store result on xml
                                self.__em_meas_tab.add_dict_measurement(msic_dict)
                                # get ocv limit
                                ocv_lo_lim, ocv_hi_lim = self.__ocv_tool.get_ocv_limit(self.batt_capacity)
                                self.__em_meas_tab.add_measurement(
                                    [(self._em_cst.OCV_NOW_LOW_LIM, ocv_lo_lim, self.__ocv_tool.UNIT),
                                     (self._em_cst.OCV_NOW_HIGH_LIM, ocv_hi_lim, self.__ocv_tool.UNIT)])
                                # compute battery capacity error
                                battery_capacity_err = (float(msic_batt["CHARGE_NOW"][0]) /
                                                        float(msic_batt["CHARGE_FULL"][0])) * 100 - self.batt_capacity
                                # store verdict
                                self._meas_list.add("BATTERY_CAPACITY_CHARGE_ERR",
                                                    (battery_capacity_err, "none"))
                                self._meas_list.add_dict("MSIC_REGISTER_CHARGE", msic_dict)
                                if self.tc_module is not None:
                                    self._meas_list.add_dict("THERMAL_MSIC_REGISTER_CHARGE", msic_dict)

                        # get thermal info
                        if len(thermal_list) > i:
                            thermal_dict = thermal_list[i]
                            if len(thermal_dict) > 1:
                                # store result on xml
                                self.__em_meas_tab.add_dict_measurement(thermal_dict[1])
                    finally:
                        # Store various information
                        self.__em_meas_tab.add_measurement(
                            [self.get_time_tuple(),
                             (self._em_cst.COMMENTS, "RUNTEST: Charging phase"),
                             (self._em_cst.REBOOT, self.phone_as_reboot)])

                        if self.tc_module is not None:
                            self.__em_meas_tab.add_measurement(
                                [self.tc_module.feed_meas_report()])
                        # reinitialize reboot variable
                        self.phone_as_reboot = False

                        # switch meas to next meas
                        self.__em_meas_tab.switch_to_next_meas()
                        # exit if condition are respected else apply verdict
                        if (self.em_core_module.batt_max_capacity == "FULL" and battery_status == self.em_core_module.batt_max_capacity) or\
                                (self.em_core_module.batt_max_capacity != "FULL" and self.batt_capacity >= max_capacity) or battery_status == "FULL":
                            keep_looping = False
                        else:
                            # apply verdict
                            self._em_meas_verdict.compare_list(self._meas_list, self._em_targets, True)
                            self._em_meas_verdict.judge(ignore_blocked_tc=True)

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
                    raise DeviceException(DeviceException.OPERATION_FAILED, tmp_txt)
                # check the board connection
                self.em_core_module.check_board_connection(tries=1, use_exception=False)

            # stop usecase if board take too long to reach battery max capacity
            if (time.time() - local_start_time) > self.em_core_module.charging_time_limit:
                tmp_txt = "Phone failed to reach %s before %s seconds" % \
                    (good_text, self.em_core_module.charging_time_limit)
                self._logger.error(tmp_txt)
                raise DeviceException(DeviceException.TIMEOUT_REACHED, tmp_txt)

        # stop all daemonized ACS process
        self.phonesystem_api.clean_autolog()

        return self.__evaluate_capacity_time_lap(charging_capacity_list,
                                                 self.__charging_milestone,
                                                 self.em_core_module.batt_min_capacity, "charge")

    def __charge_data(self, charging_capacity_list, battery_status, max_capacity, good_text):
        """
        private function served to monitor charging when there is data during charge.

        :type charging_capacity_list: list
        :param charging_capacity_list: list for computing time taken to reach given milestone

        :type battery_status: str
        :param battery_status: last battery status seen

        :type max_capacity: int
        :param max_capacity: max battery capacity reworked

        :type good_text: str
        :param good_text: str formated depending input_max_capacity value

        :rtype: str
        :rtype: message which contains detail on time lapse
        """
        keep_looping = True
        measurement_fail = 0
        # plug wall charger
        self._io_card.wall_charger_connector(True)
        # turn off screen
        if self.phonesystem_api.get_screen_status():
            self._io_card.press_power_button(0.3)
        local_start_time = time.time()
        while keep_looping:
            try:
                # try to read measurement
                msic_reg = self.update_battery_info()
                battery_status = msic_reg["BATTERY"]["STATUS"][0].upper()
                # get thermal info
                thermal_dict = self.em_api.get_thermal_sensor_info()
                # store capacity and time for milestone computing purpose
                charging_capacity_list.append((self.batt_capacity,
                                               time.time() - self.start_time, battery_status))
                # store result on xml
                self.__em_meas_tab.add_dict_measurement(msic_reg)
                self.__em_meas_tab.add_dict_measurement(thermal_dict)
                # get ocv limit
                ocv_lo_lim, ocv_hi_lim = self.__ocv_tool.get_ocv_limit(self.batt_capacity)
                self.__em_meas_tab.add_measurement(
                    [(self._em_cst.OCV_NOW_LOW_LIM, ocv_lo_lim, self.__ocv_tool.UNIT),
                     (self._em_cst.OCV_NOW_HIGH_LIM, ocv_hi_lim, self.__ocv_tool.UNIT)])

                # compute battery capacity error
                battery_capacity_err = (float(msic_reg["BATTERY"]["CHARGE_NOW"][0]) /
                                        float(msic_reg["BATTERY"]["CHARGE_FULL"][0])) * 100 - self.batt_capacity
                # store verdict
                self._meas_list.add("BATTERY_CAPACITY_CHARGE_ERR",
                                    (battery_capacity_err, "none"))
                self._meas_list.add_dict("MSIC_REGISTER_CHARGE", msic_reg)
                if self.tc_module is not None:
                    self._meas_list.add_dict("THERMAL_MSIC_REGISTER_CHARGE", msic_reg)
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
                    raise DeviceException(DeviceException.OPERATION_FAILED, tmp_txt)

                # check the board connection
                self.em_core_module.check_board_connection(tries=1, use_exception=False)
            finally:
                self.__em_meas_tab.add_measurement(
                    [self.get_time_tuple(),
                     (self._em_cst.COMMENTS, "RUNTEST: Charging phase"),
                     (self._em_cst.REBOOT, self.phone_as_reboot)])

                if self.tc_module is not None:
                    self.__em_meas_tab.add_measurement(
                        [self.tc_module.feed_meas_report()])

                # switch meas to next meas
                self.__em_meas_tab.switch_to_next_meas()
                # exit if condition are respected:
                if (self.em_core_module.batt_max_capacity == "FULL" and battery_status == self.em_core_module.batt_max_capacity) or\
                        (self.em_core_module.batt_max_capacity != "FULL" and self.batt_capacity >= max_capacity) or battery_status == "FULL":
                    keep_looping = False
                else:
                    # apply verdict
                    self._em_meas_verdict.compare_list(self._meas_list, self._em_targets, True)
                    self._em_meas_verdict.judge(ignore_blocked_tc=True)
                # reinitialize reboot variable
                self.phone_as_reboot = False

            # stop usecase if board take too long to reach battery max capacity
            if (time.time() - local_start_time) > self.em_core_module.charging_time_limit:
                tmp_txt = "Phone failed to reach %s before %s seconds" % \
                    (good_text, self.em_core_module.charging_time_limit)
                self._logger.error(tmp_txt)
                raise DeviceException(DeviceException.TIMEOUT_REACHED, tmp_txt)

        return self.__evaluate_capacity_time_lap(charging_capacity_list,
                                                 self.__charging_milestone,
                                                 self.em_core_module.batt_min_capacity, "charge")

#------------------------------------------------------------------------------

    def __discharge_no_data(self):
        """
        Discharge the board with a video playback

        :rtype: str
        :rtype: message which contains detail on time lapse
        """
        msic_reg = self.update_battery_info()
        msic_batt = msic_reg["BATTERY"]
        status = msic_batt["STATUS"][0].upper()
        discharging_capacity_list = []
        # re update target with last charge full value
        if self._em_targets["MSIC_REGISTER_DISCHARGE.BATTERY.CHARGE_NOW"] is not None:
            update_conf(self._em_targets["MSIC_REGISTER_DISCHARGE.BATTERY.CHARGE_NOW"],
                        "hi_lim", msic_reg["BATTERY"]["CHARGE_FULL"][0], "=")

        if self.batt_capacity > self.em_core_module.batt_min_capacity:
            # store capacity and time for milestone computing purpose
            discharging_capacity_list.append((self.batt_capacity,
                                              time.time() - self.start_time, status))
            # launch uecmd to help the discharge
            self.phonesystem_api.set_screen_timeout(3600)
            # deactivate set auto brightness
            self.phonesystem_api.set_brightness_mode("manual")
            # set display brightness to max value
            self.phonesystem_api.set_display_brightness(100)
            # play music
            self._system_api.adjust_specified_stream_volume("Media", self.__volume)
            self.__multimedia_api.play(self.__multimedia_file, True)
            # launch autologger
            self.em_api.set_persistent_autolog(True)
            self.em_api.add_fct_to_auto_logger(self.em_api.AUTOLOG_THERMAL, "sequenced")
            self.em_api.add_fct_to_auto_logger(self.em_api.AUTOLOG_UEVENT, "sequenced")
            self.em_api.start_auto_logger(0, 10, "sequenced")

            good_text = "DEAD"
            if self.em_core_module.batt_min_capacity > -1:
                good_text = str(self.em_core_module.batt_min_capacity) + "%"

            # reset consecutive error
            measurement_fail = 0
            connection_shutdown_counter = 0
            self.phone_as_reboot = False
            self._logger.info("Start to discharge battery until %s" % good_text)
            end_time = time.time() + self.em_core_module.charging_time_limit
            while self.batt_capacity > self.em_core_module.batt_min_capacity:
                # charge board
                if connection_shutdown_counter == 0:
                    self.em_core_module.discharge_battery(self.em_core_module.discharge_time)
                    try:
                        # get multimedia status
                        video_status = "NOT OPENED"
                        try:
                            video_status = self.__multimedia_api.is_playing()[0]
                        except AcsBaseException as e:
                            self._logger.error("error when reading multimedia playing state :%s" % str(e))

                        # parse autolog response and reset them
                        self.update_battery_info()
                        msic_list = self.em_api.get_autolog_msic_registers()
                        thermal_list = self.em_api.get_autolog_thermal_sensor_info()
                        self.em_api.reset_running_log()

                        # get the highest logs length
                        log_length = max(len(thermal_list), len(msic_list))
                        for i in range(log_length):

                            try:
                                # get battery/charger info
                                if len(msic_list) > i:
                                    msic_dict = msic_list[i]
                                    if len(msic_dict) > 1:
                                        msic_reg = msic_dict[1]
                                        # try to read measurement
                                        # get msic registers value after booting
                                        self._meas_list.add_dict("MSIC_REGISTER_DISCHARGE", msic_reg)
                                        msic_batt = msic_reg["BATTERY"]
                                        status = msic_batt["STATUS"][0].upper()
                                        # store capacity and time for milestone computing purpose
                                        discharging_capacity_list.append((self.batt_capacity, time.time() - self.start_time, status))
                                        self.__em_meas_tab.add_dict_measurement(msic_reg)
                                        # get ocv and soc information
                                        ocv_pass = self.__ocv_tool.add(self.batt_capacity, self.batt_voltage)
                                        # get ocv limit
                                        ocv_lo_lim, ocv_hi_lim = self.__ocv_tool.get_ocv_limit(self.batt_capacity)
                                        self.__em_meas_tab.add_measurement(
                                            [(self._em_cst.OCV_NOW_LOW_LIM, ocv_lo_lim, self.__ocv_tool.UNIT),
                                             (self._em_cst.OCV_NOW_HIGH_LIM, ocv_hi_lim, self.__ocv_tool.UNIT),
                                             (self._em_cst.OCV_NOW_LIM_VERDICT, ocv_pass)])

                                        self._meas_list.add_dict("MSIC_REGISTER_DISCHARGE", msic_reg)
                                        # compute battery capacity error
                                        battery_capacity_err = (float(msic_batt["CHARGE_NOW"][0]) /
                                                                float(msic_batt["CHARGE_FULL"][0])) * 100 - self.batt_capacity
                                        self._meas_list.add("BATTERY_CAPACITY_DISCHARGE_ERR",
                                                            (battery_capacity_err, "none"))

                                        # check thermal capabilities only if thermal chamber is used
                                        if self.tc_module is not None:
                                            # Store various information
                                            self._meas_list.add_dict("THERMAL_MSIC_REGISTER_DISCHARGE", msic_reg)

                                # get thermal info
                                if len(thermal_list) > i:
                                    thermal_dict = thermal_list[i]
                                    if len(thermal_dict) > 1:
                                        # store result on xml
                                        self.__em_meas_tab.add_dict_measurement(thermal_dict[1])
                                        self._meas_list.add_dict("THERMAL_CONF_DISCHARGE", thermal_dict[1])

                            finally:
                                # Store various information
                                if self.tc_module is not None:
                                    self.__em_meas_tab.add_measurement([self.tc_module.feed_meas_report()])

                                self.__em_meas_tab.add_measurement(
                                    [self.get_time_tuple(),
                                     (self._em_cst.COMMENTS, "RUNTEST:Multimedia %s discharge cycle" % self.__multimedia_type),
                                     ("R_MULTIMEDIA_PLAYING", video_status),
                                     (self._em_cst.REBOOT, self.phone_as_reboot)])

                                # generate em verdict
                                self._em_meas_verdict.compare_list(self._meas_list, self._em_targets, True)
                                self._em_meas_verdict.judge(ignore_blocked_tc=True)

                                # switch to next meas
                                self.__em_meas_tab.switch_to_next_meas()
                        # reset error
                        # restart multimedia----------------------------------------------------
                        if not video_status :
                            self.phonesystem_api.wake_screen()
                            # play music by restarting the player
                            self.__multimedia_api.play(self.__multimedia_path +
                                                       self.__multimedia_file, True)

                        measurement_fail = 0
                    #-----------------------------------------------------------------------------------------------
                    except AcsBaseException as e:
                        # try to reconnect to the board if uecmd failed
                        self._logger.error("fail to get measurement: " + str(e))
                        measurement_fail += 1

                        # stop the usecase if measurement fail several times.
                        if measurement_fail >= self._consecutive_meas_error:
                            if self.batt_voltage > self.vbatt_mos_shutdown or self.batt_voltage == -1:
                                tmp_txt = "Measurement failed after %s times, stop usecase" % self._consecutive_meas_error
                                self._logger.error(tmp_txt)
                                raise DeviceException(DeviceException.OPERATION_FAILED, tmp_txt)
                            else:
                                self._logger.info("battery must be empty, stop discharging")
                                break
                        # check and retrieve board connection only if the test target is not DEAD battery
                        if self.em_core_module.batt_min_capacity > 0:
                            self.em_core_module.check_board_connection(1, use_exception=False)

                # stop usecase if board take too long to reach battery max capacity
                if time.time() > end_time:
                    tmp_txt = "Phone failed to reach %s before %ss" % (good_text, self.em_core_module.charging_time_limit)
                    self._logger.error(tmp_txt)
                    raise DeviceException(DeviceException.TIMEOUT_REACHED, tmp_txt)

                # Restore load in case of reboot
                if self.phone_as_reboot and self.is_board_and_acs_ok():
                    # relaunch audio/video
                    self.phonesystem_api.wake_screen()
                    time.sleep(self._wait_btwn_cmd)
                    self.__multimedia_api.play(self.__multimedia_file, True)
                    # reset reboot tag here
                    self.phone_as_reboot = False

                # increase the counter if we are not in MOS only in case of DEAD battery as target
                if self.em_core_module.batt_min_capacity < 0:
                    self._logger.info("Waiting 20s to see boot transition")
                    time.sleep(20)
                    if self._device.get_boot_mode() != "MOS":
                        connection_shutdown_counter += 1
                    else:
                        connection_shutdown_counter = 0
                    # evaluate if we are off or in COS
                    if connection_shutdown_counter >= 6:
                        self._logger.info("battery must be empty, stop discharging")
                        break

            # when leaving this function stop multimedia and restart charging
            if self.is_board_and_acs_ok():
                if self.__multimedia_api is not None:
                    self.__multimedia_api.stop()

        return self.__evaluate_capacity_time_lap(discharging_capacity_list,
                                                 self.__discharging_milestone,
                                                 self.em_core_module.batt_max_capacity, "discharge")

#------------------------------------------------------------------------------

    def __discharge_data(self):
        """
        Discharge the board with a video playback

        :rtype: str
        :rtype: message which contains detail on time lapse
        """
        msic_reg = self.update_battery_info()
        msic_batt = msic_reg["BATTERY"]
        status = msic_batt["STATUS"][0].upper()
        discharging_capacity_list = []
        # re update target with last charge full value
        if self._em_targets["MSIC_REGISTER_DISCHARGE.BATTERY.CHARGE_NOW"] is not None:
            update_conf(self._em_targets["MSIC_REGISTER_DISCHARGE.BATTERY.CHARGE_NOW"],
                        "hi_lim", msic_reg["BATTERY"]["CHARGE_FULL"][0], "=")

        if self.batt_capacity > self.em_core_module.batt_min_capacity:
            # store capacity and time for milestone computing purpose
            discharging_capacity_list.append((self.batt_capacity,
                                              time.time() - self.start_time, status))
            # launch uecmd to help the discharge
            self.phonesystem_api.set_screen_timeout(3600)
            # deactivate set auto brightness
            self.phonesystem_api.set_brightness_mode("manual")
            # set display brightness to max value
            self.phonesystem_api.set_display_brightness(100)
            # stop charging through usb
            self.em_api.set_usb_charging("off")
            # play music
            self._system_api.adjust_specified_stream_volume("Media", self.__volume)
            self.__multimedia_api.play(self.__multimedia_file, True)

            good_text = "DEAD"
            if self.em_core_module.batt_min_capacity > -1:
                good_text = str(self.em_core_module.batt_min_capacity) + "%"

            # reset consecutive error
            measurement_fail = 0
            self.phone_as_reboot = False
            self._logger.info("Start to discharge battery until %s" % good_text)
            local_start_time = time.time()
            while self.batt_capacity > self.em_core_module.batt_min_capacity:
                try:
                    # try to read measurement
                    # get msic registers value after booting
                    msic_reg = self.update_battery_info()
                    self._meas_list.add_dict("MSIC_REGISTER_DISCHARGE", msic_reg)
                    msic_batt = msic_reg["BATTERY"]
                    status = msic_batt["STATUS"][0].upper()
                    # store capacity and time for milestone computing purpose
                    discharging_capacity_list.append((self.batt_capacity,
                                                      time.time() - self.start_time, status))
                    self.__em_meas_tab.add_dict_measurement(msic_reg)
                    # get ocv and soc information
                    ocv_pass = self.__ocv_tool.add(self.batt_capacity,
                                                   self.batt_voltage)
                    # get ocv limit
                    ocv_lo_lim, ocv_hi_lim = self.__ocv_tool.get_ocv_limit(self.batt_capacity)
                    self.__em_meas_tab.add_measurement(
                        [(self._em_cst.OCV_NOW_LOW_LIM, ocv_lo_lim, self.__ocv_tool.UNIT),
                         (self._em_cst.OCV_NOW_HIGH_LIM, ocv_hi_lim, self.__ocv_tool.UNIT),
                         (self._em_cst.OCV_NOW_LIM_VERDICT, ocv_pass)])

                    # get thermal
                    thermal_conf = self.em_api.get_thermal_sensor_info()
                    self.__em_meas_tab.add_dict_measurement(thermal_conf)
                    self._meas_list.add_dict("MSIC_REGISTER_DISCHARGE", msic_reg)
                    # compute battery capacity error
                    battery_capacity_err = (float(msic_batt["CHARGE_NOW"][0]) /
                                            float(msic_batt["CHARGE_FULL"][0])) * 100 - self.batt_capacity
                    self._meas_list.add("BATTERY_CAPACITY_DISCHARGE_ERR",
                                        (battery_capacity_err, "none"))

                    # check thermal capabilities only if thermal chamber is used
                    if self.tc_module is not None:
                        # Store various information
                        self._meas_list.add_dict("THERMAL_MSIC_REGISTER_DISCHARGE", msic_reg)
                        self._meas_list.add_dict("THERMAL_CONF_DISCHARGE", thermal_conf)

                    # get multimedia info------------------------------------------------------
                    video_status = "NOT OPENED"
                    try:
                        video_status = self.__multimedia_api.is_playing()[0]
                        # restart multimedia----------------------------------------------------
                        if not video_status:
                            self.phonesystem_api.wake_screen()
                            # play music by restarting the player
                            self.__multimedia_api.play(self.__multimedia_path +
                                                       self.__multimedia_file, True)
                    except AcsBaseException as e:
                        self._logger.error("error when reading multimedia playing state :%s" % str(e))
                    finally:
                        self.__em_meas_tab.add_measurement(
                            [("R_MULTIMEDIA_PLAYING", video_status)])

                    #------------------------------------------------------------------------
                    # stop charging through usb
                    self.em_api.set_usb_charging("off")
                    # reset consecutive fail
                    measurement_fail = 0

                except AcsBaseException as e:
                    # try to reconnect to the board if uecmd failed
                    self._logger.error("fail to get measurement: %s" % str(e))
                    measurement_fail += 1

                    # stop the usecase if measurement fail several times.
                    if measurement_fail >= self._consecutive_meas_error:
                        if self.batt_voltage > self.vbatt_mos_shutdown or \
                                self.batt_voltage == -1:
                            tmp_txt = "Measurement failed after %s times, stop usecase" % \
                                self._consecutive_meas_error
                            self._logger.error(tmp_txt)
                            raise DeviceException(DeviceException.OPERATION_FAILED, tmp_txt)

                        else:
                            self._logger.info("battery must be empty, stop discharging")
                            break
                    elif self._device.get_boot_mode() == "COS" and self.em_core_module.batt_min_capacity < 0:
                        self._logger.info("board booted in COS, stop discharging")
                        break
                    # check the board connection
                    self.em_core_module.check_board_connection(1, use_exception=False)
                finally:
                    # Store various information
                    if self.tc_module is not None:
                        self.__em_meas_tab.add_measurement(
                            [self.tc_module.feed_meas_report()])

                    self.__em_meas_tab.add_measurement(
                        [self.get_time_tuple(),
                         (self._em_cst.COMMENTS, "RUNTEST:Multimedia %s discharge cycle" % self.__multimedia_type),
                         (self._em_cst.REBOOT, self.phone_as_reboot)])

                    # generate em verdict
                    self._em_meas_verdict.compare_list(self._meas_list, self._em_targets, True)
                    self._em_meas_verdict.judge(ignore_blocked_tc=True)

                    # switch to next meas
                    self.__em_meas_tab.switch_to_next_meas()

                # stop usecase if board take too long to reach battery min capacity
                if (time.time() - local_start_time) > self.em_core_module.charging_time_limit:
                    tmp_txt = "Phone failed to discharge to %s before %s seconds" % \
                        (good_text, self.em_core_module.charging_time_limit)
                    self._logger.error(tmp_txt)
                    raise DeviceException(DeviceException.TIMEOUT_REACHED, tmp_txt)

                # restart uecmd if board have shutdown
                if self.has_board_reboot():
                    # stop charging through usb
                    self.em_api.set_usb_charging("off")
                    # relaunch audio/video
                    self.phonesystem_api.wake_screen()
                    time.sleep(self._wait_btwn_cmd)
                    self._system_api.adjust_specified_stream_volume("Media", self.__volume)
                    self.__multimedia_api.play(self.__multimedia_file, True)
                    # reset reboot tag here
                    self.phone_as_reboot = False

            # when leaving this function stop multimedia and restart charging
            if self.is_board_and_acs_ok():
                self.em_api.set_usb_charging("on")
                if self.__multimedia_api is not None:
                    self.__multimedia_api.stop()

        return self.__evaluate_capacity_time_lap(discharging_capacity_list,
                                                 self.__discharging_milestone,
                                                 self.em_core_module.batt_max_capacity, "discharge")
