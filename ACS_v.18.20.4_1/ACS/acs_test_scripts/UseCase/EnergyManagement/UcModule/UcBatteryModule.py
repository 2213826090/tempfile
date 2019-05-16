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
:summary: Battery module for EM usecase: io card and real battery
:author: vgombert
:since: September 17th 2013
"""

import time
from UtilitiesFWK.Utilities import str_to_bool
from ErrorHandling.AcsBaseException import AcsBaseException
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.Utilities.CommunicationUtilities import ConfigsParser
from acs_test_scripts.Utilities.EMUtilities import EMConstant
import threading


class UcBatteryModule():

    """
    Lab Energy Management base class.
    """
    __LOG_TAG = "[BATTERY_BASE]\t"
    TYPE = "BATTERY_BENCH"

    def __init__(self, uc_base):
        """
        Constructor
        """
        # collect var pointer from uc_base
        self._uc_base = uc_base
        # Var common to all usecase, these warning are known
        # and can't be remove without changing framework var
        self._tc_parameters = self._uc_base._tc_parameters
        self._logger = self._uc_base._logger
        self._io_card = self._uc_base._io_card
        self._device = self._uc_base._device
        # var from em usecase base
        self.tc_module = self._uc_base.tc_module
        self.__em_api = self._device.get_uecmd("EnergyManagement", True)
        self.__phonesystem_api = self._device.get_uecmd("PhoneSystem", True)
        self.__networking_api = self._device.get_uecmd("Networking", True)
        self.__bt_api = self._device.get_uecmd("LocalConnectivity", True)

        # var only for this module
        self.batt_max_capacity = None
        self.batt_min_capacity = None
        self.batt_start_capacity = None
        self.charge_time = None
        self.discharge_time = None
        self.__data_with_wall_charger = None

        # parse ocv target
        self.ocv_target_file = ConfigsParser("Energy_Management_OCV_limit")
        # init variables
        # var for robustness
        self._charge_empty_batt_time = 1800
        # INIT PARAMETERS HERE
        # param related to usecase
        self.__consecutive_meas_error = 5
        # param related to device
        self.charging_time_limit = 10 * 3600
        self.discharging_time_limit = 15 * 3600  # 900
        # param that are designed to be updated
        self._em_cst = EMConstant()

    def init_fg_param(self):
        """
        init parameter used to do fuel gauging
        """
        # max batt capacity to reach before going to test next step
        self.batt_max_capacity = self._tc_parameters.get_param_value("BATT_MAX_CAPACITY", "").upper()
        # min batt capacity to reach before going to test next step
        self.batt_min_capacity = self._tc_parameters.get_param_value("BATT_MIN_CAPACITY", "").upper()
        # batt capacity to reach before going to test next step, imply that a charge or discharge will be done
        self.batt_start_capacity = self._tc_parameters.get_param_value("BATT_START_CAPACITY", "").upper()
        # below are time var used to let board discharge or charge before checking capacity
        self.charge_time = self._tc_parameters.get_param_value("CHARGE_TIME", 900, default_cast_type=int)
        self.discharge_time = self._tc_parameters.get_param_value("DISCHARGE_TIME", 120, default_cast_type=int)

        if self.batt_start_capacity == "DEAD":
            self.batt_start_capacity = -1
        elif str(self.batt_start_capacity).isdigit():
            self.batt_start_capacity = int(self.batt_start_capacity)

        if self.batt_min_capacity == "DEAD":
            self.batt_min_capacity = -1
        elif str(self.batt_min_capacity).isdigit():
            self.batt_min_capacity = int(self.batt_min_capacity)

#------------------------------------------------------------------------------
    def is_batt_capacity_below_target(self, em_info, batt_target):
        """
        compare last seen battery capacity with the target and return True
        if you current capacity is strictly below it.
        Below mean not FULL or a capacity below it.
        used for setup purpose

        :type batt_target: int or str
        :param batt_target: the capacity target you want to compare with em_info

        :type em_info: dict
        :param em_info: info from uecmd get_msic_registers()

        :rtype: boolean
        :return: True if last seen capacity is below given target
        """
        result = False
        status = em_info["BATTERY"]["STATUS"][0]
        capacity = em_info["BATTERY"]["CAPACITY"][0]

        if status != "FULL":
            if str(batt_target).isdigit() and capacity < int(batt_target):
                result = True
            elif batt_target == "FULL":
                result = True

        return result

    def is_batt_capacity_above_target(self, em_info, batt_target):
        """
        compare em info capacity with the target and return True
        if you current capacity is strictly above it.
        used for setup purpose

        :type batt_target: int or str
        :param batt_target: the capacity target you want to compare with em_info

        :type em_info: dict
        :param em_info: info from uecmd get_msic_registers()

        :rtype: boolean
        :return: True if last seen capacity is above given target
        """
        result = False
        status = em_info["BATTERY"]["STATUS"][0]
        capacity = em_info["BATTERY"]["CAPACITY"][0]

        if status != "FULL":
            if str(batt_target).isdigit() and capacity > int(batt_target):
                result = True
        elif status == "FULL" and batt_target != "FULL":
            result = True

        return result

#------------------------------------------------------------------------------

    def monitor_charging(self, input_max_capacity, charge_time, meas_tab=None):
        """
        monitor the charging through dcp.

        :type input_max_capacity: int
        :param input_max_capacity: max battery capacity

        :type charge_time: int
        :param charge_time: time to spend with dcp plug in seconds

        :type meas_tab: XMLMeasurementFile
        :param meas_tab: object that represent store monitoring results

        :rtype: int
        :return: battery capacity reached
        """
        # init capacity
        msic = self._uc_base.update_battery_info()
        battery_status = msic["BATTERY"]["STATUS"][0]

        # set a fake value to capacity if status full is wanted
        good_text = "FULL"
        max_capacity = 110
        if isinstance(input_max_capacity, int) or str(input_max_capacity).isdigit():
            max_capacity = int(input_max_capacity)
            good_text = str(max_capacity) + "%"

        # Charge battery if board is not full or capacity lower than expected one
        # if we are already at FULL then exit, we can charge more
        if battery_status == "FULL":
            self._logger.warning(self.__LOG_TAG + "DUT battery is already at %s at %s%% of capacity, cant charge more" % (battery_status, self._uc_base.batt_capacity))
        elif input_max_capacity == "FULL" or (str(input_max_capacity).isdigit() and self._uc_base.batt_capacity < max_capacity):
            self._logger.info(self.__LOG_TAG + "Start to charge battery until %s before %s seconds" %
                              (good_text,
                               self.charging_time_limit))

            # do this only one time
            if self.__data_with_wall_charger is None:
                self.__data_with_wall_charger = self.is_host_connection_available_when_charger_plug(self._io_card.get_default_wall_charger())

            # case of no data allowed during charger
            if not self.__data_with_wall_charger:
                self.__charge_when_no_data(input_max_capacity, max_capacity, good_text,
                                           charge_time, meas_tab)
            # case when data is allowed
            else:
                self.__charge_when_data(input_max_capacity, max_capacity,
                                        good_text, meas_tab)
        return self._uc_base.batt_capacity

    def __charge_when_no_data(self, input_max_capacity, max_capacity, good_text,
                              charge_time, meas_tab):
        """
        private function served to monitor charging when there is no data during charge.
        when seeking for FULL state, the state will be checked from measured element during charger plug
        as the usb switching may make you lost the FULL status.
        This mean that you can rely on this function only to be near to FULL but not a FULL after leaving it.

        :type input_max_capacity: int or str
        :param input_max_capacity: max battery capacity, can be FULL or a number

        :type max_capacity: int
        :param max_capacity: max battery capacity reworked, can only be int

        :type good_text: str
        :param good_text: str formated depending input_max_capacity value

        :type charge_time: int
        :param charge_time: time to spend with dcp plug in seconds

        :type meas_tab: XMLMeasurementFile
        :param meas_tab: object that represent store monitoring results
        """
        # init var
        measurement_fail = 0
        adjusted_charging_time = charge_time
        ratio_d = charge_time
        last_capacity = self._uc_base.batt_capacity
        last_vbatt = self._uc_base.batt_voltage
        # use thread to avoid blocking action when computing measurement
        lock = threading.Lock()
        thread_list = []

        # start autolog here
        self.__em_api.clean_autolog()
        # need to set the polling
        self.__em_api.set_persistent_autolog(True)
        self.__em_api.add_fct_to_auto_logger(self.__em_api.AUTOLOG_THERMAL, "sequenced")
        self.__em_api.add_fct_to_auto_logger(self.__em_api.AUTOLOG_UEVENT, "sequenced")
        self.__em_api.start_auto_logger(0, 10, "sequenced")

        # init var
        keep_looping = True
        local_start_time = time.time()
        timeout_capacity_decrease = 900 + adjusted_charging_time
        start_time_capacity_decrease = local_start_time

        try:
            while keep_looping:
                # charge board
                self.charge_battery(adjusted_charging_time)

                try:
                    # parse autolog response and reset them
                    msic_dict = self._uc_base.update_battery_info()
                    battery_status = msic_dict["BATTERY"]["STATUS"][0].upper()
                    msic_list = self._uc_base.em_api.get_autolog_msic_registers()
                    thermal_list = self._uc_base.em_api.get_autolog_thermal_sensor_info()
                    self._uc_base.em_api.reset_running_log()

                    # move to thread next computing
                    t = threading.Thread(target=self.__threaded_feed_meas, args=(lock, msic_list, thermal_list, meas_tab,
                                                                                 "SETUP:Charging while in idle"))
                    thread_list.append(t)
                    t.start()

                    # exit if condition are respected:
                    if (battery_status == "FULL") or (self._uc_base.batt_capacity >= max_capacity and input_max_capacity != "FULL"):
                        keep_looping = False
                    else:
                        # seeking for FULL from autolog measurement
                        for i in range(len(msic_list)):
                            # get battery/charger info
                            msic_dict = msic_list[i]
                            if len(msic_dict) > 1:
                                if msic_dict[1]["BATTERY"]["STATUS"][0].upper() == "FULL":
                                    keep_looping = False
                                    break

                        # compute the smart charging time
                        earn_capacity = self._uc_base.batt_capacity - last_capacity
                        self._logger.info("%s capacity increased during %ss" % (earn_capacity, adjusted_charging_time))
                        # Detect if battery is discharging or not decreasing, if it is the case stop usecase after a while
                        if earn_capacity > 0 and keep_looping:
                            next_theorical_capacity = self._uc_base.batt_capacity + earn_capacity
                            # means that the timer is too long and we may be above the wanted capacity
                            # we need to reduce the charging time
                            # compadjust_time_for_one_iterute a ration capacity/second
                            ratio_d = float(adjusted_charging_time) / float(earn_capacity)
                            adjusted_charging_time = int(max((max_capacity - self._uc_base.batt_capacity), 1) * ratio_d)
                            if next_theorical_capacity > max_capacity:
                                # limit the discharge time to 60s at min
                                adjusted_charging_time = max(adjusted_charging_time, 60)
                            # we are below or equal to theoretical capacity thus we can wait a bigger time
                            else:
                                # limit the discharge time to 3600s at max
                                adjusted_charging_time = min(adjusted_charging_time, 3600)
                            # reinitialize the capacity increase counter
                            start_time_capacity_decrease = time.time()
                            self._logger.info("next adjusted charging time will be set to %ss" % (adjusted_charging_time))
                            last_capacity = self._uc_base.batt_capacity
                        elif keep_looping:
                            tmp_val = adjusted_charging_time
                            # if charge fail , we increase the adjust time to 1 iteration
                            adjusted_charging_time = min(adjusted_charging_time + ratio_d, 3600)
                            if tmp_val != adjusted_charging_time:
                                tmp_msg = "next adjusted charging time will be increased by %ss : %ss" % (ratio_d, adjusted_charging_time)
                            else:
                                tmp_msg = "next adjusted charging time will stay at %ss" % adjusted_charging_time
                            self._logger.info(tmp_msg)

                        # stop usecase if board take too long to reach battery capacity
                        if ((time.time() - start_time_capacity_decrease) > timeout_capacity_decrease) and keep_looping and last_vbatt > self._uc_base.batt_voltage :
                            tmp_txt = "board capacity keep decreasing during %ss instead of increasing; abort usecase" % timeout_capacity_decrease
                            self._logger.error(tmp_txt)
                            raise DeviceException(DeviceException.TIMEOUT_REACHED, tmp_txt)

                    # reset error
                    measurement_fail = 0
                except AcsBaseException as e:
                    # exit if we meet not decreasing capacity case
                    if e.get_generic_error_message() == DeviceException.TIMEOUT_REACHED:
                        raise e
                    # reinitialize the capacity increase counter
                    start_time_capacity_decrease = time.time()
                    # try to reconnect to the board if uecmd failed
                    self._logger.error(self.__LOG_TAG + "fail to get measurement: " + str(e))
                    measurement_fail += 1

                    # stop the usecase if measurement fail several times.
                    if measurement_fail >= self.__consecutive_meas_error:
                        tmp_txt = "Measurement failed after %s times, abort usecase" % \
                            self.__consecutive_meas_error
                        self._logger.error(self.__LOG_TAG + tmp_txt)
                        raise DeviceException(DeviceException.OPERATION_FAILED, tmp_txt)

                    # check the board connection
                    self.check_board_connection(tries=1, use_exception=False)

                # stop usecase if board take too long to reach battery max capacity
                if (time.time() - local_start_time) > self.charging_time_limit and keep_looping:
                    tmp_txt = "Phone failed to reach %s before %s seconds" % \
                        (good_text, self.charging_time_limit)
                    self._logger.error(self.__LOG_TAG + tmp_txt)
                    raise DeviceException(DeviceException.TIMEOUT_REACHED, tmp_txt)
            # stop all daemonized ACS process
            if self._uc_base.is_board_and_acs_ok():
                self.__phonesystem_api.clean_autolog()
        finally:
            # parse the thread list and wait then delete all started threads
            for thread_ele in thread_list:
                thread_ele.join()
                del thread_ele
            del thread_list

    def __charge_when_data(self, input_max_capacity, max_capacity, good_text, meas_tab):
        """
        private function served to monitor charging when there is data during charge.

        :type input_max_capacity: int
        :param input_max_capacity: max battery capacity

        :type max_capacity: int
        :param max_capacity: max battery capacity reworked

        :type good_text: str
        :param good_text: str formated depending input_max_capacity value

        :type charge_time: int
        :param charge_time: time to spend with dcp plug in seconds

        :type meas_tab: XMLMeasurementFile
        :param meas_tab: object that represent stored monitored results
        """
        keep_looping = True
        measurement_fail = 0
        # plug wall charger
        self._io_card.wall_charger_connector(True)
        # turn off screen
        if self.__phonesystem_api.get_screen_status():
            self._io_card.press_power_button(0.3)
        local_start_time = time.time()

        while keep_looping:
            try:
                # try to read measurement
                msic_reg = self._uc_base.update_battery_info()
                battery_status = msic_reg["BATTERY"]["STATUS"][0].upper()
                # get thermal info
                thermal_conf = self.__em_api.get_thermal_sensor_info()
                # store result on xml
                if meas_tab is not None:
                    meas_tab.add_dict_measurement(msic_reg)
                    meas_tab.add_dict_measurement(thermal_conf)
                measurement_fail = 0

            except AcsBaseException as e:
                # try to reconnect to the board if uecmd failed
                self._logger.error(self.__LOG_TAG + "fail to get measurement: " + str(e))
                measurement_fail += 1
                # stop the usecase if measurement fail several times.
                if measurement_fail >= self.__consecutive_meas_error:
                    tmp_txt = "Measurement failed after %s times, abort usecase" % \
                        self.__consecutive_meas_error
                    self._logger.error(self.__LOG_TAG + tmp_txt)
                    self._io_card.wall_charger_connector(False)
                    raise DeviceException(DeviceException.OPERATION_FAILED, tmp_txt)
                # check the board connection
                self.check_board_connection(1, use_exception=False)
            finally:
                if meas_tab is not None:
                    # Store various information
                    meas_tab.add_measurement(
                        [self._uc_base.get_time_tuple(), (self._em_cst.COMMENTS, "SETUP:Charging while in idle"),
                                            (self._em_cst.REBOOT, self._uc_base.phone_as_reboot)])

                    if self.tc_module is not None:
                        meas_tab.add_measurement([self.tc_module.feed_meas_report()])
                    # switch meas to next meas
                    meas_tab.switch_to_next_meas()

                # exit if condition are respected:
                if (input_max_capacity == "FULL" and battery_status == input_max_capacity) or\
                        (input_max_capacity != "FULL" and self._uc_base.batt_capacity >= max_capacity) or\
                        battery_status == "FULL":
                    keep_looping = False

            # stop usecase if board take too long to reach battery max capacity
            if (time.time() - local_start_time) > self.charging_time_limit:
                tmp_txt = "Phone failed to reach %s before %s seconds" % \
                    (good_text, self.charging_time_limit)
                self._logger.error(self.__LOG_TAG + tmp_txt)
                self._io_card.wall_charger_connector(False)
                raise DeviceException(DeviceException.TIMEOUT_REACHED, tmp_txt)
        # remove wall charger at the end
        self._io_card.wall_charger_connector(False)

    def monitor_discharging(self, input_min_capacity, discharge_time, meas_tab=None, load_module=None):
        """
        monitor the discharging through dcp.

        :type input_min_capacity: int
        :param input_min_capacity: max battery capacity

        :type discharge_time: int
        :param discharge_time: time to spend with dcp plug in seconds

        :type meas_tab: XMLMeasurementFile
        :param meas_tab: object that represent store monitoring results

        :type load_module: LoadModule
        :param load_module: object that handle the load activation

        :rtype: float
        :return: return the discharge ration time/capacity
        """
        self._uc_base.update_battery_info()
        # set a fake value to capacity if status dead is wanted
        # cosmetic text computing
        good_text = "DEAD"
        ratio_d = 0
        # convert min_capacity into int
        min_capacity = -1
        if str(input_min_capacity).isdigit():
            min_capacity = int(input_min_capacity)
        if min_capacity > -1:
            good_text = str(min_capacity) + "%"

        # reset consecutive error
        self._uc_base.phone_as_reboot = False
        if self._uc_base.batt_capacity > min_capacity:
            self._logger.info(self.__LOG_TAG + "Start to discharge battery until %s before %s seconds" % (good_text,
                                                                                         self.discharging_time_limit))
            # launch uecmd to help the discharge
            if load_module is None:
                self.__phonesystem_api.set_phone_lock(0)
                self.__phonesystem_api.set_vibration("on")
                self.__phonesystem_api.set_torchlight("on")
                self.__networking_api.set_wifi_power("on")
                self.__bt_api.set_bt_power("on")
                # when pushing on power button
                if not self.__phonesystem_api.get_screen_status():
                    self.__phonesystem_api.wake_screen()
            else:
                self._logger.info(self.__LOG_TAG + "load detected")
                load_module.start_load()

            # case of no data allowed during discharger
            if self._uc_base.phone_info["GENERAL"].get("DISCHARGE_TYPE") == "soft":
                ratio_d = self.__discharge_when_data(min_capacity, good_text, meas_tab, load_module)
            # case when data is allowed
            else:
                ratio_d = self.__discharge_when_no_data(min_capacity, good_text, discharge_time, meas_tab, load_module)
        return ratio_d

    def __discharge_when_no_data(self, min_capacity, good_text, discharge_time, meas_tab, load_module):
        """
        private function served to monitor discharging when there is no data during charge.
        attempt to use a smart discharging way to adjust the battery capacity

        :type min_capacity: int
        :param min_capacity: max battery capacity reworked

        :type good_text: str
        :param good_text: str formated depending input_max_capacity value

        :type discharge_time: int
        :param discharge_time: time to spend with dcp plug in seconds

        :type meas_tab: XMLMeasurementFile
        :param meas_tab: object that represent store monitoring results

        :type load_module: LoadModule
        :param load_module: object that handle the load activation

        :rtype: float
        :return: return the discharge ration time/capacity
        """
        # init var
        measurement_fail = 0
        connection_shutdown_counter = 0
        adjusted_discharging_time = discharge_time
        ratio_d = 0
        last_capacity = self._uc_base.batt_capacity

        # start autolog here
        self.__em_api.clean_autolog()
        # need to set the polling
        self.__em_api.set_persistent_autolog(True)
        self.__em_api.add_fct_to_auto_logger(self.__em_api.AUTOLOG_THERMAL, "sequenced")
        self.__em_api.add_fct_to_auto_logger(self.__em_api.AUTOLOG_UEVENT, "sequenced")
        self.__em_api.start_auto_logger(0, 5, "sequenced")
        # use thread to avoid blocking action when computing measurement
        lock = threading.Lock()
        thread_list = []
        timeout_capacity_increase = 900 + discharge_time
        start_time_capacity_increase = time.time()
        end_time = time.time() + self.discharging_time_limit
        # count the number of discharge in case of small discharge time to take into account the whole waiting time
        # when doing discharge time adjustement
        discharge_iteration = 1

        while self._uc_base.batt_capacity > min_capacity:
            # charge board
            if connection_shutdown_counter == 0:
                self.discharge_battery(adjusted_discharging_time)
                try:
                    # parse autolog response and reset them
                    self._uc_base.update_battery_info()
                    msic_list = self.__em_api.get_autolog_msic_registers()
                    thermal_list = self.__em_api.get_autolog_thermal_sensor_info()
                    self.__em_api.reset_running_log()

                    # move to thread next computement
                    t = threading.Thread(target=self.__threaded_feed_meas, args=(lock, msic_list, thermal_list, meas_tab,
                                                                                 "SETUP:Discharge with load when no data connection"))
                    thread_list.append(t)
                    t.start()
                    # compute the smart charging
                    lost_capacity = last_capacity - self._uc_base.batt_capacity
                    self._logger.info(self.__LOG_TAG + "%s capacity decreased  during %ss" % (lost_capacity, adjusted_discharging_time))
                    # Detect if battery is charging or not decreasing if it is the case stop usecase
                    if lost_capacity > 0:
                        next_theorical_capacity = self._uc_base.batt_capacity - lost_capacity
                        # means that the timer is too long and we may be below the wanted capacity
                        # we need to reduce the charging time
                        # compute a ration capacity/second
                        ratio_d = float(adjusted_discharging_time * discharge_iteration) / float(lost_capacity)
                        adjusted_discharging_time = int(max((self._uc_base.batt_capacity - min_capacity), 1) * ratio_d)
                        if next_theorical_capacity < min_capacity:
                            # limit the discharge time to 60s at min
                            adjusted_discharging_time = min(adjusted_discharging_time, 60)
                        # we are above the capacity thus we can wait a bigger time
                        else:
                            # limit the discharge time to 3600s at max
                            adjusted_discharging_time = min(adjusted_discharging_time, 3600)
                        start_time_capacity_increase = time.time()
                        self._logger.info(self.__LOG_TAG + "next adjusted discharging time will be %ss" % (adjusted_discharging_time))
                        # update last capacity only if we are not charging
                        last_capacity = self._uc_base.batt_capacity
                        discharge_iteration = 1
                    else:
                        discharge_iteration + 1

                    if load_module is not None and not ((time.time() - start_time_capacity_increase) > timeout_capacity_increase):
                        load_module.restart_load(consider_only_checkable_load=True)

                    # stop usecase if board take too long to reach battery capacity
                    if (time.time() - start_time_capacity_increase) > timeout_capacity_increase:
                        tmp_txt = "board capacity keep increasing during more than %ss instead of decreasing; abort usecase" % timeout_capacity_increase
                        self._logger.error(self.__LOG_TAG + tmp_txt)
                        raise DeviceException(DeviceException.TIMEOUT_REACHED, tmp_txt)

                    # reset error
                    measurement_fail = 0
                except AcsBaseException as e:
                    # exit if we meet not decreasing capacity case
                    if e.get_generic_error_message() == DeviceException.TIMEOUT_REACHED:
                        raise e
                    # in case of crash we consider that capacity increase timeout should be reset
                    start_time_capacity_increase = time.time()
                    # try to reconnect to the board if uecmd failed
                    self._logger.error(self.__LOG_TAG + "fail to get measurement: " + str(e))
                    measurement_fail += 1

                    # stop the usecase if measurement fail several times.
                    if measurement_fail >= self.__consecutive_meas_error:
                        if self._uc_base.batt_voltage > self._uc_base.vbatt_mos_shutdown or self._uc_base.batt_voltage == -1:
                            tmp_txt = "Measurement failed after %s times, stop usecase" % self.__consecutive_meas_error
                            self._logger.error(self.__LOG_TAG + tmp_txt)
                            raise DeviceException(DeviceException.OPERATION_FAILED, tmp_txt)
                        else:
                            self._logger.info(self.__LOG_TAG + "battery must be empty, stop discharging")
                            break
                    # check and retrieve board connection only if the test target is not DEAD battery
                    if min_capacity > 0:
                        self.check_board_connection(1, use_exception=False)

            # stop usecase if board take too long to reach battery max capacity
            if time.time() > end_time:
                tmp_txt = "Phone failed to reach %s before %ss" % (good_text, self.discharging_time_limit)
                self._logger.error(self.__LOG_TAG + tmp_txt)
                raise DeviceException(DeviceException.TIMEOUT_REACHED, tmp_txt)

            # Restore load in case of reboot
            if self._uc_base.phone_as_reboot and self._uc_base.is_board_and_acs_ok():
                # stop charging through usb
                if load_module is None:
                    self.__phonesystem_api.set_vibration("on")
                    self.__phonesystem_api.set_torchlight("on")
                else:
                    load_module.restart_load()
                self._uc_base.phone_as_reboot = False

            # increase the counter if we are not in MOS only in case of DEAD battery as target
            if min_capacity < 0:
                self._logger.info(self.__LOG_TAG + "Waiting 20s to see boot transition")
                time.sleep(20)
                if self._device.get_boot_mode() != "MOS":
                    connection_shutdown_counter += 1
                else:
                    connection_shutdown_counter = 0
                # evaluate if we are off or in COS
                if connection_shutdown_counter >= 6:
                    self._logger.info(self.__LOG_TAG + "battery must be empty, stop discharging")
                    break
        #------------------------------END----------------------------------------#
        try:
            # if device is off after discharging, exist with a warning
            if self._uc_base.is_board_and_acs_ok():
                # stop all daemonized ACS process
                self.__phonesystem_api.clean_autolog()
                # stop uecmd
                if load_module is None:
                    self.__phonesystem_api.set_vibration("off")
                    self.__phonesystem_api.set_torchlight("off")
                    self.__networking_api.set_wifi_power("off")
                    self.__bt_api.set_bt_power("off")
                else:
                    load_module.stop_load()
            else:
                tmp_txt = "The connection was lost during monitor discharging"
                self._logger.warning(self.__LOG_TAG + tmp_txt)
        finally:
            # parse the thread list and wait then delete all started threads
            for thread_ele in thread_list:
                thread_ele.join()
                del thread_ele
            del thread_list

        return ratio_d

    def __threaded_feed_meas(self, lock, msic_list, thermal_list, meas_tab, general_msg):
        """
        use a thread to fill the measurement file to avoid discharging/charging
        the board during measurement computing
        """
        lock.acquire()
        try:
            self.fill_autolog_result(msic_list, thermal_list, meas_tab, general_msg)
        finally:
            lock.release()

    def fill_autolog_result(self, msic_list, thermal_list, meas_tab, general_msg):
        """
        fill the measurement report with autolog results and add a general msg to each line
        """
        log_length = max(len(thermal_list), len(msic_list))
        if log_length > 0:
            for i in range(log_length):

                try:
                    # get battery/charger info
                    if len(msic_list) > i:
                        msic_dict = msic_list[i]
                        if len(msic_dict) > 1:
                            # store result on xml
                            if meas_tab is not None:
                                meas_tab.add_dict_measurement(msic_dict[1])
                    # get thermal info
                    if len(thermal_list) > i:
                        thermal_dict = thermal_list[i]
                        if len(thermal_dict) > 1:
                            # store result on xml
                            if meas_tab is not None:
                                meas_tab.add_dict_measurement(thermal_dict[1])

                finally:
                    if meas_tab is not None:
                        # Store various information
                        meas_tab.add_measurement([self._uc_base.get_time_tuple(),
                                                (self._em_cst.COMMENTS, general_msg),
                                                (self._em_cst.REBOOT, self._uc_base.phone_as_reboot)])
                        # reinit var
                        if self.tc_module is not None:
                            meas_tab.add_measurement([self.tc_module.feed_meas_report()])
                        # switch meas to next meas
                        meas_tab.switch_to_next_meas()

    def __discharge_when_data(self, min_capacity, good_text, meas_tab, load_module):
        """
        monitor the discharging by applying loads.
        can discharge until shutdown.

        :type min_capacity: int
        :param min_capacity: min battery capacity

        :type meas_tab: XMLMeasurementFile
        :param meas_tab: object that represent store monitoring results

        :type load_module: LoadModule
        :param load_module: object that handle the load activation

        :rtype: int
        :return: battery capacity reached
        """
        measurement_fail = 0
        connection_shutdown_counter = 0
        ratio_d = 0
        last_capacity = self._uc_base.batt_capacity

        # start autolog here
        self.__em_api.clean_autolog()
        # need to set the polling
        self.__em_api.set_persistent_autolog(True)
        self.__em_api.add_fct_to_auto_logger(self.__em_api.AUTOLOG_THERMAL, "sequenced")
        self.__em_api.add_fct_to_auto_logger(self.__em_api.AUTOLOG_UEVENT, "sequenced")
        self.__em_api.start_auto_logger(0, 10, "sequenced")
        # use thread to avoid blocking action when computing measurement
        timeout_capacity_increase = 900
        waiting_for_capacity_move = time.time()
        end_time = time.time() + self.discharging_time_limit
        capacity_increase_detected = False

        while self._uc_base.batt_capacity > min_capacity:

            if connection_shutdown_counter == 0:
                try:
                    # try to read measurement
                    msic_reg = self._uc_base.update_battery_info()
                    thermal_conf = self.__em_api.get_thermal_sensor_info()
                    # store result on xml
                    if meas_tab is not None:
                        meas_tab.add_dict_measurement(msic_reg)
                        meas_tab.add_dict_measurement(thermal_conf)
                    # this should be remove later
                    self.__em_api.set_usb_charging("off")

                    lost_capacity = last_capacity - self._uc_base.batt_capacity
                    if lost_capacity >= 0:
                        capacity_increase_detected = False
                        if lost_capacity > 0:
                            ratio_d = float(time.time() - waiting_for_capacity_move) / float(lost_capacity)
                            waiting_for_capacity_move = time.time()
                            last_capacity = self._uc_base.batt_capacity
                    elif lost_capacity < 0:
                        capacity_increase_detected = True

                    if load_module is not None and (self._uc_base.batt_capacity > min_capacity):
                        load_module.restart_load(consider_only_checkable_load=True)
                    measurement_fail = 0

                except AcsBaseException as e:
                    # If COS seen then leave if we expected a dead battery
                    # try to reconnect to the board if uecmd failed
                    self._logger.error(self.__LOG_TAG + "fail to get measurement: " + str(e))
                    measurement_fail += 1

                    # stop the usecase if measurement fail several times.
                    if measurement_fail >= self.__consecutive_meas_error:
                        if self._uc_base.batt_voltage > self._uc_base.vbatt_mos_shutdown or self._uc_base.batt_voltage == -1:
                            tmp_txt = "Measurement failed after %s times, stop usecase" % \
                                self.__consecutive_meas_error
                            self._logger.error(self.__LOG_TAG + tmp_txt)
                            raise DeviceException(DeviceException.OPERATION_FAILED, tmp_txt)
                        else:
                            self._logger.info(self.__LOG_TAG + "battery must be empty, stop discharging")
                            break
                    # check and retrieve board connection only if the test target is not DEAD battery
                    if min_capacity > 0:
                        self.check_board_connection(1, use_exception=False)
                finally:
                    if meas_tab is not None:
                        # Store various information
                        meas_tab.add_measurement([self._uc_base.get_time_tuple(),
                                                (self._em_cst.COMMENTS, "SETUP:Discharging with load with data connection on"),
                                                (self._em_cst.REBOOT, self._uc_base.phone_as_reboot)])

                        if self.tc_module is not None:
                            meas_tab.add_measurement([self.tc_module.feed_meas_report()])
                        # switch meas to next meas
                        meas_tab.switch_to_next_meas()

            # stop usecase if board take too long to reach battery capacity
            if capacity_increase_detected and ((time.time() - waiting_for_capacity_move) > timeout_capacity_increase):
                tmp_txt = "board capacity keep increasing during more than %ss instead of decreasing; abort usecase" % timeout_capacity_increase
                self._logger.error(self.__LOG_TAG + tmp_txt)
                raise DeviceException(DeviceException.TIMEOUT_REACHED, tmp_txt)
            # stop usecase if board take too long to reach battery max capacity
            elif time.time() > end_time:
                tmp_txt = "Phone failed to reach %s before %ss" % (good_text, self.discharging_time_limit)
                self._logger.error(self.__LOG_TAG + tmp_txt)
                raise DeviceException(DeviceException.TIMEOUT_REACHED, tmp_txt)

            # Restore load in case of reboot
            if self._uc_base.phone_as_reboot and self._uc_base.is_board_and_acs_ok():
                # stop charging through usb
                if load_module is None:
                    self.__phonesystem_api.set_vibration("on")
                    self.__phonesystem_api.set_torchlight("on")
                else:
                    load_module.restart_load()
                self._uc_base.phone_as_reboot = False

            # increase the counter if we are not in MOS only in case of DEAD battery as target
            if min_capacity < 0:
                self._logger.info(self.__LOG_TAG + "Waiting 20s to see boot transition")
                time.sleep(20)
                if self._device.get_boot_mode() != "MOS":
                    connection_shutdown_counter += 1
                else:
                    connection_shutdown_counter = 0
                # evaluate if we are off or in COS
                if connection_shutdown_counter >= 6:
                    self._logger.info(self.__LOG_TAG + "battery must be empty, stop discharging")
                    break

        #------------------------------END----------------------------------------#
        # if device is off after discharging, exist with a warning
        if self._uc_base.is_board_and_acs_ok():
            # restore the usb charging
            self.__em_api.set_usb_charging("on")
            # stop uecmd
            if load_module is None:
                self.__phonesystem_api.set_vibration("off")
                self.__phonesystem_api.set_torchlight("off")
                self.__networking_api.set_wifi_power("off")
                self.__bt_api.set_bt_power("off")
            else:
                load_module.stop_load()
        else:
            tmp_txt = "The connection was lost during monitor discharging"
            self._logger.warning(self.__LOG_TAG + tmp_txt)

        return ratio_d

#------------------------------------------------------------------------------

    def discharge_battery(self, timer):
        """
        discharge the battery.

        :type timer: int
        :param timer: discharging time in second
        """
        # disconnect board
        self._device.disconnect_board()
        # disconnect usb
        self._io_card.usb_connector(False)
        # wait x min
        self._logger.info(self.__LOG_TAG + "Discharge phone during %s seconds" % timer)
        time.sleep(timer)
        # connect to board
        self._io_card.usb_host_pc_connector(True)
        # wait x seconds
        time.sleep(self._uc_base.usb_sleep)
        # connect board
        self._device.connect_board()

    def charge_battery(self, timer):
        """
        charge the battery

        :type timer: int
        :param timer: charging time in second
        """
        # wake screen to be sure to turn it in S3 mode
        # when pushing on power button
        if not self.__phonesystem_api.get_screen_status():
            self.__phonesystem_api.wake_screen()
        # disconnect board
        self._device.disconnect_board()
        # connect WALL Charger
        self._io_card.wall_charger_connector(True)
        # turn off screen
        self._io_card.press_power_button(0.3)
        # wait x min
        self._logger.info(self.__LOG_TAG + "Charge phone during %s seconds" % timer)
        time.sleep(timer)
        # connect USB SDP
        self._io_card.wall_charger_connector(False)
        self._io_card.usb_host_pc_connector(True)
        # wait x seconds
        time.sleep(self._uc_base.usb_sleep)
        # connect board
        self._device.connect_board()
        # check adb connection
        self.check_board_connection()

#------------------------------------------------------------------------------

    def retrieve_off_board(self, charge_time=None):
        """
        try to bring the board to boot in mos by doing several actions.

        :type timer: int
        :param timer: charging time in second
        """
        if charge_time is None:
            charge_time = self._charge_empty_batt_time

        # get boot mode to see if we can check battery info
        boot_mode = self._device.get_boot_mode()

        # if we don't know the boot mode then plug usb host to see the mode
        if boot_mode == "UNKNOWN":
            self._io_card.usb_host_pc_connector(True)
            time.sleep(self._uc_base.usb_sleep)
            boot_mode = self._device.get_boot_mode()

        self._logger.debug(self.__LOG_TAG + "boot mode before charging : %s" % boot_mode)
        if boot_mode not in self._uc_base.phone_info["GENERAL"]["ADB_AVAILABLE_MODE"]:
            self._io_card.wall_charger_connector(False)
            self._device.hard_shutdown(False)

        self._device.disconnect_board()
        # charge the the board in the fastest way for a defined duration
        self._logger.info(self.__LOG_TAG + "try to charge board during %s seconds" % charge_time)
        # connect USB DCP
        self._io_card.wall_charger_connector(True)
        # wait time to retrieve board from 0%
        time.sleep(charge_time)
        # push power button only if boot mode was different from MOS to avoid opening power off menu
        if boot_mode != "MOS":
            # once charged, press power button to boot
            self._io_card.press_power_button(self._uc_base.pwr_btn_boot)
            # wait 2 min to let the board boot
            self._logger.debug(self.__LOG_TAG + "wait %s to boot transition happened" % self._device.get_boot_timeout())
            time.sleep(self._device.get_boot_timeout())

        # check in what mode we are
        self._io_card.usb_host_pc_connector(True)
        stop_time = time.time() + 120
        self._logger.debug(self.__LOG_TAG + "try to see boot mode before 120s")
        while time.time() < stop_time:
            boot_mode = self._device.get_boot_mode()
            if boot_mode != "UNKNOWN":
                break

        self._logger.debug(self.__LOG_TAG + "boot mode after charging : %s" % boot_mode)
        #---------------------------------------------------
        if self._uc_base.phone_info["GENERAL"]["DATA_WHILE_CHARGING"] and boot_mode in self._uc_base.phone_info["GENERAL"]["ADB_AVAILABLE_MODE"]:
            # check if we can read uevent in this mode
            #---------------------------------------------------
            if boot_mode == "MOS":
                self._device.connect_board()
            if self._device.is_available():
                self._uc_base.update_battery_info()
            # try to charge the board above MOS boot threshold
            tries = 3
            timer = 900
            self._logger.debug(self.__LOG_TAG + "check if battery voltage is below MOS boot threshold: current vbatt= %s , threshold to reach  = %s" %
                              (self._uc_base.batt_voltage, self._uc_base.vbatt_mos_boot))
            while self._uc_base.batt_voltage < self._uc_base.vbatt_mos_boot and tries > 0:
                # connect WALL Charger
                self._io_card.wall_charger_connector(True)
                # wait x min
                self._logger.info(self.__LOG_TAG + "Charge phone during %s seconds" % timer)
                time.sleep(timer)
                # connect pc host
                self._io_card.usb_host_pc_connector(True)
                # wait x seconds
                time.sleep(self._uc_base.usb_sleep)
                # connect board
                if not self._device.is_available():
                    self._device.connect_board()
                if self._device.is_available():
                    self._uc_base.update_battery_info()
                tries -= 1

            # try to boot in right mode if not MOS
            if boot_mode == "MOS":
                if not self._device.is_available():
                    self._device.connect_board()
            elif boot_mode != "UNKNOWN":
                self._device.reboot()
            else:
                self._io_card.wall_charger_connector(False)
                self.reboot_board("HARD")

        #---------------------------------------------------
        else:
            # if boot mode is not supported or unknown try to charge again
            # charge the the board in the fast way for to a defined time
            self._logger.info(self.__LOG_TAG + "try to charge again board during %s seconds" % charge_time)
            self._io_card.wall_charger_connector(True)
            time.sleep(charge_time)
            self._io_card.wall_charger_connector(False)
            self.reboot_board("HARD")

#------------------------------------------------------------------------------

    def reboot_board(self, switch_off_mode="HARD"):
        """
        Reboot the board

        :type switch_off_mode: str
        :param switch_off_mode:
                    SOFT to do a soft shutdown
                    HARD to do an hard shutdown(default)

        :rtype: boolean
        :return: True if board has booted and is alive, False otherwise
        """
        self._logger.info(self.__LOG_TAG + "trying to reboot board")

        if str(switch_off_mode).upper() == "HARD":
            self._device.hard_shutdown(False)
            self._device.switch_on(settledown_duration=10,
                                   simple_switch_mode=True)
        elif str(switch_off_mode).upper() == "SOFT":
            self._device.reboot()
        else:
            tmp_txt = "Unknown mode %s" % str(switch_off_mode)
            self._logger.error(self.__LOG_TAG + tmp_txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, tmp_txt)
        self._uc_base.phone_as_reboot = True

#------------------------------------------------------------------------------

    def __reconnect_board(self):
        """
        Reconnect the board
        """
        self._logger.info(self.__LOG_TAG + "trying to reconnect board")
        # disconnect board
        self._device.disconnect_board()
        # disconnect usb in any case
        self._io_card.usb_connector(False)
        # turn board OFF
        time.sleep(self._uc_base.usb_sleep)
        # connect USB SDP
        self._io_card.usb_host_pc_connector(True)
        # wait x seconds
        time.sleep(self._uc_base.usb_sleep)
        # connect board
        if self._device.get_state() == "alive":
            self._device.connect_board()

#------------------------------------------------------------------------------

    def check_board_connection(self, tries=3,
                               use_exception=True, only_reconnect=False):
        """
        Try to check then establish an connection to DUT by connecting SDP and
        opening a communication socket

        :type tries: int
        :param tries: number connection tries

        :type only_reconnect: boolean
        :param only_reconnect: if set to True, board connection
                               will be establish only by reconnecting usb

        :type use_exception: boolean
        :param use_exception: true to raise an exception if failed to boot several time,
                              false to do nothing

        :rtype: boolean
        :return: return True if the connection was successful or False if not
        """
        self._logger.info(self.__LOG_TAG + "check board connection")
        failed_tries = 0
        # try to connect board to see if the developer forget to establish a connection
        board_state = self._device.get_state()
        if board_state == "alive" and not self._device.is_available():
            self._device.connect_board()
        # secondly try first to play with usb data cable by doing plug/unplug
        elif board_state != "alive":
            # try to reconnect board.
            self.__reconnect_board()

        # if we reach here  and there still no connection , turn off acs connection
        if not self._uc_base.is_board_and_acs_ok():
            self._device.disconnect_board()

            # reboot board in the worst case
            while not self._uc_base.is_board_and_acs_ok():
                if failed_tries < tries:
                    # only play on usb reconnection
                    if only_reconnect:
                        self._logger.error(self.__LOG_TAG +
                            "DUT connection failed, trying to reconnect board")
                        # disconnect usb
                        self._io_card.usb_connector(False)
                        # wait X seconds
                        time.sleep(self._uc_base.usb_sleep)
                        # Connect USB PC/Host
                        self._io_card.usb_host_pc_connector(True)
                        # Open connection
                        time.sleep(self._uc_base.usb_sleep)
                        if self._device.get_state() == "alive":
                            self._device.connect_board()

                    # try to reboot the board instead
                    else:
                        self._logger.error(self.__LOG_TAG +
                            "DUT connection failed, trying to reboot board")
                        # leave time to board to write on internal logs
                        time.sleep(10)
                        # hardware shutdown
                        self._device.hard_shutdown(False)
                        # switch on the board
                        self._device.switch_on(settledown_duration=10,
                                               simple_switch_mode=True)
                        self._uc_base.phone_as_reboot = True

                    failed_tries += 1
                else:
                    if use_exception:
                        tmp_txt = "DUT connection failed after %s try(ies) abort usecase" % \
                                  failed_tries
                        self._logger.error(self.__LOG_TAG + tmp_txt)
                        raise DeviceException(DeviceException.CONNECTION_LOST, tmp_txt)
                    else:
                        self._logger.error(self.__LOG_TAG + "DUT connection failed after %s try(ies) continuous usecase " %
                                           failed_tries)
                        return False

        self._logger.info(self.__LOG_TAG + "board connection successful")
        return True

#-----------------------------------------------------------------------

    def configure_slot(self):
        """
        .. deprecated:: use Uc module instead
        Configure network simulator slots.

        :rtype: dict
        :return: return a dict that contains slot configuration with given keys:
                "DL_STATE"    downlink on/off state
                "DL_VALUE"    downlink level
                "UL_STATE"    uplink on/off state
                "UL_VALUE"    uplink PCL value
        .. warning:: WILL BE DELETED SOON
        """
        active_slot = []
        separator = ""

        slots_conf = {
            "DL_STATE": "",
            "DL_VALUE": "",
            "UL_STATE": "",
            "UL_VALUE": ""
        }
        # get used slot that is at true from Testcase parameter
        for slot_nbr in range(8):
            name = ("USE_SLOT_%s" % slot_nbr)
            use_slot = self._tc_parameters.get_param_value(name)
            if use_slot is not None:
                if str_to_bool(use_slot):
                    active_slot.append(slot_nbr)

        # configure slots
        for value in range(8):
            if value in active_slot:
                name = "SLOT_%s_PCL" % str(value)
                slots_conf["DL_STATE"] += ("%sON" % separator)
                slots_conf["UL_STATE"] += ("%sON" % separator)

                # Read SLOT_x_PCL from test case xml file
                # make the uc crash if tc parameter is not numeric
                pcl_configuration = int(
                    self._tc_parameters.get_param_value(name))
                slots_conf["DL_VALUE"] += ("%s0" % separator)
                slots_conf["UL_VALUE"] += "%s%s" % (separator,
                                                      str(pcl_configuration))
                separator = ","
            else:
                slots_conf["DL_STATE"] += ("%sOFF" % separator)
                slots_conf["UL_STATE"] += ("%sOFF" % separator)
                slots_conf["DL_VALUE"] += ("%s0" % separator)
                slots_conf["UL_VALUE"] += ("%s0" % separator)
                separator = ","

        return slots_conf

#------------------------------------------------------------------------------

    def plug_charger(self, charger="WALL_CHARGER", ext_ps=None):
        """
        plug the charger and the external power supply if expected

        :type charger: Charger type defined in iocard type
        :param charger: charger to unplug
        :type ext_ps: Boolean
        :param ext_ps: if true, connect external power supply
                        if it is available
        """
        # check charger type
        if charger == self._io_card.AC_CHGR:
            # Connect AC CHARGER
            self._io_card.ac_charger_connector(True)
        elif charger in [self._io_card.SDP, self._io_card.CDP, self._io_card.ACA, self._io_card.DCP, self._io_card.USB_HOST_PC]:
            # Connect USB SDP or CDP
            self._io_card.simulate_insertion(charger)
        elif charger == self._io_card.WALL_CHARGER:
            # connect WALL charger
            self._io_card.wall_charger_connector(True)
        elif charger.upper() == "NONE":
            self._logger.info(self.__LOG_TAG + "[EM BASE] Any charger %s plugged" % charger)
        else:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                   "Parameter %s is invalid to plug charger function" % charger)
        time.sleep(2)

    def unplug_charger(self, charger="WALL_CHARGER"):
        """
        unplug the charger

        :type charger: Charger type defined in iocard type
        :param charger: charger to unplug
        """
        # check charger type
        if charger == self._io_card.AC_CHGR:
            # Connect USB
            self._io_card.ac_charger_connector(False)
        elif charger in [self._io_card.SDP, self._io_card.USB_HOST_PC]:
            # unConnect USB
            self._device.disconnect_board()
            self._io_card.usb_connector(False)
        elif charger in [self._io_card.CDP, self._io_card.ACA, self._io_card.DCP]:
            # unConnect USB
            self._io_card.usb_connector(False)
        elif charger == self._io_card.WALL_CHARGER:
            # unConnect wall charger
            self._io_card.wall_charger_connector(False)
            # Connect external supply

    def is_host_connection_available_when_charger_plug(self, charger, keep_charger_if_data=False):
        """
        Test if your charger allow data with pc host.
        may keep the tested charger plugged if data conenction is seen
        :rtype:boolean
        :return: True if data is available after plugging the charger, False otherwise
        """
        # check that we have data with the given charger type
        self._logger.info(self.__LOG_TAG + "detect if when the charger %s is plugged, a data connection with HOST is still available" % charger)
        self._device.disconnect_board()
        if charger != "NONE":
            self._io_card.simulate_insertion(charger)
        else:
            self._io_card.remove_cable("ALL")

        time.sleep(self._uc_base.usb_sleep)
        data = (self._device.get_boot_mode() not in ["UNKNOWN", "POS"])
        # switch to pc host only if we have no data
        if not data:
            txt = " no data available when the charger is plugged"
        else:
            txt = " data is available when the charger is plugged"

        # restore the charger if option is false
        if not keep_charger_if_data or not data:
            self._io_card.usb_host_pc_connector(True)
            time.sleep(self._uc_base.usb_sleep)

        self._device.connect_board()
        self._logger.info(self.__LOG_TAG + txt)

        return data

#------------------------------------------------------------------------------

    def clean_up(self):
        """
        charge the board if necessary and retrieve applications logs.
        """
        # Check that we are in the right boot mode, if not try to boot in the right one
        if self._device.get_boot_mode() not in ["MOS", "UNKNOWN"]:
            self._device.reboot(skip_failure=True)
        # if board if off , try tosee if it is not because of an bad plugged cable
        if self._device.get_boot_mode() == "UNKNOWN":
            self._io_card.usb_host_pc_connector(True)
            time.sleep(self._uc_base.usb_sleep)
            if self._device.get_boot_mode() == "MOS":
                self._device.connect_board()

        # check if phone battery is too low and
        # charge it for a while (even if the board is down)

        # this try except give a final chance to get aplog in case of a crash
        try :
            if self._uc_base.is_board_and_acs_ok():
                self._uc_base.update_battery_info()
            # enter here if we are not in MOS or in MOS with low voltage or capacity
            if (self._device.get_boot_mode() != "MOS") or\
                (-1 < self._uc_base.batt_voltage <= self._uc_base.vbatt_mos_shutdown) or\
                   (-1 < self._uc_base.batt_capacity <= 10):
                self.retrieve_off_board()
        except AcsBaseException as e:
            self._logger.error(self.__LOG_TAG + "error happened during board charge process in clean_up")
            raise e

        finally:
            # try again to boot the board if the charging was useless
            if not self._uc_base.is_board_and_acs_ok():
                # check board connection and reboot if necessary
                self.check_board_connection()

            if self._uc_base.is_board_and_acs_ok():
                self._uc_base.get_application_logs()

#-----------------------------------------------------------------------

    def reset_thermal_critical_state(self, restore_thermal_captor=None,
                                     timeout=60, os_to_reboot="MOS"):
        """
        reset the phone from thermal critical state and avoid critical shutdown

        :type restore_thermal_captor: str
        :param restore_thermal_captor: thermal captor to reset
        :type timeout: int
        :param timeout: time (in second) to wait to try to restore thermal captor
        :type os_to_reboot: str
        :param os_to_reboot: os to reboot
        """
        # try to reset the thermal captor temp in case
        if self.__em_api.restore_thermal_captor_temp(restore_thermal_captor,
                                                    15000, timeout=5):
            # return if sucess
            return

        # try to restore the thermal captro after a reboot
        max_tries = 3
        tries = 0
        # disconnect board
        self._device.disconnect_board()
        while tries <= max_tries:
            self._logger.info("[THERMAL] try to reset thermal critical state")
            # unplug USB
            self.unplug_charger(self._io_card.SDP)
            # turn board ON
            self._io_card.press_power_button(self._uc_base.pwr_btn_boot)
            # wait a little bit to avoid COS or flash mode
            time.sleep(5)
            # Connect USB PC/Host
            self.plug_charger(self._io_card.SDP)
            # spam to restor captor file during timeout second
            self.__em_api.restore_thermal_captor_temp(restore_thermal_captor,
                                                     15000, timeout)
            time.sleep(20)
            if self._device.get_boot_mode() == "MOS":
                self._logger.info("[THERMAL] the board leave critical temp"
                                  + "configuration")
                break

            tries += 1
        # check_the numbers of tries
        if tries > max_tries:
            tmp_txt = "[THERMAL] connection and boot failed due to thermal test ; "
            self._logger.error(tmp_txt)
            raise DeviceException(DeviceException.CONNECTION_LOST, tmp_txt)

        # reboot the board one more time
        self._device.reboot(os_to_reboot)

#------------------------------------------------------------------------------

    def adjust_batt_temp(self, min_temp, max_temp, operation_timeout, tc_module=None, meas_report=None):
        """
        wait that the DUT battery temperature is inside the temperature range before a define timeout
        use if it is a setup condition.
        """
        # use default thermal module if not specify
        if tc_module is None:
            tc_module = self.tc_module
        # compute the average temp
        last_batt_temp = self._uc_base.update_battery_info()["BATTERY"]["TEMP"][0]
        if tc_module is not None and not (min_temp <= last_batt_temp <= max_temp):
            self._logger.info(self.__LOG_TAG + "check that DUT is between [%s;%s] degree celsius" % (min_temp, max_temp))
            exceed_timeout = True
            temperature = int((min_temp + max_temp) / 2)
            self.__phonesystem_api.sleep_screen()
            tc_module.get_eq().wait_for_temperature(temperature)
            # hardcoded 1h of timeout and 300s for a change in temperature on the board
            timeout_to_see_temp_change = 300
            start_time_to_see_temp_change = time.time()
            timeout = time.time() + operation_timeout

            while time.time() < timeout:
                self._logger.info(self.__LOG_TAG + "wait 60s")
                time.sleep(60)
                try:
                    # try to read measurement
                    msic_reg = self._uc_base.update_battery_info()
                    thermal_conf = self.__em_api.get_thermal_sensor_info()
                    if meas_report is not None:
                        # store result on xml
                        meas_report.add_dict_measurement(msic_reg)
                        meas_report.add_dict_measurement(thermal_conf)
                    local_temp = msic_reg["BATTERY"]["TEMP"][0]

                    if last_batt_temp != local_temp:
                        start_time_to_see_temp_change = time.time()
                    last_batt_temp = local_temp

                    if (time.time() - start_time_to_see_temp_change) > timeout_to_see_temp_change:
                        self._logger.info(self.__LOG_TAG + "DUT temperature did not change in %ss, adjusting chamber temperature" % timeout_to_see_temp_change)
                        if local_temp < min_temp:
                            temperature += 1

                        elif  local_temp > max_temp:
                            temperature -= 1
                        tc_module.get_eq().set_temperature(temperature)
                        start_time_to_see_temp_change = time.time()

                except AcsBaseException as e:
                    # try to catch why uecmd may fail
                    if not self._uc_base.is_board_and_acs_ok():
                        txt = "connection with DUT lost during temperature change to let it see [%s;%s] degree celsius" % (min_temp, max_temp)
                    else:
                        txt = "error happened during temperature change to let it see [%s;%s] degree celsius : %s" % (min_temp, max_temp, str(e))
                    self._logger.error(self.__LOG_TAG + txt)
                    raise DeviceException(DeviceException.OPERATION_FAILED, txt)
                finally:
                    if meas_report is not None:
                        # Store various information
                        meas_report.add_measurement([self._uc_base.get_time_tuple(),
                                                (self._em_cst.COMMENTS, "Waiting for the DUT to reach [%s;%s] degree celsius" % (min_temp, max_temp))])
                        # switch meas to next meas
                        meas_report.switch_to_next_meas()
                # exit if the battery temp match with test temp
                if min_temp <= last_batt_temp <= max_temp:
                    self._logger.info(self.__LOG_TAG + "The DUT has reached %s degree celsius which is between expected target [%s;%s] !" % (last_batt_temp, min_temp, max_temp))
                    exceed_timeout = False
                    break

            if exceed_timeout:
                txt = "timeout exceeded (%ss) to move the DUT temperature in between [%s,%s] degree celsius" % (str(operation_timeout), min_temp, max_temp)
                self._logger.error(self.__LOG_TAG + txt)
                raise DeviceException(DeviceException.OPERATION_FAILED, txt)

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # to be call in uc setup
        pass

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        # to be call in uc teardown
        pass
