"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

:organization: INTEL QCTV
:summary: EM - check that the platform battery information during charge discharge cycle : capacity jump
:author: jortetx
:since: 06/06/2014
"""
import time
import os

from acs_test_scripts.Utilities.EMUtilities import XMLMeasurementFile
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from acs_test_scripts.UseCase.EnergyManagement.UcModule.LoadModule import LoadModule


class LabEmBattInfoChargeDischarge(EmUsecaseBase):
    """
    Lab Energy Management class.
    """
    DEDICATED_BENCH = "BATTERY_BENCH"

    def __init__(self, tc_name, global_config):

        EmUsecaseBase.__init__(self, tc_name, global_config)

        # get the tc parameters
        self.__min_range = self._tc_parameters.get_param_value("MIN_RANGE", default_cast_type=int)
        self.__max_range = self._tc_parameters.get_param_value("MAX_RANGE", default_cast_type=int)
        self.__charging_timeout = self._tc_parameters.get_param_value("CHARGING_TIMEOUT", default_cast_type=int)
        self.__discharging_timeout = self._tc_parameters.get_param_value("DISCHARGING_TIMEOUT", default_cast_type=int)
        self.__polling_delay = self._tc_parameters.get_param_value("DATA_POLLING_DELAY", default_cast_type=int)
        self.__load = self._tc_parameters.get_param_value("LOAD")
        self.__target = str(self._tc_parameters.get_param_value("TARGET")).lower()
        self.__crit_batt_voltage = self._tc_parameters.get_param_value("CRIT_VBATT_VOLTAGE", default_cast_type=float)
        self.__full_voltage_th = self._tc_parameters.get_param_value("FULL_VOLTAGE_THRESHOLD", default_cast_type=float)
        self.__full_volt_offset = self._tc_parameters.get_param_value("FULL_VOLTAGE_OFFSET", default_cast_type=float)
        self._critt_batt_offset = self._tc_parameters.get_param_value("CRIT_VBATT_OFFSET", default_cast_type=float)

        # vars
        self.__time_to_charge = 500
        self.__time_to_discharge = 500
        self.__reports_file_name = os.path.join(self._saving_directory, "EM_meas_report.xml")
        # create a report file
        self.__report_file_handler = XMLMeasurementFile(self.__reports_file_name)

        # Activate Loadmodule instance
        self.__load_module = LoadModule()
        self.__load_module.add_load(self.__load)

    # ---------------------------------------

    def set_up(self):

        EmUsecaseBase.set_up(self)

        # get the starting capacity
        self.update_battery_info()

        self._logger.info("Device capacity is %s : going to %s" % (self.batt_capacity, self.__max_range))

        # initial state is dut charge at max_range
        if int(self.batt_capacity) < self.__max_range:
            self.em_core_module.monitor_charging(self.__max_range, self.__time_to_charge, self.__report_file_handler)
        elif int(self.batt_capacity) > self.__max_range:
            self.em_core_module.monitor_discharging(self.__max_range, self.__time_to_discharge, self.__report_file_handler, load_module=self.__load_module)

        return Global.SUCCESS, "No errors"

    # ---------------------------------------

    def run_test_body(self):

        EmUsecaseBase.run_test_body(self)

        # device is in initial state
        self._logger.info("Starting discharge to %s percent of capacity " % self.__min_range)

        _, discharging_result = self._start_logged_discharging()

        if self.__target == "capacity_jump":
            is_a_capacity_jump_while_discharge, msg = self._seek_capajump_and_log_values(discharging_result)
            msg += " While discharging"
            if is_a_capacity_jump_while_discharge:
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        elif self.__target == "voltage_ocv":
            is_vocv_ok = self._check_vocv_and_log_value(discharging_result, self.__min_range)

            if not is_vocv_ok:
                msg = "Discharging : Voltage ocv at %s is not at the expected value" % self.__min_range
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)
            else:
                msg = "Discharging : Voltage OCV is at the expected value at %s of capacity" % self.__min_range

        # charge
        self._logger.info("Starting charge to %s percent of capacity " % self.__max_range)
        _, charging_result = self._start_logged_charging()

        if self.__target == "capacity_jump":
            # result and save to file
            is_a_capacity_jump_while_charge, msg = self._seek_capajump_and_log_values(charging_result)
            msg += "While charging"

            if is_a_capacity_jump_while_charge:
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        elif self.__target == "voltage_ocv":
            is_vocv_ok = self._check_vocv_and_log_value(discharging_result, self.__max_range)

            if not is_vocv_ok:
                msg += " - Charging :Voltage ocv at %s is not at the expected value" % self.__max_range
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)
            else:
                msg += " - Charging :Voltage OCV is at the expected value at %s of capacity" % self.__max_range

        return Global.SUCCESS, msg

    # ---------------------------------------

    def tear_down(self):

        EmUsecaseBase.tear_down(self)
        self.em_core_module.clean_up()
        return Global.SUCCESS, "No errors"

    # ---------------------------------------

    def _check_vocv_and_log_value(self, results, capa_to_check):

        result_at_expected_capa = []

        for element in results:
            capa = int(element[1]['BATTERY']['CAPACITY'][0])
            vocv = float(element[1]['BATTERY']['VOLTAGE'][0])

            # log measurement
            self.__report_file_handler.add_dict_measurement(element[1])
            self.__report_file_handler.switch_to_next_meas()

            if capa == int(capa_to_check):
                result_at_expected_capa.append(vocv)

        # in case of discharging

        if capa_to_check == self.__min_range:
            for vocv in result_at_expected_capa:
                if self.__crit_batt_voltage <= vocv <= self.__crit_batt_voltage + self._critt_batt_offset:
                    return True

        # in case of charging

        elif capa_to_check == self.__max_range:
            for vocv in result_at_expected_capa:
                if self.__full_voltage_th - self.__full_volt_offset <= vocv <= self.__full_voltage_th:
                    return True

        return False

    # ---------------------------------------

    def _seek_capajump_and_log_values(self, results):
        """
        try to find capacity jump in the results,
        """
        previous_value = 0
        capacity_jump_found = False

        for element in results:
            capa = int(element[1]['BATTERY']['CAPACITY'][0])

            # log measurement
            self.__report_file_handler.add_dict_measurement(element[1])
            self.__report_file_handler.switch_to_next_meas()

            # test is there a capacity jump
            if abs(capa - previous_value) > 1 and (self.__max_range >= previous_value >= self.__min_range):
                msg = "A capacity jump found between %s and %s" % (capa, previous_value)
                self._logger.info(msg)
                capacity_jump_found = True

            previous_value = capa

        if not capacity_jump_found:
            msg = "No capacity jump found"
            self._logger.info(msg)

        return capacity_jump_found, msg

    # ---------------------------------------

    def _start_logged_discharging(self):
        # clean old logs
        self.em_api.clean_autolog()

        # set log to be persistent even if the board reboot
        self.em_api.set_persistent_autolog(False)
        phone_start_time = self.get_time_from_board()

        # choose function to put in logger
        self.em_api.add_fct_to_auto_logger(self.em_api.AUTOLOG_UEVENT, "sequenced")
        self.em_api.start_auto_logger(60, 10, "sequenced")
        # start maintenance discharging
        self._logger.info("Logged discharging started")

        self.__load_module.start_load()

        discharging_start_time = time.time()

        discharge_done = False

        while (time.time() - discharging_start_time) < self.__discharging_timeout and discharge_done is False:
            time.sleep(self.__polling_delay)
            # measurement if board alive check the capacity value
            boot_mode = self._device.get_boot_mode()
            if boot_mode == "MOS":
                self.update_battery_info()
                if self.batt_capacity <= self.__min_range:
                    discharge_done = True
                self.__load_module.restart_load(consider_only_checkable_load=True)
            # board is not alive charge a while
            else:
                self.em_core_module.retrieve_off_board(charge_time=600)
                discharge_done = True

        boot_mode = self._device.get_boot_mode()

        # check board connection
        if boot_mode != "MOS":
            # if board is not alive charge it for a while
            self.em_core_module.retrieve_off_board()
            self.em_core_module.check_board_connection()

        self._logger.info("Logged discharging stopped, compute result")
        # stop logger
        self.em_api.stop_auto_logger("sequenced")

        # get result from log
        msic_result = self.em_api.get_autolog_msic_registers()

        # remove parasite logs
        if len(msic_result) > 2:
            msic_result.pop(-1)

        self.__load_module.stop_load()

        return phone_start_time, msic_result

    # ---------------------------------------

    def _start_logged_charging(self):
        # clean old logs
        self.em_api.clean_autolog()

        # set log to be persistent even if the board reboot
        self.em_api.set_persistent_autolog(False)
        phone_start_time = self.get_time_from_board()

        # choose function to put in logger
        self.em_api.add_fct_to_auto_logger(self.em_api.AUTOLOG_UEVENT, "sequenced")
        self.em_api.start_auto_logger(60, 10, "sequenced")
        # start charging
        self._logger.info("Logged charging started")
        # disconnect board
        self._device.disconnect_board()
        # connect DCP
        self._io_card.wall_charger_connector(True)
        # wait x min
        self._logger.info("Charging during %s seconds" % self.__charging_timeout)

        # 2 var for time stamp to cover logs checking
        host_start_time = time.time()

        while (time.time() - host_start_time) < self.__charging_timeout:
            # measure capacity
            self._device.disconnect_board()
            self._io_card.usb_host_pc_connector(False)
            self._io_card.wall_charger_connector(True)
            time.sleep(self.__polling_delay)
            self._io_card.wall_charger_connector(False)
            self._io_card.usb_host_pc_connector(True)
            time.sleep(self.usb_sleep)
            self._device.connect_board()
            self.update_battery_info()
            if self.batt_capacity >= self.__max_range:
                # max capacity is raised
                break

        boot_mode = self._device.get_boot_mode()

        # check board connection
        if boot_mode != "MOS":
            # if board is not alive charge it for a while
            self.em_core_module.retrieve_off_board(charge_time=1)
            self.em_core_module.check_board_connection()

        self._logger.info("Logged charging stopped, compute result")

        # stop logger
        self.em_api.stop_auto_logger("sequenced")

        # get result from log
        msic_result = self.em_api.get_autolog_msic_registers()

        # remove parasite logs
        if len(msic_result) > 2:
            msic_result.pop(-1)

        return phone_start_time, msic_result
