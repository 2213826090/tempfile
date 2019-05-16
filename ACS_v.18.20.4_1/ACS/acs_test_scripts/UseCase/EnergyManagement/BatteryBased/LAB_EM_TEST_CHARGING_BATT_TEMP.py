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
:summary: EM - check that the charge RESUME or STOP when exiting DUT temperature change
:author: vgomberx
:since: 20/08/2014
"""
import os
import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.EMUtilities import XMLMeasurementFile
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.AcsBaseException import AcsBaseException
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.UseCase.EnergyManagement.UcModule.ThermalChamberModule import ThermalChamberModule
from acs_test_scripts.UseCase.EnergyManagement.UcModule.LoadModule import LoadModule


class LabEmTestChargingBattTemp(EmUsecaseBase):

    """
    Lab Energy Management class.
    """
    DEDICATED_BENCH = "BATTERY_BENCH"
    __ACTION = {"CHARGE_RESUME": {"START": "OVERHEAT",
                                  "STOP": "GOOD"},
                "CHARGE_STOP": {"START": "GOOD",
                                "STOP": "OVERHEAT"}}

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LAB_EM_BASE Init function
        EmUsecaseBase.__init__(self, tc_name, global_config)

        # init fuel gauging parameters
        self.em_core_module.init_fg_param()

        # High load
        load = self._tc_parameters.get_param_value("SETUP_LOAD")
        self.__charger_type = self._tc_parameters.get_param_value("CHARGER_TYPE")
        self.__timeout_dut_adjust_stop_temp = self._tc_parameters.get_param_value("TIME_TO_ADJUST_TO_STOP_TEMPERATURE", default_cast_type=int)
        self.__timeout_dut_adapt_start_temp = self._tc_parameters.get_param_value("TIME_TO_ADJUST_TO_START_TEMPERATURE", 3600, default_cast_type=int)

        # Read TEMPERATURE from test case xml file
        self.tc_module = None
        self.__start_max_temperature = self._tc_parameters.get_param_value("START_MAX_TEMPERATURE")
        self.__start_min_temperature = self._tc_parameters.get_param_value("START_MIN_TEMPERATURE")
        self.__stop_temp = self._tc_parameters.get_param_value("STOP_TEMPERATURE")
        self.__action_to_peform = str(self._tc_parameters.get_param_value("WHAT_TO_CHECK")).upper()
        # ROOM is only for debug purpose
        if "ROOM" not in [self.__stop_temp, self.__start_max_temperature, self.__start_min_temperature]:
            self.__start_max_temperature = int(self.__start_max_temperature)
            self.__start_min_temperature = int(self.__start_min_temperature)
            # get temperature chamber equipment if not room temperature
            self.__stop_temp = int(self.__stop_temp)
            self.tc_module = ThermalChamberModule(self._ambient_temperature)
            # inform module that this is not a default thermal test
            self.__start_temperature = int((self.__start_min_temperature + self.__start_max_temperature) / 2)
            self.tc_module.set_test_type("SPECIFIC")
        else:
            self.__start_temperature = "ROOM"

        # Summarize the parameters
        self._logger.info("Type of the charger: %s" % self.__charger_type)
        self._logger.info("Test start MAX temperature: %s" % self.__start_max_temperature)
        self._logger.info("Test start MIN temperature: %s" % self.__start_min_temperature)
        self._logger.info("Test stop temperature: %s" % self.__stop_temp)
        self._logger.info("What we want to check: %s" % self.__action_to_peform)
        self._logger.info("Time to see change on DUT behavior at tested temperature: %s" % self.__timeout_dut_adjust_stop_temp)

        # Load Module Initialization
        self.__load_module = LoadModule()
        self.__load_module.add_load(load)

        # Call ConfigsParser to parse Energy_Management
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_TEST_CHARGING_BATT_TEMP", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())
        # get target and initialize measurement
        self._em_meas_verdict.load_target(self._em_targets, self.tcd_to_test)

        # measurement file
        meas_file_name = os.path.join(self._saving_directory,
                                      "EM_meas_report.xml")
        self.__em_meas_tab = XMLMeasurementFile(meas_file_name)
        self.__charger_is_data_cable = None

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # Call the UseCaseBase Setup function
        EmUsecaseBase.set_up(self)
        # Check specific option if they are chosen
        if self.__charger_type not in self._io_card.SUPPORTED_DEVICE_TYPE:
            txt = "io card does not support charger type %s " % self.__charger_type
            self._logger.error(txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, txt)

        # Update Battery Information
        em_info = self.update_battery_info()
        if self.em_core_module.is_batt_capacity_above_target(em_info, self.em_core_module.batt_max_capacity):

            # Discharge if we are too high
            self.em_core_module.monitor_discharging(self.em_core_module.batt_max_capacity,
                                     self.em_core_module.discharge_time,
                                     self.__em_meas_tab, self.__load_module)

        if self.tc_module is not None:
            self.tc_module.set_up_eq()
            self.tc_module.get_eq().set_regulation(True)
            self.tc_module.get_eq().set_temperature(self.__start_temperature)

        # set the temperature
        self.em_api.set_thermal_management(False)
        # a reboot is required to apply this change
        self._device.reboot()
        self.phonesystem_api.set_screen_timeout(3600)

        # do this job one time in case of B2B
        if self.__charger_is_data_cable is None:
            self.__charger_is_data_cable = self.em_core_module.is_host_connection_available_when_charger_plug(self.__charger_type, True)
            self._logger.info("test sequences will adjust to this")

        # wait that the board see the test temperature
        remaining_time = time.time()
        self.em_core_module.adjust_batt_temp(self.__start_min_temperature, self.__start_max_temperature,
                                             self.__timeout_dut_adapt_start_temp, self.tc_module, self.__em_meas_tab)

        remaining_time = max(self.__timeout_dut_adapt_start_temp - (time.time() - remaining_time), 120)
        self.__track_battery_health_change(remaining_time)

        return Global.SUCCESS, "No errors"

    def __track_battery_health_change(self, test_timeout):
        """
        track a battery health change
        """
        exceed_timeout = True
        timeout = time.time() + test_timeout
        crash__alowed = 3
        while time.time() < timeout:
            try:
                # try to read measurement
                msic_reg = self.update_battery_info()
                thermal_conf = self.em_api.get_thermal_sensor_info()
                # store result on xml
                self.__em_meas_tab.add_dict_measurement(msic_reg)
                self.__em_meas_tab.add_dict_measurement(thermal_conf)
                # temporary workaround to detect the end condition
                batt_health = str(msic_reg["BATTERY"]["HEALTH"][0]).upper()
                if self.__ACTION[self.__action_to_peform]["START"] == batt_health:
                    exceed_timeout = False
                    break

            except AcsBaseException as e:
                crash__alowed -= 1
                # try to catch why uecmd may fail
                if not self.is_board_and_acs_ok():
                    txt = "connection with DUT lost during change waiting on DUT battery health"
                else:
                    txt = "error happened during change waiting on DUT on DUT battery health: " + str(e)
                self._logger.error(txt)
                if crash__alowed <= 0:
                    raise DeviceException(DeviceException.OPERATION_FAILED, txt)
            finally:
                # Store various information
                self.__em_meas_tab.add_measurement([self.get_time_tuple(),
                                        (self._em_cst.COMMENTS, "SETUP:Waiting for the DUT to modify its battery health inside [%s;%s] degree celsius" % (self.__start_min_temperature,
                                                                                                                                            self.__start_max_temperature))])
                # switch meas to next meas
                self.__em_meas_tab.switch_to_next_meas()
                # exist if we see the expected battery health

        if exceed_timeout:
            txt = "timeout exceeded (%ss) to see a change on battery health inside [%s,%s] degree celsius" % (str(test_timeout),
                                                                                                              self.__start_min_temperature,
                                                                                                              self.__start_max_temperature)

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        """
        # Call LAB_EM_BASE Run function
        EmUsecaseBase.run_test_body(self)

        if self.__charger_is_data_cable:
            self.__test_with_data(self.__stop_temp)
        else:
            self.__test_without_data(self.__stop_temp)

        # value comparing are done in private function
        self._em_meas_verdict.judge()

        return self._em_meas_verdict.get_current_result_v2()

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        # call tear down after some operations
        EmUsecaseBase.tear_down(self)
        if self.tc_module is not None:
            # Go to the ambient temperature
            self.tc_module.release_eq()

        # clean the board state and retrieve logs
        self.em_core_module.clean_up()

        # restore thermal management setting
        self.em_api.set_thermal_management(True)
        # a reboot is required to apply this change
        self._device.reboot()
        return Global.SUCCESS, "No errors"

    #------------------------------------------------------------------------------

    def __test_with_data(self, temperature):
        """
        test charging resume or stop while a data cable is plugged
        """
        self._logger.info("test if %s when we move from [%s;%s] degree celsius to %s degree celsius with an active data connection" % (self.__action_to_peform,
                                                                                                                                            self.__start_min_temperature,
                                                                                                                                             self.__start_max_temperature,
                                                                                                                                             self.__stop_temp))
        # when we are here, it means that we are already at START charging temperature condition
        msic_reg = self.update_battery_info()
        self._meas_list.add_dict("EM_INFO_START_TEMP", msic_reg)
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets, clean_meas_list=True)

        # change the test temperature
        if self.tc_module is not None:
            self.tc_module.get_eq().set_temperature(temperature)

        # monitor temp until we are outside the start temp
        test_timeout = self.__timeout_dut_adjust_stop_temp
        last_batt_temp = self.update_battery_info()["BATTERY"]["TEMP"][0]
        if self.__start_min_temperature <= last_batt_temp <= self.__start_max_temperature:
            consumed_time = time.time()
            self._logger.info("check that DUT is moving outside START temperature [%s;%s] degree celsius" % (self.__start_min_temperature, self.__start_max_temperature))
            exceed_timeout = True
            timeout = time.time() + test_timeout

            while time.time() < timeout:
                try:
                    # try to read measurement
                    msic_reg = self.update_battery_info()
                    thermal_conf = self.em_api.get_thermal_sensor_info()
                    # store result on xml
                    self.__em_meas_tab.add_dict_measurement(msic_reg)
                    self.__em_meas_tab.add_dict_measurement(thermal_conf)
                    last_batt_temp = msic_reg["BATTERY"]["TEMP"][0]

                except AcsBaseException as e:
                    # try to catch why uecmd may fail
                    if not self.is_board_and_acs_ok():
                        txt = "connection with DUT lost during temperature change to exit range [%s;%s] degree celsius" % (self.__start_min_temperature,
                                                                                                                           self.__start_max_temperature)
                    else:
                        txt = "error happened during temperature change to  exit range [%s;%s] degree celsius : %s" % (self.__start_min_temperature,
                                                                                                                       self.__start_max_temperature, str(e))
                    self._logger.error(txt)
                    raise DeviceException(DeviceException.OPERATION_FAILED, txt)
                finally:
                    # Store various information
                    self.__em_meas_tab.add_measurement([self.get_time_tuple(),
                                            (self._em_cst.COMMENTS, "RUNTEST:Waiting for the DUT to move outside [%s;%s] degree celsius" % (self.__start_min_temperature,
                                                                                                                                            self.__start_max_temperature))])
                    # switch meas to next meas
                    self.__em_meas_tab.switch_to_next_meas()
                # exit if the battery temp match with test temp
                if not (self.__start_min_temperature <= last_batt_temp <= self.__start_max_temperature):
                    self._logger.info("The DUT has reached %s degree celsius which is outside [%s;%s] !" % (last_batt_temp,
                                                                                                            self.__start_min_temperature,
                                                                                                            self.__start_max_temperature))
                    exceed_timeout = False
                    break
            # remove the time spent to track for the change from allowed test duration
            consumed_time = time.time() - consumed_time
            test_timeout = test_timeout - consumed_time
            if exceed_timeout:
                txt = "timeout exceeded (%ss) to move the DUT temperature outside [%s,%s] degree celsius" % (str(self.__timeout_dut_adjust_stop_temp),
                                                                                                             self.__start_min_temperature,
                                                                                                             self.__start_max_temperature)
                self._logger.error(txt)
                raise DeviceException(DeviceException.OPERATION_FAILED, txt)

        # once here we wait until a change is seen
        result = False
        exceed_timeout = True
        msic_reg = None
        time_take_to_see_a_change = time.time()
        timeout = time.time() + test_timeout
        while time.time() < timeout and not result:
            try:
                # try to read measurement
                msic_reg = self.update_battery_info()
                thermal_conf = self.em_api.get_thermal_sensor_info()
                # store result on xml
                self.__em_meas_tab.add_dict_measurement(msic_reg)
                self.__em_meas_tab.add_dict_measurement(thermal_conf)
                # temporary workaround to detect the end condition
                batt_health = str(msic_reg["BATTERY"]["HEALTH"][0]).upper()
                if batt_health != self.__ACTION[self.__action_to_peform]["START"]:
                    self._meas_list.add_dict("EM_INFO_STOP_TEMP", msic_reg)
                    if self._em_meas_verdict.test_list(self._meas_list, self._em_targets):
                        self._meas_list.add("TIME_TO_SEE_A_CHANGE", time.time() - time_take_to_see_a_change, "second")
                        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets, clean_meas_list=True)
                        result = True

            except AcsBaseException as e:
                # try to catch why uecmd may fail
                if not self.is_board_and_acs_ok():
                    txt = "connection with DUT lost during change waiting on DUT behavior"
                else:
                    txt = "error happened during change waiting on DUT behavior: " + str(e)
                self._logger.error(txt)
                raise DeviceException(DeviceException.OPERATION_FAILED, txt)
            finally:
                # Store various information
                self.__em_meas_tab.add_measurement([self.get_time_tuple(),
                                        (self._em_cst.COMMENTS, "RUNTEST:Waiting for the DUT to modify its behavior outside [%s;%s] degree celsius" % (self.__start_min_temperature,
                                                                                                                                            self.__start_max_temperature))])
                # switch meas to next meas
                self.__em_meas_tab.switch_to_next_meas()
                # exist if we see the expected battery health
                if result:
                    exceed_timeout = False
                    break

        if exceed_timeout:
            txt = "timeout exceeded (%ss) to see a change on battery/charger behavior outside [%s,%s] degree celsius" % (str(self.__timeout_dut_adjust_stop_temp),
                                                                                                                         self.__start_min_temperature,
                                                                                                                         self.__start_max_temperature)
            # take the last em info read
            if msic_reg is not None:
                self._meas_list.add_dict("EM_INFO_STOP_TEMP", msic_reg)
                self._em_meas_verdict.compare_list(self._meas_list, self._em_targets, clean_meas_list=True)
            self._logger.error(txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, txt)

    def __test_without_data(self, temperature):
        """
        test charging resume or stop while a data cable is not plugged
        """
        self._logger.info("test if %s when we move from [%s;%s] degree celsius to %s degree celsius without an active data connection" % (self.__action_to_peform,
                                                                                                                                            self.__start_min_temperature,
                                                                                                                                             self.__start_max_temperature,
                                                                                                                                             self.__stop_temp))
        self.em_api.clean_autolog()
        # choose function to put in logger
        self.em_api.add_fct_to_auto_logger(self.em_api.AUTOLOG_UEVENT, "sequenced")
        self.em_api.add_fct_to_auto_logger(self.em_api.AUTOLOG_THERMAL, "sequenced")
        self.em_api.set_autolog_duration(self.__timeout_dut_adjust_stop_temp)
        # start  non persistent autolog with a short period of data polling
        self.em_api.start_auto_logger(30, 10, "sequenced")

        # switch charger
        self._device.disconnect_board()
        self._io_card.simulate_insertion(self.__charger_type)
        self._logger.info("wait 60s to capture the first em info at START temperature")
        time.sleep(60)
        # remove the time spent in adjusting the temperature chamber and wait what it left
        remaning_time = time.time()
        self.tc_module.get_eq().set_temperature(temperature)
        self.tc_module.get_eq().wait_for_temperature(temperature)
        remaning_time = time.time() - remaning_time
        remaning_time = self.__timeout_dut_adjust_stop_temp + 60 - remaning_time

        if remaning_time > 0:
            self._logger.info("wait %ss to see any change on DUT behavior" % str(remaning_time))
            time.sleep(self.__timeout_dut_adjust_stop_temp + 60)

        self._io_card.usb_host_pc_connector(True)
        # wait x seconds
        time.sleep(self.usb_sleep)
        # connect board
        self._device.connect_board()
        self.em_api.stop_auto_logger()
        # parse autolog response and reset them
        msic_list = self.em_api.get_autolog_msic_registers()
        thermal_list = self.em_api.get_autolog_thermal_sensor_info()
        self.em_api.clean_autolog()

        log_length = len(msic_list)
        start_time = None
        stop_time = None
        battery_health_change = False
        for i in range(log_length):

            try:
                # get battery/charger info
                if len(msic_list) > i:
                    msic_dict = msic_list[i]
                    if len(msic_dict) > 1:
                        msic_dict = msic_dict[1]
                        # first track when we are outside the range
                        if start_time is None:
                            batt_temp = msic_dict["BATTERY"]["TEMP"][0]
                            if not (self.__start_min_temperature <= batt_temp <= self.__start_max_temperature):
                                start_time = msic_dict["TIME_STAMP"][0]
                                start_time = time.mktime(time.strptime(start_time, "%Y-%m-%d_%Hh%M.%S"))

                        # else track when we notice a change on battery health
                        elif stop_time is None:
                            if msic_dict["BATTERY"]["HEALTH"][0].upper() != self.__ACTION[self.__action_to_peform]["START"]:
                                battery_health_change = True
                                self._meas_list.add_dict("EM_INFO_STOP_TEMP", msic_dict)
                                # test if the verdict comparison is true
                                if self._em_meas_verdict.test_list(self._meas_list, self._em_targets):
                                    self._em_meas_verdict.compare_list(self._meas_list, self._em_targets, clean_meas_list=True)
                                    stop_time = msic_dict["TIME_STAMP"][0]
                                    stop_time = time.mktime(time.strptime(stop_time, "%Y-%m-%d_%Hh%M.%S"))
                                    stop_time -= start_time

                        # store result on xml
                        self.__em_meas_tab.add_dict_measurement(msic_dict)
                    # get thermal info
                    if len(thermal_list) > i:
                        thermal_dict = thermal_list[i]
                        if len(thermal_dict) > 1:
                            # store result on xml
                            self.__em_meas_tab.add_dict_measurement(thermal_dict[1])

            finally:
                self.__em_meas_tab.add_measurement(
                    [self.get_time_tuple(), (self._em_cst.COMMENTS,
                        "RUNTEST:Waiting temperature change without data cable plug")])
                # switch meas to next meas
                self.__em_meas_tab.switch_to_next_meas()

        # compute verdict by taking the last batt info seen
        if log_length > 0:
            self._meas_list.add_dict("EM_INFO_START_TEMP", msic_list[1][1])
        # get the duration it takes to see the right info
        if stop_time is not None:
            self._meas_list.add("TIME_TO_SEE_A_CHANGE", stop_time, "second")

        if not battery_health_change:
            self._meas_list.add_dict("EM_INFO_STOP_TEMP", msic_list[-1][1])

        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets, clean_meas_list=True)
