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
:summary: EM - Japanese PSE regulation Compliance
:author: jvauchex, vgomberx
:since: 06/01/2014
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


class LabEmBattJapaneseRegulationCompliance(EmUsecaseBase):

    """
    Lab Energy Management class.
    """
    DEDICATED_BENCH = "BATTERY_BENCH"

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LAB_EM_BASE Init function
        EmUsecaseBase.__init__(self, tc_name, global_config)

        # init fuel gauging parameters
        self.em_core_module.init_fg_param()

        # High load
        load = self._tc_parameters.get_param_value("LOAD")
        self.__charger_type = self._tc_parameters.get_param_value("CHARGER_TYPE")
        self.__timeout_dut_adjust_charging_param = self._tc_parameters.get_param_value("TIME_TO_LET_DUT_ADJUST_CHARGING_PARAM", default_cast_type=int)
        self.__timeout_dut_adapt_temp = self._tc_parameters.get_param_value("TIME_TO_LET_DUT_ADAPT_TO_TEST_TEMP", 3600, default_cast_type=int)

        # Read TEMPERATURE from test case xml file
        self.tc_module = None
        self.__max_temperature = self._tc_parameters.get_param_value("MAX_TEMPERATURE")
        self.__min_temperature = self._tc_parameters.get_param_value("MIN_TEMPERATURE")
        if "ROOM" not in [self.__min_temperature, self.__max_temperature]:
            # get temperature chamber equipment if not room temperature
            self.__max_temperature = int(self.__max_temperature)
            self.__min_temperature = int(self.__min_temperature)
            self.tc_module = ThermalChamberModule(self._ambient_temperature)
            # inform module that this is not a default thermal test
            self.tc_module.set_test_type("SPECIFIC")
            self.__temperature = int((self.__min_temperature + self.__max_temperature) / 2)
        else:
            self.__temperature = "ROOM"

        # Summarize the parameters
        self._logger.info("Type of the charger: %s" % self.__charger_type)
        self._logger.info("Test MAX temperature: %s" % self.__max_temperature)
        self._logger.info("Test MIN temperature: %s" % self.__min_temperature)
        self._logger.info("Load to use during test: %s" % load)
        self._logger.info("Time to see change on DUT behavior at tested temperature: %s" % self.__timeout_dut_adjust_charging_param)

        # Load Module Initialization
        self.__load_module = LoadModule()
        self.__load_module.add_load(load)

        # Call ConfigsParser to parse Energy_Management
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_BATT_JAPANESE_REGULATION_COMPLIANCE", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())
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

        if self.tc_module is not None:
            self.tc_module.set_up_eq()
            self.tc_module.get_eq().set_regulation(True)
            start_temp = self.__temperature
            low_temp = self.phone_info["BATTERY"]["THERMAL_CHARGING_LOW_LIMIT"]
            high_temp = self.phone_info["BATTERY"]["THERMAL_CHARGING_HIGH_LIMIT"]
            # avoid setting temperature at not charging temp but near it
            if start_temp <= low_temp:
                self._logger.info("test temperature at %s is <= the min temperature for charging:%s, adjusting it" % (self.__temperature, low_temp))
                start_temp = low_temp + 10
            elif start_temp >= high_temp :
                self._logger.info("test temperature at %s is >= the max temperature for charging:%s, adjusting it" % (self.__temperature, high_temp))
                start_temp = high_temp - 10
            self.tc_module.get_eq().set_temperature(start_temp)

        # set the temperature
        self.em_api.set_thermal_management(False)
        # a reboot is required to apply this change
        self._device.reboot()

        # do this job one time in case of B2B
        if self.__charger_is_data_cable is None:
            self.__charger_is_data_cable = self.em_core_module.is_host_connection_available_when_charger_plug(self.__charger_type)
            self._logger.info("test sequences will adjust to this")

        # Update Battery Information
        em_info = self.update_battery_info()
        if self.em_core_module.is_batt_capacity_above_target(em_info, self.em_core_module.batt_max_capacity):

            # Discharge part
            self.em_core_module.monitor_discharging(self.em_core_module.batt_max_capacity,
                                     self.em_core_module.discharge_time,
                                     self.__em_meas_tab, self.__load_module)
        # wait that the board see the test temperature
        self.__wait_for_temp(self.__temperature)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        """
        # Call LAB_EM_BASE Run function
        EmUsecaseBase.run_test_body(self)

        if self.__charger_is_data_cable:
            self.__test_with_data(self.__temperature)
        else:
            self.__test_without_data(self.__temperature)

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

    def __wait_for_temp(self, temperature):
        """
        wait that the DUT see the test temperature
        """
        # heat the DUT
        self._logger.info("check that DUT is between [%s;%s] degree celsius" % (self.__min_temperature, self.__max_temperature))
        last_batt_temp = self.update_battery_info()["BATTERY"]["TEMP"][0]
        if self.tc_module is not None and not (self.__min_temperature <= last_batt_temp <= self.__max_temperature):
            self.phonesystem_api.sleep_screen()
            exceed_timeout = True
            self.tc_module.get_eq().set_temperature(temperature)
            self.tc_module.get_eq().wait_for_temperature(temperature)
            # hardcoded 1h of timeout and 300s for a change in temperature on the board
            timeout_to_see_temp_change = 300
            start_time_to_see_temp_change = time.time()
            timeout = time.time() + self.__timeout_dut_adapt_temp
            while time.time() < timeout:
                self._logger.info("wait 60s")
                time.sleep(60)
                try:
                    # try to read measurement
                    msic_reg = self.update_battery_info()
                    thermal_conf = self.em_api.get_thermal_sensor_info()
                    # store result on xml
                    self.__em_meas_tab.add_dict_measurement(msic_reg)
                    self.__em_meas_tab.add_dict_measurement(thermal_conf)
                    local_temp = msic_reg["BATTERY"]["TEMP"][0]

                    if last_batt_temp != local_temp:
                        start_time_to_see_temp_change = time.time()
                    last_batt_temp = local_temp

                    # stop usecase if board take too long to reach battery capacity
                    if (time.time() - start_time_to_see_temp_change) > timeout_to_see_temp_change:
                        self._logger.info("DUT temperature did not change in %ss, adjusting chamber temperature" % timeout_to_see_temp_change)
                        if local_temp < self.__min_temperature:
                            temperature += 1

                        elif  local_temp > self.__max_temperature:
                            temperature -= 1
                        self.tc_module.get_eq().set_temperature(temperature)
                        start_time_to_see_temp_change = time.time()

                except AcsBaseException as e:
                    # try to catch why uecmd may fail
                    if not self.is_board_and_acs_ok():
                        txt = "connection with DUT lost during temperature change to be between [%s;%s] degree celsius" % (self.__min_temperature, self.__max_temperature)
                    else:
                        txt = "error happened during temperature change to let it see [%s;%s] degree celsius : %s" % (self.__min_temperature, self.__max_temperature, str(e))
                    self._logger.error(txt)
                    raise DeviceException(DeviceException.OPERATION_FAILED, txt)
                finally:
                    # Store various information
                    self.__em_meas_tab.add_measurement([self.get_time_tuple(),
                                            (self._em_cst.COMMENTS, "SETUP:Waiting for the DUT to reach [%s;%s] degree celsius" % (self.__min_temperature, self.__max_temperature))])
                    # switch meas to next meas
                    self.__em_meas_tab.switch_to_next_meas()
                # exit if the battery temp match with test temp
                if self.__min_temperature <= last_batt_temp <= self.__max_temperature:
                    self._logger.info("The DUT has reached %s degree celsius which is between expected target [%s;%s] !" % (last_batt_temp, self.__min_temperature, self.__max_temperature))
                    exceed_timeout = False
                    break

            if exceed_timeout:
                txt = "timeout exceeded (%ss) to heat up the DUT to %s degree celsius" % (str(self.__timeout_dut_adapt_temp), temperature)
                self._logger.error(txt)
                raise DeviceException(DeviceException.OPERATION_FAILED, txt)

    def __test_with_data(self, temperature):
        """
        test that we can see the CC & CV expected for this temperature
        with a data cable plug
        """
        self._logger.info("testing charger and battery behavior at [%s;%s] degree celsius while data is active" % (self.__min_temperature, self.__max_temperature))
        exceed_timeout = True
        msic_reg = self.update_battery_info()
        self._meas_list.add_dict("EM_INFO", msic_reg)
        result = self._em_meas_verdict.compare_list(self._meas_list, self._em_targets, clean_meas_list=True)
        if result:
            exceed_timeout = False

        # timeout to see CC and CV updated
        timeout = time.time() + self.__timeout_dut_adjust_charging_param
        while time.time() < timeout and not result:
            self._logger.info("wait 60s")
            time.sleep(60)
            try:
                # try to read measurement
                msic_reg = self.update_battery_info()
                thermal_conf = self.em_api.get_thermal_sensor_info()
                # store result on xml
                self.__em_meas_tab.add_dict_measurement(msic_reg)
                self.__em_meas_tab.add_dict_measurement(thermal_conf)
                # compare values with targets
                self._meas_list.add_dict("EM_INFO", msic_reg)
                result = self._em_meas_verdict.test_list(self._meas_list, self._em_targets)
                if result:
                    self._em_meas_verdict.compare_list(self._meas_list, self._em_targets, clean_meas_list=True)

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
                                        (self._em_cst.COMMENTS, "RUNTEST:Waiting for the DUT behavior to change at [%s;%s] degree" % (self.__min_temperature, self.__max_temperature))])
                # switch meas to next meas
                self.__em_meas_tab.switch_to_next_meas()

        if not result:
            self._meas_list.add_dict("EM_INFO", msic_reg)
            self._em_meas_verdict.compare_list(self._meas_list, self._em_targets, clean_meas_list=True)

        if exceed_timeout:
            txt = "timeout exceeded (%ss) to wait for a change on DUT behavior" % str(self.__timeout_dut_adjust_charging_param)
            self._logger.error(txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, txt)

    def __test_without_data(self, temperature):
        """
        test that we can see the CC & CV expected for this temperature
        without a data cable plug
        """
        self._logger.info("testing charger and battery behavior at  [%s;%s] degree celsius while data is not active" % (self.__min_temperature, self.__max_temperature))
        self.em_api.clean_autolog()
        # choose function to put in logger
        self.em_api.add_fct_to_auto_logger(self.em_api.AUTOLOG_UEVENT, "sequenced")
        self.em_api.add_fct_to_auto_logger(self.em_api.AUTOLOG_THERMAL, "sequenced")
        self.em_api.set_autolog_duration(self.__timeout_dut_adjust_charging_param)
        # start  non persistent autolog with a short period of data polling
        self.em_api.start_auto_logger(30, 10, "sequenced")

        # switch charger
        self._device.disconnect_board()
        # connect WALL Charger
        self._io_card.simulate_insertion(self.__charger_type)
        # wait x min
        self._logger.info("wait %ss to see any change on DUT behavior" % str(self.__timeout_dut_adjust_charging_param + 60))
        time.sleep(self.__timeout_dut_adjust_charging_param + 60)
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
        # fill autolog
        self.em_core_module.fill_autolog_result(msic_list, thermal_list, self.__em_meas_tab,
                     "RUNTEST:Waiting for the DUT behavior to change when it is between [%s;%s] degree" % (self.__min_temperature, self.__max_temperature))
        # compute verdict by taking the last batt info seen
        if len(msic_list) > 0 and len(msic_list[-1]) > 0:
            self._meas_list.add_dict("EM_INFO", msic_list[-1][1])
            self._em_meas_verdict.compare_list(self._meas_list, self._em_targets, clean_meas_list=True)
