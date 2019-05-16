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
:summary: EM - Battery Full Identification
:author: jvauchex, vgomberx
:since: 10/02/2014
"""
import os
import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.EMUtilities import XMLMeasurementFile
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.UseCase.EnergyManagement.UcModule.LoadModule import LoadModule
from ErrorHandling.AcsConfigException import AcsConfigException


class LabEmBattFullIdentification(EmUsecaseBase):

    """
    Lab Energy Management class.
    """
    DEDICATED_BENCH = "BATTERY_BENCH"
    __ACTION = ["CAPACITY_JUMP_NEAR_FULL",
                "CAPACITY_AT_WHICH_FIRST_FULL_APPEAR",
                "TIME_TO_CHARGE_TO_FULL_FROM_START_CAPACITY",
                "TIME_TO_SEE_FULL_AT_MAX_CAPACITY",
                "CHARGE_STOP_ONCE_FULL"]

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LAB_EM_BASE Init function
        EmUsecaseBase.__init__(self, tc_name, global_config)

        # init fuel gauging parameters
        self.em_core_module.init_fg_param()
        # Type of the charger
        self.__charger_type = self._tc_parameters.get_param_value("CHARGER_TYPE")
        self.__load = self._tc_parameters.get_param_value("LOAD_TO_HELP_DISCHARGE_IN_SETUP")
        # charge to Full Timeout
        self.__charge_full_timeout = self._tc_parameters.get_param_value("CHARGE_TO_FULL_DURATION", default_cast_type=int)
        # capacity that match with first full
        self.__first_full_capacity = self._tc_parameters.get_param_value("CAPACITY_AT_WHICH_FULL_FIRST_APPEAR", 100, default_cast_type=int)
        self.__what_to_check = str(self._tc_parameters.get_param_value("WHAT_TO_CHECK")).upper()

        # Summarize the parameters
        self._logger.info("Charger type to plug during FULL charging: %s" % self.__charger_type)
        self._logger.info("Capacity at which FULL shall first appear: %d" % self.__first_full_capacity)
        self._logger.info("Test expected start capacity : %s" % self.em_core_module.batt_start_capacity)

        # Load Module Initialization
        self.__load_module = LoadModule()
        self.__load_module.add_load(self.__load)

        # Call ConfigsParser to parse Energy_Management
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_BATT_FULL_IDENTIFICATION", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets, consider_all_target=True)

        # measurement file
        meas_file_name = os.path.join(self._saving_directory,
                                      "EM_meas_report.xml")
        self.__em_meas_tab = XMLMeasurementFile(meas_file_name)

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # Call the UseCaseBase Setup function
        EmUsecaseBase.set_up(self)

        # Check expected result first
        if self.__what_to_check not in self.__ACTION:
            txt = "wrong value for TC parameter WHAT_TO_CHECK %s, can only be %s" % (self.__what_to_check,
                                                                                          str(self.__ACTION))
            self._logger.error(txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, txt)

        # Update Battery Information
        em_info = self.update_battery_info()

        if self.em_core_module.is_batt_capacity_above_target(em_info, self.em_core_module.batt_start_capacity):
            # Discharge part
            self.em_core_module.monitor_discharging(self.em_core_module.batt_start_capacity,
                                     self.em_core_module.discharge_time,
                                     self.__em_meas_tab, self.__load_module)
        elif self.em_core_module.is_batt_capacity_below_target(em_info, self.em_core_module.batt_start_capacity):
            # Charge part
            self.em_core_module.monitor_charging(self.em_core_module.batt_start_capacity,
                                     self.em_core_module.charge_time,
                                     self.__em_meas_tab)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        Check the end of the main battery charge
        """
        # Call LAB_EM_BASE Run function
        EmUsecaseBase.run_test_body(self)

        # first of all charge to full
        em_info_list = self.__charge_without_data()
        start_index = None
        full_seen = False
        index = 0
        # search for the first iteration of the capacity at which full appear
        for em_dico in em_info_list:
            if em_dico["BATTERY"]["CAPACITY"][0] == self.__first_full_capacity and start_index is None:
                start_index = index
            if em_dico["BATTERY"]["STATUS"][0] == "FULL":
                full_seen = True
                break
            index += 1

        # all the test above consider that the FULL status has already been seen
        # else an error is raised here
        if not full_seen:
            txt = "Battery FULL status was not reached after waiting %ss with %s plugged" % (self.__charge_full_timeout, self.__charger_type)
            self._logger.error(txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, txt)

        if self.__what_to_check == "CAPACITY_AT_WHICH_FIRST_FULL_APPEAR":
            self.__check_first_full_capacity(em_info_list)
        elif self.__what_to_check == "TIME_TO_SEE_FULL_AT_MAX_CAPACITY":
            self.__check_first_full_timeout(em_info_list, start_index)
        elif self.__what_to_check == "TIME_TO_CHARGE_TO_FULL_FROM_START_CAPACITY":
            self.__check_charge_to_full_timeout(em_info_list)
        elif self.__what_to_check == "CAPACITY_JUMP_NEAR_FULL":
            self.__check_capacity_jump(em_info_list, start_index)
        elif self.__what_to_check == "CHARGE_STOP_ONCE_FULL":
            self.__check_charge_stop(em_info_list)

        # compare values with targets
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets, clean_meas_list=True)
        self._em_meas_verdict.judge()

        return self._em_meas_verdict.get_current_result_v2()

    def tear_down(self):
        """
        End and dispose the test
        """
        # call tear down after some operations
        EmUsecaseBase.tear_down(self)
        # clean the board state and retrieve logs
        self.em_core_module.clean_up()
        self.__load_module.clean()

        return Global.SUCCESS, "No errors"

    def __charge_without_data(self):
        """
        charge without a data cable plug

        :rtype: list
        :return: list of em dictionary
        """
        adjusted_time = 90
        self._logger.info("Trying to charge board from %s%% to FULL during %ss" % (self.batt_capacity, self.__charge_full_timeout))
        self.em_api.clean_autolog()
        # choose function to put in logger
        self.em_api.add_fct_to_auto_logger(self.em_api.AUTOLOG_UEVENT, "sequenced")
        self.em_api.add_fct_to_auto_logger(self.em_api.AUTOLOG_THERMAL, "sequenced")
        self.em_api.set_autolog_duration(self.__charge_full_timeout)
        # start non persistent autolog with a short period of data polling
        self.em_api.start_auto_logger(30, 10, "sequenced")

        # switch charger
        self._device.disconnect_board()
        # connect WALL Charger
        self._io_card.simulate_insertion(self.__charger_type)
        # wait x min
        self._logger.info("waiting %ss (+ %ss adjusted) to reach FULL status" % (self.__charge_full_timeout, adjusted_time))
        time.sleep(self.__charge_full_timeout + adjusted_time)

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

        # get the highest logs length
        log_length = max(len(thermal_list), len(msic_list))
        em_info_list = []
        for i in range(log_length):

            try:
                # get battery/charger info
                if len(msic_list) > i:
                    msic_dict = msic_list[i]
                    if len(msic_dict) > 1:
                        msic_dict = msic_dict[1]
                        # store result on xml
                        em_info_list.append(msic_dict)
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
                        "RUNTEST:charging board to FULL")])
                # switch meas to next meas
                self.__em_meas_tab.switch_to_next_meas()

        return em_info_list

    def __check_capacity_jump(self, em_info_list, start_index):
        """
        check if there is capacity jump when charging to full
        """
        self._logger.info("Check if capacity jump happen when we are near to FULL ...")
        # Check if start index is superior than 0 , else we can't evaluate the jump state
        if start_index in [0, None]:
            txt = "Can't compute capacity jump as the first measurement was already at %s%% which is the capacity where battery FULL should firstly appear" % self.__first_full_capacity
            self._logger.error(txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, txt)

        end_capacity = em_info_list[start_index]["BATTERY"]["CAPACITY"][0]
        start_capacity = em_info_list[start_index - 1]["BATTERY"]["CAPACITY"][0]
        measured_jump = end_capacity - start_capacity

        self._meas_list.add("CAPACITY_JUMP_AT_MAX_CAPACITY", measured_jump, "none")

    def __check_charge_to_full_timeout(self, em_info_list):
        """
        Check the time it takes to see status FULL from start capacity
        """
        self._logger.info("Check the time it takes to see status FULL from the start capacity at %s%%" % self.em_core_module.batt_start_capacity)
        stop_time = None
        start_time = em_info_list[0]["TIME_STAMP"][0]
        start_time = time.mktime(time.strptime(start_time, "%Y-%m-%d_%Hh%M.%S"))

        # search for the FULL
        for em_dico in em_info_list:
            if em_dico["BATTERY"]["STATUS"][0] == "FULL":
                stop_time = em_dico["TIME_STAMP"][0]
                stop_time = time.mktime(time.strptime(stop_time, "%Y-%m-%d_%Hh%M.%S"))
                break

        # raise error if full is not seen
        if stop_time is None:
            txt = "Battery FULL status was never seen"
            self._logger.error(txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, txt)

        self._meas_list.add("TIME_TO_CHARGE_TO_FULL", stop_time - start_time, "second")

    def __check_first_full_capacity(self, em_info_list):
        """
        check that the first full seen appear at the right capacity
        """
        self._logger.info("check that the first FULL seen appear at the right capacity")
        full_capacity = None
        for em_dico in em_info_list:
            if em_dico["BATTERY"]["STATUS"][0] == "FULL":
                full_capacity = em_dico["BATTERY"]["CAPACITY"][0]
                break

        if full_capacity != self.__first_full_capacity:
            txt = "Battery FULL status was seen at %s%% of capacity instead of expecting %s%%" % (full_capacity, self.__first_full_capacity)
            self._logger.error(txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, txt)

        self._meas_list.add("CAPACITY_AT_WHICH_FIRST_FULL_APPEAR", full_capacity, "none")

    def __check_first_full_timeout(self, em_info_list, start_index):
        """
        Check the time it takes to see status FULL once max capacity is seen
        """
        self._logger.info("Check the time it takes to see status FULL from the first %s%% of capacity seen ..." % self.__first_full_capacity)

        if start_index is None:
            txt = "Can't compute the time it takes to see FULL status once %s%% capacity is reached: this capacity was never reached" % self.__first_full_capacity
            self._logger.error(txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, txt)

        stop_time = None
        stop_capacity = None
        start_time = em_info_list[start_index]["TIME_STAMP"][0]
        start_time = time.mktime(time.strptime(start_time, "%Y-%m-%d_%Hh%M.%S"))

        # search for the FULL
        for em_dico in em_info_list:
            if em_dico["BATTERY"]["STATUS"][0] == "FULL":
                stop_time = em_dico["TIME_STAMP"][0]
                stop_time = time.mktime(time.strptime(stop_time, "%Y-%m-%d_%Hh%M.%S"))
                stop_capacity = em_dico["BATTERY"]["CAPACITY"][0]
                break

        # raise error if full is not seen
        if stop_capacity is not None and stop_capacity != self.__first_full_capacity:
            txt = "Battery FULL status was seen at %s%% of capacity instead of expecting %s%%" % (stop_capacity, self.__first_full_capacity)
            self._logger.error(txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, txt)

        self._meas_list.add("TIME_TO_SEE_FULL_AT_MAX_CAPACITY", stop_time - start_time, "second")

    def __check_charge_stop(self, em_info_list):
        """
        Check that the charge stop once FULL is reached
        """
        self._logger.info("Check that the charge stop once FULL is reached")
        result = False
        # search for the FULL
        for em_dico in em_info_list:
            if em_dico["BATTERY"]["STATUS"][0] == "FULL" and em_dico["CHARGER"]["ENABLE_CHARGING"][0] == 0:
                self._meas_list.add_dict("EM_INFO", em_dico)
                result = True
                break

        if not result:
            txt = "The charge did not stop after battery FULL reached"
            self._logger.error(txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, txt)
