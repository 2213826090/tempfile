"""
:copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
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
:summary: EM - check that there is no capacity jump when a charger is plugged
:author: jortetx
:since: 20/01/2015
"""
import time
import os

from acs_test_scripts.Utilities.EMUtilities import XMLMeasurementFile
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from acs_test_scripts.UseCase.EnergyManagement.UcModule.LoadModule import LoadModule


class LabEmBattCapacityJumpOnPlug(EmUsecaseBase):
    """
    Lab Energy Management class.
    """
    DEDICATED_BENCH = "BATTERY_BENCH"

    def __init__(self, tc_name, global_config):

        EmUsecaseBase.__init__(self, tc_name, global_config)
        # init fuel gauging parameters
        self.em_core_module.init_fg_param()
        # get the tc parameters
        self.__tested_capacity = self._tc_parameters.get_param_value("TESTED_CAPACITY", default_cast_type=int)
        self.__charger = self._tc_parameters.get_param_value("CHARGER")
        self.__load = self._tc_parameters.get_param_value("LOAD")
        self.__jump_allowed = self._tc_parameters.get_param_value("JUMP_ALLOWED", default_cast_type=float)
        self.__time_to_log = self._tc_parameters.get_param_value("PLUG_DURATION", default_cast_type=int)

        # vars
        self.__time_to_charge = 60
        self.__time_to_discharge = 60
        self.__reports_file_name = os.path.join(self._saving_directory, "EM_meas_report.xml")
        # create a report file
        self.__em_meas_tab = XMLMeasurementFile(self.__reports_file_name)

        # Activate Loadmodule instance
        self.__load_module = LoadModule()
        self.__load_module.add_load(self.__load)

    # ---------------------------------------

    def set_up(self):
        EmUsecaseBase.set_up(self)
        # Update Battery Information
        em_info = self.update_battery_info()

        # we want to start at given capacity
        if self.em_core_module.is_batt_capacity_above_target(em_info, self.__tested_capacity):
            # Discharge part
            self.em_core_module.monitor_discharging(self.__tested_capacity,
                                     self.em_core_module.discharge_time,
                                     self.__em_meas_tab, self.__load_module)
        elif self.em_core_module.is_batt_capacity_below_target(em_info, self.__tested_capacity):
            # charge part
            self.em_core_module.monitor_charging(self.__tested_capacity,
                                         self.em_core_module.charge_time,
                                         self.__em_meas_tab)

        return Global.SUCCESS, "No errors"

    # ---------------------------------------

    def run_test_body(self):

        EmUsecaseBase.run_test_body(self)
        start_delay = 30
        # clean autolog
        self.em_api.clean_autolog()

        # choose to log msic register in autolog
        self.em_api.add_fct_to_auto_logger(self.em_api.AUTOLOG_UEVENT, "sequenced")

        # set time to log
        self.em_api.set_autolog_duration(self.__time_to_log)

        # start the autologger we put a first delay to have time to unplugged usb_HOST
        self.em_api.start_auto_logger(start_delay, 1)
        start_time = time.time()
        # disconnect board
        self._device.disconnect_board()
        self._io_card.remove_cable("ALL")

        # wait to be sure that the log is started
        time_left = 5 + start_delay - (time.time() - start_time)
        if time_left > 0:
            self._logger.info("waiting %ss to sync the logger start time with %s insertion" % (start_delay, self.__charger))
            time.sleep(time_left)

        # plug charger
        self._io_card.simulate_insertion(self.__charger)
        # wait a while
        self._logger.info("waiting %ss to let the board see the %s insertion and trigger a possible capacity jump" % (self.__time_to_log, self.__charger))
        time.sleep(max(self.__time_to_log - start_delay, 0))

        # connect usb_HOST
        self._io_card.remove_cable(self.__charger)
        self._io_card.usb_host_pc_connector(True)
        time.sleep(self.usb_sleep)
        # connect board
        self._device.connect_board()
        # check boot mode
        boot_mode = self._device.get_boot_mode()
        if boot_mode != "MOS":
            error_msg = "The board is seen not booted in MOS but in %s at the end of the test , cant compute result" % (boot_mode)
            self._logger.error(error_msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, error_msg)

        # stop auto logger
        self.em_api.stop_auto_logger()
        # get the results
        msic_list = self.em_api.get_autolog_msic_registers()

        # check the difference between first and last measure
        first_capacity_recorded = int(msic_list[0][1]['BATTERY']['CAPACITY'][0])
        last_capacity_recorded = int(msic_list[-1][1]['BATTERY']['CAPACITY'][0])
        # store result for analysis
        self.em_core_module.fill_autolog_result(msic_list, [], self.__em_meas_tab, "capacity jump test")

        diff_of_capacity = abs(last_capacity_recorded - first_capacity_recorded)
        if diff_of_capacity > self.__jump_allowed:
            # log measurement in file
            msg = "Capacity jump after a %s plug is more than %f" % (self.__charger, self.__jump_allowed)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        pass_msg = "No capacity jump greater than %s%% was seen during the test after plugging %s charger at %s%% of battery capacity" % (self.__jump_allowed,
                                                                                                                                        self.__charger,
                                                                                                                                        self.__tested_capacity)
        return Global.SUCCESS, pass_msg
    # ---------------------------------------

    def tear_down(self):
        EmUsecaseBase.tear_down(self)
        self.em_core_module.clean_up()
        return Global.SUCCESS, "No errors"
