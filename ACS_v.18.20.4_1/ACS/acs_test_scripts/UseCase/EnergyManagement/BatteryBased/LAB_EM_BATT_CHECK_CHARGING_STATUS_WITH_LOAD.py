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
:summary: EM - check the status is charging when a charger is plugged with a load
:author: jortetx
:since: 20/01/2015
"""
import time
import os

from acs_test_scripts.Utilities.EMUtilities import XMLMeasurementFile
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from acs_test_scripts.UseCase.EnergyManagement.UcModule.UcBatteryModule import UcBatteryModule
from acs_test_scripts.UseCase.EnergyManagement.UcModule.LoadModule import LoadModule


class LabEmBattCheckChargingStatusWithLoad(EmUsecaseBase):
    """
    Lab Energy Management class.
    """
    DEDICATED_BENCH = "BATTERY_BENCH"

    def __init__(self, tc_name, global_config):

        EmUsecaseBase.__init__(self, tc_name, global_config)

        # get base module
        self.em_core_module = UcBatteryModule(self)

        # get the tc parameters
        self.__tested_capacity = self._tc_parameters.get_param_value("TESTED_CAPACITY", default_cast_type=int)
        self.__charger = self._tc_parameters.get_param_value("CHARGER")
        self.__status = self._tc_parameters.get_param_value("STATUS").upper()
        self.__load = self._tc_parameters.get_param_value("LOAD")

        # vars
        self.__time_to_charge = 60
        self.__time_to_discharge = 60
        self.__reports_file_name = os.path.join(self._saving_directory, "EM_meas_report.xml")
        self.__time_to_log = 60
        # create a report file
        self.__report_file_handler = XMLMeasurementFile(self.__reports_file_name)

        # Activate Loadmodule instance
        self.__load_module = LoadModule()
        self.__load_module.add_load(self.__load)

        # delay to launch autologger
        self.__autolog_delay = 20

    # ---------------------------------------

    def set_up(self):

        EmUsecaseBase.set_up(self)

        # get the tested capacity
        batt_info = self.update_battery_info()

        # initial state is dut charge at max_range
        if self.em_core_module.is_batt_capacity_below_target(batt_info, self.__tested_capacity):
            self._logger.info("Device capacity is %s : going to %s" % (self.batt_capacity, self.__tested_capacity))
            self.em_core_module.monitor_charging(self.__tested_capacity, self.__time_to_charge, self.__report_file_handler)
        else:
            self._logger.info("Device capacity is %s : going to %s" % (self.batt_capacity, self.__tested_capacity))
            self.em_core_module.monitor_discharging(self.__tested_capacity, self.__time_to_discharge, self.__report_file_handler, load_module=self.__load_module)

        return Global.SUCCESS, "No errors"

    # ---------------------------------------

    def run_test_body(self):

        EmUsecaseBase.run_test_body(self)

        status_list = []

        # clean autolog
        self.em_api.clean_autolog()

        # choose to log msic register in autolog
        self.em_api.add_fct_to_auto_logger(self.em_api.AUTOLOG_UEVENT, "sequenced")

        # set time to log
        self.em_api.set_autolog_duration(self.__time_to_log)

        # start the autologger we put a first delay to have time to unplugged usb_HOST
        self.em_api.start_auto_logger(self.__autolog_delay, 1, "sequenced")

        # start the load
        self.__load_module.start_load()

        # disconnect board
        self._device.disconnect_board()
        self._io_card.remove_cable("ALL")

        # plug charger
        self._io_card.simulate_insertion(self.__charger)

        # wait a while
        time.sleep(self.__time_to_log + self.__autolog_delay)

        # connect usb_HOST
        self._io_card.usb_host_pc_connector(True)

        # connect board
        self._device.connect_board()
        time.sleep(self.usb_sleep)

        # stop auto logger
        self.em_api.stop_auto_logger()

        # get the results
        msic_list = self.em_api.get_autolog_msic_registers()

        # check if tested status is seen
        for measure_index in range(len(msic_list)):
            status_list.append(msic_list[measure_index][1]['BATTERY']['STATUS'][0].upper())

        # fill autolog result in file
        self.em_core_module.fill_autolog_result(msic_list, [], self.__report_file_handler, "Results of autologger")

        # raise an error if status is not seen
        if self.__status not in status_list:
            msg = "Status %s not seen on plug" % self.__status
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        return Global.SUCCESS, "No errors"
    # ---------------------------------------

    def tear_down(self):

        EmUsecaseBase.tear_down(self)
        # stop load
        self.em_core_module.clean_up()
        self.__load_module.stop_load()

        return Global.SUCCESS, "No errors"

