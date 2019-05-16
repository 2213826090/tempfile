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

:organization: INTEL MCG PSI
:summary: check that the battery board used the new battery technology with Li-Ion/polymer : 4.35V
:author: jortetx, vgomberx
:since: Sept 11 2014
"""
import os
import time

from acs_test_scripts.Utilities.EMUtilities import XMLMeasurementFile
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase


class LabEmNewBatTechno(EmUsecaseBase):

    """
    Lab Energy Management base class.
    """

    DEDICATED_BENCH = "BATTERY_BENCH"

    def __init__(self, tc_name, global_config):

        EmUsecaseBase.__init__(self, tc_name, global_config)
        # init fuel gauging parameters
        self.em_core_module.init_fg_param()
        # get parameters
        self.__high_voltage_range = self._tc_parameters.get_param_value("HIGH_VOLTAGE_RANGE", default_cast_type=float)
        self.__low_voltage_range = self._tc_parameters.get_param_value("LOW_VOLTAGE_RANGE", default_cast_type=float)
        self.__time_to_reach_full = self._tc_parameters.get_param_value("TIME_TO_CHARGE_TO_FULL", default_cast_type=int)

        # create a report file to save all charge/discharge value for debug
        self.__em_meas_tab = XMLMeasurementFile(os.path.join(self._saving_directory, "em_meas_report.xml"))

    def set_up(self):
        """
        charge board to FULL or near it
        """
        EmUsecaseBase.set_up(self)
        # Update Battery Information
        em_info = self.update_battery_info()
        # we want to start above a minimal capacity
        if self.em_core_module.is_batt_capacity_below_target(em_info, self.em_core_module.batt_min_capacity):
            # charge part
            self.em_core_module.monitor_charging(self.em_core_module.batt_min_capacity,
                                         self.em_core_module.charge_time,
                                         self.__em_meas_tab)

        return Global.SUCCESS, "No errors"

    def run_test_body(self):
        EmUsecaseBase.run_test_body(self)
        self.__test_without_data()
        # if the test fail then an error will be raised before reaching here
        passed_msg = "battery Voltage was between %s and %s all along the FULL state seen" % (self.__low_voltage_range, self.__high_voltage_range)
        return Global.SUCCESS, passed_msg

    def tear_down(self):
        EmUsecaseBase.tear_down(self)
        self.em_core_module.clean_up()
        return Global.SUCCESS, "No errors"

    def __test_without_data(self):
        """
        charge battery to full
        """
        self.em_api.clean_autolog()
        # choose function to put in logger
        self.em_api.add_fct_to_auto_logger(self.em_api.AUTOLOG_UEVENT, "sequenced")
        self.em_api.add_fct_to_auto_logger(self.em_api.AUTOLOG_THERMAL, "sequenced")
        self.em_api.set_autolog_duration(self.__time_to_reach_full)
        # start  non persistent autolog with a short period of data polling
        self.em_api.start_auto_logger(30, 10, "sequenced")
        # switch charger
        self._device.disconnect_board()
        # connect WALL Charger
        self._io_card.wall_charger_connector(True)
        # wait x min
        self._logger.info("waiting %ss + %ss to reach battery FULL" % (str(self.__time_to_reach_full), 60))
        time.sleep(self.__time_to_reach_full + 60)
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

        # init var
        status = "UNKNOWN"
        full_seen_one_time = False
        self.em_core_module.fill_autolog_result(msic_list, thermal_list, self.__em_meas_tab, "RUNTEST: trying to charge to FULL")
        for msic_dict in msic_list:
            if len(msic_dict) > 1:
                msic_dict = msic_dict[1]
                # check that we reach full and stay in this state
                status = str(msic_dict["BATTERY"]["STATUS"][0]).upper()
                if status == "FULL":
                    voltage = msic_dict["BATTERY"]["VOLTAGE"][0]
                    full_seen_one_time = True
                    if not (self.__low_voltage_range <= voltage <= self.__high_voltage_range):
                        msg = "battery Voltage (%s) is not between %s and %s at FULL state" % (voltage, self.__low_voltage_range, self.__high_voltage_range)
                        self._logger.error(msg)
                        raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # if full is never seen, raise an error with the last status seen
        if not full_seen_one_time:
            msg = "battery status did not reach FULL but is seen in %s at the end of the measurement done" % status
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
