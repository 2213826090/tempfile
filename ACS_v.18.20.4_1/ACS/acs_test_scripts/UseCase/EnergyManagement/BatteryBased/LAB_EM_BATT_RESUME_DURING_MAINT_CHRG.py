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
:summary: check the charging resume during maintenance charging when battery is discharged from the FULL state
:author: jortetx
:since: July 20 2014
"""
import os
import time

from acs_test_scripts.Utilities.EMUtilities import XMLMeasurementFile
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from acs_test_scripts.UseCase.EnergyManagement.UcModule.LoadModule import LoadModule


class LabEmBattResumeDuringMaintChrg(EmUsecaseBase):

    """
    Lab Energy Management base class.
    """
    DEDICATED_BENCH = "BATTERY_BENCH"

    def __init__(self, tc_name, global_config):

        EmUsecaseBase.__init__(self, tc_name, global_config)

        # get parameters
        self.__charger_to_use = self._tc_parameters.get_param_value("CHARGER")
        self.__load = self._tc_parameters.get_param_value("LOAD")
        self.__time_to_charge_full = self._tc_parameters.get_param_value("TIME_TO_FULL", default_cast_type=int)
        self.__exit_maint_threshold = self._tc_parameters.get_param_value("EXIT_MAINTENANCE_CHARGING_THRESHOLD", default_cast_type=float)
        self.__time_to_stabilize_board = self._tc_parameters.get_param_value("TIME_TO_STABILIZE_BOARD", default_cast_type=int)
        self.__test_type = str(self._tc_parameters.get_param_value("TEST_TYPE")).upper()
        self.__test_timeout = self._tc_parameters.get_param_value("TEST_TIMEOUT", default_cast_type=int)

        # create a report file to save all charge/discharge value for debug
        self.__report_file_handler = XMLMeasurementFile(os.path.join(self._saving_directory, "em_meas_report.xml"))

        # instance of load module
        self.__load_module = LoadModule()
        self.__load_module.add_load(self.__load)

        # constant
        self.__full_status = "FULL"
        self.__charging_status = "CHARGING"

    # ---------------------------------------

    def set_up(self):

        EmUsecaseBase.set_up(self)

        if not self.__test_type in ["STAY_IN_FULL", "EXIT_MAINTENANCE_TRESHOLD"]:
            msg = "Parameter TEST_TYPE should be STAY_IN_FULL or EXIT_MAINTENANCE_TRESHOLD"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # this test can be done only if we have data
        if not self.em_core_module.is_host_connection_available_when_charger_plug(self.__charger_to_use):
            msg = "Data is necessary to perform the test, not data is seen when we plug %s" % self.__charger_to_use
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, msg)

        # make sure that the status is FULL
        result = self.update_battery_info()
        batt_status = result["BATTERY"]["STATUS"][0].upper()

        if batt_status != self.__full_status:
            self.em_core_module.monitor_charging(self.__full_status, self.__time_to_charge_full, self.__report_file_handler)

        # at this time we plug the charger and monitor the full state
        self._logger.info("try to see if we can see battery FULL state when %s is plugged" % self.__charger_to_use)
        self._device.disconnect_board()
        self._io_card.simulate_insertion(self.__charger_to_use)
        time.sleep(self.usb_sleep)
        self._device.connect_board()
        timeout = 300
        full_seen = False
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.__get_battery_status("SETUP:SEEKING FOR FULL") == self.__full_status:
                full_seen = True
                break

        if not full_seen:
            msg = "FULL status was not seen after %ss with %s plugged, stopping the test" % (timeout, self.__charger_to_use)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        return Global.SUCCESS, "No errors"

    # ---------------------------------------

    def run_test_body(self):

        EmUsecaseBase.run_test_body(self)
        time_after_treshold = 0
        time_begin_test = time.time()
        full_test_passed = False
        charging_test_passed = False
        comment = "RUNTEST:SEEKING FOR FULL STATE DURING MAINTENANCE"

        # battery is full, starting loads
        self.__load_module.start_load()

        while False in [full_test_passed, charging_test_passed]:

            # check if test timeout is reached
            if (time.time() - time_begin_test) >= self.__test_timeout:
                msg = "test timeout is reached, device does not discharge correctly"
                self._logger.error(msg)
                raise DeviceException(DeviceException.TIMEOUT_REACHED, msg)

            # check if voltage is above threshold
            batt_status = self.__get_battery_status(comment)

            # case : above threshold
            if float(self.batt_voltage) > self.__exit_maint_threshold:
                # status should be FULL
                if batt_status != self.__full_status:
                    if self.__test_type in "STAY_IN_FULL":
                        msg = "Status is %s it should be %s when battery voltage is above EXIT_MAINTENANCE_CHARGING_THRESHOLD" % (batt_status, self.__full_status)
                        self._logger.error(msg)
                        raise DeviceException(DeviceException.OPERATION_FAILED, msg)
                else:
                    if not full_test_passed:
                        full_test_passed = True
            # case : below threshold
            else:
                comment = "RUNTEST:SEEKING FOR MAINTENANCE CHARGING EXIT"
                if self.__test_type in "STAY_IN_FULL":
                    break
                # status should be charging
                if time_after_treshold == 0:
                    time_after_treshold = time.time()
                if batt_status != self.__charging_status and (time.time() - time_after_treshold) > self.__time_to_stabilize_board:
                    msg = "Status is %s it should be %s when battery voltage is below EXIT_MAINTENANCE_CHARGING_THRESHOLD" % (batt_status, self.__charging_status)
                    self._logger.error(msg)
                    raise DeviceException(DeviceException.OPERATION_FAILED, msg)
                elif batt_status == self.__charging_status:
                    if not charging_test_passed:
                        changing_time = time.gmtime(time.time() - time_after_treshold)
                        log_msg = "Battery status change to %s %d min %ds after threshold reached " % (self.__charging_status, changing_time.tm_min, changing_time.tm_sec)
                        self._logger.info(log_msg)
                        charging_test_passed = True

            if False in [full_test_passed, charging_test_passed]:
                # make sure that LOADs still ON
                self.__load_module.restart_load(consider_only_checkable_load=True)

        # if we reach here , it means that the test is pass
        if self.__test_type in "STAY_IN_FULL":
            pass_msg = "Battery status stay at FULL while voltage is above EXIT_MAINTENANCE_TRESHOLD"
        else:
            pass_msg = "Battery status change to CHARGING when voltage reach EXIT_MAINTENANCE_TRESHOLD"
        return Global.SUCCESS, pass_msg

    # ---------------------------------------

    def tear_down(self):

        EmUsecaseBase.tear_down(self)
        self.em_core_module.clean_up()
        self.__load_module.clean()
        return Global.SUCCESS, "No errors"

    # ---------------------------------------

    def __get_battery_status(self, comment):
        """
        launch command to get battery info , return them as dict
        also update protected var that target voltage & capacity

        :param log: Log the battery info in the measurement file or not
        """
        result = self.update_battery_info()
        batt_status = result["BATTERY"]["STATUS"][0].upper()
        self.__report_file_handler.add_dict_measurement(result)
        self.__report_file_handler.add_measurement([self.get_time_tuple(),
                                            (self._em_cst.COMMENTS, comment)])

        self.__report_file_handler.switch_to_next_meas()
        return batt_status
