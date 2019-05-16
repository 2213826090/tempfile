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
:summary: Energy Management automatic graceful shutdown at very low battery
:author: dbatutx, vgomberx
:since: 05/11/2011
"""
import time
import os
from acs_test_scripts.Utilities.EMUtilities import XMLMeasurementFile
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.AcsBaseException import AcsBaseException
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.UseCase.EnergyManagement.UcModule.LoadModule import LoadModule


class LabEmBattLowCapShutdown(EmUsecaseBase):

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
        # Read the battery capacity expected for the init of the test
        self.__discharge_timeout = self._tc_parameters.get_param_value("DISCHARGE_TIMEOUT", default_cast_type=int)
        self.__cable_type = self._tc_parameters.get_param_value("CABLE_TYPE_DURING_DISCHARGE")
        load_to_apply_in_setup = self._tc_parameters.get_param_value("LOAD_TO_HELP_DISCHARGE")

        #-----------------------------------------------------------------------
        # Get load Parameters
        self.__load_module = LoadModule()
        self.__load_module.add_load(load_to_apply_in_setup)

        #-----------------------------------------------------------------------

        # Call ConfigsParser to parse Energy_Management
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_BATT_LOW_CAP_SHUTDOWN", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets, self.tcd_to_test)
        # Initialize EM measurement file xml object
        meas_file_name = os.path.join(self._saving_directory, "EM_meas_report.xml")
        self.__em_meas_tab = XMLMeasurementFile(meas_file_name)
        self.__charger_is_data_cable = None
        self.__parser_api = self._device.get_uecmd("Aplog", True)

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # Call the UseCaseBase Setup function
        EmUsecaseBase.set_up(self)

        # Check if cable type is supported by your io card
        if self.__cable_type not in self._io_card.SUPPORTED_DEVICE_TYPE and self.__cable_type != "NONE":
            txt = "io card does not support cable type %s " % self.__cable_type
            self._logger.error(txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, txt)

        # do this job one time in case of B2B
        if self.__charger_is_data_cable is None:
            self.__charger_is_data_cable = self.em_core_module.is_host_connection_available_when_charger_plug(self.__cable_type)
            self._logger.info("test sequences will adjust to this")

        # init capacity
        em_info = self.update_battery_info()
        # discharge battery
        if self.em_core_module.is_batt_capacity_above_target(em_info, self.em_core_module.batt_max_capacity):

            self.em_core_module.monitor_discharging(self.em_core_module.batt_max_capacity,
                                     self.em_core_module.discharge_time,
                                     self.__em_meas_tab, self.__load_module)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        run test
        """
        EmUsecaseBase.run_test_body(self)
        # begin by discharge the battery in order to reach 0% of capacity
        self.update_battery_info()
        # inject a start tag with the host date as unique id
        start_tag = "START_WAITING_FOR_SHUTDOWN-" + time.strftime("-%Y-%m-%d_%Hh%M.%S")
        if not self.__parser_api.inject_tag(start_tag):
            error_msg = "The tag [%s] failed to be injected on aplogs " % (start_tag)
            self._logger.error(error_msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, error_msg)

        # choose the right way to discharge
        if self.__charger_is_data_cable:
            self.__discharge_to_off_with_data()
        else:
            self.__discharge_to_off_without_data()

        # retrieve the board connection will exist if MOS is not reached
        self.__retrieve_the_board_to_mos()
        # retrieve logs if no data cable
        if not self.__charger_is_data_cable:
            self.__retrieve_autolog_result()

        # inject a start tag with the host date as unique id
        stop_tag = "STOP_WAITING_FOR_SHUTDOWN-" + time.strftime("-%Y-%m-%d_%Hh%M.%S")
        if not self.__parser_api.inject_tag(stop_tag):
            error_msg = "The tag [%s] failed to be injected on aplogs " % (stop_tag)
            self._logger.error(error_msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, error_msg)

        # get shutdown reason
        shutdown_reason = self.__parser_api.find_txt_between_tag(start_tag, stop_tag,
                                                                 self.__parser_api.PUPDR_MISC_TAG["SHUTDOWN_REASON"],
                                                                 raise_error=False)
        if len(shutdown_reason) > 0:
            shutdown_reason = self.__parser_api.parse_shutdown_reason(shutdown_reason[0])
        else:
            shutdown_reason = "NOT FOUND IN DUT LOG"
        self._meas_list.add("SHUTDOWN_REASON", shutdown_reason, "none")

        # get last capacity seen by dut log
        capacity = self.__parser_api.find_txt_between_tag(start_tag,
                                                        self.__parser_api.PUPDR_MISC_TAG["KERNEL_BOOT"],
                                                        self.__parser_api.EM_TAG["BATTERY_INFO"],
                                                        raise_error=False)
        capacity = self.__parser_api.parse_battery_info(capacity)
        # Take the last capacity seen from the list
        if len(capacity) > 0:
            capacity = capacity[-1][0]
        else:
            capacity = "NOT FOUND IN YOUR LOG"

        self._meas_list.add("LAST_BATT_CAPACITY_SEEN", capacity, "none")

        # generate em verdict
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets, True)
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
        if self.is_board_and_acs_ok():
            self.__load_module.clean()
            # restore setting that does not persist on reboot

        return Global.SUCCESS, "No errors"

    def __discharge_to_off_with_data(self):
        """
        discharge code when there is a data cable plug
        """
        self._logger.info("Start to discharge battery until board is off with a data cable connected")
        local_timeout = time.time() + self.__discharge_timeout
        connection_shutdown_counter = 0
        timeout_capacity_increase = 900
        start_capacity_increase = time.time()
        last_capacity = self.batt_capacity

        self.__load_module.start_load()
        # we check the connection with the board to determine when the shutdown happen.
        while connection_shutdown_counter < 5:
            if connection_shutdown_counter == 0:
                try:
                    # get uevent info
                    msic_dict = self.update_battery_info()
                    self.__em_meas_tab.add_dict_measurement(msic_dict)
                    # get thermal
                    thermal_conf = self.em_api.get_thermal_sensor_info()
                    self.__em_meas_tab.add_dict_measurement(thermal_conf)
                    # restart some load if they stopped
                    self.__load_module.restart_load(consider_only_checkable_load=True)
                    # reset consecutive fail
                    measurement_fail = 0

                    # detect that capacity is not increasing
                    lost_capacity = last_capacity - self.batt_capacity
                    if lost_capacity >= 0:
                        start_capacity_increase = time.time()

                    last_capacity = self.batt_capacity
                    # stop usecase if capacity increase instead of decreasing
                    if (time.time() - start_capacity_increase) > timeout_capacity_increase:
                        tmp_txt = "board capacity keep increasing during %ss instead of decreasing; abort usecase" % timeout_capacity_increase
                        self._logger.error(tmp_txt)
                        raise DeviceException(DeviceException.TIMEOUT_REACHED, tmp_txt)

                except AcsBaseException as e:
                    # try to reconnect to the board if uecmd failed
                    self._logger.error("fail to get measurement during discharge: %s" % str(e))
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

                finally:
                    # Store various information
                    self.__em_meas_tab.add_measurement([self.get_time_tuple(),
                                                      (self._em_cst.COMMENTS, "RUNTEST: discharge until shutdown with a data cable plug")])
                    # switch to next meas
                    self.__em_meas_tab.switch_to_next_meas()

            # increase the counter if we are not in MOS
            if self._device.get_boot_mode() != "MOS":
                connection_shutdown_counter += 1
            else:
                connection_shutdown_counter = 0

            # stop usecase if board take too long to turn off
            if  connection_shutdown_counter < 5 and time.time() > local_timeout:
                self._em_meas_verdict.judge(ignore_blocked_tc=True)
                tmp_txt = "Board failed to turn off before %ss" % self.__discharge_timeout
                self._logger.error(tmp_txt)
                raise DeviceException(DeviceException.OPERATION_FAILED, tmp_txt)

    def __discharge_to_off_without_data(self):
        """
        discharge board until shutdown
        designed for a short discharge period(like discharging from 20% to 0).
        """
        # start  non persistent autolog with a short period of data polling
        # choose function to put in logger
        self.em_api.clean_autolog()
        self.em_api.add_fct_to_auto_logger(self.em_api.AUTOLOG_THERMAL, "sequenced")
        self.em_api.add_fct_to_auto_logger(self.em_api.AUTOLOG_UEVENT, "sequenced")
        self.em_api.start_auto_logger(60, 5, "sequenced")
        self.__load_module.start_load()
        # try to evaluate the time needed to discharge board to off------------------------------------------------------------------------#

        # disconnect board and plug the wanted cable
        self._device.disconnect_board()
        if self.__cable_type == "NONE":
            self._io_card.remove_cable("ALL")
        else:
            self._io_card.simulate_insertion(self.__cable_type)

        # wait for 0% of battery capacity or connection lost
        self._logger.info("Waiting %ss to let the board discharge from %s%% to off" % (self.__discharge_timeout, self.batt_capacity))
        time.sleep(self.__discharge_timeout)

    def __retrieve_the_board_to_mos(self):
        """
        retrieve the board
        """
        # plug a data cable to see if the board boot in COS or MOS
        self._io_card.usb_host_pc_connector(True)
        time.sleep(self.usb_sleep)
        # try to see what is the boot mode just after plugging cable
        mode = self._device.get_boot_mode()
        self._logger.info("boot mode just after plugging usb pc host connection : %s" % mode)
        if mode == "UNKNOWN":
            time_to_wait = 300
            # wait for a while to let the board boot
            self._logger.info("Waiting at most %ss to see the boot mode" % time_to_wait)
            start_time = time.time()
            while time.time() - start_time < time_to_wait:
                mode = self._device.get_boot_mode()
                if mode in ["COS", "MOS"]:
                    self._logger.info("Board seen booted in %s after %ss" % (mode, time.time() - start_time))
                    break

        if mode != "MOS":
            # plug a charge and charge it for a while
            self._io_card.wall_charger_connector(True)
            time_to_wait = 1800
            self._logger.info("Board booted mode is %s waiting %ss to charge it with a wall charger" % (mode, time_to_wait))
            time.sleep(time_to_wait)
            # plug a data cable to see if the board boot is still in COS
            self._io_card.wall_charger_connector(False)
            self._io_card.usb_host_pc_connector(True)
            time.sleep(self.usb_sleep)
            mode = self._device.get_boot_mode()
            end_time = 300
            self._logger.info("Waiting at most %ss to detect the boot mode" % end_time)
            start_time = time.time()
            while time.time() - start_time < end_time:
                mode = self._device.get_boot_mode()
                if mode != "UNKNOWN":
                    self._logger.info("Board seen booted in %s after %ss" % (mode, time.time() - start_time))
                    break

            # if the boot mode is not unknown, try to reboot the board
            if mode == "UNKNOWN":
                self.em_core_module.reboot_board("HARD")
            # if in MOS connect board
            elif mode != "MOS":
                self._device.reboot(skip_failure=True)
            mode = self._device.get_boot_mode()

        else:
            # else we are in MOS thus connecting ACS
            self._device.connect_board()

        # finally do action here only if board is well booted
        if not self.is_board_and_acs_ok():
            tmp_txt = "failed to retrieve the board in MAIN OS and connect ACS after a potential shutdown, last boot mode seen was %s" % mode
            self._logger.error(tmp_txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, tmp_txt)

    def __retrieve_autolog_result(self):
        """
        retrieve data from autolog. Used when there is no data cable plugged.
        put this as optional as it is only for debug purpose
        """
        # get info from autologger
        try:
            self.em_api.stop_auto_logger()
            msic_list = self.em_api.get_autolog_msic_registers()
            thermal_list = self.em_api.get_autolog_thermal_sensor_info()
            self.em_api.clean_autolog()
            self.em_core_module.fill_autolog_result(msic_list, thermal_list, self.__em_meas_tab,
                                                    "RUNTEST : Discharge until shutdown without a data cable")
        except DeviceException as e:
            txt = "FAILED to retrieve EM info due to following error :%s" % str(e)
            self._logger.error(txt)
            self._logger.info("these info are optional, the test will continue")
