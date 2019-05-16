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
:summary: EM - Test for fuel gauging
:author: cheurtex
:since: 30/06/2015
"""
import os
import time
import threading
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.AcsBaseException import AcsBaseException
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.UseCase.EnergyManagement.UcModule.LoadModule import LoadModule
from acs_test_scripts.Utilities.EMUtilities import XMLMeasurementFile


class LabEmFuelGauging(EmUsecaseBase):
    """
    Lab Energy Management class.
    """
    __CYCLE_BEHAVIOR_VAL = ["CHARGE_DISCHARGE", "DISCHARGE_CHARGE", "CHARGE_ONLY", "DISCHARGE_ONLY"]
    DEDICATED_BENCH = "BATTERY_BENCH"

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LAB_EM_BASE Init function
        EmUsecaseBase.__init__(self, tc_name, global_config)

        # init fuel gauging parameters
        self.em_core_module.init_fg_param()
        self.__raw_behavior = str(self._tc_parameters.get_param_value("CYCLE_BEHAVIOR")).upper()
        load = self._tc_parameters.get_param_value("LOAD")
        self.__charger_type = self._tc_parameters.get_param_value("CHARGER_TYPE")

        # Load Module Initialization
        self.__load_module = LoadModule()
        self.__load_module.add_load(load)

        # Initialize EM  xml object
        # measurement file
        meas_file_name = os.path.join(self._saving_directory, "EM_meas_report.xml")
        self.__em_meas_tab = XMLMeasurementFile(meas_file_name)
        # track the case where DUT is lost to gather em info later
        self.__fail_to_get_em_info = False

    # ------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # Call the UseCaseBase Setup function
        EmUsecaseBase.set_up(self)

        if self.__raw_behavior not in self.__raw_behavior :
            tmp_txt = "Unknown value for CYCLE_BEHAVIOR, expected value must be in % and we got %s" % (self.__CYCLE_BEHAVIOR_VAL, self.__raw_behavior)
            self._logger.error(tmp_txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, tmp_txt)

        return Global.SUCCESS, "No errors"

    def run_test_body(self):
        """
        Execute the test
        """
        # Call LAB_EM_BASE Run function
        EmUsecaseBase.run_test_body(self)
        self.__fail_to_get_em_info = False
        # if we reach here it means that we have the correct value in behavior
        sorted_behavior = self.__raw_behavior.split("_")
        for action in sorted_behavior:
            action = action.strip()
            # Charge/discharge battery depending of fuel gauging option used
            if  action == "CHARGE" :
                # after a discharge board may be totally off , try to charge it a little bit
                if not self.is_board_and_acs_ok():
                    if self.__boot_in_mos_board(self.em_core_module.charge_time):
                        self.__bring_board_above_vbatt_MOS_boot()
                        self.__retrieve_lost_em_info()

                # Update Battery Information
                em_info = self.update_battery_info()
                if self.em_core_module.is_batt_capacity_below_target(em_info, self.em_core_module.batt_max_capacity):
                    self.em_core_module.monitor_charging(self.em_core_module.batt_max_capacity, self.em_core_module.charge_time, self.__em_meas_tab)
                else:
                    self._logger.info("Cant charge the board as battery capacity is above the max capacity")

            elif action == "DISCHARGE":
                # Update Battery Information
                em_info = self.update_battery_info()
                if self.em_core_module.is_batt_capacity_above_target(em_info, self.em_core_module.batt_min_capacity):
                    self.__monitor_discharging_with_load(self.em_core_module.batt_min_capacity, self.em_core_module.discharge_time, self.__em_meas_tab, self.__load_module)

                else:
                    self._logger.info("Cant discharge the board as battery capacity is below the min capacity")

        return Global.SUCCESS, "No errors"

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
            time.sleep(self.usb_sleep)
            if self._device.get_boot_mode() == "MOS":
                self._device.connect_board()

        # check if phone battery is too low and
        # charge it for a while (even if the board is down)

        # this try except give a final chance to get aplog in case of a crash
        try :
            if self.is_board_and_acs_ok():
                self.update_battery_info()
            # enter here if we are not in MOS or in MOS with low voltage or capacity
            if (self._device.get_boot_mode() != "MOS") or (-1 < self.batt_voltage <= self.vbatt_mos_shutdown) or(-1 < self.batt_capacity <= 10):
                self.__boot_in_mos_board()
        except AcsBaseException as e:
            self._logger.error("error happened during board charge process in clean_up")
            raise e

        finally:
            if self.is_board_and_acs_ok():
                self.get_application_logs()

    def tear_down(self):
        """
        End and dispose the test
        """
        # call tear down after some operations
        EmUsecaseBase.tear_down(self)
        if self.is_board_and_acs_ok():
            self.__load_module.stop_load()
            self.__retrieve_lost_em_info()

        # clean the board state and retrieve logs
        self.clean_up()
        if self.is_board_and_acs_ok():
            self.__retrieve_lost_em_info()

        return Global.SUCCESS, "No errors"

    # ------------------------------------------------------------------------------

    def __retrieve_lost_em_info(self):
        """
        retrieve lost em info
        """
        if self.__fail_to_get_em_info == True:
            self._logger.info("try to retrieve lost EM information")
            try :
                em_info = self.em_api.get_autolog_msic_registers()
            except AcsBaseException as e:
                self._logger.error(e)
                em_info = None

            try:
                thermal_list = self.em_api.get_autolog_thermal_sensor_info()
            except AcsBaseException as e:
                self._logger.error(e)
                thermal_list = None

            if thermal_list is not None or em_info is not None:
                self.em_core_module.fill_autolog_result(em_info, thermal_list, self.__em_meas_tab, "log from DUT connection lost")
                self.__fail_to_get_em_info = False

    def __monitor_discharging_with_load(self, input_min_capacity, discharge_time, meas_tab=None, load_module=None):
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
        self.update_battery_info()
        # set a fake value to capacity if status dead is wanted
        # cosmetic text computing
        good_text = "DEAD"
        # convert min_capacity into int
        min_capacity = -1
        if str(input_min_capacity).isdigit():
            min_capacity = int(input_min_capacity)
        if min_capacity > -1:
            good_text = str(min_capacity) + "%"

        # reset consecutive error
        if self.batt_capacity > min_capacity:
            self._logger.info("Start to discharge battery until %s before %s seconds" % (good_text, self.em_core_module.discharging_time_limit))
            # launch uecmd to help the discharge
            self._logger.info("load detected")
            self.__load_module.start_load()
            self.__discharge_when_no_data(min_capacity, good_text, discharge_time, meas_tab, load_module)

#------------------------------------------------------------------------------------------------

    def __boot_in_mos_board(self, charge_time=None):
        """
        try to bring the board to boot in mos by doing several actions.

        :type charge_time: int
        :param charge_time: charging time in second

        :rtype : bool
        :return : True if boot mode is MOS
        """
        self._logger.info("Try to boot board in MOS")
        if charge_time is None:
            charge_time = self.em_core_module._charge_empty_batt_time

        # get boot mode to see if we can check battery info
        boot_mode = self._device.get_boot_mode()
        if boot_mode != "MOS":
            # if we don't know the boot mode then plug usb host to see the mode
            self._device.disconnect_board()
            if boot_mode == "UNKNOWN":
                self._io_card.usb_host_pc_connector(True)
                time.sleep(self.usb_sleep)
                boot_mode = self._device.get_boot_mode()

            self._logger.debug("boot mode before charging : %s" % boot_mode)
            # charge the board in the fastest way for a defined duration
            self._logger.info("try to charge board during %ss" % charge_time)
            # connect USB DCP
            self._io_card.wall_charger_connector(True)
            # wait time to retrieve board from 0%
            time.sleep(charge_time)
            # Following step consider that we do not have data while charging , this way to do cover more DUT types.
            if boot_mode in ["COS", "UNKNOWN"]:
                cos_transition = 120
                # once charged, press power button to boot
                self._io_card.press_power_button(self.pwr_btn_boot)
                # wait some time with the wall charger inserted to let the board charge a little bit if the boot did not work
                self._logger.debug("wait %ss for boot transition happening after pushing power button" % cos_transition)
                time.sleep(cos_transition)

            boot_timeout = self._device.get_boot_timeout()
            # check in which mode we are
            self._io_card.usb_host_pc_connector(True)
            stop_time = time.time() + boot_timeout
            self._logger.debug("try to see boot mode before %ss" % boot_timeout)
            while time.time() < stop_time:
                boot_mode = self._device.get_boot_mode()
                if boot_mode != "UNKNOWN":
                    break

            if  boot_mode != "MOS":
                # force to reboot in MOS in case of POS or ROS
                self._device.reboot()

            boot_mode = self._device.get_boot_mode()
            self._logger.debug("boot mode after charging %ss and trying to boot is : %s" % (charge_time, boot_mode))

        return boot_mode == "MOS"

    def __bring_board_above_vbatt_MOS_boot(self):
        """
        After a boot, we need to ensure that VBATT is enough high to avoid shutdown due to cable removal
        this is typically the case when you are able to boot in MOS with 0% of capacity
        """
        self._logger.info("Try to bring board above MOS")
        boot_mode = self._device.get_boot_mode()
        if boot_mode == "MOS":
            self._device.connect_board()

        if self._device.is_available():
            capacity = self.update_battery_info()["BATTERY"]["CAPACITY"][0]
            # create the msg but dont mean to use it
            msg = "Capacity is below 1 percent, trying to charge the board a little bit before continuing the test"
        else:
            capacity = 0
            msg = "fail to read capacity after boot, we will charge the board and try again"

        if capacity < 1:
            self._logger.info(msg)
            # try to charge the board above MOS boot threshold
            tries = 2
            timer = 450

            while capacity < 1 and tries > 0:
                # connect WALL Charger
                self._device.disconnect_board()
                self._io_card.wall_charger_connector(True)
                # wait x min
                self._logger.info("Charge phone during %ss" % timer)
                time.sleep(timer)
                # connect pc host
                self._io_card.usb_host_pc_connector(True)
                # wait x seconds
                time.sleep(self.usb_sleep)
                # connect board
                boot_mode = self._device.get_boot_mode()
                if boot_mode != "UNKNOWN":
                    self._device.connect_board()
                    if self._device.is_available():
                        capacity = self.update_battery_info()["BATTERY"]["CAPACITY"][0]
                tries -= 1

            # try to boot in right mode if not MOS
            boot_mode = self._device.get_boot_mode()
            if boot_mode == "MOS":
                if not self._device.is_available():
                    self._device.connect_board()
            elif boot_mode != "UNKNOWN":
                self._device.reboot()

            if not self.is_board_and_acs_ok():
                boot_mode = self._device.get_boot_mode()
                txt = "Fail to make board boot in MOS after having charging it, current boot mode is %s" % boot_mode
                self._logger.error(txt)
                raise DeviceException(DeviceException.OPERATION_FAILED, txt)

#---------------------------------------------------------------------------------
    def __discharge_when_no_data(self, min_capacity, good_text, discharge_time, meas_tab, load_module):
        """
        private function served to monitor discharging when there is no data during charge.
        attempt to use a smart discharging way to adjust the battery capacity

        :type load_module: LoadModule
        :param load_module: object that handle the load activation
        """
        # init var
        self.__fail_to_get_em_info = False
        measurement_fail = 0
        connection_shutdown_counter = 0
        adjusted_discharging_time = discharge_time
        ratio_d = 0
        last_capacity = self.batt_capacity
        phone_as_reboot = False

        # start autolog here
        self.em_api.clean_autolog()
        # need to set the polling
        self.em_api.set_persistent_autolog(True)
        self.em_api.add_fct_to_auto_logger(self.em_api.AUTOLOG_THERMAL, "sequenced")
        self.em_api.add_fct_to_auto_logger(self.em_api.AUTOLOG_UEVENT, "sequenced")
        self.em_api.start_auto_logger(0, 5, "sequenced")

        # use thread to avoid blocking action when computing measurement
        lock = threading.Lock()
        thread_list = []
        timeout_capacity_increase = 900 + discharge_time
        start_time_capacity_increase = time.time()
        end_time = time.time() + self.em_core_module.discharging_time_limit
        # count the number of discharge in case of small discharge time to take into account the whole waiting time
        # when doing discharge time adjustment
        discharge_iteration = 1

        while self.batt_capacity > min_capacity and measurement_fail < 4:
            self.__fail_to_get_em_info = True

            # discharge only if we have data before
            if connection_shutdown_counter == 0:
                self.em_core_module.discharge_battery(adjusted_discharging_time)
                if self._device.get_boot_mode() != "MOS":
                    connection_shutdown_counter += 1
                    # consider stopping the test if shutdown happen when capacity was low
                    if connection_shutdown_counter < 0 and last_capacity < 10 :
                        self._logger.info("connection lost happened when capacity was <10, stopping the discharge")
                        break
            else :
                # try to boot the board in MOS
                boot_mode = self._device.get_boot_mode()
                if boot_mode == "UNKNOWN":
                    self._io_card.press_power_button(self.pwr_btn_boot)
                    start_time = time.time()
                    self._logger.info("Waiting atmost 60s to see boot transition")
                    while time.time() - start_time < 60:
                        boot_mode = self._device.get_boot_mode()
                        if boot_mode != "UNKNOWN":
                            break

                # Push power button
                elif boot_mode != "MOS":
                    self._device.reboot(skip_failure=True)

                if self._device.get_boot_mode() != "MOS":
                    connection_shutdown_counter += 1
                    if connection_shutdown_counter >= 2:
                        self._logger.info("battery must be empty or refuse to boot in MOS, stop discharging")
                        break
                else:
                    self._device.connect_board()
                    connection_shutdown_counter = 0
                    phone_as_reboot = True

            if connection_shutdown_counter == 0:
                try:
                    # parse autolog response and reset them
                    self.update_battery_info()

                    if self._device.get_boot_mode() == "MOS":
                        msic_list = self.em_api.get_autolog_msic_registers()
                        thermal_list = self.em_api.get_autolog_thermal_sensor_info()
                        self.em_api.reset_running_log()

                        # move to thread next computing
                        t = threading.Thread(target=self.__threaded_feed_meas, args=(lock, msic_list, thermal_list, meas_tab,
                                                                                     "SETUP:Discharge with load when no data connection"))
                        thread_list.append(t)
                        t.start()
                    self.__fail_to_get_em_info = False
                    # compute the smart charging
                    lost_capacity = last_capacity - self.batt_capacity
                    self._logger.info("%s capacity decreased  during %ss" % (lost_capacity, adjusted_discharging_time))
                    # Detect if battery is charging or not decreasing if it is the case stop usecase
                    if lost_capacity > 0:
                        next_theorical_capacity = self.batt_capacity - lost_capacity
                        # means that the timer is too long and we may be below the wanted capacity
                        # we need to reduce the charging time
                        # compute a ration capacity/second
                        ratio_d = float(adjusted_discharging_time * discharge_iteration) / float(lost_capacity)
                        adjusted_discharging_time = int(max((self.batt_capacity - min_capacity), 1) * ratio_d)
                        if next_theorical_capacity < min_capacity:
                            # limit the discharge time to 60s at min
                            adjusted_discharging_time = min(adjusted_discharging_time, 60)
                        # we are above the capacity thus we can wait a bigger time
                        else:
                            # limit the discharge time to 3600s at max
                            adjusted_discharging_time = min(adjusted_discharging_time, 3600)
                        start_time_capacity_increase = time.time()
                        self._logger.info("next adjusted discharging time will be %ss" % adjusted_discharging_time)
                        # update last capacity only if we are not charging
                        last_capacity = self.batt_capacity
                        discharge_iteration = 1
                    else:
                        discharge_iteration + 1

                    if load_module is not None and not ((time.time() - start_time_capacity_increase) > timeout_capacity_increase):
                        load_module.restart_load(consider_only_checkable_load=True)

                    # stop usecase if board take too long to reach battery capacity
                    if (time.time() - start_time_capacity_increase) > timeout_capacity_increase:
                        tmp_txt = "board capacity keep increasing during more than %ss instead of decreasing; abort usecase" % timeout_capacity_increase
                        self._logger.error(tmp_txt)
                        raise DeviceException(DeviceException.TIMEOUT_REACHED, tmp_txt)

                    # reset error
                    measurement_fail = 0
                except Exception as e:
                    # exit if we meet not decreasing capacity case
                    if e.get_generic_error_message() == DeviceException.TIMEOUT_REACHED:
                        raise e
                    # in case of crash we consider that capacity increase timeout should be reset
                    start_time_capacity_increase = time.time()
                    # try to reconnect to the board if uecmd failed
                    self._logger.error("fail to get measurement: " + str(e))
                    measurement_fail += 1

                # stop usecase if board take too long to reach battery max capacity
                if time.time() > end_time:
                    tmp_txt = "Phone failed to reach %s before %ss" % (good_text, self.em_core_module.discharging_time_limit)
                    self._logger.error(tmp_txt)
                    raise DeviceException(DeviceException.TIMEOUT_REACHED, tmp_txt)

                # Restore load in case of reboot
                if phone_as_reboot and self.is_board_and_acs_ok():
                    # stop charging through usb
                    load_module.restart_load()
                    phone_as_reboot = False

        #------------------------------WHILE END----------------------------------------#
        try:
            # if device is off after discharging, exist with a warning
            if self.is_board_and_acs_ok():
                self.phonesystem_api.clean_autolog()
                load_module.stop_load()
            else:
                tmp_txt = "The connection was lost during monitor discharging"
                self._logger.warning(tmp_txt)
        except Exception as e :
            tmp_txt = "non blocking error happen when trying to clean board after discharge: %s" % str(e)
            self._logger.error(tmp_txt)
        finally:
            # parse the thread list and wait then delete all started threads
            for thread_ele in thread_list:
                thread_ele.join()
                del thread_ele
            del thread_list

    def __threaded_feed_meas(self, lock, msic_list, thermal_list, meas_tab, general_msg):
        """
        use a thread to fill the measurement file to avoid discharging/charging
        the board during measurement computing
        """
        lock.acquire()
        try:
            self.em_core_module.fill_autolog_result(msic_list, thermal_list, meas_tab, general_msg)
        finally:
            lock.release()
