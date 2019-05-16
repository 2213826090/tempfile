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
:summary: Energy Management Battery Monitor - Bench cycle in 2G
:author: vgombert
:since: 08/23/2011
"""
import time
import os

from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.EMUtilities import update_conf, XMLMeasurementFile, OcvComputingTool
from acs_test_scripts.Device.UECmd.UECmdTypes import VOICE_CALL_STATE

from ErrorHandling.TestEquipmentException import TestEquipmentException
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsBaseException import AcsBaseException


class LabEmBattBenchCycle2g(EmUsecaseBase):

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
        # Read BATT_MIN_CAPACITY from test case xml file
        self.em_core_module.init_fg_param()
        #-----------------------------------------------------------------------
        # Read Band from test case xml file (str)
        self.__cell_band = str(self._tc_parameters.get_param_value("CELL_BAND"))
        # Read CELL_SERVICE from test case xml file
        self.__cell_service = str(self._tc_parameters.get_param_value("CELL_SERVICE"))
        # Read TCH_ARFCN from test case xml file
        self.__tch_arfcn = int(self._tc_parameters.get_param_value("TCH_ARFCN"))
        # Read UPLINK_CHANNEL from test case xml file
        self.__uplink_channel = int(self._tc_parameters.get_param_value("UPLINK_CHANNEL"))
        # Read CELL_POWER from test case xml file
        self.__cell_power = int(self._tc_parameters.get_param_value("CELL_POWER"))
        # Read CALL_ORIGIN from test case xml file
        self.__call_origin = str(self._tc_parameters.get_param_value("CALL_ORIGIN")).upper()
        # Read DATA_CALL_MODE from test case xml file
        self.__data_call_mode = str(self._tc_parameters.get_param_value("DATA_CALL_MODE"))
        # Read registrationTimeout from Phone_Catalog.xml
        self._registration_timeout = \
            int(self._dut_config.get("registrationTimeout"))

        # Create cellular network simulator and retrieve 2G APIs
        self.__ns = self._em.get_cellular_network_simulator("NETWORK_SIMULATOR1")
        self.__ns_2g = self.__ns.get_cell_2g()
        self.__data_2g = self.__ns_2g.get_data()
        #-----------------------------------------------------------------------

        # Initialize EM  xml object
        # measurement file
        meas_file_name = os.path.join(self._saving_directory,
                                      "EM_meas_report.xml")
        self.__em_meas_tab = XMLMeasurementFile(meas_file_name)
        # enable Global Measurement file
        name = os.path.join(self._campaign_folder,
                            self._em_cst.GLOBAL_MEAS_FILE)
        self.__em_meas_tab.enable_global_meas(name, self._name)
        # enable OCV computing
        # get OCV target file
        target = self.em_core_module.ocv_target_file.parse_em_ocv_targets(
            self._device.get_phone_model(), self._tct)
        self.__ocv_tool = OcvComputingTool(target)

#-----------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # Call the UseCaseBase Setup function
        EmUsecaseBase.set_up(self)

        if self.__call_origin not in ["SIMULATOR", "PHONE"]:
            tmp_txt = "invalid CALL_ORIGIN parameter value : %s " % \
                      self.__call_origin
            self._logger.error(tmp_txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, tmp_txt)

        # set usb charging on
        self.em_api.set_usb_charging("on")

        # Configure CMU
        # Connect to cellular network simulator
        self.__ns.init()
        # Perform Full Preset
        self.__ns.perform_full_preset()
        # Set cell band using CELL_BAND parameter
        self.__ns_2g.set_band(self.__cell_band)
        # Set cell off
        self.__ns_2g.set_cell_off()
        # Set cell service using CELL_SERVICE parameter
        self.__ns_2g.set_cell_service(self.__cell_service)
        # Set Traffic Channel Arfcn using TCH_ARFCN parameter
        self.__ns_2g.set_tch_arfcn(self.__tch_arfcn)
        # Set cell power using CELL_POWER parameter
        self.__ns_2g.set_cell_power(self.__cell_power)
        # set data uplink channel
        self.__data_2g.set_data_channel(self.__uplink_channel)
        # configure cellular network burst slots
        slots_conf = self.em_core_module.configure_slot()

        # Set main timeslot to 3 and set slot configs
        self.__data_2g.set_custom_multislot_config(
            3, slots_conf["DL_STATE"],
            slots_conf["DL_VALUE"],
            slots_conf["UL_STATE"],
            slots_conf["UL_VALUE"])

        # get capability targets
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_BATT_BENCH_CYCLE_2G", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

        # init verdict value
        msic = self.update_battery_info()

        if self._em_targets["MSIC_REGISTER_CHARGE.BATTERY.CHARGE_NOW"] is not None:
            update_conf(self._em_targets["MSIC_REGISTER_CHARGE.BATTERY.CHARGE_NOW"],
                        "hi_lim", msic["BATTERY"]["CHARGE_FULL"][0], "=")
        # setting to be dne only if chamber is here and used
        if self.tc_module is not None:
            if self._em_targets["THERMAL_MSIC_REGISTER_CHARGE.BATTERY.TEMP"] is not None:
                update_conf(self._em_targets["THERMAL_MSIC_REGISTER_CHARGE.BATTERY.TEMP"],
                            ["lo_lim", "hi_lim"], self._tct, "*")
            if self._em_targets["THERMAL_MSIC_REGISTER_DISCHARGE.BATTERY.TEMP"] is not None:
                update_conf(self._em_targets["THERMAL_MSIC_REGISTER_DISCHARGE.BATTERY.TEMP"],
                            ["lo_lim", "hi_lim"], self._tct, "*")

        # Charge battery
        if (self.em_core_module.batt_start_capacity != "FULL" and str(self.em_core_module.batt_start_capacity).isdigit() and
           self.batt_capacity > int(self.em_core_module.batt_start_capacity)):
            self.em_core_module.monitor_discharging(self.em_core_module.batt_start_capacity, self.em_core_module.discharge_time,
                                     self.__em_meas_tab)
        else:
            self.em_core_module.monitor_charging(self.em_core_module.batt_start_capacity,
                                  self.em_core_module.charge_time,
                                  self.__em_meas_tab)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        EmUsecaseBase.run_test_body(self)

        # charge board
        self.__charge_phase_mos()

        # setting for discharge loop
        # set CMU cell phone ON
        self.__data_2g.set_data_cell_on()
        # register phone
        self.__data_2g.data_register_dut(None,
                                         self._registration_timeout)
        # Check registration status on DUT
        self.modem_api.check_cdk_registration_bfor_timeout(
            self._registration_timeout)
        # establish call
        if self.__call_origin == "SIMULATOR":
            self.__data_2g.data_call(self.__data_call_mode)
            self.__data_2g.check_data_call_connected(60)
        elif self.__call_origin == "PHONE":
            self.voicecall_api.dial("OOOO12121121")

        # discharge the board
        self.__discharge_phase()

        return(self._em_meas_verdict.get_global_result(),
               self._em_meas_verdict.generate_msg_verdict())

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        # call tear down after some operations
        EmUsecaseBase.tear_down(self)

        # clean the measurement file from last ocv computing
        self.__em_meas_tab.remove_meas(
            self.__ocv_tool.get_constant_list())
        # compute ocv and store it in measurement file
        result = self.__ocv_tool.compute_as_meas()
        map(self.__em_meas_tab.add_computed_meas, result)  # pylint: disable=W0141
        # retrieve measurement from test
        self.__em_meas_tab.generate_global_file()
        # as measurement is kept over B2B iteration,
        # just reset global file to its init state
        self.__em_meas_tab.reset_global_file()

        # set CMU cell phone OFF
        if self.__data_2g is not None:
            self.__data_2g.set_data_cell_off()
            # Disconnect from the equipment (Network simulator)
            self.__ns.release()

        if self.is_board_and_acs_ok():
            self.em_api.set_usb_charging("on")

        # clean the board state and retrieve logs
        self.em_core_module.clean_up()

        if self.is_board_and_acs_ok():
            self.phonesystem_api.set_screen_timeout(30)
            self.phonesystem_api.clean_daemon_files()

        return Global.SUCCESS, "No errors"

# private-----------------------------------------------------------------------------

    def __charge_phase_mos(self):
        """
        Charge the board with a dcp, screen off
        """
        # start charging through usb
        self.em_api.set_usb_charging("on")
        # init capacity
        msic = self.update_battery_info()
        battery_status = msic["BATTERY"]["STATUS"][0].upper()

        # set a fake value to capacity if status full is wanted
        max_capacity = 10000
        good_text = "FULL"
        if self.em_core_module.batt_max_capacity.isdigit():
            max_capacity = int(self.em_core_module.batt_max_capacity)
            good_text = str(max_capacity) + "%"

        measurement_fail = 0
        # Charge battery, if board is already full then leave
        if (self.em_core_module.batt_max_capacity == "FULL" and battery_status != self.em_core_module.batt_max_capacity) or\
                (self.em_core_module.batt_max_capacity != "FULL" and self.batt_capacity < max_capacity) or battery_status != "FULL":
            self._logger.info("Start to charge battery until %s before %s seconds" %
                              (good_text,
                               self.em_core_module.charging_time_limit))

            keep_looping = True
            msic_charge_time = abs(self.em_core_module.charge_time - 10)
            local_start_time = time.time()
            while keep_looping:
                # charge board
                try:
                    # start scheduled operation and get result
                    # first num is the time to wait before reaching dcp plug state
                    task_id = self.em_api.poll_multi_msic_registers(30, msic_charge_time, 10)
                    self.em_core_module.charge_battery(self.em_core_module.charge_time)
                    msic_list = self.em_api.get_multi_msic_registers(task_id)

                    # get the highest logs length
                    for msic_dict in msic_list:
                        try:
                            # get battery/charger info
                            if len(msic_dict) > 1:
                                msic_batt = msic_dict["BATTERY"]
                                self.batt_capacity = msic_batt["CAPACITY"][0]
                                self.batt_voltage = msic_batt["VOLTAGE"][0]
                                battery_status = msic_batt["STATUS"][0].upper()
                                # store result on xml
                                self.__em_meas_tab.add_dict_measurement(msic_dict)
                                # get ocv limit
                                ocv_lo_lim, ocv_hi_lim = self.__ocv_tool.get_ocv_limit(self.batt_capacity)
                                self.__em_meas_tab.add_measurement(
                                    [(self._em_cst.OCV_NOW_LOW_LIM, ocv_lo_lim, self.__ocv_tool.UNIT),
                                     (self._em_cst.OCV_NOW_HIGH_LIM, ocv_hi_lim, self.__ocv_tool.UNIT)])
                                # compute battery capacity error
                                battery_capacity_err = (float(msic_batt["CHARGE_NOW"][0]) /
                                                        float(msic_batt["CHARGE_FULL"][0])) * 100 - self.batt_capacity
                                # store verdict
                                self._meas_list.add("BATTERY_CAPACITY_CHARGE_ERR",
                                                    (battery_capacity_err, "none"))
                                self._meas_list.add_dict("MSIC_REGISTER_CHARGE", msic_dict)
                                if self.tc_module is not None:
                                    self._meas_list.add_dict("THERMAL_MSIC_REGISTER_CHARGE", msic_dict)

                        finally:
                            # Store various information
                            self.__em_meas_tab.add_measurement(
                                [self.get_time_tuple(),
                                 (self._em_cst.COMMENTS, "RUNTEST: Charging phase"),
                                 (self._em_cst.REBOOT, self.phone_as_reboot)])

                            if self.tc_module is not None:
                                self.__em_meas_tab.add_measurement(
                                    [self.tc_module.feed_meas_report()])

                            # reinitialize reboot variable
                            self.phone_as_reboot = False

                            # switch meas to next meas
                            self.__em_meas_tab.switch_to_next_meas()
                            # exit if condition are respected else apply verdict
                            if (self.em_core_module.batt_max_capacity == "FULL" and battery_status == self.em_core_module.batt_max_capacity) or\
                                    (self.em_core_module.batt_max_capacity != "FULL" and self.batt_capacity >= max_capacity) or battery_status == "FULL":
                                keep_looping = False
                            else:
                                # apply verdict
                                self._em_meas_verdict.compare_list(self._meas_list, self._em_targets, True)
                                self._em_meas_verdict.judge(ignore_blocked_tc=True)

                    # reset error
                    measurement_fail = 0
                except AcsBaseException as e:
                    # try to reconnect to the board if uecmd failed
                    self._logger.error("fail to get measurement: " + str(e))
                    measurement_fail += 1
                    # stop the usecase if measurement fail several times.
                    if measurement_fail >= self._consecutive_meas_error:
                        tmp_txt = "Measurement failed after %s times, abort usecase" % \
                            self._consecutive_meas_error
                        self._logger.error(tmp_txt)
                        raise DeviceException(DeviceException.OPERATION_FAILED, tmp_txt)

                    # check the board connection
                    self.em_core_module.check_board_connection(tries=1, use_exception=False)

                # stop usecase if board take too long to reach battery max capacity
                if (time.time() - local_start_time) > self.em_core_module.charging_time_limit:
                    tmp_txt = "Phone failed to reach %s before %s seconds" % \
                        (good_text, self.em_core_module.charging_time_limit)
                    self._logger.error(tmp_txt)
                    raise DeviceException(DeviceException.TIMEOUT_REACHED, tmp_txt)

            # stop all daemonized ACS process & clean measurement done
            self._meas_list.clean()
            self.phonesystem_api.clean_daemon_files()

#------------------------------------------------------------------------------

    def __discharge_phase(self):
        """
        discharge the board with a 2G call
        """
        measurement_fail = 0
        # re update target with last charge full value
        msic = self.em_api.get_msic_registers()
        if self._em_targets["MSIC_REGISTER_DISCHARGE.BATTERY.CHARGE_NOW"] is not None:
            update_conf(self._em_targets["MSIC_REGISTER_DISCHARGE.BATTERY.CHARGE_NOW"],
                        "hi_lim", msic["BATTERY"]["CHARGE_FULL"][0], "=")

        # discharge loop -----------------------------------------------------------------------------------
        msic_reg = self.update_battery_info()

        if self.batt_capacity > self.em_core_module.batt_min_capacity:
            # stop charging through usb
            self.em_api.set_usb_charging("off")
            # launch uecmd to help the discharge
            self.phonesystem_api.set_screen_timeout(3600)
            # deactivate set auto brightness
            self.phonesystem_api.set_brightness_mode("manual")
            # set display brightness to max value
            self.phonesystem_api.set_display_brightness(100)
            self.phonesystem_api.set_phone_lock(0)
            # reinitialize consecutive error
            self.phone_as_reboot = False

            good_text = "DEAD"
            if self.em_core_module.batt_min_capacity > -1:
                good_text = str(self.em_core_module.batt_min_capacity) + "%"

            # reset consecutive error
            measurement_fail = 0
            self.phone_as_reboot = False
            self._logger.info("Start to discharge battery until %s" % good_text)
            local_start_time = time.time()
            while self.batt_capacity > self.em_core_module.batt_min_capacity:
                try:
                    # try to read measurement
                    # get msic registers value after booting
                    msic_reg = self.update_battery_info()
                    self._meas_list.add_dict("MSIC_REGISTER_DISCHARGE", msic_reg)
                    msic_batt = msic_reg["BATTERY"]
                    self.__em_meas_tab.add_dict_measurement(msic_reg)
                    # get ocv and soc information
                    ocv_pass = self.__ocv_tool.add(self.batt_capacity,
                                                   self.batt_voltage)
                    # get ocv limit
                    ocv_lo_lim, ocv_hi_lim = self.__ocv_tool.get_ocv_limit(self.batt_capacity)
                    self.__em_meas_tab.add_measurement(
                        [(self._em_cst.OCV_NOW_LOW_LIM, ocv_lo_lim, self.__ocv_tool.UNIT),
                         (self._em_cst.OCV_NOW_HIGH_LIM, ocv_hi_lim, self.__ocv_tool.UNIT),
                         (self._em_cst.OCV_NOW_LIM_VERDICT, ocv_pass)])

                    # compute battery capacity error
                    battery_capacity_err = (float(msic_batt["CHARGE_NOW"][0]) /
                                            float(msic_batt["CHARGE_FULL"][0])) * 100 - self.batt_capacity
                    self._meas_list.add("BATTERY_CAPACITY_DISCHARGE_ERR",
                                        (battery_capacity_err, "none"))

                    # check thermal capabilities only if thermal chamber is used
                    if self.tc_module is not None:
                        # Store various information
                        self._meas_list.add_dict("THERMAL_MSIC_REGISTER_DISCHARGE", msic_reg)

                    # get the call state------------------------------------------------------------------------
                    call_state = self.__cell_service
                    if self.__call_origin == "SIMULATOR":
                        # Check data call state
                        try:
                            self.__data_2g.check_data_call_connected(10)
                            call_state += "_DATA_CALL_CONNECTED"
                        except TestEquipmentException as error:
                            self._logger.error(error)
                            call_state += "_DATA_CALL_DISCONNECTED"
                            self.__data_2g.data_call(self.__data_call_mode)
                    elif self.__call_origin == "PHONE":
                        # Check cs call state
                        call_state_tmp = self.voicecall_api.get_state()
                        call_state += "_CALL_" + str(call_state_tmp)
                        # pylint: disable=E1101
                        if call_state_tmp == VOICE_CALL_STATE.NOCALL:
                            self.voicecall_api.dial("OOOO12121121")

                    # Store various information
                    self.__em_meas_tab.add_measurement(
                        [("REGISTRATION", self.modem_api.get_network_registration_status()),
                         ("VOICE_CALL_" + self.__call_origin, call_state)])
                    #------------------------------------------------------------------------
                    # stop charging through usb
                    self.em_api.set_usb_charging("off")
                    # reset consecutive fail
                    measurement_fail = 0

                except AcsBaseException as e:
                    # try to reconnect to the board if uecmd failed
                    self._logger.error("fail to get measurement: %s" % str(e))
                    measurement_fail += 1

                    # stop the usecase if measurement fail several times.
                    if measurement_fail >= self._consecutive_meas_error:
                        if self.batt_voltage > self.vbatt_mos_shutdown or \
                                self.batt_voltage == -1:
                            tmp_txt = "Measurement failed after %s times, stop usecase" % \
                                self._consecutive_meas_error
                            self._logger.error(tmp_txt)
                            raise DeviceException(DeviceException.OPERATION_FAILED, tmp_txt)
                        else:
                            self._logger.info("battery must be empty, stop discharging")
                            break
                    elif self._device.get_boot_mode() == "COS" and self.em_core_module.batt_min_capacity < 0:
                        self._logger.info("board booted in COS, stop discharging")
                        break
                    # check the board connection
                    self.em_core_module.check_board_connection(1, use_exception=False)
                finally:
                    # Store various information
                    if self.tc_module is not None:
                        self.__em_meas_tab.add_measurement(
                            [self.tc_module.feed_meas_report()])

                    self.__em_meas_tab.add_measurement(
                        [self.get_time_tuple(),
                         (self._em_cst.COMMENTS, "RUNTEST:discharge phase test with 2G call "),
                         (self._em_cst.REBOOT, self.phone_as_reboot)])

                    # generate em verdict
                    self._em_meas_verdict.compare_list(self._meas_list, self._em_targets, True)
                    self._em_meas_verdict.judge(ignore_blocked_tc=True)

                    # switch to next meas
                    self.__em_meas_tab.switch_to_next_meas()

                # stop usecase if board take too long to reach battery min capacity
                if (time.time() - local_start_time) > self.em_core_module.charging_time_limit:
                    tmp_txt = "Phone failed to discharge to %s before %s seconds" % \
                        (good_text, self.em_core_module.charging_time_limit)
                    self._logger.error(tmp_txt)
                    raise DeviceException(DeviceException.TIMEOUT_REACHED, tmp_txt)

                # restart uecmd if board have shutdown
                if self.has_board_reboot():
                    # stop charging through usb
                    self.em_api.set_usb_charging("off")
                    # establish a call
                    if self.__call_origin == "SIMULATOR":
                        try:
                            self.__data_2g.data_call(self.__data_call_mode)
                            self.__data_2g.check_data_call_connected(10)
                        except TestEquipmentException as error:
                            self._logger.error(error)
                    elif self.__call_origin == "PHONE":
                        self.voicecall_api.dial("OOOO12121121")

                    self.phone_as_reboot = False

            # when leaving this function stop multimedia and restart charging
            if self.is_board_and_acs_ok():
                self.em_api.set_usb_charging("on")
