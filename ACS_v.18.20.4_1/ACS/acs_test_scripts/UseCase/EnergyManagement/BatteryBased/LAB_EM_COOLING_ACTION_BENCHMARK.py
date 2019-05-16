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

:organization: INTEL MCG PSI
:summary: This usecase measure the time it takes to trigger a thermal cooling action during a benchmark execution
:author: vgomberx
:since: 20/04/2015
"""

import time, os, calendar
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.Utilities.EMUtilities import XMLMeasurementFile
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from acs_test_scripts.UseCase.EnergyManagement.UcModule.LoadModule import LoadModule
from acs_test_scripts.UseCase.EnergyManagement.UcModule.BenchmarkModule import BenchmarkModule
from Core.Report.Live.LiveReporting import LiveReporting
from UtilitiesFWK.Utilities import Global, str_to_bool


class LabEmCoolingActionBenchmark(EmUsecaseBase):

    """
    Montinor cooling action during benchmark
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
        self.__zone_to_inspect = str(self._tc_parameters.get_param_value("THERMAL_ZONE_TO_MONITOR", "ANY")).upper().strip()

        setup_load = self._tc_parameters.get_param_value("SETUP_TECHNO", "")
        self.__cooldown = self._tc_parameters.get_param_value("SETUP_TIME_COOLDOWN_BOARD", 1800, default_cast_type=int)
        self.__start_temp = self._tc_parameters.get_param_value("SETUP_START_TEMPERATURE", 30, default_cast_type=int)
        use_camera_to_cooldown = self._tc_parameters.get_param_value("SETUP_COOLDOWN_WITH_CAMERA", False , default_cast_type=str_to_bool)
        self.__temp_camera = None
        if use_camera_to_cooldown:
            self.__temp_camera = self._em.get_thermal_camera("THERMAL_CAMERA")

        self.__benchmark_module = BenchmarkModule()
        self.__load_module = LoadModule()
        self.__load_module.add_load(setup_load)

        # measurement file
        meas_file_name = os.path.join(self._saving_directory, "EM_meas_report.xml")
        self.__em_meas_tab = XMLMeasurementFile(meas_file_name)

        meas_file_name = os.path.join(self._saving_directory, "temperature_report.xml")
        self.__temp_meas_tab = XMLMeasurementFile(meas_file_name)

        meas_file_name = os.path.join(self._saving_directory, "benchmark_score_report.xml")
        self.__benchamrk_tab = XMLMeasurementFile(meas_file_name)

        self.__ref_temp = None
        self.__pic_folder = self._saving_directory
        self.__host_test_start_time = None
        self.__bk_delay = 30
        self.__parser_api = self._device.get_uecmd("Aplog", True)
        # Call ConfigsParser to parse Energy_Management
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_COOLING_ACTION_BENCHMARK", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())
        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets, consider_all_target=True)

#------------------------------------------------------------------------------

    def __push_score(self , score_list):
        """
        take in entry the result for get_score() and format the benchmark score
        for TCR upload in order to upload all the score together

        :type score_list: list
        :param score_list: a list of dictionary containing score from benchmarks

        :rtype: dict of dict
        :return:  return a dict of dictionary
        """
        result = {}
        i = 0
        # get the highest number of digit in score list, for example 2 for 10
        max_iter_count = len(str(len(score_list)))
        max_logged_iteration = 1

        # search for the highest logged iteration
        for dico in score_list:
            digit_length = len(str(dico.get(BenchmarkModule.DITER, 1)))
            # we consider that the value is always a digit when it is returned
            if digit_length > max_logged_iteration:
                max_logged_iteration = digit_length

        for dico in score_list:
            i += 1
            # push dico on sysfs
            self.__benchamrk_tab.add_dict_measurement(dico)
            self.__benchamrk_tab.add_measurement([self.get_time_tuple(),
                                (self._em_cst.COMMENTS, "Getting benchmark scores")])
            self.__benchamrk_tab.switch_to_next_meas()

            # push dico on TCR
            if len (dico) > 1:
                score_dico = {}
                score_iter = dico.get(BenchmarkModule.DITER)
                if score_iter is None:
                    i_str = str(i).zfill(max_iter_count)
                    iter_tag = "BENCHMARK_MANUAL_COUNT_ITERATION_%s" % i_str
                else:
                    score_iter = str(score_iter).zfill(max_logged_iteration)
                    iter_tag = "BENCHMARK_ITERATION_%s" % score_iter
                for key, value in dico.iteritems():
                    # store everything under iteration tag except iteration
                    if key != BenchmarkModule.DITER:
                        score_dico[key] = value
                result[iter_tag] = score_dico

        if len(result) > 0:
            LiveReporting.instance().update_running_tc_info(test_info=result)
        return result

    def set_up(self):
        """
        Initialize the test
        """
        EmUsecaseBase.set_up(self)
        self.__benchmark_module.setup()
        if self.__temp_camera is not None:
            # init camera connection
            self.__temp_camera.init()
            self.__temp_camera.delete_all_pictures()

        # check that logs you want to parse exist:
        stop_tag = "TEST_IF_COOLING_INTENT_EXIST-" + time.strftime("-%Y-%m-%d_%Hh%M.%S")
        self.__parser_api.inject_tag(stop_tag)
        cooling_intents = self.__parser_api.find_txt_between_tag(self.__parser_api.PUPDR_MISC_TAG.get("MAIN_BOOT"), stop_tag,
                                                                 self.__parser_api.THERMAL_TAG.get("COOLING_INTENT"),
                                                                 raise_error=False)
        if len(cooling_intents) <= 0:
            # it means that cooling intent does not exist,  we cant do the test
            error_msg = "No thermal throttling/de-throttling message has been seen since last boot, check if thermal service is working on your DUT or an update of the thermal parser may be necessary"
            self._logger.error(error_msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, error_msg)

        ####################################### CHARGE BOARD ###########################################
        # Update Battery Information
        em_info = self.update_battery_info()
        if self.em_core_module.is_batt_capacity_below_target(em_info, self.em_core_module.batt_min_capacity):
            # charge part
            self.em_core_module.monitor_charging(self.em_core_module.batt_min_capacity,
                                         self.em_core_module.charge_time,
                                         self.__em_meas_tab)

        ####################################### COOL DOWN BOARD ###########################################
        self.phonesystem_api.set_screen_timeout(3600)
        self._device.switch_off()
        # remove any cable after this
        self._io_card.remove_cable("ALL")

        if self.__temp_camera is not None:
            # do the thermal camera setting during the off phase to let the board cool down
            self._setup_camera()
            # wait a given time to let the board cool down
            self._monitor_board_temp(self.__cooldown, self.__start_temp)
        else:
            # wait a given time to let the board cool down
            wait_time = self.__cooldown + 15
            self._logger.info("waiting %ss+15s to let the board cool down while it should be OFF" % str(self.__cooldown))
            time.sleep(wait_time)

        # turn on board and launch every load
        self._device.switch_on()
        # this measurement is done only for debugging purpose
        self.em_api.get_thermal_sensor_info()
        # start all environment load
        self.__load_module.start_load()

        if self.__temp_camera is not None:
            # collect a ref temperature before starting the test once board is on
            cam_meas = self.__temp_camera.get_measurement_from_box()
            self.__temp_meas_tab.add_dict_measurement(cam_meas)
            self.__temp_meas_tab.add_measurement([self.get_time_tuple(),
                            (self._em_cst.COMMENTS, "Setup: Temperature just before starting the test")])
            self.__temp_meas_tab.switch_to_next_meas()

        ####################################### START BENCHMARK ###########################################
        self.phonesystem_api.wake_screen()
        self.phonesystem_api.set_phone_lock(False)
        self.__benchmark_module.execute_test(background=True, delay=self.__bk_delay)
        self.__host_test_start_time = time.time()

        start_timeout = 60
        running = False
        start_time = time.time()
        self._logger.info("wait at most %ss to see that benchmark has been launched" % str(start_timeout))
        while time.time() - start_time < start_timeout:
            running = self.__benchmark_module.is_running()
            if running:
                break

        if not running:
            txt = "benchmark fail to start after waiting %ss" % str(start_timeout)
            self._logger.error(txt)
            self._logger.error("benchmark execution logs for debugging : %s " % self.__benchmark_module.get_output_log())
            raise DeviceException(DeviceException.OPERATION_FAILED, txt)
        self.__benchmark_module.check_crash()
        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        run test
        """
        EmUsecaseBase.run_test_body(self)
        ####################################### TAG TEST START, WAIT FOR TEST, THEN TAG TEST END ###########################################
        # inject a start tag with the host date as unique id
        start_tag = "START_WAITING_FOR_INTENT-" + time.strftime("-%Y-%m-%d_%Hh%M.%S")
        if not self.__parser_api.inject_tag(start_tag):
            error_msg = "The tag [%s] failed to be injected on logs" % (start_tag)
            self._logger.error(error_msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, error_msg)

        # remove any cable
        self._device.disconnect_board()
        self._io_card.remove_cable("ALL")
        bk_duration = self.__benchmark_module.get_exec_whole_duration()
        delay_left = self.__bk_delay - (time.time() - self.__host_test_start_time)
        if delay_left > 1:
            self._logger.info("Waiting %ss to align benckmark launching time with beginning of waiting duration" % str(delay_left))
            time.sleep(delay_left)

        start_time = time.time()
        self._logger.info("waiting %ss of benchmark execution" % str(bk_duration))
        time.sleep(bk_duration)
        stop_time = time.time()
        # reconnect usb cable
        warning_crash = False
        self._io_card.usb_host_pc_connector(True)
        time.sleep(self.usb_sleep)
        self._device.connect_board()

        # check if board did not went off during the test
        boot_mode = self._device.get_boot_mode()
        if boot_mode != "MOS":
            error_msg = "The board is seen not booted in MOS but in %s at the end of the test , cant compute result" % (boot_mode)
            self._logger.error(error_msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, error_msg)
        self._device.screenshot(filename=os.path.join(self.__pic_folder, "test_end_board_screen.png"))

        # inject a stop tag with the host date as unique id
        stop_tag = "STOP_WAITING_FOR_INTENT-" + time.strftime("-%Y-%m-%d_%Hh%M.%S")
        if not self.__parser_api.inject_tag(stop_tag):
            error_msg = "The tag [%s] failed to be injected on logs " % (stop_tag)
            self._logger.error(error_msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, error_msg)

        ####################################### CHECK BENCHMARK CRASH ###########################################
        # first check that there were no crashes
        start_date = self.__benchmark_module.get_start_date()
        crash_date, _ = self.__benchmark_module.get_crash()

        # if a crash happen but after that the test end , then the test may be passed with a warning
        if start_date is not None:
            if crash_date is not None:
                bk_exact_duration = crash_date - start_date
                host_exact_duration = stop_time - start_time
                if bk_exact_duration <= host_exact_duration:
                    self.__benchmark_module.check_crash()
                else:
                    warning_crash = True
        else:
            error_msg = "Cant retrieve benchmark start date, it seems that the benchmark may have not been started"
            self._logger.error(error_msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, error_msg)

        # try to stop the benchmark, if it fail, continue the measurement
        try:
            self.__benchmark_module.stop_execution()
        except Exception as e:
            self._logger.error("error happen when stopping benchmark %s" % str(e))

        # upload benchmark result
        try :
            score_list = self.__benchmark_module.get_score()
            self.__push_score(score_list)
        except Exception as e:
            self._logger.error("Error happen when trying to upload benchmark: %s" % str(e))

        ####################################### SEARCH FOR COOLING ACTION ###########################################
        # get start intent to extract the time
        start_intent = self.__parser_api.find_txt_between_tag(start_tag, stop_tag,
                                                                 self.__benchmark_module.BENCHMARK_START_TAG, raise_error=True)[0]
        # "ACS_TAG_INJECTOR: START_WAITING_FOR_INTENT--2014-12-22_11h40.22"
        cooling_intents = self.__parser_api.find_txt_between_tag(start_tag, stop_tag,
                                                                 self.__parser_api.THERMAL_TAG.get("COOLING_INTENT"),
                                                                 raise_error=False)
        action_seen = False
        local_msg = ""
        action_before_benchmark = False
        # parse every zone to analyze only the wanted one
        for cooling_intent in cooling_intents:
            result = self.__parser_api.parse_thermal_cooling_event(cooling_intent)
            if result is not None:
                zone, level, temperature = result
                zone = zone.upper()

                # this way allow to compare simple str to several monitored zone
                # if we are in ANY case, compute the first seen and leave the test here
                if self.__zone_to_inspect in ["ANY", zone]:
                    # get the time structure for the cooling intent
                    thermal_time_structure, thermal_millisec = self.__parser_api.get_log_date(cooling_intent)
                    time_cooling_happen = float(calendar.timegm(thermal_time_structure)) + (float(thermal_millisec) * 0.001)

                    # get the time structure for start intent
                    start_time_structure, start_millisec = self.__parser_api.get_log_date(start_intent)
                    time_intent_sent = float(calendar.timegm(start_time_structure)) + (float(start_millisec) * 0.001)

                    spend_time = time_cooling_happen - time_intent_sent
                    if spend_time < 0:
                        self._meas_list.add("TIME_TAKEN_TO_SEE_FIRST_COOLING_ACTION", spend_time, "SECOND")
                        local_msg += "A cooling action has been seen %ss before benchmark start and after board was cool down : zone=%s level=%s temperature=%s.\n" % (abs(spend_time), zone, level, temperature)
                        action_before_benchmark = True
                    else:
                        self._meas_list.add("TIME_TAKEN_TO_SEE_FIRST_COOLING_ACTION", spend_time, "SECOND")
                        local_msg += "First cooling action seen %ss after benchmark start: zone=%s level=%s temperature=%s.\n" % (spend_time, zone, level, temperature)
                        action_seen = True
                        break

        msg = "no cooling action seen."
        if local_msg != "":
            msg = local_msg
        if action_before_benchmark == True:
            msg += "A manual check on cooling actions before benchmark is required to see if it is a real FAIL or PASS.."
        # generate em verdict
        if not action_seen:
            self._meas_list.add("TIME_TAKEN_TO_SEE_FIRST_COOLING_ACTION", bk_duration, "SECOND")
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets, True)
        self._em_meas_verdict.judge()
        verdict, verdict_msg = self._em_meas_verdict.get_current_result_v2()
        msg = msg + "\n" + verdict_msg
        if warning_crash :
            msg = "A crash happened but it was after the test end, result can be computed.\n" + msg
        msg += "\nBenchmark scores may be seen in file %s" % self.__benchamrk_tab.get_report_path()
        return verdict, msg

    def tear_down(self):
        """
        End and dispose the test
        """
        EmUsecaseBase.tear_down(self)
        benchmark_stopped = False
        # stop benchmark here to prevent it from running in loop while charging below
        if self.is_board_and_acs_ok():
            try:
                self.__benchmark_module.clean()
                benchmark_stopped = True
            except Exception as e:
                self._logger.error("error happen when stopping benchmark %s" % str(e))

        #  stop camera anyway
        if self.__temp_camera is not None:
            try:
                self.__temp_camera.delete_all_pictures()
                self.__temp_camera.set_linear_temperature_mode(False)
            except Exception as e:
                self._logger.error("error happen when releasing thermal camera %s" % str(e))
            finally:
                self.__temp_camera.release()

        # stop all load
        self.em_core_module.clean_up()
        self.__load_module.stop_load()
        if not benchmark_stopped:
            self.__benchmark_module.clean()
        # stop benchmark
        return Global.SUCCESS, "No errors"

    def _setup_camera(self):
        """
        setup the camera
        """
        self.__temp_camera.auto_setup()
        x, y, h, w = self.__temp_camera.get_image_geometry()
        self.__temp_camera.set_linear_temperature_mode(True)
        self.__temp_camera.configure_measurement_box(True, x, y, h, w)

    def _monitor_board_temp(self, monitor_timeout, target_temp):
        """
        monitor board temperature until it reach given value or a value below it.
        :todo: this function only monitor for cooling condition,
                it may be change to offer the possibility to monitor for heating one
        """
        start_temp_seen = False
        wait_time = monitor_timeout + 15
        self._logger.info("waiting at most %ss+15s to let the board cool down below %s degree" % (str(monitor_timeout), str(target_temp)))
        start_time = time.time()
        while time.time() - start_time < wait_time:
            cam_meas = self.__temp_camera.get_measurement_from_box()

            try:
                self.__temp_meas_tab.add_dict_measurement(cam_meas)
                self.__temp_meas_tab.add_measurement([self.get_time_tuple(),
                                    (self._em_cst.COMMENTS, "Setup: cooling down the board")])
                self.__temp_meas_tab.switch_to_next_meas()
            except Exception as e:
                msg = "error happen when filling temperature value in file: %s" % str(e)
                self._logger.error(msg)

            if cam_meas["MAXT"][0] <= target_temp:
                self._logger.info("the MAX temperature seen is %s, test can continue " % str(cam_meas["MAXT"]))
                # take a ref picture
                pic_path = self.__temp_camera.take_picture("ref_phone_off")
                self.__temp_camera.pull_picture(pic_path, self.__pic_folder)
                start_temp_seen = True
                self.__ref_temp = cam_meas["MAXT"][0]
                break

        if not start_temp_seen:
            cam_meas = self.__temp_camera.get_measurement_from_box()
            pic_path = self.__temp_camera.take_picture("pic_temp_after_cooldown_fail")
            self.__temp_camera.pull_picture(pic_path, self.__pic_folder)
            error_msg = "The board fail to cool down below %s degree after spending %ss in OFF state, the last temperature seen was %s degree" % (target_temp,
                                                                                                                                                  monitor_timeout,
                                                                                                                                                  str(cam_meas["MAXT"]))
            self._logger.error(error_msg)
            self._device.switch_on()
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, error_msg)
