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
:summary: This usecase measure the max temperature seen during a benchmark execution
:author: vgomberx
:since: 18/03/2015
"""

import time, os, tempfile
from shutil import move
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.Utilities.EMUtilities import XMLMeasurementFile
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from acs_test_scripts.UseCase.EnergyManagement.UcModule.LoadModule import LoadModule
from acs_test_scripts.UseCase.EnergyManagement.UcModule.BenchmarkModule import BenchmarkModule
from Core.Report.Live.LiveReporting import LiveReporting
from UtilitiesFWK.Utilities import Global


class LabEmTempMonitorBenchmark(EmUsecaseBase):

    """
    Monitor temperature during benchmark
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

        setup_load = self._tc_parameters.get_param_value("SETUP_TECHNO", "")
        self.__cooldown = self._tc_parameters.get_param_value("SETUP_TIME_COOLDOWN_BOARD", 120, default_cast_type=int)
        self.__start_temp = self._tc_parameters.get_param_value("SETUP_START_TEMPERATURE", 30, default_cast_type=int)
        self.__temp_target = self._tc_parameters.get_param_value("TARGET_MAX_TEMPERATURE", default_cast_type=int)

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

        self.__temp_camera = self._em.get_thermal_camera("THERMAL_CAMERA")
        self.__pic_folder = self._saving_directory
        self.__ref_temp = None
        self.__host_test_start_time = None
        self.__bk_delay = 30

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
        # Call LabPwrMeasBase base Setup function
        EmUsecaseBase.set_up(self)

        # check that all necessary file exist
        self.__benchmark_module.setup()
        # init camera connection
        self.__temp_camera.init()
        # call the auto setup
        self.__temp_camera.delete_all_pictures()
        # Update Battery Information
        em_info = self.update_battery_info()
        if self.em_core_module.is_batt_capacity_below_target(em_info, self.em_core_module.batt_min_capacity):
            # charge part
            self.em_core_module.monitor_charging(self.em_core_module.batt_min_capacity,
                                         self.em_core_module.charge_time,
                                         self.__em_meas_tab)

        self.phonesystem_api.set_screen_timeout(3600)
        self._device.switch_off()
        # remove any cable after this
        self._io_card.remove_cable("ALL")
        # do the thermal camera setting during the off phase to let the board cool down
        self._setup_camera()
        # wait a given time to let the board cool down
        self._monitor_board_temp(self.__cooldown, self.__start_temp)
        # turn on board and launch every load
        self._device.switch_on()
        # this measurement is done only for debugging purpose
        self.em_api.get_thermal_sensor_info()
        # start all environment load
        self.__load_module.start_load()
        # collect a ref temperature before starting the test once board is on
        cam_meas = self.__temp_camera.get_measurement_from_box()
        self.__temp_meas_tab.add_dict_measurement(cam_meas)
        self.__temp_meas_tab.add_measurement([self.get_time_tuple(),
                        (self._em_cst.COMMENTS, "Setup: Temperature just before starting the test")])
        self.__temp_meas_tab.switch_to_next_meas()

        # start benchmark
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

        # remove any cable
        self._device.disconnect_board()
        self._io_card.remove_cable("ALL")
        bk_duration = self.__benchmark_module.get_exec_whole_duration()
        self._logger.info("Monitoring temperature during a total duration of %ss " % bk_duration)
        # do the monitoring job
        tmpd = tempfile.gettempdir()
        maxt = self.__temp_camera.get_measurement_from_box()["MAXT"]
        last_max_temp_seen = maxt[0]
        unit = maxt[1]
        pic_nb = 0
        max_temp_list = []
        delay_left = self.__bk_delay - (time.time() - self.__host_test_start_time)
        if delay_left > 1:
            self._logger.info("Waiting %ss to align benckmark launching with temperature recording" % str(delay_left))
            time.sleep(delay_left)

        start_time = time.time()
        while time.time() - start_time < bk_duration:
        # we want to monitor and see the highest temperature seen by the board
            cam_meas = self.__temp_camera.get_measurement_from_box()
            max_temp = cam_meas["MAXT"][0]
            if max_temp > last_max_temp_seen:
                # save a picture of each highest temperature seen
                pic_path = self.__temp_camera.take_picture("max_during_benchmark")
                pic_exte = os.path.splitext(pic_path)[1]
                final_file_name = "max_during_benchmark_" + str(pic_nb)
                if pic_exte != "":
                    final_file_name += "." + pic_exte
                host_pic_path = self.__temp_camera.pull_picture(pic_path, tmpd, final_file_name)
                max_temp_list.append((max_temp, host_pic_path, time.time()))
                last_max_temp_seen = max_temp
                pic_nb += 1
            try:
                self.__temp_meas_tab.add_dict_measurement(cam_meas)
                self.__temp_meas_tab.add_measurement([self.get_time_tuple(),
                                                    (self._em_cst.COMMENTS, "Runtest: benchmark execution")])
                self.__temp_meas_tab.switch_to_next_meas()
            except Exception as e:
                msg = "error happen when filling temperature value in file: %s" % str(e)
                self._logger.error(msg)
            time.sleep(1)
        stop_time = time.time()

        # reconnect usb cable
        verdict_msg = ""
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
        # first that there were no crashes and that all iterations have been done
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

        # try to stop the benchmark , if it fail, continue the measurement
        try:
            self.__benchmark_module.stop_execution()
        except Exception as e:
            self._logger.error("error happen when stopping benchmark %s" % str(e))

        bk_expected_iter = -1
        if self.__benchmark_module.is_based_on_iteration():
            bk_expected_iter = self.__benchmark_module.get_expected_iter()
            iteration, _ = self.__benchmark_module.get_last_complete_iter()
            if iteration < bk_expected_iter:
                error_msg = "benchmark iteration failed to iter the expected %s time, only %s iteration seen" % (bk_expected_iter, iteration)
                self._logger.error(error_msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, error_msg)

        # upload benchmark result
        try :
            score_list = self.__benchmark_module.get_score()
            self.__push_score(score_list)
        except Exception as e:
            self._logger.error("Error happen when trying to upload benchmark: %s" % str(e))

        # get execution info to see if all benchmark run has been done
        verdict = Global.FAILURE
        # search for the picture that match with the right data
        if len (max_temp_list) > 0:
            for temp, pic, pic_time in reversed(max_temp_list):
                # check that the measurement time was done at or before the benchmark test end

                # if based on a iteration number , we consider only picture taken during the benchmark run
                if (self.__benchmark_module.is_based_on_iteration() and pic_time > (start_time + bk_duration)):
                    verdict_msg = "the picture date was done after the benchmark iteration end, cant compute result!"
                    continue

                # otherwise we consider all picture
                move(pic, self.__pic_folder)
                verdict_msg = "The ref temperature when board is OFF was at %s %s.\n" % (self.__ref_temp, unit)
                if self.__benchmark_module.is_based_on_iteration():
                    verdict_msg += "The max temperature seen on the DUT during %s iteration of benchmark execution is %s %s" % (bk_expected_iter, temp, unit)
                else:
                    verdict_msg += "The max temperature seen on the DUT during %ss of benchmark execution is %s %s" % (bk_duration, temp, unit)

                if temp > self.__temp_target :
                    verdict_msg += " which is above"
                else:
                    verdict_msg += " which is below or equal to"
                    verdict = Global.SUCCESS
                verdict_msg += " the target not to exceed : %s degree" % (self.__temp_target)
                break

        else:
            verdict_msg = "the temperature before starting the test was at %s %s and was never exceed during the test, something may went wrong during your test" % (last_max_temp_seen, unit)

        if warning_crash :
            verdict_msg = "A crash happened but it was after the test end, result can be computed." + verdict_msg

        verdict_msg += "\nBenchmark scores may be seen in file %s" % self.__benchamrk_tab.get_report_path()

        self._logger.info(verdict_msg)
        return verdict, verdict_msg


    def store_benchmark_score(self):
        """
        store benchmark score in a file for tracking
        """
        score_list = self.__benchmark_module.get_score()
        if len(score_list > 0):
            for dico in list :
                self.__temp_meas_tab.add_dict_measurement(dico)
                self.__temp_meas_tab.add_measurement([self.get_time_tuple(),
                                (self._em_cst.COMMENTS, "Getting benchmark scores")])

    def tear_down(self):
        """
        End and dispose the test
        """
        EmUsecaseBase.tear_down(self)
        # stop benchmark here to prevent it from running in loop while charging below
        benchmark_stopped = False
        if self.is_board_and_acs_ok():
            try:
                self.__benchmark_module.clean()
                benchmark_stopped = True
            except Exception as e:
                self._logger.error("error happen when stopping benchmark %s" % str(e))

        # stop benchmark here to prevent it from running in loop while charging below
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


