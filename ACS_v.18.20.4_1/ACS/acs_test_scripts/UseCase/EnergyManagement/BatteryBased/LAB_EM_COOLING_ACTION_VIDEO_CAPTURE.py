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
:summary: This usecase measure the time it takes to trigger a thermal cooling action during a video capture
:author: vgomberx
:since: 16/12/2014
"""

import time, calendar, os
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.Utilities.EMUtilities import XMLMeasurementFile
from acs_test_scripts.UseCase.EnergyManagement.UcModule.VideoCaptureModule import VideoCaptureModule
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from acs_test_scripts.UseCase.EnergyManagement.UcModule.LoadModule import LoadModule
from acs_test_scripts.Lib.EM.VideoPlayback import MediaPlayer
from UtilitiesFWK.Utilities import Global, str_to_bool


class LabEmCoolingActionVideoCapture(EmUsecaseBase):

    """
    Class Lab cooling action during Video Capture
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
        self.__record_duration = self._tc_parameters.get_param_value("VIDEO_RECORD_DURATION", 120, default_cast_type=int)
        self.__host_media = self._tc_parameters.get_param_value("MEDIA_TO_RECORD", "", default_cast_type=str)
        self.__host_monitor = self._tc_parameters.get_param_value("MONITOR_ON_WHICH_MEDIA_IS_PLAY", -1, default_cast_type=int)

        setup_load = self._tc_parameters.get_param_value("SETUP_TECHNO", "")
        self.__start_temp = self._tc_parameters.get_param_value("SETUP_START_TEMPERATURE", 30, default_cast_type=int)
        self.__cooldown = self._tc_parameters.get_param_value("SETUP_TIME_COOLDOWN_BOARD", 120, default_cast_type=int)
        use_camera_to_cooldown = self._tc_parameters.get_param_value("SETUP_COOLDOWN_WITH_CAMERA", False , default_cast_type=str_to_bool)
        self.__temp_camera = None
        if use_camera_to_cooldown:
            self.__temp_camera = self._em.get_thermal_camera("THERMAL_CAMERA")

        # Call ConfigsParser to parse Energy_Management
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_COOLING_ACTION_VIDEO_CAPTURE", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())
        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets, consider_all_target=True)
        self.__video_capture_mod = VideoCaptureModule()
        self.__parser_api = self._device.get_uecmd("Aplog", True)
        self.__load_module = LoadModule()
        self.__load_module.add_load(setup_load)
        self.__video_record_api = self._device.get_uecmd("VideoRecorder", True)
        self.__media_player = None
        if self.__host_media != "":
            self.__media_player = MediaPlayer()
        # measurement file
        meas_file_name = os.path.join(self._saving_directory,
                                      "EM_meas_report.xml")
        self.__em_meas_tab = XMLMeasurementFile(meas_file_name)
        self.__ref_temp = None
        self.__pic_folder = self._saving_directory
        meas_file_name = os.path.join(self._saving_directory, "temperature_report.xml")
        self.__temp_meas_tab = XMLMeasurementFile(meas_file_name)

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # Call LabPwrMeasBase base Setup function
        EmUsecaseBase.set_up(self)
        # first turn off board for a while to allow it to cool down
        if self.__media_player is None:
            self._logger.warning("no media was set to be launched on the HOST side, no media will be play during video capture")
        else:
            if not os.path.exists(self.__host_media):
                error_msg = "The video %s you want to play on the host side does not exist" % self.__host_media
                self._logger.error(error_msg)
                raise AcsConfigException(AcsConfigException.FILE_NOT_FOUND, error_msg)

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

        ####################################### SETUP CAMERAS ###########################################
        if self.__temp_camera is not None:
            # init camera connection
            self.__temp_camera.init()
            self.__temp_camera.delete_all_pictures()
        # clean video storage
        self.__video_record_api.clean_video_storage()
        # Update Battery Information
        ####################################### CHARGE BOARD ###########################################
        em_info = self.update_battery_info()
        if self.em_core_module.is_batt_capacity_below_target(em_info, self.em_core_module.batt_min_capacity):
            # charge part
            self.em_core_module.monitor_charging(self.em_core_module.batt_min_capacity,
                                         self.em_core_module.charge_time,
                                         self.__em_meas_tab)

        ####################################### COOLDOWN BOARD ###########################################
        self.phonesystem_api.set_screen_timeout(3600)
        self.__video_capture_mod.setup()

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

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        run test
        """
        EmUsecaseBase.run_test_body(self)
        # inject a start tag with the host date as unique id
        start_tag = "START_WAITING_FOR_INTENT-" + time.strftime("-%Y-%m-%d_%Hh%M.%S")
        if not self.__parser_api.inject_tag(start_tag):
            error_msg = "The tag [%s] failed to be injected on logs" % (start_tag)
            self._logger.error(error_msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, error_msg)

        # start HOST Video
        if not self.__media_player is None:
            self.__media_player.play(self.__host_media, self.__host_monitor)
            # media may take time to start
            start_time = time.time()
            while time.time() - start_time < 10:
                time.sleep(1)
                if self.__media_player.is_playing():
                    break

            if not self.__media_player.is_playing():
                error_msg = "Fail to start the video playback on the HOST side"
                self._logger.error(error_msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, error_msg)

        # start video recording
        self.phonesystem_api.wake_screen()
        self.phonesystem_api.set_phone_lock(False)
        raw_video_filename, _ = self.__video_capture_mod.start_recording()
        # Check if the video file exist
        if not self.__video_record_api.check_video_exist(raw_video_filename):
            self._logger.error("Recording fail to start, retry again")
            raw_video_filename, _ = self.__video_capture_mod.start_recording()
            # Check if the video file exist
            if not self.__video_record_api.check_video_exist(raw_video_filename):
                screen_path = self._device.screenshot(filename=os.path.join(self.__pic_folder, "camera_record_start_fail.png"))
                error_msg = "Fail to see the raw video file generated during the record, it seems that the video record may has not started."
                if screen_path is not None:
                    error_msg += "\nPlease check the screenshot at %s." % screen_path
                self._logger.error(error_msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, error_msg)

        # remove any cable
        self._device.disconnect_board()
        self._io_card.remove_cable("ALL")

        self._logger.info("waiting %ss to let the video recording be done" % str(self.__record_duration))
        time.sleep(self.__record_duration)

        # reconnect usb cable
        self._io_card.usb_host_pc_connector(True)
        time.sleep(self.usb_sleep)
        self._device.connect_board()
        # check if board did not went off during the test
        boot_mode = self._device.get_boot_mode()
        if boot_mode != "MOS":
            error_msg = "The board is seen not booted in MOS but in %s at the end of the test , cant compute result" % (boot_mode)
            self._logger.error(error_msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, error_msg)

        # get a screenshot to check if camera is still recording
        self._device.screenshot(filename=os.path.join(self.__pic_folder, "test_end_board_screen.png"))
        self.__video_capture_mod.stop_recording()
        # inject a stop tag with the host date as unique id
        stop_tag = "STOP_WAITING_FOR_INTENT-" + time.strftime("-%Y-%m-%d_%Hh%M.%S")
        if not self.__parser_api.inject_tag(stop_tag):
            error_msg = "The tag [%s] failed to be injected on logs " % (stop_tag)
            self._logger.error(error_msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, error_msg)

        # stop HOST Video
        if not self.__media_player is None:
            self.__media_player.stop()

        # get start intent to extract the time
        start_intent = self.__parser_api.find_txt_between_tag(start_tag, stop_tag,
                                                                 self.__video_record_api.CAMERA_TAG.get("RECORD_START"), raise_error=True)[0]
        # "ACS_TAG_INJECTOR: START_WAITING_FOR_INTENT--2014-12-22_11h40.22"
        cooling_intents = self.__parser_api.find_txt_between_tag(start_tag, stop_tag,
                                                                 self.__parser_api.THERMAL_TAG.get("COOLING_INTENT"),
                                                                 raise_error=False)

        action_seen = False
        local_msg = ""
        action_before_video = False
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

                    # if the time between cooling action and the video camera start is < 0 then it is a pass but with a warning
                    spend_time = time_cooling_happen - time_intent_sent

                    if spend_time < 0:
                        self._meas_list.add("TIME_TAKEN_TO_SEE_FIRST_COOLING_ACTION", spend_time, "SECOND")
                        local_msg += "A cooling action has been seen %ss before video recording start: zone=%s level=%s temperature=%s.\n" % (abs(spend_time), zone, level, temperature)
                        action_before_video = True
                    else:
                        self._meas_list.add("TIME_TAKEN_TO_SEE_FIRST_COOLING_ACTION", spend_time, "SECOND")
                        local_msg += "First cooling action seen %ss after video recording start: zone=%s level=%s temperature=%s.\n" % (spend_time, zone, level, temperature)
                        action_seen = True
                        break

        msg = "no cooling action seen."
        if local_msg != "":
            msg = local_msg
        if action_before_video == True:
            msg += "A manual check on cooling actions before video start is required to see if it is a real FAIL or PASS.."

        # generate em verdict
        if not action_seen:
            self._meas_list.add("TIME_TAKEN_TO_SEE_FIRST_COOLING_ACTION", self.__record_duration, "SECOND")
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets, True)
        self._em_meas_verdict.judge()
        verdict, verdict_msg = self._em_meas_verdict.get_current_result_v2()
        msg = msg + "\n" + verdict_msg
        return verdict, msg

    def tear_down(self):
        """
        End and dispose the test
        """
        EmUsecaseBase.tear_down(self)
        if self.__temp_camera is not None:
            try:
                self.__temp_camera.delete_all_pictures()
                self.__temp_camera.set_linear_temperature_mode(False)
            except Exception as e:
                self._logger.error("error happen when releasing thermal camera %s" % str(e))
            finally:
                self.__temp_camera.release()
        if not self.__media_player is None:
            self.__media_player.stop()

        # stop all load
        self.em_core_module.clean_up()
        self.__load_module.stop_load()
        # stop any remaining video capture
        self.__video_record_api.force_stop()
        self.__video_record_api.restore_camera_setup()
        self.__video_record_api.clean_video_storage()

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
