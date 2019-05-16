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
:summary: This usecase measure the max temperature seen during a video record
:author: vgomberx
:since: 05/03/2015
"""

import time, os
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsBaseException import AcsBaseException
from ErrorHandling.AcsConfigException import  AcsConfigException
from acs_test_scripts.Utilities.EMUtilities import XMLMeasurementFile
from acs_test_scripts.UseCase.EnergyManagement.UcModule.VideoCaptureModule import VideoCaptureModule
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from acs_test_scripts.UseCase.EnergyManagement.UcModule.LoadModule import LoadModule
from acs_test_scripts.Lib.EM.VideoPlayback import MediaPlayer
from UtilitiesFWK.Utilities import Global


class LabEmTempMonitorVideoCapture(EmUsecaseBase):

    """
    Class Lab temperature during Video Capture
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
        self.__record_duration = self._tc_parameters.get_param_value("VIDEO_RECORD_DURATION", 120, default_cast_type=int)
        self.__cooldown = self._tc_parameters.get_param_value("SETUP_TIME_COOLDOWN_BOARD", 120, default_cast_type=int)
        self.__start_temp = self._tc_parameters.get_param_value("SETUP_START_TEMPERATURE", 30, default_cast_type=int)
        self.__host_media = self._tc_parameters.get_param_value("MEDIA_TO_RECORD", "", default_cast_type=str)
        self.__host_monitor = self._tc_parameters.get_param_value("MONITOR_ON_WHICH_MEDIA_IS_PLAY", -1, default_cast_type=int)
        self.__temp_target = self._tc_parameters.get_param_value("TARGET_MAX_TEMPERATURE", default_cast_type=int)

        # load targets in order to measure iteration
        self.__video_capture_mod = VideoCaptureModule()
        self.__load_module = LoadModule()
        self.__load_module.add_load(setup_load)
        self.__video_record_api = self._device.get_uecmd("VideoRecorder", True)
        self.__media_player = None
        if self.__host_media != "":
            self.__media_player = MediaPlayer()
        # measurement file
        meas_file_name = os.path.join(self._saving_directory, "EM_meas_report.xml")
        self.__em_meas_tab = XMLMeasurementFile(meas_file_name)

        meas_file_name = os.path.join(self._saving_directory, "temperature_report.xml")
        self.__temp_meas_tab = XMLMeasurementFile(meas_file_name)
        self.__temp_camera = self._em.get_thermal_camera("THERMAL_CAMERA")
        self.__pic_folder = self._saving_directory
        self.__ref_temp = None

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # Call LabPwrMeasBase base Setup function
        EmUsecaseBase.set_up(self)
        # reset the ref value
        self.__ref_temp = None
        # check that media exist
        if self.__media_player is None:
            self._logger.warning("no media was set to be launched on the HOST side, no media will be play during video capture")
        else:
            if not os.path.exists(self.__host_media):
                error_msg = "The video %s you want to play on the host side does not exist" % self.__host_media
                self._logger.error(error_msg)
                raise AcsConfigException(AcsConfigException.FILE_NOT_FOUND, error_msg)

        # clean video storage
        self.__video_record_api.clean_video_storage()
        # init camera connection
        self.__temp_camera.init()
        self.__temp_camera.delete_all_pictures()
        # Update Battery Information
        em_info = self.update_battery_info()
        if self.em_core_module.is_batt_capacity_below_target(em_info, self.em_core_module.batt_min_capacity):
            # charge part
            self.em_core_module.monitor_charging(self.em_core_module.batt_min_capacity,
                                         self.em_core_module.charge_time,
                                         self.__em_meas_tab)

        self.__video_capture_mod.setup()
        self.phonesystem_api.set_screen_timeout(3600)
        self._device.switch_off()
        # remove any cable after this
        self._io_card.remove_cable("ALL")
        # do the thermal camera setting during the off phase to let the board cooldown
        self._setup_camera()
        # wait a given time to let the board cool down
        start_temp_seen = False
        wait_time = self.__cooldown + 15
        self._logger.info("waiting at most %ss+15s to let the board cool down below %s degree" % (str(self.__cooldown), str(self.__start_temp)))
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

            if cam_meas["MAXT"][0] <= self.__start_temp:
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
            error_msg = "The board fail to cool down below %s degree after spending %ss in OFF state, the last temperature seen was %s degree" % (self.__start_temp,
                                                                                                                                                  self.__cooldown,
                                                                                                                                                  str(cam_meas["MAXT"]))
            self._logger.error(error_msg)
            self._device.switch_on()
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, error_msg)

        # turn on board and launch every load
        self._device.switch_on()
        self.em_api.get_thermal_sensor_info()
        # start all environment load
        self.__load_module.start_load()

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
                raise AcsBaseException(AcsBaseException.OPERATION_FAILED, error_msg)

        # collect a ref temperature before starting the test once board is on
        cam_meas = self.__temp_camera.get_measurement_from_box()
        self.__temp_meas_tab.add_dict_measurement(cam_meas)
        self.__temp_meas_tab.add_measurement([self.get_time_tuple(),
                        (self._em_cst.COMMENTS, "Setup: Temperature just before starting the test")])
        self.__temp_meas_tab.switch_to_next_meas()
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
                screen_path = self._device.screenshot(filename=os.path.join(self.__pic_folder, "camera_record_fail.png"))
                error_msg = "Fail to see the raw video file generated during the record, it seems that the video record may has not started."
                if screen_path is not None:
                    error_msg += "\nPlease check the screenshot at %s." % screen_path
                self._logger.error(error_msg)
                raise DeviceException(DeviceException.FILE_SYSTEM_ERROR, error_msg)

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

        self._logger.info("Monitoring temperature during %ss " % str(self.__record_duration))
        # do the monitoring job
        maxt = self.__temp_camera.get_measurement_from_box()["MAXT"]
        last_max_temp_seen = maxt[0]
        unit = maxt[1]
        pic_path = None
        start_time = time.time()
        while time.time() - start_time < self.__record_duration:
        # we want to monitor and see the highest temperature seen by the board
            cam_meas = self.__temp_camera.get_measurement_from_box()
            max_temp = cam_meas["MAXT"][0]
            if max_temp > last_max_temp_seen:
                # save a picture of each highest temperature seen
                pic_path = self.__temp_camera.take_picture("max_during_record")
                last_max_temp_seen = max_temp
            try:
                self.__temp_meas_tab.add_dict_measurement(cam_meas)
                self.__temp_meas_tab.add_measurement([self.get_time_tuple(),
                                                    (self._em_cst.COMMENTS, "Runtest: video record ongoing")])
                self.__temp_meas_tab.switch_to_next_meas()
            except Exception as e:
                msg = "error happen when filling temperature value in file: %s" % str(e)
                self._logger.error(msg)

        # reconnect usb cable
        self._io_card.usb_host_pc_connector(True)
        if pic_path is not None:
            self.__temp_camera.pull_picture(pic_path, self.__pic_folder)

        time.sleep(self.usb_sleep)
        self._device.connect_board()
        # check if board did not went off during the test
        boot_mode = self._device.get_boot_mode()
        if boot_mode != "MOS":
            error_msg = "The board is seen not booted in MOS but in %s at the end of the test , cant compute result" % (boot_mode)
            self._logger.error(error_msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, error_msg)

        # get a proof that video was still running
        self._device.screenshot(filename=os.path.join(self.__pic_folder, "test_end_board_screen.png"))
        self.__video_capture_mod.stop_recording()
        # stop HOST Video
        if not self.__media_player is None:
            self.__media_player.stop()

        verdict = Global.FAILURE
        msg = "The ref temperature when board is OFF was at %s %s.\n" % (self.__ref_temp, unit)
        msg += "The max temperature seen on the DUT during %ss of video recording is %s %s" % (self.__record_duration,
                                                                                           last_max_temp_seen, unit)
        if last_max_temp_seen > self.__temp_target :
            msg += " which is above"
        else:
            msg += " which is below or equal to"
            verdict = Global.SUCCESS
        msg += " the target not to exceed : %s degree" % (self.__temp_target)
        self._logger.info(msg)
        return verdict, msg

    def tear_down(self):
        """
        End and dispose the test
        """
        EmUsecaseBase.tear_down(self)
        try:
            self.__temp_camera.set_linear_temperature_mode(False)
            self.__temp_camera.delete_all_pictures()
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