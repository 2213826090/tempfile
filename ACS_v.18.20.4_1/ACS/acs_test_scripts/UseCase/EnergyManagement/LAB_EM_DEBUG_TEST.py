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
:summary: EM - this UseCase test all Uecmd used by all EM/PUPDR/THERMAL UseCase
:author: dbatutx, vgombert
:since: 23/11/2012
"""

from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from acs_test_scripts.UseCase.Misc.UECMD_TEST_TOOL import UECmdTestTool
from acs_test_scripts.UseCase.EnergyManagement.UcModule.MultimediaModule import MultimediaModule
from UtilitiesFWK.Utilities import Global
import time


class LabEmDebugTest(EmUsecaseBase, UECmdTestTool):

    """
    Lab Energy Management class.
    """
    # This is the default bench type where this test can be run
    DEDICATED_BENCH = "BATTERY_BENCH"

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call the LabEmBaseCommon Init function
        EmUsecaseBase.__init__(self, tc_name, global_config)  # pylint: disable=W0233
        UECmdTestTool.__init__(self, self.get_name(), self._logger, self._device)

        # Read UECMD_TYPE_LIST from TC parameters
        self.__uecmd_type = self._tc_parameters.get_param_value("UECMD_TYPE_LIST", "").upper()

        # Call ConfigsParser to parse Energy_Management
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_DEBUG_TEST", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        self._mum = MultimediaModule("Video")

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        EmUsecaseBase.set_up(self)
        UECmdTestTool.set_up(self, self.tc_order)

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets, self.tcd_to_test)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        """
        EmUsecaseBase.run_test_body(self)

        if "THERMAL" in self.__uecmd_type:
            # test thermal
            self.__thermal_test()
        if "MSIC" in self.__uecmd_type:
            # test msic uecmd
            self.__msic_test()
        if "BCU" in self.__uecmd_type:
            # test bcu uecmd
            self.__bcu_test()
        if "AUTOLOG" in self.__uecmd_type:
            # test autolog uecmd
            self.__autolog_test()
        if "SCREEN" in self.__uecmd_type:
            # test screen uecmd
            self.__screen_test()
        if "STATUS" in self.__uecmd_type:
            # test status uecmd
            self.__status_test()
        if "CONSUMPTION" in self.__uecmd_type:
            # test consumption uecmd
            self.__consumption_test()
        if "MULTIMEDIA" in self.__uecmd_type:
            # test multimedia uecmd
            self.__multimedia_test()

        # compare values with targets
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets, True)
        self._em_meas_verdict.judge(True)

        return self._em_meas_verdict.get_current_result_v2()

    def tear_down(self):
        """
        End and dispose the test
        """
        EmUsecaseBase.tear_down(self)
        # retrieve logs
        if self.DEDICATED_BENCH == "BATTERY_BENCH":
            self.get_application_logs()
            # clean schedule created object
            self.phonesystem_api.clean_daemon_files()

        return Global.SUCCESS, "No errors"

#---------------------------------------------------------------------------

    def __thermal_test(self):
        """
        test all uecmd linked to thermal feature

        :return: None
        """
        tag = "THERMAL_TEST"

        def get_scheduled_thermal_info():
            """
            local function
            """
            pid = self.em_api.get_thermal_sensor_info("scheduled", 10)
            time.sleep(15)
            return self.em_api.get_thermal_sensor_info("read", pid)

        fct_list = [  # test the get thermal info
            {self._FCT: self.em_api.get_thermal_sensor_info},
            # test the scheduled get thermal"
            {self._FCT: get_scheduled_thermal_info,
             self._DEP: ["get_thermal_sensor_info"]},
            # test the set temp validity
            {self._FCT: self.em_api.load_thermal_file},
            # test the fake thermal builder validity
            {self._FCT: self.em_api.modify_thermal_threshold,
             self._PARMS: [100000, "SUPER_FAKE_SENSOR"]},
             {self._FCT: self.em_api.update_thermal_file},
            # test the thermal restoration validity
            {self._FCT: self.em_api.restore_thermal_file}]

        self.__evaluate_fct_list(fct_list, tag)

#---------------------------------------------------------------------------

    def __msic_test(self):
        """
        test all uecmd linked to msic feature

        :return: None
        """
        tag = "MSIC_TEST"

        def get_scheduled_msic_info():
            """
            local function
            """
            pid = self.em_api.get_msic_registers("scheduled", 10)
            time.sleep(15)
            return self.em_api.get_msic_registers("read", pid)

        def get_scheduled_interrupt_info():
            """
            local function
            """
            pid = self.em_api.get_proc_interrupt("scheduled", 10)
            time.sleep(15)
            return self.em_api.get_proc_interrupt("read", pid)

        fct_list = [  # test msic register validity
            {self._FCT: self.em_api.get_msic_registers},
            # test schedule validity
            {self._FCT: get_scheduled_msic_info,
             self._DEP: ["get_msic_registers"]},
            # test interrupt validity
            {self._FCT: self.em_api.get_proc_interrupt},
            {self._FCT: get_scheduled_interrupt_info,
             self._DEP: ["get_proc_interrupt"]},
            # test the charge level validity
            {self._FCT: self.em_api.get_charger_level},
            {self._FCT: self.em_api.set_usb_charging,
             self._PARMS: ["on"]}]

        self.__evaluate_fct_list(fct_list, tag)

#---------------------------------------------------------------------------
    def __bcu_test(self):
        """
        test all uecmd linked to bcu feature

        :return: None
        """
        tag = "BCU_TEST"
        fct_list = [  # test bcu uecmd
            {self._FCT: self.em_api.get_bcu_status},
            {self._FCT: self.em_api.get_bcu_activation_status},
            {self._FCT: self.em_api.get_bcu_interrupt}]

        self.__evaluate_fct_list(fct_list, tag)

#---------------------------------------------------------------------------

    def __autolog_test(self):
        """
        test all uecmd linked to autolog feature

        :return: None
        """
        def configure_autolog():
            """
            local function
            """
            # test autolog clean
            self.em_api.clean_autolog()
            # test the autolog set up persistent
            self.em_api.set_persistent_autolog(True)

        def test_thermal_autolog():
            """
            local function
            """
            self.em_api.add_fct_to_auto_logger(self.em_api.AUTOLOG_THERMAL, "sequenced")
            self.em_api.start_auto_logger(0, 10, "sequenced")
            time.sleep(40)
            self.em_api.stop_auto_logger("sequenced")
            return self.em_api.get_autolog_thermal_sensor_info()

        def test_msic_autolog():
            """
            local function
            """
            self.em_api.add_fct_to_auto_logger(self.em_api.AUTOLOG_UEVENT, "sequenced")
            self.em_api.start_auto_logger(0, 10, "sequenced")
            time.sleep(20)
            self.em_api.reset_running_log()
            time.sleep(20)
            self.em_api.stop_auto_logger("sequenced")
            return self.em_api.get_autolog_msic_registers()

        tag = "AUTOLOG_TEST"
        fct_list = [{self._FCT: configure_autolog},
                    {self._FCT: test_msic_autolog},
                    {self._FCT: test_thermal_autolog},
                    {self._FCT: self.em_api.clean_autolog}]

        self.__evaluate_fct_list(fct_list, tag)

#---------------------------------------------------------------------------
    def __screen_test(self):
        """
        test all uecmd linked to screen feature

        :return: None
        """
        tag = "SCREEN_TEST"
        fct_list = [  # test the  set brightness mode validity
            {self._FCT: self.phonesystem_api.set_brightness_mode,
             self._PARMS: ["manual"]},
            # test the  set screen timeout validity
            {self._FCT: self.phonesystem_api.set_screen_timeout,
             self._PARMS: [3600]},
            # test the  set display brightness validity
            {self._FCT: self.phonesystem_api.set_display_brightness,
             self._PARMS: [100]},
            # test the  set phone lock validity
            {self._FCT: self.phonesystem_api.set_phone_lock,
             self._PARMS: ["off"]},
            # test the  wake screen validity
            {self._FCT: self.phonesystem_api.wake_screen},
            # test the disable lock screen validity
            {self._FCT: self.phonesystem_api.disable_lockscreen},
            # test the get backlight level  validity
            {self._FCT: self.phonesystem_api.get_backlight_level}]

        self.__evaluate_fct_list(fct_list, tag)

#---------------------------------------------------------------------------

    def __status_test(self):
        """
        test all uecmd linked to status feature

        :return: None
        """
        tag = "STATUS_TEST"
        self._device.reboot()

        def parse_aplog():
            """
            local function
            """
            parser_api = self._device.get_uecmd("Aplog", True)
            msg_to_search = "CHIMATA CHAPAPA"
            start_tag = "START_TAG-" + time.strftime("-%Y-%m-%d_%Hh%M.%S")
            parser_api.inject_tag(start_tag)
            parser_api.inject_tag(msg_to_search)
            stop_tag = "STOP_TAG-" + time.strftime("-%Y-%m-%d_%Hh%M.%S")
            parser_api.inject_tag(stop_tag)

            # get shutdown reason
            found_entries = parser_api.find_txt_between_tag(start_tag, stop_tag,
                                                                 msg_to_search,
                                                                 raise_error=False)
            return len(found_entries) > 0

        fct_list = [  # test the get cpu freq validity
            {self._FCT: self.phonesystem_api.get_cpu_freq,
             self._PARMS: [0]},
            # test the get boot wake source validity
            {self._FCT: self.phonesystem_api.get_boot_wake_source},
            # test the get screen status validity
            {self._FCT: self.phonesystem_api.get_screen_status},
            {self._FCT: parse_aplog,
             self._EXP_RES:True},
            # test the read phone device validity
            {self._FCT: self.phonesystem_api.read_phone_device,
             self._PARMS: ["MOS"],
             self._EXP_RES:"DEVICE"},
            # test the clean daemon files validity
            {self._FCT: self.phonesystem_api.clean_daemon_files}]

        self.__evaluate_fct_list(fct_list, tag)

#---------------------------------------------------------------------------

    def __consumption_test(self):
        """
        test all uecmd linked to consumption feature

        :return: None
        """
        tag = "CONSUMPTION_TEST"
        fct_list = [
            # test the set vibration test validity
            {self._FCT: self.phonesystem_api.set_vibration,
             self._PARMS: ["on"]},
            # test the get vibration test validity
            {self._FCT: self.phonesystem_api.get_vibration_state,
             self._EXP_RES: True},
            {self._FCT: self.phonesystem_api.set_vibration,
             self._PARMS: ["off"]},
            # test the set torchlight test validity
            {self._FCT: self.phonesystem_api.set_torchlight,
             self._PARMS: ["on"]},
            {self._FCT: self.phonesystem_api.set_torchlight,
             self._PARMS: ["off"]},
            {self._FCT: self.bt_api.set_bt_power,
             self._PARMS: ["on"]},
            {self._FCT: self.bt_api.set_bt_power,
             self._PARMS: ["off"]},
            {self._FCT: self.networking_api.set_wifi_power,
             self._PARMS: ["on"]},
            {self._FCT: self.networking_api.set_wifi_power,
             self._PARMS: ["off"]}]

        self.__evaluate_fct_list(fct_list, tag)

#---------------------------------------------------------------------------

    def __multimedia_test(self):
        """
        test all uecmds linked to multimedia feature

        :return: None
        """
        video_record_api = self._device.get_uecmd("VideoRecorder", True)
        tag = "MULTIMEDIA_TEST"
        fct_list = [
            # test the set vibration test validity
            {self._FCT: self._mum.start_media},
            # test the get vibration test validity
            {self._FCT: self._mum.is_media_running, self._EXP_RES: True},
            {self._FCT: self._mum.stop_media},
            {self._FCT: video_record_api.setup_camera},
            {self._FCT: video_record_api.record},
            {self._FCT: video_record_api.stop_record},
            {self._FCT: video_record_api.check_video_exist, self._PARMS: ["/fake/path"], self._EXP_RES: False},
            {self._FCT: video_record_api.clean_video_storage},
            {self._FCT: video_record_api.restore_camera_setup,
             self._FCT: video_record_api.force_stop}]

        self.__evaluate_fct_list(fct_list, tag)

    def __evaluate_fct_list(self, fct_list, tag):
        """
        sub function to simplify redundant call

        :type fct: list
        :param fct: a list of dict describing functions to test

        :type tag: str
        :param tag: tag associated to these tests. Will be used for EM reports.
        """
        verdict = True

        for fct2test in fct_list:
            # do a logical operation to catch fail
            result = self._check_uecmd(fct2test)
            if (result in [self._FAIL, self._BLOCKED]) and verdict:
                verdict = False

        if verdict:
            self._meas_list.add(tag, "PASS", "none")
            self._device.get_logger().info(str(tag) + " passed")
        else:
            self._meas_list.add(tag, "FAIL", "None")
            self._device.get_logger().info(str(tag) + " failed")
