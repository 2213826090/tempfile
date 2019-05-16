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
:summary: plug a charger to trigger a boot
:author: vgombert
:since: 08/05/2013
"""
import time
import os
import shutil
from distutils import archive_util
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from UtilitiesFWK.Utilities import Global, str_to_bool
import acs_test_scripts.Utilities.EMUtilities as EMUtil


class LabEmTurnOffTurnOn(EmUsecaseBase):

    """
    Lab Energy Management base class.
    """
    DEDICATED_BENCH = "BATTERY_BENCH"

    def __init__(self, tc_name, global_config):
        """
        Advanced ON OFF test that also provide action to charge the board
        """

        # Call UseCase base Init function
        EmUsecaseBase.__init__(self, tc_name, global_config)
        # init fuel gauging parameters
        self.em_core_module.init_fg_param()
        # time to wait to boot
        self.__boot_timeout = self._tc_parameters.get_param_value("BOOT_TIMEOUT", self._device.get_boot_timeout(), default_cast_type=int)
        # time to wait just after the boot action and before wait to successfully boot
        self.__after_perform_boot_time = self._tc_parameters.get_param_value("TIME_TO_WAIT_AFTER_JUST_PERFORM_BOOT", 10, default_cast_type=int)
        # time to wait to shutdown
        self.__shutdown_timeout = self._tc_parameters.get_param_value("SHUTDOWN_TIMEOUT", default_cast_type=int)
        # cable to insert to make board boot
        self.__cable_type = self._tc_parameters.get_param_value("CABLE_TYPE")
        self.__keep_cable_during_shutdown = self._tc_parameters.get_param_value("KEEP_CABLE_DURING_SHUTDOWN", True, default_cast_type=str_to_bool)
        # cable to insert to make board boot
        self.__max_iteration = self._tc_parameters.get_param_value("MAX_ITERATION", default_cast_type=int)
        # time to wait to shutdown
        self.__max_fail = self._tc_parameters.get_param_value("MAX_CONSECUTIVE_FAIL", 0, default_cast_type=int)
        self.__min_capacity_stop = self._tc_parameters.get_param_value("MIN_CAPACITY_TO_STOP_TEST", 0, default_cast_type=int)
        # time to wait to shutdown
        # Initialize EM  xml object
        # measurement file
        meas_file_name = os.path.join(self._saving_directory,
                                      "EM_meas_report.xml")
        self.__em_meas_tab = EMUtil.XMLMeasurementFile(meas_file_name)
        self.__parser_api = self._device.get_uecmd("Aplog", True)
        self.__fail_boot = 0
        self.__pass_boot = 0

#-----------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # in setup board should be on
        EmUsecaseBase.set_up(self)
        if self.__cable_type != self._io_card.USB_HOST_PC:
            if not self.em_core_module.is_host_connection_available_when_charger_plug(self.__cable_type,
                                                                                  keep_charger_if_data=True):
                msg = "Data is necessary to perform the test, not data is seen when we plug %s" % self.__charger_to_use
                self._logger.error(msg)
                raise AcsConfigException(AcsConfigException.OPERATION_FAILED, msg)

        # Update Battery Information
        em_info = self.update_battery_info()
        if self.em_core_module.is_batt_capacity_below_target(em_info, self.em_core_module.batt_min_capacity):
            # charge part
            self.em_core_module.monitor_charging(self.em_core_module.batt_min_capacity,
                                         self.em_core_module.charge_time,
                                         self.__em_meas_tab)

        return (Global.SUCCESS, "No errors")

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        do iterative OFF the ON board by using a end user way:
        soft shutdown through UI
        boot from power button press
        in case of unknown state hard shutdown with button press
        """
        EmUsecaseBase.run_test_body(self)

        iteration = 0
        last_injected_tag = ""
        consecutive_fail = 0
        last_fail_iteration = None
        must_leave = False
        dopull = False

        while iteration < self.__max_iteration:
            mode = self._device.get_boot_mode()
            iteration += 1
            error_msg = ""
            # -------------------------------------------------------------OFF -------------------------------
            self._logger.info("turn OFF board")
            off_stamp = "TURN OFF iteration %s" % iteration + time.strftime("-%Y-%m-%d_%Hh%M.%S")
            self._logger.info(off_stamp)
            # if there is a boot mode seen, try to inject time stamp
            if mode in ["MOS", "COS"]:
                # init injected tag only if it was not done before
                if self.__parser_api.inject_tag(off_stamp) and last_injected_tag == "":
                    last_injected_tag = off_stamp
            # TURN OFF ACTION
            mode = self.__turn_off_board(mode)

            # -------------------------------------------------------------ON -------------------------------
            # if we are in MOS it means that the shutdown fail
            if mode != "MOS":
                # TURN ON ACTION
                mode = self.__turn_on_board()
                msg = "board seen booted in %s mode at ITERATION %s" % (mode, iteration)
                self._logger.info(msg)
                if mode == "MOS":
                    self._logger.info("Wait 5s to settle down the boot")
                    time.sleep(5)
                    self.__pass_boot += 1
                    error_msg = ""
                else:
                    self.__fail_boot += 1
                    error_msg = "Board seen booted in %s" % mode

            else:
                msg = "board has not shutdown and seen in %s mode before any boot action at iteration %s" % (mode, iteration)
                self._logger.info(msg)
                error_msg = msg

            if mode != "UNKNOWN":
                try:
                    # if we are in MOS reconnect ACS to gather em info
                    if mode == "MOS":
                        self._device.connect_board()
                    else:
                        self._device.disconnect_board()

                    self.__parser_api.inject_tag(msg)
                    if dopull:
                        # try to pull application log from tag
                        self.__custom_get_application_logs(last_injected_tag, last_fail_iteration, iteration)
                        dopull = False

                    last_injected_tag = ""
                    last_fail_iteration = None
                    # pull em info
                    if mode in ["MOS", "COS", "ROS"]:
                        em_info = self.update_battery_info()
                        if em_info["BATTERY"]["CAPACITY"][0] == self.__min_capacity_stop:
                            must_leave = True
                        self.__em_meas_tab.add_dict_measurement(em_info)
                except Exception as e:
                    error_msg = "Python crash after boot: " + str(e)
                    self._logger.error(error_msg)

            else:
                # init this var only one time to mark the last failed attempt
                if last_fail_iteration is None:
                    last_fail_iteration = iteration
                dopull = True

            self.__em_meas_tab.add_measurement([self.get_time_tuple(),
                                            (self._em_cst.COMMENTS, "RUNTEST"),
                                            ("BOOT_MODE", mode),
                                            ("ITERATION", iteration),
                                            ("ERROR_MSG", error_msg)])
            self.__em_meas_tab.switch_to_next_meas()

            if error_msg != "":
                consecutive_fail += 1
            else:
                consecutive_fail = 0

            if  0 < consecutive_fail >= self.__max_fail:
                self._logger.error("boot failed consecutively more than %s times, stopping the usecase" % self.__max_fail)
                break
            if must_leave:
                self._logger.error("baord capacity reach 0 stopping usecase")
                break

        msg = "Number of boot attempt done %s (%s PASS, %s FAIL)" % (iteration, self.__pass_boot, self.__fail_boot)
        return (Global.SUCCESS, msg)

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        # call tear down after some operations
        EmUsecaseBase.tear_down(self)
        # clean the board state and retrieve logs
        self.em_core_module.clean_up()

        return (Global.SUCCESS, "success")

    def __turn_off_board(self, mode):
        """
        try to turn off the board by using the UI menu when possible.
        """
        # then turn OFF the board by using a end user way
        if mode == "MOS":
            # turn off from UI
            if self.__keep_cable_during_shutdown:
                self.phonesystem_api.wake_screen()
                self._io_card.press_power_button(3)
                time.sleep(0.5)
                self.phonesystem_api.select_pwoff_menu("ok")
                self._device.disconnect_board()
            else:
                # use a delayed off action
                self.phonesystem_api.perform_graceful_shutdown(15)
                self._device.disconnect_board()
                self._io_card.remove_cable(self.__cable_type)
                self._logger.info("wait 15s to let the scheduled shutdown be performed")
                time.sleep(15)
        else:
            if not self.__keep_cable_during_shutdown:
                self._io_card.remove_cable(self.__cable_type)
            # if we are in COS, removing the cable should be enough to shutdown
            if mode != "COS" or self.__keep_cable_during_shutdown:
                # if board does not respond perform a force shutdown
                self._io_card.press_power_button(10)

        if self.__keep_cable_during_shutdown and mode != "UNKNOWN":
            # actively wait for the shutdown
            self._logger.info("Wait at most %ss to see connection lost during shutdown" % self.__shutdown_timeout)
            mode = self._device.get_boot_mode()
            start_time = time.time()
            while (time.time() - start_time < self.__shutdown_timeout) and (mode != "UNKNOWN"):
                mode = self._device.get_boot_mode()
                time.sleep(1)

            if mode == "UNKNOWN":
                self._logger.info("Wait 15s for shutdown stabilization")
                time.sleep(15)
        else:
            mode = "UNKNOWN"
            self._logger.info("Wait at most %ss to shutdown" % self.__shutdown_timeout)
            time.sleep(self.__shutdown_timeout)

        return mode

    def __turn_on_board(self):
        """
        turn on the board by using press button
        """
        self._logger.info("turn ON board")
        # if inserted cable is a data cable then
        self._io_card.press_power_button(self.pwr_btn_boot)
        self._logger.info("Wait %ss before performing any action" % self.__after_perform_boot_time)
        time.sleep(self.__after_perform_boot_time)

        # we need to plug the cable here:
        if not self.__keep_cable_during_shutdown:
            self._io_card.simulate_insertion(self.__cable_type)

        # actively wait for the boot
        self._logger.info("Wait %ss at mos for booting board" % self.__boot_timeout)
        mode = self._device.get_boot_mode()
        start_time = time.time()
        while (time.time() - start_time < self.__boot_timeout) and (mode == "UNKNOWN"):
            mode = self._device.get_boot_mode()
            time.sleep(1)

        return mode

    def __custom_get_application_logs(self, tag, fail_iter, curr_iteration):
        """
        get the log of a fail boot attempt from the board, zip it and store it on usecase result.
        """
        pulled_files = []
        directory = os.path.join(self._saving_directory,
                                    "crash_iter_" + str(fail_iter) + "_" + time.strftime("%Y-%m-%d_%Hh%M.%S"))

        self._logger.info("try to get logs since failed iteration %s, we are currently in iteration %s" % (fail_iter, curr_iteration))
        # try to pull application log from tag
        if tag != "":
            pulled_files = self._device.get_application_logs(
                    directory, tag)

        # if fail to pull file for any reason then pull all file
        if pulled_files is None or len(pulled_files) == 0:
            # if tag not found pull all logs
            pulled_files = self._device.get_application_logs(
                directory)

        # zip only if there is file to zip
        if pulled_files is not None and len(pulled_files) > 0:
            archive_util.make_archive(directory, "zip",
                                      root_dir=directory)
            time.sleep(1)
            shutil.rmtree(directory, ignore_errors=True)
            if os.path.isdir(directory):
                self._logger.warning("fail do delete tempdir %s" % directory)
