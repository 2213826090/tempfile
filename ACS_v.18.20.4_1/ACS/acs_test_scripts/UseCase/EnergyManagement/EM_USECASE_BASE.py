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
:summary: this is the base of all EM usecase implemented only
common code to power supply and non power supply test
:author: vgomberx
:since: 07/29/2013
"""

import sys
import time
import os
import shutil
from distutils import archive_util
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from Core.Report.SecondaryTestReport import SecondaryTestReport
from acs_test_scripts.UseCase.EnergyManagement.UcModule.OverMind import OverMind
from acs_test_scripts.UseCase.EnergyManagement.UcModule.ThermalChamberModule import ThermalChamberModule
from acs_test_scripts.UseCase.EnergyManagement.UcModule.UcBatteryModule import UcBatteryModule
from acs_test_scripts.UseCase.EnergyManagement.UcModule.UcPowerSupplyModule import UcPowerSupplyModule
from acs_test_scripts.Equipment.IOCards.Interface.IIOCard import IIOCard
from acs_test_scripts.Utilities.CommunicationUtilities import ConfigsParser
from acs_test_scripts.Utilities.EMUtilities import EMConstant
import acs_test_scripts.Utilities.EMUtilities as EMUtil
from Core.Report.Live.LiveReporting import LiveReporting
from UtilitiesFWK.Utilities import Global


class EmUsecaseBase(UseCaseBase):

    """
    Lab Energy Management base class.
    """
    # by default test are set to be run with only normal battery
    DEDICATED_BENCH = "BASIC_BENCH"

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)
        self._overmind = None
        # init em stamp
        self.em_stamp = self._name
        self._uc_id_logged = False
        # override IO CARD with a not None type but not implemented type
        if self._io_card is None:
            self._io_card = IIOCard()

        # get em parameter from device catalog here
        self.phone_info = self._device.get_em_parameters()
        # feed the overmind
        self._overmind = OverMind()
        self._overmind.init(device=self._device,
                                 tc_parameters=self._tc_parameters,
                                 equipment_manager=self._em,
                                 bench_config=global_config.benchConfig,
                                 dut_config=self._dut_config)

        # Get uecmd api
        self.em_api = self._device.get_uecmd("EnergyManagement")
        self.phonesystem_api = self._device.get_uecmd("PhoneSystem")
        self.modem_api = self._device.get_uecmd("Modem")
        self.voicecall_api = self._device.get_uecmd("VoiceCall")
        self.networking_api = self._device.get_uecmd("Networking")
        self.bt_api = self._device.get_uecmd("LocalConnectivity")

        # init temperature chamber
        self._tct = self._tc_parameters.get_param_value("TCT", default_cast_type=int)
        if self._tct is not None:
            self.tc_module = ThermalChamberModule(self._tct)
        else:
            self.tc_module = None

        # Each raw measurement data & xml files are
        # stored in a folder bearing the name
        # of the TC + a time stamp
        directory_name = self._name + "_" + time.strftime("%Y-%m-%d_%Hh%M.%S")
        report_tree = global_config.campaignConfig.get("campaignReportTree")
        self._saving_directory = report_tree.create_subfolder(directory_name)
        campaign_pass_rate = float(global_config.campaignConfig.get("targetB2bPassRate"))
        # resume file
        self._campaign_folder = report_tree.get_report_path()
        # verdict file
        self._em_meas_verdict = EMUtil.EMMeasurement(self._saving_directory, campaign_pass_rate)

        # Call ConfigsParser to parse Energy_Management
        external_target_path = self._device.get_config("EMExternalTarget")
        if external_target_path is not None and os.path.exists(external_target_path):
            # Call ConfigsParser to parse Energy_Management
            self._logger.info("[EM BASE] External EM target file found : %s" % external_target_path)
            self._target_file = ConfigsParser(external_target_path, config_folder="")
        else:
            # If no external target file , parse default one
            self._logger.info("[EM BASE] No external target file found, use default EM target file")
            self._target_file = ConfigsParser("Energy_Management")

        # Enable ACS secondary reports
        self._secondary_report = SecondaryTestReport(
            self._device.get_report_tree().get_report_path())

        # init variables
        # var for robustness
        self.start_time = time.time()
        self.__phone_date = ""
        self._ambient_temperature = 25
        self.phone_as_reboot = False
        self._consecutive_meas_error = 5
        # pylint: disable=W0212
        # Ignore used of protected member, will be change if a getter is created later
        self.pwr_btn_boot = self._device._prss_pwr_btn_time_switch_on
        self.pwr_btn_off = self._device._prss_pw_btn_time_switch_off

        # INIT PARAMETERS HERE
        # param related to usecase
        self._em_cst = EMUtil.EMConstant
        self._em_targets = None
        # add usb sleep for avoiding to lost connection during usb recognition
        self.usb_sleep = self._device._usb_sleep_duration  # pylint: disable=w0212

        # param related to device
        self.vbatt_mos_shutdown = self.phone_info["BATTERY"]["VBATT_MOS_SHUTDOWN"] + 0.1
        self.vbatt_mos_boot = self.phone_info["BATTERY"]["VBATT_MOS_BOOT"] + 0.1

        # param that are designed to be updated
        self.batt_capacity = -1
        self.batt_voltage = -1
        # measurement list where we store measurement inside
        self._meas_list = EMUtil.MeasurementList()

        # init core module at the end
        self.em_core_module = None
        bench_type = self._tc_parameters.get_param_value("BENCH_TYPE", self.DEDICATED_BENCH).upper()
        if bench_type == "POWER_SUPPLY_BENCH":
            self.em_core_module = UcPowerSupplyModule(self)
        elif bench_type == "BATTERY_BENCH":
            self.em_core_module = UcBatteryModule(self)
        self._logger.info("Loaded EM TEST_SCRIPT configuration for %s" % self.DEDICATED_BENCH)
        # get the list of TCD to test, it can have several entries
        # if the parameter is missing or empty ( meaning None) the old verdict system will be used.
        self.tcd_to_test = self._tc_parameters.get_param_value(EMConstant.TC_PARAMETER_TO_CHECK, "")
        if type(self.tcd_to_test) is str:
            self.tcd_to_test = filter(None, self.tcd_to_test.split(";"))
            self.tcd_to_test = map(str.strip, self.tcd_to_test)

#------------------------------------------------------------------------------

    def get_application_logs(self):
        """
        get the log from the board, zip it and store it on usecase result.
        """
        pulled_files = []
        directory = os.path.join(self._saving_directory,
                                 "aplogs_" + time.strftime("%Y-%m-%d_%Hh%M.%S"))
        # create the destination directory
        if not os.path.exists(directory):
            os.mkdir(directory)

        # try to pull application log from tag
        if self._uc_id_logged:
            pulled_files = self._device.get_application_logs(
                directory, self.em_stamp)

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

#------------------------------------------------------------------------------

    def get_time_tuple(self):
        """
        return a tuple for logging time in xml measurement.
        Time will be evaluate in hours.

        :rtype: tuple
        :return: (str name, float value, str unit)
        """
        result = (self._em_cst.TIME_TAG,
                  (time.time() - self.start_time) / 3600,
                  self._em_cst.HR)
        return result

#------------------------------------------------------------------------------

    def get_time_from_board(self):
        """
        get the time from the board itself.

        :rtype: int
        :return: time taken from the board in seconds
        """
        result = self.phonesystem_api.get_date()
        time_tuple = time.strptime(result, "%Y %m-%d %H:%M:%S")
        return int(time.mktime(time_tuple))

#------------------------------------------------------------------------------
    def update_battery_info(self):
        """
        launch command to get battery info , return them as dict
        also update protected var that target voltage & capacity

        :rtype: dict
        :return: uevent charger & battery info
        """
        result = self.em_api.get_msic_registers()
        self.batt_capacity = result["BATTERY"]["CAPACITY"][0]
        self.batt_voltage = result["BATTERY"]["VOLTAGE"][0]
        return result

#------------------------------------------------------------------------------

    def is_board_and_acs_ok(self):
        """
        check if board is alive and ACS connected

        :rtype: bool
        :return: True if all is ok to communicate with board, False otherwise
        """
        result = False
        if self._device.get_boot_mode() == "MOS" and self._device.is_available() :
            result = True

        return result

    def has_board_reboot(self):
        """
        check if board is alive and has rebooted

        :rtype: bool
        :return: True if board rebooted, False otherwise
        """
        result = False
        if self.phone_as_reboot and self._device.get_state() == "alive":
            result = True

        return result
#------------------------------------------------------------------------------

    def _wait_smart_time(self, origin, time_used_for_schedule_cmd):
        """
        Function used to wait a judicious time and let the read of the msic register to be executed.
        """
        now = time.time()
        safety_timer = 8
        if now < (origin + time_used_for_schedule_cmd + safety_timer):

            time2wait = int((origin + time_used_for_schedule_cmd + safety_timer)
                            - now)

            if time2wait > time_used_for_schedule_cmd + safety_timer:
                # this should never occurs
                self._logger.warning("Timer computing error " +
                                     "(time used for schedule cmd: %d)"
                                     % time_used_for_schedule_cmd)
                time2wait = time_used_for_schedule_cmd + safety_timer

            self._logger.debug("waiting %s second(s) remaining to elapse the %s s timer"
                               % (time2wait, time_used_for_schedule_cmd))
            time.sleep(time2wait)

        else:
            self._logger.debug("waiting 0 second(s) to elapse the %s s timer"
                               % time_used_for_schedule_cmd)

#------------------------------------------------------------------------------

    def set_up(self):
        """
        setup is designed to be done only once
        put here settings that persist all along a test like :
         - equipment connection and configuration.
         - turning on feature
        """
        # before all, update and inject EM stamp
        self.em_stamp = self._name + time.strftime("-%Y-%m-%d_%Hh%M.%S")
        self._uc_id_logged = self._device.inject_device_log("i", self._em_cst.EM_INJECT_TAG, self.em_stamp)
        # Call the UseCaseBase Setup function
        UseCaseBase.set_up(self)
        self.em_core_module.set_up()

        # temperature settings
        if self.tc_module is not None and self.tc_module.is_default_test():
            self.tc_module.set_up_eq()
            self.tc_module.adjust_temp_according_to_test_value()
        # one shot measurement
        if self._device.get_state() == "alive":
            # reset reboot var
            self.phone_as_reboot = False
        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def run_test(self):
        """
        the heart of your test.
        measurement should be done in this function.
        No test clean should be on it except if it is need by the test steps.
        """
        UseCaseBase.run_test(self)

        # temperature settings
        if self.tc_module is not None and self.tc_module.is_default_test():
            self.tc_module.adjust_temp_according_to_current_value()

        # implementation for B2B continuous
        # Do before_runtest without a try, if it can't be done then the DUT must be KO
        self.run_test_start()
        result = Global.FAILURE
        result_msg = ""
        import traceback
        try:
            result, result_msg = self.run_test_body()
        # the warning is known and left here to remember it
        except Exception as ee:
            _, exc_tb = sys.exc_info()[1:]
            self._logger.error("Error happen in RUN TEST BODY : %s" % str(ee))
            self._logger.error(str(traceback.format_tb(exc_tb)))
            self._logger.error("Trying to execute RUN TEST STOP before finishing test")
            result_msg = str(ee)
        finally:
            # in any case do a cleaning
            try:
                self.run_test_stop()
                # the warning is known and left here to remember it
            except Exception as e:
                _, exc_tb = sys.exc_info()[1:]
                self._logger.error("Exception during RUN TEST BODY: %s" % str(e))
                self._logger.error(str(traceback.format_tb(exc_tb)))

        return result, result_msg

    def run_test_start(self):
        """
        put here non persistent settings like:
        - feature which need to be set again after a reboot
        - initialization of a boolean
        - establish a call
        """
        # Reset the global result
        self._logger.debug("\tRUN TEST BEGIN")
        self._em_meas_verdict.reset_current_verdict()

    def run_test_body(self):
        """
        the heart of your test.
        measurement should be done in this function.
        No test clean should be on it except if it is need by the test steps.
        """
        self._logger.debug("\tRUN TEST BODY")
        return Global.SUCCESS, "No actions"

    def run_test_stop(self):
        """
        put here temporary cleaning like :
        - emptying folder
        - reboot the board if KO
        - equipment settings necessary to re apply settings on before_runtest
        - verdict computing
        """
        self._logger.debug("\tRUN TEST END")

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        tear down is here to clean your test for next testcase
        put here final cleaning like:
        - equipment release
        - board charging

        """
        # Call the UseCaseBase tear down function
        UseCaseBase.tear_down(self)
        self.em_core_module.tear_down()

        # sink last measurement and reset meas_dict
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets, True)
        self._em_meas_verdict.judge()

        # feed awr secondary reports
        result = self._em_meas_verdict.get_local_result_v2()
        self._secondary_report.add_result_from_dict(result, self.get_name(), self.tc_order)
        # feed TCR
        LiveReporting.instance().update_running_tc_info(test_info=result)

        if self.tc_module is not None and self.tc_module.is_default_test():
            self.tc_module.release_eq()

        return Global.SUCCESS, "No errors"


    def finalize(self):
        UseCaseBase.finalize(self)
        if self._overmind is not None:
            del self._overmind

        return Global.SUCCESS, "No errors"
