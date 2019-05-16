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
:summary: Use Case Base for power measurement
:since: 13/08/2010
:author: ssa & dgo
"""

import time
import os
import shutil
from acs_test_scripts.Utilities.PnPUtilities import PnPConfiguration
from acs_test_scripts.Utilities.PnPUtilities import PnPResults

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from Core.PathManager import Paths

class LabPwrMeasBase(UseCaseBase):

    """
    Lab Power Measurement Base class.
    """
    _sleep_duration = None
    _settle_time = None
    _is_raw_data_saved = None
    _is_power_calculation_needed = None

    def __init__(self, tc_name, global_config):
        """
        Constructor

        :type tc_name: BaseConf
        :param tc_name: Configuration of the usecase

        :type global_config: Dictionnary
        :param global_config: Global configuration of the campaign
        """

        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        self.global_config = global_config

        # Initialize variables
        self._sleep_mode = None
        self._audio_file = None
        self._volume = 10
        self.__screenshot = []
        val_screen = self._tc_parameters.get_param_value("SCREENSHOT")
        self.__display = self._tc_parameters.get_param_value("DISPLAY")
        if val_screen is not None:
            self.__screenshot = [val.lower().strip() for val in val_screen.split(",")]

        # Get the campaign report tree object
        self._report_tree = global_config.campaignConfig.get("campaignReportTree")

        # Each raw measurement data & xml files are
        # stored in a folder bearing the name
        # of the TC + a time stamp
        self._tc_name = os.path.basename(self.get_name())
        self._tc_date = ""

        self.__finish_test = False

        # Initialize power technology dictionary to off
        self._technology_power = []

        self.__adbConnectionTimeout = self._device.get_config("adbConnectTimeout", 30, float)

        # If the device was disconnected before due to an error
        # we must reconnect it explicitly at the beginning of the test
        # else the commands will fail and the test will be blocked
        self._system_api = self._device.get_uecmd("System")
        return_code = self._system_api.wait_for_device(self.__adbConnectionTimeout)
        if not return_code:
            time.sleep(30)

        if not self._device.is_available():
            self._device.connect_board()

        # Instantiate generic UECmd for all power measurement Ucs
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")
        self._networking_api = self._device.get_uecmd("Networking")
        self._localization_api = self._device.get_uecmd("Location")
        self._localconnectivity_api = self._device.get_uecmd("LocalConnectivity")
        self._audio_api = self._device.get_uecmd("Audio")
        self._video_api = self._device.get_uecmd("Video")
        self._sysdebug_apis = self._device.get_uecmd("SysDebug")
        self._sleep_mode_api = self._device.get_uecmd("SleepMode")

        # Initialize object and keys used for parsing of pnp target file
        self._patlib = None
        self._verdict_rail = None

        # Set target and Failure str value to empty str
        self._test_meas_duration = 180

        self.__failure_file = os.path.join(self._execution_config_path,
                                           self._device.get_config("FailureFile"))
        self.__target_file = os.path.join(self._execution_config_path,
                                          self._device.get_config("TargetFile"))
        self.__adbConnectionTimeout = self._device.get_config("adbConnectTimeout", 10, float)
        self.__usbReplugRetries = self._device.get_config("usbReplugRetries", 1, int)

        self._pnp_config = PnPConfiguration(self._dut_config.get("Name"),
                                            os.path.basename(self.get_name()))

        self._music_player = None
        self._device_uptime_begin = None

#------------------------------------------------------------------------------

    def _retrieve_sleep_duration(self):
        """
        Get sleep duration before making the measurement with PatLib.
        """
        sleep_parameter = self._tc_parameters.get_param_value("SLEEP_DURATION")
        if sleep_parameter is not None and sleep_parameter != "" and sleep_parameter.isdigit():
            self._sleep_duration = int(sleep_parameter)
        else:
            # in seconds
            self._sleep_duration = 5
            self._logger.warning("Sleep duration defaulted to %d seconds" % self._sleep_duration)

#------------------------------------------------------------------------------

    def _retrieve_settle_time(self):
        """
        Settle time only for residency/current measure UCs
        """
        settle_time = self._tc_parameters.get_param_value("SETTLE_TIME")
        if settle_time is not None and settle_time != "" and settle_time.isdigit():
            self._settle_time = int(settle_time)
        else:
            self._settle_time = 60
            self._logger.warning("Settle time defaulted to %d seconds" % self._settle_time)

#------------------------------------------------------------------------------
    def _retrieve_pat_parameters(self):
        """
        Get the rail to compute verdict.

        Try to read the verdict from Bench_Config.
        If it's not found, the verdict will be read in the UseCase configuration.
        """
        pat = self.global_config.benchConfig.get_parameters("POWER_ANALYZER_TOOL")
        self._verdict_rail = pat.get_param_value("VerdictRail")
        self._logger.debug("PAT Verdict rail from BenchConfig is %s" % self._verdict_rail)

        patlib = self._pnp_config.get("patlib")
        verdict_rail = patlib.get("verdict_rail")
        if patlib and verdict_rail:
            self._verdict_rail = verdict_rail
            self._logger.warning("PAT Verdict rail from Pnp_Config is %s" %
                                 self._verdict_rail)

        if not self._verdict_rail:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "verdict rail is not set")

#------------------------------------------------------------------------------

    def _get_patconf_file(self):
        """
        Get the PAT configuration file

        Try to read the PAT configuration file from Bench_Config.
        If it's not found, the configuration file will be read in the
        UseCase configuration.

        :rtype: String
        :return: PAT configuration file
        """
        pat = self.global_config.benchConfig.get_parameters("POWER_ANALYZER_TOOL")
        conf_file = pat.get_param_value("ConfFile")
        self._logger.debug("PAT configuration file from BenchConfig is %s" % conf_file)

        patlib = self._pnp_config.get("patlib")
        patconffile = patlib.get("config_file")
        if patlib and patconffile:
            conf_file = patconffile
            self._logger.warning("PAT configuration file from Pnp_Config is %s" % conf_file)

        if not conf_file:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "Pat ConfFile is not set : %s" % conf_file)

        return os.path.join(self._execution_config_path, conf_file)

#------------------------------------------------------------------------------

    def _retrieve_test_duration(self):
        """
        Get the test duration

        :rtype: Integer
        :return: Test duration
        """
        test_duration = self._tc_parameters.get_param_value("TEST_DURATION")
        if test_duration and test_duration.isdigit():
            self._logger.debug("Set test duration for measure : %s" % test_duration)
            self._test_meas_duration = int(test_duration)

        if not test_duration:
            self._logger.warning("Set test duration to default value : %d" %
                                 self._test_meas_duration)

        return self._test_meas_duration

#------------------------------------------------------------------------------

    def _configure_pat(self):
        """
        Configure Pat from Pat Conf file
        """
        self._patlib = self._em.get_power_analyzer_tool("POWER_ANALYZER_TOOL")

        patconf = self._get_patconf_file()
        self._patlib.configure(patconf)

#------------------------------------------------------------------------------

    def _switch_off(self):
        """
        Control USB from I/O Card: disconnect USB
        """

        # Update device uptime
        updated, self._device_uptime_begin = self._device._update_device_up_state(0)
        if not updated:
            self._device_uptime_begin = None

        # If No I/O Card : do nothing
        if self._io_card is None:
            self._sysdebug_apis.reset() # for debug tests without iocard
            self._logger.warning("_switch_off : No IO_CARD detected")
            return

        # Close connection to the board
        self._device.disconnect_board()

        if self._sleep_mode and "s3" not in self._sleep_mode.lower():
            sleep_parameter = int(self._tc_parameters.get_param_value("SLEEP_DURATION"))
            self._sysdebug_apis.reset(sleep_parameter)
        else:
            self._sysdebug_apis.reset()

        # Unplug the USB
        self._io_card.usb_host_pc_connector(False)
        # unplug wall charger only if it is AC_CHGR
        if self._device.get_default_wall_charger() == self._io_card.AC_CHGR:
            # Unplug wall charger
            self._io_card.wall_charger_connector(False)

#------------------------------------------------------------------------------

    def _switch_on(self):
        """
        Control USB from I/O Card: Connect USB
        """
        # If No I/O Card : do nothing
        if self._io_card is not None:
            ret_code = False
            for cnt in range (0, self.__usbReplugRetries + 1):
                # plug wall charger only if it is AC_CHGR
                if self._device.get_default_wall_charger() == self._io_card.AC_CHGR:
                    # Plug wall charger
                    self._io_card.wall_charger_connector(True)
                # Plug the USB
                self._io_card.usb_host_pc_connector(True)

                ret_code = self._system_api.wait_for_device(self.__adbConnectionTimeout)
                if not ret_code:
                    if cnt < self.__usbReplugRetries:
                        self._logger.warning("timeout on wait-for-device, trying to unplug/replug (try %s/%s)" %
                                          (str(cnt + 1), str(self.__usbReplugRetries)))
                        self._io_card.usb_host_pc_connector(False)
                        # Unplug wall charger only if it is AC_CHGR
                        if self._device.get_default_wall_charger() == self._io_card.AC_CHGR:
                            self._io_card.wall_charger_connector(False)
                        time.sleep(10)
                    continue

                self._logger.debug("device retrieved after %s tries" % str(cnt + 1))
                self._sysdebug_apis.fetch()
                break

            if not ret_code:
                raise DeviceException(DeviceException.OPERATION_FAILED,
                                      "Could not retrieve the device after %s plug/unplug cycles" %
                                      str(self.__usbReplugRetries))

        else:
            self._logger.warning("_switch_on : No IO_CARD detected")
            self._sysdebug_apis.fetch() # for debug tests without iocard

        if not self._device.is_available():
            # Open connection to the board
            self._device.connect_board()

        if self._device_uptime_begin:
            updated, uptime = self._device._update_device_up_state(self._device_uptime_begin)
            if updated and not self._device.is_up:
                self._logger.warning("the device uptime was %s before the measurement, %s now !"
                                      % (str(self._device_uptime_begin), str(uptime)))
                raise DeviceException(DeviceException.OPERATION_FAILED,
                                      "Device rebooted during the measurement")

#------------------------------------------------------------------------------

    def _stabilize(self):
        """
        Stabilization of the board
        """
        # Wait so platform sleeps in the right mode (USB unplugged)
        self._logger.info("Stabilize board current state for %d seconds ...",
                          self._sleep_duration)
        time.sleep(self._sleep_duration)

#------------------------------------------------------------------------------

    def _start_power_measure(self):
        """
        Start Pat power measurement acquisition
        """
        self._patlib.start_acquisition(True)

#------------------------------------------------------------------------------

    def _stop_power_measure(self):
        """
        Start Pat power measurement acquisition
        """
        self._logger.info("Stop power measurement acquisition")
        self._patlib.stop_acquisition()

#------------------------------------------------------------------------------

    def _set_technologies(self, technologies_on):
        """
        Enable technologies (Cellular, Wifi, Bluetooth, GPS, FM) on the phone
        :type technologies_on: str array
        :param technologies_on: List of technologies to activate if available
                                - cellular
                                - wifi
                                - bluetooth
                                - gps
                                - fm
        """

        # Get available technologies installed on CDK
        available_technologies = \
            self._networking_api.get_available_technologies()

        # dictionary of function that turn on specific technology
        uecmd_list = {"cellular": self._networking_api.set_flight_mode,
                      "wifi": self._networking_api.set_wifi_power,
                      "bluetooth": self._localconnectivity_api.set_bt_power,
                      "gps": self._localization_api.set_gps_power,
                      "fm": self._phonesystem_api.set_fm_power}

        # activate/deativate each technology accordingly
        for technology in available_technologies:

            technology = str(technology).lower()

            # Activate/Deactivate technology modules
            if technology in uecmd_list.keys():
                if technology in technologies_on:
                    self._logger.info("Switch ON %s technology ..." % technology)
                    if technology == "cellular":
                        uecmd_list[technology]("off")
                    else:
                        uecmd_list[technology]("on")
                else:
                    self._logger.info("Switch OFF %s technology ..." % technology)
                    if technology == "cellular":
                        uecmd_list[technology]("on")
                    else:
                        uecmd_list[technology]("off")
            else:
                raise AcsConfigException(AcsConfigException.FEATURE_NOT_IMPLEMENTED,
                                         "No UECmd for technology %s " % technology)

#------------------------------------------------------------------------------

    def __capture_screenshot(self):
        """
        capture a screenshot of the board
        """
        # create a sub folder in the report folder which contains screenshots
        result_folder = "SCREEN/%s" % self._tc_name
        report_dir = self._device.get_report_tree()
        report_dir.create_subfolder(result_folder)
        self.__base_report_path = report_dir.get_subfolder_path(result_folder)
        filename = self.__base_report_path + "/" + time.strftime("%Y%m%d.%H%M") + ".png"
        self._device.screenshot(filename=filename)

#------------------------------------------------------------------------------

    def _run_test_begin(self):
        """
        Action before starting power measurement
        """
        pass

#------------------------------------------------------------------------------

    def _verdict(self):
        """
        Compute verdict for the test
        """
        attributes = {
            "id": self._tc_name,
            "date": self._tc_date,
            "residency_mode": self._sleep_mode_api.get_sleep_mode(),
            "duration": str(self._test_meas_duration),
            "verdict_rail": str(self._verdict_rail),
            "verdict": "UNDEFINED"
        }
        results = PnPResults(self._report_tree,
                             self._dut_config.get("Name"),
                             self.__failure_file,
                             self.__target_file,
                             attributes)

        raw_pat_file = None
        dat_pat_file = None
        js_out_file = None
        if self._is_raw_data_saved:
            raw_pat_file = os.path.join(self._report_tree.get_report_path(),
                                        "PowerMeasurementData_%s_%s.xml" %
                                        (self._tc_date.replace(" ", "_").replace(":", ""), self._tc_name))
            dat_pat_file = os.path.join(self._report_tree.get_report_path(),
                                        "PowerMeasurementData_%s_%s" %
                                        (self._tc_date.replace(" ", "_").replace(":", ""), self._tc_name),
                                        self._verdict_rail + ".dat")
            js_out_file = os.path.join(self._report_tree.get_report_path(),
                                        "PowerMeasurementData_%s_%s" %
                                        (self._tc_date.replace(" ", "_").replace(":", ""), self._tc_name),
                                        "graph_values.js")

        # Add power measure only if test has finished
        if self.__finish_test:
            results.append(self._patlib.report(self._is_power_calculation_needed, raw_pat_file, dat_pat_file, js_out_file))

            # Add power_graph.html into folder
            if js_out_file:
                srcname = os.path.join(Paths.TEST_SUITES, "FT", "pnp", "REPORT", "power_graph.html")
                destname = os.path.join(self._report_tree.get_report_path(),
                                        "PowerMeasurementData_%s_%s" %
                                        (self._tc_date.replace(" ", "_").replace(":", ""), self._tc_name))
                shutil.copy(srcname, destname)

        results.append(self._sysdebug_apis.report())

        ver_pwr, msg_pwr = results.get_power_verdict(self._verdict_rail)
        ver_res, msg_res = results.get_residency_verdict(self._sleep_mode_api.get_sleep_mode(),
                                                         self._sleep_duration,
                                                         self._test_meas_duration)

        if Global.FAILURE in [ver_pwr, ver_res]:
            results.update({"verdict": "FAIL"})
        else:
            results.update({"verdict": "PASS"})

        results.write()

        if Global.FAILURE in [ver_pwr, ver_res]:
            return Global.FAILURE, msg_pwr + ";" + msg_res
        else:
            return Global.SUCCESS, msg_pwr + ";" + msg_res

#------------------------------------------------------------------------------

    def _run_test_end(self):
        """
        Action to do at the end of run_test step : stop power measurement,
        and get verdict and comments of the test.
        """
        if self._sleep_mode is not None:
            # Clear sleep mode for platform
            # It is called before base teardown to guarantee that the sleep mode is cleared
            self._sleep_mode_api.clear()

#------------------------------------------------------------------------------

    def _wait(self):
        """
        Wait until the end of the measure
        """
        # Wait during power measurement acquisition given from PAT config file
        time.sleep(self._test_meas_duration)

#------------------------------------------------------------------------------
    def _finalize_set_up(self):
        """
        """
        pass

    def set_up(self):
        """
        Set up the test configuration
        """
        # Call UseCase base Setup function
        UseCaseBase.set_up(self)

        sysdbg_modules_config = self._tc_parameters.get_param_value("SYSDEBUG_MODULES")
        self._sysdebug_apis.init(sysdbg_modules_config)

        self._retrieve_pat_parameters()
        self._retrieve_test_duration()
        self._retrieve_sleep_duration()
        self._retrieve_settle_time()

        # Get indication whether we should save or not PatLib raw data
        self._is_raw_data_saved = self._tc_parameters.get_pat_raw_data_saving_mode()

        # Get indication whether we should ask PatLib to calculate power
        # from voltage & current data
        self._is_power_calculation_needed = self._tc_parameters.get_pat_power_calculation_mode()

        # Configure Pat from Pat Conf file
        self._configure_pat()

        # Set needed technologies
        self._set_technologies(self._technology_power)

        # Ensure to not have any previous lock
        self._phonesystem_api.clear_pwr_lock()

        self._finalize_set_up()

        if self._sleep_mode is not None:
            # Initialize sleep mode for platform
            self._sleep_mode_api.init(self._sleep_mode, self._settle_time,
                                      self._audio_file, self._volume, self._music_player)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        UseCaseBase.run_test(self)

        self._tc_date = time.strftime("%Y-%m-%d %H:%M:%S")

        while not self._sysdebug_apis.synchronize():
            time.sleep(10)

        self._run_test_begin()
        if "before" in self.__screenshot:
            self.__capture_screenshot()
        if self.__display is not None and self.__display.lower() == "off":
            self._phonesystem_api.display_off()
        self._switch_off()
        self._stabilize()
        self._sysdebug_apis.start()

        self._logger.info("Start power measurement acquisition for %d seconds",
                          self._test_meas_duration)
        self._start_power_measure()

        verdict = (None, None)
        try:
            self._wait()
            self.__finish_test = True
        finally:
            # Stop Pat power measurement acquisition
            self._stop_power_measure()
            self._sysdebug_apis.stop()

            self._switch_on()

            if "after" in self.__screenshot:
                self.__capture_screenshot()

            self._run_test_end()
            verdict = self._verdict()

        return verdict
