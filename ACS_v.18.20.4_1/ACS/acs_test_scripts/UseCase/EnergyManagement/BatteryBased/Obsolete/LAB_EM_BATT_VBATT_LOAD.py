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
:summary: Energy Management batt vbatt load
:author: vgombert
:since: 12/02/2011
"""
import time
import os

from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from UtilitiesFWK.Utilities import Global
import acs_test_scripts.Utilities.EMUtilities as EMUtil

from ErrorHandling.TestEquipmentException import TestEquipmentException
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.AcsBaseException import AcsBaseException
from ErrorHandling.DeviceException import DeviceException


class LabEmBattVbattLoad(EmUsecaseBase):

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

        # save global config into an attribute in
        # order to retrieve value in set_up method
        self.__global_config = global_config

        # Read LOAD from test case xml file
        self._load = \
            str(self._tc_parameters.get_param_value("LOAD"))
        # Read BATT_MIN_CAPACITY from test case xml file
        # Init fuel gauging param
        self.em_core_module.init_fg_param()
        # Read STRESS_PERIOD from test case xml file
        self.hard_stress_time = \
            int(self._tc_parameters.get_param_value(
                "STRESS_PERIOD"))
        # Read NO_STRESS_PERIOD from test case xml file
        self.soft_stress_time = \
            int(self._tc_parameters.get_param_value(
                "NO_STRESS_PERIOD"))

        if self._load in ["HEAVY", "MEDIUM"]:
            # Read Band from test case xml file (str)
            self._cell_band = str(
                self._tc_parameters.get_param_value("CELL_BAND"))
            # Read CELL_SERVICE from test case xml file
            self._cell_service = \
                str(self._tc_parameters.get_param_value("CELL_SERVICE"))
            # Read TCH_ARFCN from test case xml file
            self._tch_arfcn = int(
                self._tc_parameters.get_param_value("TCH_ARFCN"))
            # Read UPLINK_CHANNEL from test case xml file
            self._uplink_channel = int(
                self._tc_parameters.get_param_value("UPLINK_CHANNEL"))
            # Read CELL_POWER from test case xml file
            self._cell_power = \
                int(self._tc_parameters.get_param_value("CELL_POWER"))

            # Read registrationTimeout from Phone_Catalog.xml
            self._registration_timeout = \
                int(self._dut_config.get("registrationTimeout"))

            # Create cellular network simulator and retrieve 2G APIs
            self._ns = self._em.get_cellular_network_simulator("NETWORK_SIMULATOR1")
            self._ns_2g = self._ns.get_cell_2g()
            self._data_2g = self._ns_2g.get_data()
            self._test_mode_2g = self._ns_2g.get_test_mode()

            # Get Multimedia Parameters
            self._audio_file = self._tc_parameters.get_param_value("AUDIO_FILE")
            self._volume = int(self._tc_parameters.get_param_value("VOLUME"))

            # Get path to multimedia files
            self._multimedia_path = self._device.multimedia_path

            # import special api
            self._audio_api = self._device.get_uecmd("Audio")
            self._system_api = self._device.get_uecmd("System")

        if self._load in ["HEAVY"]:

            # Get FTP server parameters
            server = self.__global_config.benchConfig.\
                get_parameters("WIFI_SERVER")
            self._ftp_ip_address = server.get_param_value("IP")
            self._ftp_username = server.get_param_value("username")
            self._ftp_password = server.get_param_value("password")
            if server.has_parameter("ftp_path"):
                self._ftp_path = server.get_param_value("ftp_path")
            else:
                self._ftp_path = ""

            # get file to download info
            self._dlfilename = os.path.join(
                self._ftp_path,
                self._tc_parameters.get_param_value("DL_FILE"))

            # get wifi parameters
            self._wifi_router_name = None
            self._standard = None
            self._ssid = ""
            self._standard_read = None
            self._security = None
            self._security_read = None
            self._passphrase = None
            self._is_router_name_found = True
            self._is_router_found = False
            supported_wifi_standard = ['a', 'b', 'g', 'n', 'an', 'bg', 'gb',
                                       'bgn', 'ngb', 'n2.4G', 'n5G', 'off']

            bench_config = self.__global_config.benchConfig
            # Retrieve wifi access point
            self._wifi_router_name = \
                str(self._tc_parameters.get_param_value("WIFI_ACCESS_POINT"))

            # if wifi router name is not specified, exit
            if self._wifi_router_name in [None, "", "NONE"]:
                self._is_router_name_found = False

            elif(bench_config.has_parameter(self._wifi_router_name)
                 and self.__global_config.benchConfig.
                 get_parameters(self._wifi_router_name) != ""):

                self._is_router_found = True
                self._wifirouter = self.__global_config.benchConfig.\
                    get_parameters(self._wifi_router_name)

                ssid_read = self._wifirouter.get_param_value("SSID")
                self._standard_read = self._wifirouter.get_param_value("standard")
                self._security_read = self._wifirouter.\
                    get_param_value("WIFI_SECURITY").upper()

                if ssid_read != "":
                    self._ssid = ssid_read

                if self._standard_read in supported_wifi_standard:
                    self._standard = self._standard_read

                # retrieve wifi router parameters
                if self._security_read in ("NONE", "OPEN"):

                    self._logger.debug("OPEN" 'Wifi Security type selected,' +
                                       'getting parameters')
                    self._security = self._security_read

                elif self._security_read in ("WEP", "WPA", "WPA2"):

                    self._security = self._security_read
                    self._logger.debug(str(self._security) +
                                       " Wifi Security selected, getting parameters")

                    passphrase = self._wifirouter.get_param_value("passphrase")
                    self._passphrase = passphrase

        # Initialize EM xml object
        # measurement file
        meas_file_name = os.path.join(self._saving_directory,
                                      "EM_meas_report.xml")
        self.__em_meas_tab = EMUtil.XMLMeasurementFile(meas_file_name)
        # enable Global Measurement file
        name = os.path.join(self._campaign_folder,
                            self._em_cst.GLOBAL_MEAS_FILE)
        self.__em_meas_tab.enable_global_meas(name, self._name)

        # init variables
        self.__total_test = 0
        self._slots_conf = []
        self._data_call_mode = "TEST_MODE_B"
        self.measurement_fail = 0
        self._em_targets = None

#-----------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # Call the UseCaseBase Setup function
        EmUsecaseBase.set_up(self)

        # set usb charging on
        self.em_api.set_usb_charging("on")

        # Check LOAD option
        if self._load not in ["LIGHT", "MEDIUM", "HEAVY"]:
            tmp_txt = "unknown LOAD option value :" + str(self._load)
            self._logger.error(tmp_txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, tmp_txt)

        if self._load in ["HEAVY", "MEDIUM"]:
            # Configure CMU
            # Connect to cellular network simulator
            self._ns.init()

            # Perform Full Preset
            self._ns.perform_full_preset()

            # Set cell band using CELL_BAND parameter
            self._ns_2g.set_band(self._cell_band)

            # Set cell off
            self._ns_2g.set_cell_off()

            # Set cell service using CELL_SERVICE parameter
            self._ns_2g.set_cell_service(self._cell_service)

            # Set Traffic Channel Arfcn using TCH_ARFCN parameter
            self._ns_2g.set_tch_arfcn(self._tch_arfcn)

            # Set cell power using CELL_POWER parameter
            self._ns_2g.set_cell_power(self._cell_power)

            # set data uplink channel
            self._data_2g.set_data_channel(self._uplink_channel)

            # configure cellular network burst slots
            self._slots_conf = self.em_core_module.configure_slot()

            # Set main timeslot to 3 and set slot configs
            self._data_2g.set_custom_multislot_config(
                3, self._slots_conf["DL_STATE"],
                self._slots_conf["DL_VALUE"],
                self._slots_conf["UL_STATE"],
                self._slots_conf["UL_VALUE"])

        if self._load in ["HEAVY"]:
            # Check initialization and raise exception if something went wrong
            if not self._is_router_name_found:
                msg = "No Wifi router has been specified by the user"
                raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG, msg)

            if not self._is_router_found:
                msg = "router name" + self._wifi_router_name + "is missing"
                raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG, msg)

            if self._ssid == "":
                msg = str(self._ssid) + ": wifi ssid is missing"
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

            if self._standard is None:
                # unknown wifi security
                if self._standard_read == "":
                    msg = "wifi standard is missing"
                else:
                    msg = self._standard_read \
                        + ":Unknown wifi standard is missing"
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

            if self._security is None:
                # unknown wifi security
                if self._security_read == "":
                    msg = "missing wifi security"
                else:
                    msg = self._security_read + ": unknown wifi security"
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

            if(self._security in ("WEP", "WPA", "WPA2")
               and self._passphrase in ("", None)):
                msg = "Passphrase %s is missing in bench configuration file"
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # check keys on dict to avoid reading empty files
        msic = self.update_battery_info()

        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_BATT_VBATT_LOAD", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

        # update charger full design value
        if self._em_targets["MSIC_REGISTER_STRESS.BATTERY.CHARGE_NOW"] is not None:
            EMUtil.update_conf(
                self._em_targets["MSIC_REGISTER_STRESS.BATTERY.CHARGE_NOW"],
                "hi_lim", msic["BATTERY"]["CHARGE_FULL_DESIGN"][0],
                "=")
        if self.tc_module is not None:
            # update battery temperature expected depending of TCT
            if self._em_targets["THERMAL_MSIC_REGISTER_STRESS.BATTERY.TEMP"] is not None:
                EMUtil.update_conf(
                    self._em_targets["THERMAL_MSIC_REGISTER_STRESS.BATTERY.TEMP"],
                    ["lo_lim", "hi_lim"], self._tct, "*")

            if self._em_targets["THERMAL_MSIC_REGISTER_NO_STRESS.BATTERY.TEMP"] is not None:
                EMUtil.update_conf(
                    self._em_targets["THERMAL_MSIC_REGISTER_NO_STRESS.BATTERY.TEMP"],
                    ["lo_lim", "hi_lim"], self._tct, "*")

        # Charge battery
        self.em_core_module.monitor_charging(self.em_core_module.batt_max_capacity, self.em_core_module.charge_time,
                              self.__em_meas_tab)

        if self._load in ["HEAVY", "MEDIUM"]:
            # set CMU cell phone ON
            self._data_2g.set_data_cell_on()

        # launch uecmd to help the discharge
        self.phonesystem_api.set_screen_timeout(3600)
        # deactivate set auto brightness
        self.phonesystem_api.set_brightness_mode("manual")
        # set display brightness to max value
        self.phonesystem_api.set_display_brightness(100)

        if self._load in ["HEAVY"]:
            # setup wifi connection
            self.__setup_wifi()

            # connect wifi
            self.networking_api.\
                wifi_connect(self._ssid)

        if self._load in ["HEAVY", "MEDIUM"]:
            # register phone
            self._data_2g.data_register_dut(None,
                                            self._registration_timeout)

        # reset consecutive error
        self.measurement_fail = 0

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        EmUsecaseBase.run_test_body(self)

        self._logger.info("Start to discharge battery until %s%%" %
                          self.em_core_module.batt_min_capacity)
        while (self.batt_capacity > self.em_core_module.batt_min_capacity and
               self.measurement_fail < self._consecutive_meas_error):
            self.stress_discharge(self.hard_stress_time)
            self.soft_discharge(self.soft_stress_time)

        return(self._em_meas_verdict.get_global_result(),
               self._em_meas_verdict.save_data_report_file())

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        # call tear down
        EmUsecaseBase.tear_down(self)

        # retrieve measurement from test
        self.__em_meas_tab.generate_global_file()

        if self._load in ["HEAVY", "MEDIUM"]:
            # set CMU cell phone OFF
            self._data_2g.set_data_cell_off()

            # Disconnect from the equipment (Network simulator)
            self._ns.release()

        # reboot the board to clean it from all setting
        self.em_core_module.reboot_board()

        # clean the board state and retrieve logs
        self.em_core_module.clean_up()

        if  self.is_board_and_acs_ok() and self.batt_capacity != -1:
            self.phonesystem_api.set_screen_timeout(30)
            if self._load in ["HEAVY"]:
                self.networking_api.set_wifi_power("off")
            self.phonesystem_api.clean_daemon_files()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def stress_discharge(self, timeout):
        """
        stress the board during discharge
        """
        start_time = time.time()
        # turn on data cell
        if self._load in ["HEAVY", "MEDIUM"]:
            self._data_2g.set_data_cell_on()

        # launch uecmd only if board is available
        if self.is_board_and_acs_ok():

            try:
                # give write access
                self._device.set_filesystem_rw()
                # stop charging through usb
                self.em_api.set_usb_charging("off")

                # stress cpu
                self.phonesystem_api.stress_cpu("on")
                # stress emmc
                self.phonesystem_api.stress_emmc("on")
                # wake phone
                self.phonesystem_api.wake_screen()

                if self._load in ["HEAVY"]:
                    # connect wifi
                    self.networking_api.\
                        wifi_connect(self._ssid)
                    # start to download  file through ftp
                    self.networking_api.\
                        start_ftp_xfer(self._uecmd_types.XFER_DIRECTIONS.DL,  # pylint: disable=E1101
                                       self._ftp_ip_address,
                                       self._ftp_username,
                                       self._ftp_password,
                                       self._dlfilename,
                                       self._device.get_ftpdir_path(),
                                       True)
                    # set flash
                    self.phonesystem_api.set_torchlight("on")

                if self._load in ["HEAVY", "MEDIUM"]:
                    # set vibration
                    self.phonesystem_api.set_vibration("on")
                    # set music volume
                    self._system_api.adjust_specified_stream_volume("Media", self._volume)
                    # play music
                    self._audio_api.play(self._multimedia_path + self._audio_file, True)

                    # establish data call
                    try:
                        self._data_2g.data_call(self._data_call_mode)
                        self._data_2g.check_data_call_connected(10)
                    except TestEquipmentException as error:
                        self._logger.error(str(error))

                # init failed measurement counter
                self.phone_as_reboot = False
                self.measurement_fail = 0
            except AcsBaseException as e:
                self._logger.error("fail to set hard stress setup : " + str(e))

        # start to discharge
        self._logger.info("Stress discharge cycle during %s" % timeout)
        while (self.batt_capacity > self.em_core_module.batt_min_capacity and
              (time.time() - start_time) < timeout):

            try:
                # try to read measurement
                self.__total_test += 1
                self._logger.info("TEST iteration_%s" % str(self.__total_test))

                # get msic registers value after booting
                msic_reg = self.update_battery_info()
                self.__em_meas_tab.add_dict_measurement(msic_reg)

                # get thermal
                thermal_conf = self.em_api.get_thermal_sensor_info()
                self.__em_meas_tab.add_dict_measurement(thermal_conf)

                self._meas_list.add_dict("MSIC_REGISTER_STRESS", msic_reg)

                # check thermal capabilities only if thermal chamber is used
                if self.tc_module is not None:
                    # Store various information
                    self.__em_meas_tab.add_measurement(
                        [self.tc_module.feed_meas_report()])
                    self._meas_list.add_dict("THERMAL_MSIC_REGISTER_STRESS", msic_reg)
                    self._meas_list.add_dict("THERMAL_CONF_STRESS", thermal_conf)

                # get the bcu state
                bcu = self.em_api.get_bcu_status()
                # Store various information
                self.__em_meas_tab.add_measurement(
                    [("BCU", bcu)])

                if self._load in ["HEAVY", "MEDIUM"]:
                    # get the call state
                    call_state = self._cell_service
                    reg_state = self.modem_api.get_network_registration_status()

                    # Check data call state
                    try:
                        self._data_2g.check_data_call_connected(10)
                        call_state += "_DATA_CALL_CONNECTED"
                    except TestEquipmentException as error:
                        self._logger.error(str(error))
                        call_state += "_DATA_CALL_DISCONNECTED"
                        # try to re - establish data call
                        try:
                            self._data_2g.data_call(self._data_call_mode)
                            self._data_2g.check_data_call_connected(10)
                        except TestEquipmentException as error:
                            self._logger.error(str(error))

                    # get BER
                    ber = "CANT_GET_BER"
                    if reg_state != "unregistered":
                        try:
                            self._test_mode_2g.configure_ber_measurement(200,
                                                                         self._cell_power)
                            ber = self._test_mode_2g.get_ber()
                        except TestEquipmentException as error:
                            self._logger.error(str(error))

                    # Store various information
                    self.__em_meas_tab.add_measurement([("BER", ber)])

                    # Store various information
                    self.__em_meas_tab.add_measurement(
                        [("REGISTRATION", reg_state),
                         ("VOICE_CALL", call_state)])

                    # get multimedia info
                    volume = self._system_api.get_specified_stream_volume("Media")
                    self.__em_meas_tab.add_measurement([("VOLUME", volume)])

                    self._meas_list.add("BER", (ber, "none"))
                    self._meas_list.add("VOLUME", (volume, "none"))

                    # check vibration
                    vibration = self.phonesystem_api.get_vibration_state()
                    self.__em_meas_tab.add_measurement(
                        [("VIBRATION", vibration)])
                    self._meas_list.add("VIBRATION", (vibration, "none"))

                # check system info
                brightness_level = self.phonesystem_api.get_backlight_level()
                cpu_freq = self.phonesystem_api.get_cpu_freq()
                charge_current = self.em_api.get_charger_level()

                self.__em_meas_tab.add_measurement(
                    [("BRIGHTNESS_LEVEL", brightness_level),
                     ("CPU_FREQ", cpu_freq),
                     ("CHARGER_CURRENT", charge_current)])

                self._meas_list.add("BRIGHTNESS_LEVEL", (brightness_level, "none"))
                self._meas_list.add("CPU_FREQ", (cpu_freq, "none"))

                if self._load in ["HEAVY"]:
                    # check ftp xfer info
                    wifi_transfer = self.networking_api.get_ftp_xfer_status()
                    self.__em_meas_tab.add_measurement(
                        [("WIFI_FTP_TRANSFER", wifi_transfer)])
                    self._meas_list.add("WIFI_FTP_TRANSFER", (wifi_transfer, "none"))

                    # restart wifi if necessary
                    if wifi_transfer == "notrunning":
                        self.networking_api.stop_ftp_xfer("id")
                        self.networking_api.\
                            start_ftp_xfer(self._uecmd_types.XFER_DIRECTIONS.DL,  # pylint: disable=E1101
                                           self._ftp_ip_address,
                                           self._ftp_username,
                                           self._ftp_password,
                                           self._dlfilename,
                                           self._device.get_ftpdir_path(),
                                           True)

                # reset consecutive fail
                self.measurement_fail = 0

                # TEMPS TO FORCE USB CHARGING OFF stop charging through usb
                self.em_api.set_usb_charging("off")

                self.phonesystem_api.stress_emmc("off")
                self.phonesystem_api.stress_emmc("on")

            except AcsBaseException as e:
                # Just log error, board will be rebooted in next iteration
                self._logger.error("fail to get measurement: %s" % str(e))
                self.measurement_fail += 1

                # stop the usecase if measurement fail several times.
                if self.measurement_fail >= self._consecutive_meas_error:
                    tmp_txt = "Measurement failed after %s times, stop usecase" % \
                        self._consecutive_meas_error
                    self._logger.error(tmp_txt)
                    if self.batt_voltage > self.vbatt_mos_shutdown or \
                            self.batt_voltage == -1:
                        raise DeviceException(DeviceException.OPERATION_FAILED, tmp_txt)
                    else:
                        self._logger.info("battery must be empty, stop usecase")
                        break
            finally:
                # Store various information
                self.__em_meas_tab.add_measurement(
                    [self.get_time_tuple(),
                     ("COMMENTS", "Discharging with hard stess test"),
                     ("REBOOT", self.phone_as_reboot)])

                # generate em verdict
                self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
                self._em_meas_verdict.judge(ignore_blocked_tc=True)
                self._meas_list.clean()

                # switch to next meas
                self.__em_meas_tab.switch_to_next_meas()
                self.phone_as_reboot = False

            # check the board connection
            self.em_core_module.check_board_connection(1, False)

            # restart flash , audio and call
            try:
                if self.has_board_reboot():
                    self._device.set_filesystem_rw()
                    # set brightness to max value
                    self.phonesystem_api.set_display_brightness(100)
                    # stop charging through usb
                    self.em_api.set_usb_charging("off")
                    self.phonesystem_api.stress_cpu("on")
                    self.phonesystem_api.stress_emmc("on")

                    if self._load in ["HEAVY"]:
                        # set flash
                        self.phonesystem_api.set_torchlight("on")

                        # restart wifi transfer
                        self.networking_api.\
                            wifi_connect(self._ssid)
                        self.networking_api.\
                            start_ftp_xfer(self._uecmd_types.XFER_DIRECTIONS.DL,  # pylint: disable=E1101
                                           self._ftp_ip_address,
                                           self._ftp_username,
                                           self._ftp_password,
                                           self._dlfilename,
                                           self._device.get_ftpdir_path(),
                                           True)

                    if self._load in ["HEAVY", "MEDIUM"]:
                        # launch music
                        time.sleep(self._wait_btwn_cmd)
                        self._system_api.adjust_specified_stream_volume("Media", self._volume)
                        self._audio_api.play(self._multimedia_path + self._audio_file, True)
                        # establish data call
                        try:
                            self._data_2g.data_call(self._data_call_mode)
                            self._data_2g.check_data_call_connected(60)
                        except TestEquipmentException as error:
                            self._logger.error(str(error))

                        # set vibration at the end
                        self.phonesystem_api.set_vibration("on")

            except AcsBaseException as e:
                self._logger.error("fail to relaunch uemcd : " + str(e))

        return Global.SUCCESS, "No actions"

#------------------------------------------------------------------------------

    def soft_discharge(self, timeout):
        """
        stress the board during discharge
        """
        start_time = time.time()

        if self._load in ["HEAVY", "MEDIUM"]:
            # set CMU cell phone OFF
            self._data_2g.set_data_cell_off()

        if self.is_board_and_acs_ok():
            try:
                # try to deactivate
                if self._load in ["HEAVY"]:
                    # stop to download  file through ftp
                    self.networking_api.stop_ftp_xfer("id")
                    self.networking_api.wifi_disconnect(self._ssid)
                    # stop all daemonized process
                    self.phonesystem_api.clean_daemon_files()

                # stop charging through usb
                self.em_api.set_usb_charging("off")

                if self._load in ["HEAVY", "MEDIUM"]:
                    # set vibration
                    self.phonesystem_api.set_vibration("off")
                    # stop  music
                    self._audio_api.stop()

                # stop stress emmc
                self.phonesystem_api.stress_emmc("off")
                # stop stress cpu
                self.phonesystem_api.stress_cpu("off")
                # wake phone
                self.phonesystem_api.wake_screen()

                # init failed measurement counter
                self.phone_as_reboot = False
                self.measurement_fail = 0
            except AcsBaseException as e:
                self._logger.error("fail to set hard stress setup : " + str(e))

        # start to discharge
        self._logger.info("Soft discharge cycle during %s" % timeout)
        while (self.batt_capacity > self.em_core_module.batt_min_capacity and
              ((time.time() - start_time) < timeout)):

            try:
                # try to read measurement
                self.__total_test += 1
                self._logger.info("TEST iteration_%s" % str(self.__total_test))

                # get msic registers value
                msic_reg = self.update_battery_info()
                self.__em_meas_tab.add_dict_measurement(msic_reg)

                # get thermal
                thermal_conf = self.em_api.get_thermal_sensor_info()
                self.__em_meas_tab.add_dict_measurement(thermal_conf)

                # get the bcu state
                bcu = self.em_api.get_bcu_status()
                # Store various information
                self.__em_meas_tab.add_measurement(
                    [("BCU", bcu)])

                self._meas_list.add_dict("MSIC_REGISTER_NO_STRESS", msic_reg)

                if self.tc_module is not None:
                    # Store various information
                    self.__em_meas_tab.add_measurement(
                        [self.tc_module.feed_meas_report()])
                    self._meas_list.add_dict("THERMAL_MSIC_REGISTER_NO_STRESS",
                                             msic_reg)
                    self._meas_list.add_dict("THERMAL_CONF_NO_STRESS",
                                             thermal_conf)

                # reset consecutive fail
                self.measurement_fail = 0

                # TEMPS TO FORCE USB CHARGING OFF stop charging through usb
                self.em_api.set_usb_charging("off")

            except AcsBaseException as e:
                # Just log error, board will be rebooted in next iteration
                self._logger.error("fail to get measurement: " + str(e))
                self.measurement_fail += 1

                # stop the usecase if measurement fail several times.
                if self.measurement_fail >= self._consecutive_meas_error:
                    tmp_txt = "Measurement failed after %s times, stop usecase" % \
                        self._consecutive_meas_error
                    self._logger.error(tmp_txt)
                    if self.batt_voltage > self.vbatt_mos_shutdown or \
                            self.batt_voltage == -1:
                        raise DeviceException(DeviceException.OPERATION_FAILED, tmp_txt)
                    else:
                        self._logger.info("battery must be empty, stop usecase")
                        break
            finally:
                # Store various information
                self.__em_meas_tab.add_measurement(
                    [self.get_time_tuple(),
                     ("COMMENTS", "Discharging with soft stress test"),
                     ("REBOOT", self.phone_as_reboot)])

                # switch to next meas
                self.__em_meas_tab.switch_to_next_meas()
                self.phone_as_reboot = False

            # check the board connection
            self.em_core_module.check_board_connection(1, False)

            # restart flash , audio and call
            try:
                if self.has_board_reboot():
                    # TEMPS TO FORCE USB CHARGING OFF stop charging through usb
                    self.em_api.set_usb_charging("off")

            except AcsBaseException as e:
                self._logger.error("fail to relaunch uemcd : " + str(e))

#------------------------------------------------------------------------------

    def __setup_wifi(self):
        """
        setup wifi connection but does not connect it.
        """
        # turn off wifi
        self.networking_api.set_wifi_power("off")
        time.sleep(self._wait_btwn_cmd)
        # turn on wifi
        self.networking_api.set_wifi_power("on")
        time.sleep(self._wait_btwn_cmd)
        # disconnect all wifi
        self.networking_api.wifi_disconnect_all()

        time.sleep(self._wait_btwn_cmd)

        if self._security not in ("NONE", "OPEN"):

            self._logger.info("Setting passphrase " + str(self._passphrase) +
                              " for " + str(self._ssid) + "...")
            self.networking_api.\
                set_wificonfiguration(self._ssid,
                                      self._passphrase,
                                      self._security)

            time.sleep(self._wait_btwn_cmd)
