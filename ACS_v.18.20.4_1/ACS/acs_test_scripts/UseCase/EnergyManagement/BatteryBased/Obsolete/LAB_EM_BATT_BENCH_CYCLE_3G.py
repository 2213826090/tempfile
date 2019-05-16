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
:summary: Energy Management Battery Monitor - Bench cycle in 3G
:author: vgombert
:since: 08/23/2011
"""
import time
import os
import acs_test_scripts.Utilities.RegistrationUtilities as RegUtil
import acs_test_scripts.Utilities.EMUtilities as EMUtil

from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from UtilitiesFWK.Utilities import Global, get_conversion_toolbox
from ErrorHandling.AcsBaseException import AcsBaseException
from ErrorHandling.DeviceException import DeviceException


class LabEmBattBenchCycle3g(EmUsecaseBase):

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
        self.em_core_module.init_fg_param()

        # Read Band from test case xml file (str)
        self._cell_band = str(self._tc_parameters.get_param_value("CELL_BAND"))

        # Read TCH_ARFCN from test case xml file
        self._ul_uarfcn = int(self._tc_parameters.get_param_value("UL_UARFCN"))

        # Read mobile power from test case xml file (int)
        self._mobile_power = int(self._tc_parameters.get_param_value("MOBILE_POWER"))

        # Read CELL_POWER from test case xml file
        self._cell_power = \
            int(self._tc_parameters.get_param_value("CELL_POWER"))

        # Instantiate ConversionToolBox into conv_toolbox object
        self._conversion_toolbox = get_conversion_toolbox()

        # Read registrationTimeout from Phone_Catalog.xml
        self._registration_timeout = \
            int(self._dut_config.get("registrationTimeout"))

        # Create cellular network simulator and retrieve 3g APIs
        self._ns = self._em.get_cellular_network_simulator("NETWORK_SIMULATOR1")
        self._ns_3g = self._ns.get_cell_3g()

        # Initialize EM  xml object
        # measurement file
        meas_file_name = os.path.join(self._saving_directory,
                                      "EM_meas_report.xml")
        self.__em_meas_tab = EMUtil.XMLMeasurementFile(meas_file_name)
        # enable Global Measurement file
        name = os.path.join(self._campaign_folder,
                            self._em_cst.GLOBAL_MEAS_FILE)
        self.__em_meas_tab.enable_global_meas(name, self._name)
        # init variables
        self._total_test = 0

#-----------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # Call the UseCaseBase Setup function
        EmUsecaseBase.set_up(self)

        # set usb charging on
        self.em_api.set_usb_charging("on")

        # Configure CMU
        # Connect to cellular network simulator
        self._ns.init()

        # check that channel is allow for this band
        self._conversion_toolbox.convert_wcdma_channelscript_to_array(
            self._cell_band,
            self._ul_uarfcn)

        # Perform Full Preset
        self._ns.perform_full_preset()

        # Set cell band using CELL_BAND parameter
        self._ns_3g.set_band("BAND" + self._cell_band)

        # Set cell off
        self._ns_3g.set_cell_off()

        # Set Traffic Channel Arfcn using TCH_ARFCN parameter
        self._ns_3g.set_uplink_arfcn(self._ul_uarfcn)

        # Set mobile power using MOBILE_POWER parameter
        self._ns_3g.set_ms_power(self._mobile_power)

        # Set cell power using CELL_POWER parameter
        self._ns_3g.set_cell_power(self._cell_power)

        # init capacity
        msic = self.update_battery_info()

        # get capability targets
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_BATT_BENCH_CYCLE_2G", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

        # init verdict value
        if self._em_targets["MSIC_REGISTER_PLUG.BATTERY.CHARGE_NOW"] is not None:
            EMUtil.update_conf(
                self._em_targets["MSIC_REGISTER_PLUG.BATTERY.CHARGE_NOW"],
                "hi_lim", msic["BATTERY"]["CHARGE_FULL"][0], "=")

        if self.tc_module is not None:
            if self._em_targets["THERMAL_MSIC_REGISTER_PLUG.BATTERY.TEMP"] is not None:
                EMUtil.update_conf(
                    self._em_targets["THERMAL_MSIC_REGISTER_PLUG.BATTERY.TEMP"],
                    ["lo_lim", "hi_lim"], self._tct, "*")

        # Charge battery
        self.em_core_module.monitor_charging(self.em_core_module.batt_max_capacity, self.em_core_module.charge_time,
                              self.__em_meas_tab)

        # set CMU cell phone ON
        self._ns_3g.set_cell_on()
        # register phone

        RegUtil.check_dut_registration_before_timeout(self._ns_3g,
                                                      self.networking_api,
                                                      self._logger,
                                                      None,
                                                      self._registration_timeout)

        # Check registration status on DUT
        self.modem_api.check_cdk_registration_bfor_timeout(
            self._registration_timeout)
        self.voicecall_api.dial("OOOO12121121")

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        EmUsecaseBase.run_test_body(self)

        # configure the board to discharge quicker
        self._logger.info("Start to discharge battery until %s%%" %
                          self.em_core_module.batt_min_capacity)

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
        measurement_fail = 0

        self.phone_as_reboot = False
        # discharge loop
        self._logger.info("Start to discharge battery until %s%%" %
                          self.em_core_module.batt_min_capacity)

        while self.batt_capacity > self.em_core_module.batt_min_capacity:

            # update total test value
            self._total_test += 1
            self._logger.info("TEST iteration_%s" % str(self._total_test))

            batt_charge_Low = "FAILED_TO_COMPUTE"
            batt_charge_High = "FAILED_TO_COMPUTE"
            try:
                # get msic registers value after booting
                msic_reg = self.update_battery_info()
                msic_batt = msic_reg["BATTERY"]

                batt_charge_Low = msic_batt["CHARGE_NOW"][0] - \
                    5 * msic_batt["CURRENT_NOW"][0]
                batt_charge_High = msic_batt["CHARGE_NOW"][0] + \
                    5 * msic_batt["CURRENT_NOW"][0]

                if self._em_targets["MSIC_REGISTER_UNPLUG.BATTERY.CHARGE_NOW"] is not None:
                    EMUtil.update_conf(
                        self._em_targets["MSIC_REGISTER_UNPLUG.BATTERY.CHARGE_NOW"],
                        ["lo_lim"], batt_charge_Low, "=")
                    EMUtil.update_conf(
                        self._em_targets["MSIC_REGISTER_UNPLUG.BATTERY.CHARGE_NOW"],
                        ["hi_lim"], batt_charge_High, "=")

                thermal_conf = self.em_api.get_thermal_sensor_info()

                # store result on xml
                self.__em_meas_tab.add_dict_measurement(msic_reg)
                self.__em_meas_tab.add_dict_measurement(thermal_conf)

                # check thermal capabilities only if thermal chamber is used
                if self.tc_module is not None:
                    # Store various information
                    self.__em_meas_tab.add_measurement(
                        [self.tc_module.feed_meas_report()])
                    self._meas_list.add_dict("THERMAL_MSIC_REGISTER_PLUG", msic_reg)
                    self._meas_list.add_dict("THERMAL_CONF_PLUG", thermal_conf)

                # get the call state
                call_state = "BAND" + self._cell_band
                # Check cs call state
                call_state += "_CS_CALL_" + str(self.voicecall_api.get_state())

                # Store various information
                self.__em_meas_tab.add_measurement(
                    [("REGISTRATION",
                      self.modem_api.get_network_registration_status()),
                     ("VOICE_CALL", call_state)])

            except AcsBaseException as e:
                # try to reconnect to the board if uecmd failed
                self._logger.error("fail to get measurement for OFF ON cycle SDP PLUG TEST: %s"
                                   % str(e))
                measurement_fail += 0.5
            finally:
                # Store various information
                self.__em_meas_tab.add_measurement(
                    [self.get_time_tuple(),
                     ("COMMENTS", "OFF ON cycle SDP PLUG TEST")])

                # switch to next meas
                self.__em_meas_tab.switch_to_next_meas()
#------------------------------------------------------------------------------------------

            try:
                # start charging through usb
                self.em_api.set_usb_charging("on")

                # schedule commands
                if self._device.get_state() == "alive":
                    pid = self.em_api.get_msic_registers("scheduled",
                                                          15)
                    thermal_pid = self.em_api.get_thermal_sensor_info("scheduled",
                                                                       15)

                # Charge board
                self.em_core_module.charge_battery(20)

                # read msic result
                msic_reg = self.em_api.get_msic_registers("read",
                                                           pid)
                # stop charging through usb
                self.em_api.set_usb_charging("off")

                msic_batt = msic_reg["BATTERY"]
                self.batt_capacity = msic_batt["CAPACITY"][0]
                self.batt_voltage = msic_batt["VOLTAGE"][0]

                # get thermal information
                thermal_conf = self.em_api.get_thermal_sensor_info("read",
                                                                    thermal_pid)

                # create XML files
                self.__em_meas_tab.add_dict_measurement(msic_reg)
                self.__em_meas_tab.add_dict_measurement(thermal_conf)

                # compute verdict
                self._meas_list.add_dict("MSIC_REGISTER_UNPLUG",
                                         msic_reg)

                # get the call state
                call_state = "BAND" + self._cell_band
                # Check cs call state
                call_state += "_CS_CALL_" + str(self.voicecall_api.get_state())

                self.__em_meas_tab.add_measurement(
                    [("REGISTRATION",
                      self.modem_api.get_network_registration_status()),
                     ("VOICE_CALL", call_state)])

                # clean scheduled task
                self.phonesystem_api.clean_daemon_files()
                measurement_fail = 0

            except AcsBaseException as e:
                # try to reconnect to the board if uecmd failed
                self._logger.error("fail to get measurement for OFF ON cycle SDP UNPLUG TEST:" + str(e))
                measurement_fail += 0.5

                # stop the usecase if measurement fail several times.
                if measurement_fail >= self._consecutive_meas_error:
                    tmp_txt = "Measurement failed after %s times, stop usecase" % \
                        self._consecutive_meas_error
                    self._logger.error(tmp_txt)
                    if self.batt_voltage > self.vbatt_mos_shutdown or self.batt_voltage == -1:
                        raise DeviceException(DeviceException.PROHIBITIVE_MEASURE, tmp_txt)
                    else:
                        self._logger.info("battery must be empty, stop usecase")
                        break

                # check the board connection
                self.em_core_module.check_board_connection()

            finally:
                self.__em_meas_tab.add_measurement(
                    [self.get_time_tuple(),
                     ("COMMENTS", "OFF ON cycle CABLE UNPLUG TEST"),
                     ("REBOOT", self.phone_as_reboot)])

                # generate em verdict
                self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
                self._em_meas_verdict.judge(ignore_blocked_tc=True)
                self._meas_list.clean()

                # switch to next meas
                self.__em_meas_tab.switch_to_next_meas()

            # restart uecmds to help the discharge if phone as reboot
            if self.has_board_reboot():
                self.phonesystem_api.set_phone_lock(0)
                # stop charging through usb
                self.em_api.set_usb_charging("off")

                # establish a call
                self.voicecall_api.dial("OOOO12121121")

                self.phone_as_reboot = False

            self._logger.info(
                "waiting board discharge during  %s seconds" %
                self.em_core_module.discharge_time)
            time.sleep(self.em_core_module.discharge_time)

        return(self._em_meas_verdict.get_global_result(),
               self._em_meas_verdict.save_data_report_file())

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        # call tear down after some operations
        EmUsecaseBase.tear_down(self)

        # retrieve measurement from test
        self.__em_meas_tab.generate_global_file()

        # set CMU cell phone OFF
        self._ns_3g.set_cell_off()

        # Disconnect from the equipment (Network simulator)
        self._ns.release()

        if self.is_board_and_acs_ok():
            self.em_api.set_usb_charging("on")

        # clean the board state and retrieve logs
        self.em_core_module.clean_up()

        if self.is_board_and_acs_ok():
            self.phonesystem_api.set_screen_timeout(30)
            self.phonesystem_api.clean_daemon_files()

        return Global.SUCCESS, "No errors"
