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
:summary: Energy Management - idle 3G voicecall with no charger
:author: vgombert
:since: 20/02/2012(feb)
"""
import time
import os
import acs_test_scripts.Utilities.RegistrationUtilities as RegUtil

from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase

from UtilitiesFWK.Utilities import Global, get_conversion_toolbox
import acs_test_scripts.Utilities.EMUtilities as EMUtil

from ErrorHandling.AcsBaseException import AcsBaseException
from ErrorHandling.DeviceException import DeviceException


class LabEmBattIdleVc3gNoChgr(EmUsecaseBase):

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
        # init fuel gauging parameters
        self.em_core_module.init_fg_param()
        # Read Band from test case xml file (str)
        self._cell_band = str(self._tc_parameters.get_param_value("CELL_BAND"))

        # Read UL_UARFCN from test case xml file
        self._ul_uarfcn = int(self._tc_parameters.get_param_value("UL_UARFCN"))

        # Read mobile power from test case xml file (int)
        self._mobile_power = int(self._tc_parameters.get_param_value("MOBILE_POWER"))

        # Read CELL_POWER from test case xml file
        self._cell_power = \
            int(self._tc_parameters.get_param_value("CELL_POWER"))

        # Read TEST_DURATION from test case xml file
        self._test_duration = \
            int(self._tc_parameters.get_param_value(
                "TEST_DURATION"))

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

        # check keys on dict to avoid reading empty files
        self.update_battery_info()

        # get capability targets
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_BATT_IDLE_3G_VC_NO_CHGR", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

        # init verdict value
        if self.tc_module is not None:
            if self._em_targets["THERMAL_MSIC_REGISTER.BATTERY.TEMP"] is not None:
                EMUtil.update_conf(
                    self._em_targets["THERMAL_MSIC_REGISTER.BATTERY.TEMP"],
                    ["lo_lim"], self._tct, "=")

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

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        start to discharge board
        """
        EmUsecaseBase.run_test_body(self)

        self._logger.info("get info during idle voice call %s seconds" % self._test_duration)

        # stop charging
        self.em_api.set_usb_charging("off")

        # set screen timeout to 15 sec
        self.phonesystem_api.set_screen_timeout(15)

        # wait 30 sec
        time.sleep(30)

        # establish call
        self.voicecall_api.dial("OOOO12121121")

        # init var
        measurement_fail = 0
        start_time = time.time()

        # OFF ON loop
        while (time.time() - start_time) < self._test_duration:

            try:
                # try to read measurement
                self._total_test += 1
                self._logger.info("TEST iteration_%s" % str(self._total_test))

                # get msic registers value after booting
                msic_reg = self.update_battery_info()
                self.__em_meas_tab.add_dict_measurement(msic_reg)

                # get thermal info
                thermal_conf = self.em_api.get_thermal_sensor_info()
                self.__em_meas_tab.add_dict_measurement(thermal_conf)

                # check system info
                brightness_level = self.phonesystem_api.get_backlight_level()
                cpu_freq = self.phonesystem_api.get_cpu_freq()
                charge_current = self.em_api.get_charger_level()

                self._meas_list.add_dict("MSIC_REGISTER", msic_reg)
                # check thermal capabilities only if thermal chamber is used
                if self.tc_module is not None:
                    # Store various information
                    self.__em_meas_tab.add_measurement(
                        [self.tc_module.feed_meas_report()])
                    self._meas_list.add_dict("THERMAL_MSIC_REGISTER", msic_reg)
                    self._meas_list.add_dict("THERMAL_CONF", thermal_conf)

                self.__em_meas_tab.add_measurement(
                    [("BRIGHTNESS_LEVEL", brightness_level),
                     ("CPU_FREQ", cpu_freq),
                     ("CHARGER_CURRENT", charge_current)])

                self._meas_list.add("BRIGHTNESS_LEVEL",
                                    (brightness_level, "none"))
                self._meas_list.add("CPU_FREQ",
                                    (cpu_freq, "none"))
                self._meas_list.add("CHARGER_CURRENT",
                                    (cpu_freq, "none"))

                self.__em_meas_tab.add_measurement(
                    [("REGISTRATION", self.modem_api.get_network_registration_status()),
                     ("VOICE_CALL", self.voicecall_api.get_state())])

                # stop to charge through usb
                self.em_api.set_usb_charging("off")
                measurement_fail = 0

            except AcsBaseException as e:
                # Just log error, board will be rebooted in next iteration
                self._logger.error("fail to get measurement : %s" % str(e))
                measurement_fail += 1

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

                # Store various information
                self.__em_meas_tab.add_measurement(
                    [self.get_time_tuple(),
                     ("COMMENTS", "OFF ON cycle discharge")])

                # generate em verdict
                self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
                self._em_meas_verdict.judge(ignore_blocked_tc=True)
                self._meas_list.clean()

                # switch to next meas
                self.__em_meas_tab.switch_to_next_meas()

                # reinitialize reboot variable
                self.phone_as_reboot = False

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

        # clean the board state and retrieve logs
        self.em_core_module.clean_up()

        return Global.SUCCESS, "No errors"
