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
:summary: Energy Management Battery Monitor - Manage Burst Control
:author: vgombert
:since: 08/23/2011
"""
import os

from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase

from UtilitiesFWK.Utilities import Global, str_to_bool
import acs_test_scripts.Utilities.EMUtilities as EMUtil

from ErrorHandling.TestEquipmentException import TestEquipmentException
from ErrorHandling.AcsBaseException import AcsBaseException
from ErrorHandling.DeviceException import DeviceException


class LabEmBattMonitorBurstCtrl(EmUsecaseBase):

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

        # Read Band from test case xml file (str)
        self._cell_band = str(self._tc_parameters.get_param_value("CELL_BAND"))
        # Read CELL_SERVICE from test case xml file
        self._cell_service = \
            str(self._tc_parameters.get_param_value("CELL_SERVICE"))
        # Read TCH_ARFCN from test case xml file
        self._tch_arfcn = int(self._tc_parameters.get_param_value("TCH_ARFCN"))
        # Read UPLINK_CHANNEL from test case xml file
        self._uplink_channel = int(self._tc_parameters.get_param_value("UPLINK_CHANNEL"))
        # Read CELL_POWER from test case xml file
        self._cell_power = \
            int(self._tc_parameters.get_param_value("CELL_POWER"))

        # Read DATA_CALL from test case xml file
        self._data_call = str_to_bool(self._tc_parameters.get_param_value("DATA_CALL"))
        # Read DATA_CALL_MODE from test case xml file
        self._data_call_mode = str(self._tc_parameters.get_param_value("DATA_CALL_MODE"))

        # Init fuel gauging param
        self.em_core_module.init_fg_param()

        # Read registrationTimeout from Phone_Catalog.xml
        self._registration_timeout = \
            int(self._dut_config.get("registrationTimeout"))

        # Create cellular network simulator and retrieve 2G APIs
        self._ns = self._em.get_cellular_network_simulator("NETWORK_SIMULATOR1")
        self._ns_2g = self._ns.get_cell_2g()
        self._data_2g = self._ns_2g.get_data()
        self._vc_2g = self._ns_2g.get_voice_call()
        self._test_mode_2g = self._ns_2g.get_test_mode()

        # Initialize EM  xml object
        meas_file_name = os.path.join(self._saving_directory,
                                      "EM_meas_report.xml")
        self.__em_meas_tab = EMUtil.XMLMeasurementFile(meas_file_name)
        # enable Global Measurement file
        name = os.path.join(self._campaign_folder,
                            self._em_cst.GLOBAL_MEAS_FILE)
        self.__em_meas_tab.enable_global_meas(name, self._name)
        # Call ConfigsParser to parse Energy_Management
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_BATT_MONITOR_BURST_CTRL", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

        # init variables
        self._total_test = 0
        self._shutdown_timeout = 60
        self._network_type = "2G"
        self._slots_conf = []

        # prepare to launch scheduled test depending of battery capacity
        # enter test like below (battery_min, battery_max, test id): function_to_call
        self.scheduled_test = {
            (65, 81, "1"): self.__ber_test,
            (12, 19, "2"): self.__ber_test,
            (1, 5, "3"): self.__ber_test,
        }

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

        # check keys on dict to avoid reading empty files
        msic_batt = self.em_api.get_msic_registers()["BATTERY"]

        # init capacity
        self.batt_capacity = msic_batt["CAPACITY"][0]
        self.batt_voltage = msic_batt["VOLTAGE"][0]

        # Charge battery
        self.em_core_module.monitor_charging(self.em_core_module.batt_max_capacity, self.em_core_module.charge_time,
                              self.__em_meas_tab)

        # set CMU cell phone ON
        self._data_2g.set_data_cell_on()

        # register phone
        self._data_2g.data_register_dut(None, self._registration_timeout)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        EmUsecaseBase.run_test_body(self)
        test_id = "DEFAULT"

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

        # establish call
        if self._data_call:
            try:
                self._data_2g.data_call(self._data_call_mode)
                self._data_2g.check_data_call_connected(60)
            except TestEquipmentException as error:
                self._logger.error(error)
        else:
            self.voicecall_api.dial("OOOO12121121")

        # get BER
        ber = "CANT_GET_BER"
        try:
            self._test_mode_2g.configure_ber_measurement(500, self._cell_power)
            ber = self._test_mode_2g.get_ber()
        except TestEquipmentException as error:
            self._logger.error(error)

        # generate xml measurement
        self._meas_list.add("REGISTRATION_" + str(test_id),
                           (self.modem_api.get_network_registration_status(), "none"))

        self._meas_list.add("BER_" + str(test_id), (ber, "none"))

        # switch to next meas
        self._em_meas_verdict.compare_list(self._meas_list,
                                           self._em_targets)
        self._meas_list.clean()
        self._em_meas_verdict.save_data_report_file()

        # configure the board to discharge quicker
        self._logger.info("Start to discharge battery until %s%%" %
                          self.em_core_module.batt_min_capacity)

        # launch uecmd to help the discharge
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
            # discharge board

            try:
                # launch test depending of battery capacity------------------------------------------#
                test_found = False
                for test_limit in self.scheduled_test:
                    if test_limit[0] < self.batt_capacity < test_limit[1]:
                        test_found = True
                        self.scheduled_test[test_limit](test_limit[2])
                        break

                if not test_found:
                    self.__default_test()
                # launch test depending of battery capacity------------------------------------------#

                # get the call state
                call_state = self._cell_band

                reg_state = self.modem_api.get_network_registration_status()
                if self._data_call:
                    # Check data call state
                    try:
                        self._data_2g.check_data_call_connected(10)
                        call_state += "_DATA_CALL_CONNECTED"
                    except TestEquipmentException as error:
                        self._logger.error(error)
                        call_state += "_DATA_CALL_DISCONNECTED"
                        # try to re - establish call
                        try:
                            self._data_2g.data_call(self._data_call_mode)
                            self._data_2g.check_data_call_connected(10)
                        except TestEquipmentException as error:
                            self._logger.error(error)
                else:
                    # Check cs call state
                    temp_call = self.voicecall_api.get_state()
                    call_state += "_CS_CALL_" + str(temp_call)

                    # pylint: disable=E1101
                    if temp_call not in [self._uecmd_types.VOICE_CALL_STATE.INCOMING,
                                         self._uecmd_types.VOICE_CALL_STATE.ACTIVE]:
                        # establish a call
                        self.voicecall_api.dial("OOOO12121121")
                    # pylint: enable=E1101

                # Store various information
                self.__em_meas_tab.add_measurement(
                    [("REGISTRATION", reg_state),
                     ("VOICE_CALL", call_state)])

                # keep usb discharging
                self.em_api.set_usb_charging("off")
                # reset consecutive failed measurement
                measurement_fail = 0

            except AcsBaseException as e:
                # try to reconnect to the board if uecmd failed
                self._logger.error("fail to get measurement : %s" % str(e))
                measurement_fail += 1

                # stop the usecase if measurement fail several times.
                if measurement_fail >= self._consecutive_meas_error:
                    tmp_txt = "Measurement failed after %s times, abort usecase" % \
                        self._consecutive_meas_error
                    self._logger.error(tmp_txt)
                    raise DeviceException(DeviceException.PROHIBITIVE_MEASURE, tmp_txt)

                # check the board connection
                self.em_core_module.check_board_connection()

            finally:
                self.__em_meas_tab.add_measurement(
                    [self.get_time_tuple(),
                     ("REBOOT", self.phone_as_reboot)])

                # Store test information
                self.__em_meas_tab.switch_to_next_meas()
                self.phone_as_reboot = False

            # restart uecmds to help the discharge if phone as reboot
            if self.has_board_reboot():
                self.phonesystem_api.set_phone_lock(0)
                # stop charging through usb
                self.em_api.set_usb_charging("off")

                # establish a call
                if self._data_call:
                    try:
                        self._data_2g.data_call(self._data_call_mode)
                        self._data_2g.check_data_call_connected(10)
                    except TestEquipmentException as error:
                        self._logger.error(error)
                else:
                    self.voicecall_api.dial("OOOO12121121")

                self.phone_as_reboot = False

        # compute verdict
        self._em_meas_verdict.judge(ignore_blocked_tc=True)
        self._meas_list.clean()

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
        self._data_2g.set_data_cell_off()

        # Disconnect from the equipment (Network simulator)
        self._ns.release()

        if self.is_board_and_acs_ok():
            self.em_api.set_usb_charging("on")

        # call tear down after some operations
        self.em_core_module.clean_up()

        if self.is_board_and_acs_ok():
            self.phonesystem_api.set_screen_timeout(30)
            self.phonesystem_api.clean_daemon_files()

        return Global.SUCCESS, "No errors"

#-----------------------------------------------------------------------

    def __ber_test(self, test_id):
        """
        Specific test that measure BER value
        with vibration and flash active
        """
        self._logger.info("BER test")
        self._total_test += 1

        # prepare phone for measurement
        self.phonesystem_api.set_phone_lock(0)
        self.phonesystem_api.set_torchlight("on")
        self.phonesystem_api.set_vibration("on")

        # try to read measurement
        msic_reg = self.update_battery_info()

        # store result on xml
        self.__em_meas_tab.add_dict_measurement(msic_reg)

        # get registration state
        reg_state = (self.modem_api.get_network_registration_status(),
                     "none")

        # try to get BER
        ber = "CANT_GET_BER"
        try:
            self._test_mode_2g.configure_ber_measurement(500, self._cell_power)
            ber = self._test_mode_2g.get_ber()
        except TestEquipmentException as error:
            self._logger.error(error)

        # compare measurement
        self._meas_list.add_dict("MSIC_REGISTER_" + str(test_id), msic_reg)
        self._meas_list.add("BER_" + str(test_id), (ber, "none"))
        self._meas_list.add("REGISTRATION_" + str(test_id),
                            reg_state)

        self._em_meas_verdict.compare_list(self._meas_list,
                                           self._em_targets)
        self._meas_list.clean()
        self._em_meas_verdict.save_data_report_file()
        # Store various information
        self.__em_meas_tab.add_measurement(
            [("COMMENTS", "msic ber registration Discharging"),
             ("BER", ber)])

        self.phonesystem_api.set_torchlight("off")
        self.phonesystem_api.set_vibration("off")

    def __default_test(self):
        """
        default test
        """
        self._logger.info("default test")
        # try to read measurement
        msic_reg = self.update_battery_info()
        # store result on xml
        self.__em_meas_tab.add_dict_measurement(msic_reg)
        self.__em_meas_tab.add_measurement(
            [("COMMENTS", "default Discharging test")])
