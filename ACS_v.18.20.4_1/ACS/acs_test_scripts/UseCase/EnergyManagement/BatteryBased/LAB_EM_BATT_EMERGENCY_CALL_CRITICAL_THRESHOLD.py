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

:organization: INTEL QCTV
:summary: EM - Call an emergency call when the battery critical threshold is reached
Call an emergency call when the battery capacity is equal to 5%
:author: jvauchex
:since: 12/11/2013
"""
import os
import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.EMUtilities import XMLMeasurementFile
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from acs_test_scripts.Device.UECmd import UECmdTypes
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.UseCase.EnergyManagement.UcModule.TelephonyModule3G import TelephonyModule3g
from acs_test_scripts.UseCase.EnergyManagement.UcModule.TelephonyModule2G import TelephonyModule2g
from acs_test_scripts.UseCase.EnergyManagement.UcModule.LoadModule import LoadModule
from ErrorHandling.AcsBaseException import AcsBaseException
from acs_test_scripts.Device.UECmd.UECmdTypes import VOICE_CALL_STATE


class LabEmBattEmergencyCallCriticalThreshold(EmUsecaseBase):

    """
    Lab Energy Management class.
    """
    DEDICATED_BENCH = "BATTERY_BENCH"

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LAB_EM_BASE Init function
        EmUsecaseBase.__init__(self, tc_name, global_config)
        # init fuel gauging parameters
        self.em_core_module.init_fg_param()
        # misc parameter
        self.__battery_critical_threshold = self._tc_parameters.get_param_value("BATTERY_CRITICAL_THRESHOLD", default_cast_type=int)
        load_to_apply_in_setup = self._tc_parameters.get_param_value("LOAD_DURING_DISCHARGE", "")
        # call parameter
        self.__call_timeout = self._tc_parameters.get_param_value("CALL_HOLD_TIME", default_cast_type=int)
        self.__hands_free_activation = self._tc_parameters.get_param_value("HANDS_FREE_ACTIVATION", default_cast_type="str_to_bool")
        self.__fake_emergency_phone_number = self._tc_parameters.get_param_value("PHONE_NUMBER", "012345")
        call_type = self._tc_parameters.get_param_value("CALL_TYPE")

        # Summarize the parameters
        self._logger.info("[TESTCASE]\tBattery Critical Threshold: %s%%" % self.__battery_critical_threshold)
        self._logger.info("[TESTCASE]\tEmergency Phone Number : %s" % self.__fake_emergency_phone_number)
        self._logger.info("[TESTCASE]\tthe holding time of the call when battery reach critical level: %ss" % self.__call_timeout)
        self._logger.info("[TESTCASE]\tType of the call : %s" % call_type)
        self._logger.info("[TESTCASE]\tActivation of Hands Free : %s" % self.__hands_free_activation)

        # Initial Emergency Number
        if call_type == "3G":
            # Initialization of module 3g
            self.__telmodule = TelephonyModule3g()
        elif call_type == "2G":
            # Initialization of module 3g
            self.__telmodule = TelephonyModule2g()
        else:
            txt = "Type of the call is not correct => Quit the test"
            self._logger.error(txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, txt)

        # measurement file
        meas_file_name = os.path.join(self._saving_directory, "EM_meas_report.xml")
        self.__em_meas_tab = XMLMeasurementFile(meas_file_name)

        # Store the initial list of emergency number from UECmdTypes
        self.__initial_ue_command_em_number = UECmdTypes.EMERGENCY_NUMBERS_LIST

        # init load module used to discharge board
        self.__load_module = LoadModule()
        self.__load_module.add_load(load_to_apply_in_setup)
        self.__initial_emergency_numbers = None
        self.__original_airplane_state = None

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # Call the UseCaseBase Setup function
        EmUsecaseBase.set_up(self)
        # deactivate flight mode for this test
        self.__original_airplane_state = self.networking_api.get_flight_mode()
        if self.__original_airplane_state:
            self.networking_api.set_flight_mode(False)

        # setup network simulator
        self.__telmodule.set_up_eq()
        # Brightness Configuration
        self.phonesystem_api.set_phone_lock(0)
        # set the screen on and brightness to 100%
        self.phonesystem_api.set_screen_timeout(3600)
        # deactivate set auto brightness
        self.phonesystem_api.set_brightness_mode("manual")
        # set display brightness to max value
        self.phonesystem_api.set_display_brightness(100)
        # Wake up the screen
        self.phonesystem_api.wake_screen()
        # wait for registration
        self.__telmodule.register_to_network()

        # Check if the fake number is a real emergency number
        if self.__fake_emergency_phone_number in UECmdTypes.EMERGENCY_NUMBERS_LIST:
            txt = "Fake phone number is a real emergency phone number => Quit the test, know emergency number are %s" % str(UECmdTypes.EMERGENCY_NUMBERS_LIST)
            self._logger.error(txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, txt)

        if self.__initial_emergency_numbers is None:
            initial_emergency_numbers = self.modem_api.get_emergency_numbers()
            if self.__fake_emergency_phone_number not in initial_emergency_numbers:
                self.__initial_emergency_numbers = initial_emergency_numbers
            else:
                self.__initial_emergency_numbers = initial_emergency_numbers.remove(self.__fake_emergency_phone_number)

        # Add the fake emergency phone number
        self.modem_api.set_emergency_numbers([self.__fake_emergency_phone_number], False)
        # Update Battery Information
        self.update_battery_info()
        # Discharge the battery if we are above critical threshold
        if self.batt_capacity > self.__battery_critical_threshold:
            self.em_core_module.monitor_discharging(self.__battery_critical_threshold,
                                self.em_core_module.discharge_time, self.__em_meas_tab, self.__load_module)

            # Check the board is still alive
            self._logger.info("Check if we haven't lost the board connection after discharging")
            if not self.is_board_and_acs_ok():
                cause_txt = "At the beginning of the test, the board status is not in MOS, abort testcase !"
                self._logger.info(cause_txt)
                raise DeviceException(DeviceException.DUT_BOOT_ERROR, cause_txt)

        # try to reboot in MOS if we discharge too much
        if self._device.get_boot_mode() == "COS":
            self._device.reboot()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        Check the end of the main battery charge
        """
        # Call LAB_EM_BASE Run function
        EmUsecaseBase.run_test_body(self)

        # Check the battery capacity to 0%
        self._logger.info("BEGIN OF THE TEST : Battery capacity is equal to %d%%" % self.batt_capacity)

        # if registration has been lost, register again
        if self.modem_api.get_network_registration_status() not in ["registered", "roaming"]:
            self.__telmodule.register_to_network()
        # if fake number has been lost restore it here
        if self.__fake_emergency_phone_number not in self.modem_api.get_emergency_numbers():
            self.modem_api.set_emergency_numbers([self.__fake_emergency_phone_number], False)
        # First Part, Check emergency call alone
        # establish voice call
        self._logger.info("Board is in MOS, Initiate the call !")
        self.voicecall_api.emergency_dial(self.__fake_emergency_phone_number)

        # Add the hands free
        if self.__hands_free_activation:
            self._logger.info("Activate the Speaker !")
            self.phonesystem_api.switch_audio_output("speaker")

        # Discharge the board
        return self.__discharge_when_data(0)

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        # call tear down after some operations
        EmUsecaseBase.tear_down(self)

        # release equipment
        self.__telmodule.release_eq()

        # clean the board state and retrieve logs
        self.em_core_module.clean_up()

        if self.is_board_and_acs_ok():
            # restore flight mode if it was turn off and previous test
            if self.__original_airplane_state:
                self.networking_api.set_flight_mode(True)

            if self.__initial_emergency_numbers is not None:
                # Restore the list of emergency number
                self.modem_api.set_emergency_numbers(self.__initial_emergency_numbers, True)
                # Check the effect of the command
                if self.modem_api.get_emergency_numbers() != self.__initial_emergency_numbers:
                    txt = "Could not restore the list of emergency number"
                    self._logger.error(txt)
                    raise DeviceException(DeviceException.OPERATION_FAILED, txt)

        # restore the hands free
        if self.__hands_free_activation:
            self._logger.info("restore speaker")
            self.phonesystem_api.switch_audio_output("headset")

        return Global.SUCCESS, "No errors"

    #------------------------------------------------------------------------------

    def __discharge_when_data(self, min_capacity):
        """
        monitor the discharging by applying loads.
        can discharge until shutdown.

        :type min_capacity: int
        :param min_capacity: min battery capacity

        :rtype: int
        :return: battery capacity reached
        """
        self.update_battery_info()
        # find away to compute this
        timeout = 3600
        end_time = time.time() + timeout
        # discharge until 0% is reached
        last_capacity = self.batt_capacity
        capacity_increase_timeout = 900
        capa_inc_detect_once = False
        start_time_capacity_increase = time.time()

        while self.batt_capacity > min_capacity:

            try:
                # try to read measurement
                msic_reg = self.update_battery_info()
                thermal_conf = self.em_api.get_thermal_sensor_info()
                # store result on xml
                self.__em_meas_tab.add_dict_measurement(msic_reg)
                self.__em_meas_tab.add_dict_measurement(thermal_conf)
                # check call state
                call_state = self.voicecall_api.get_state()
                # pylint: disable=E1101
                if call_state == VOICE_CALL_STATE.NOCALL:
                    self._logger.warning("Call is OFF => Re-Activate the call !")
                    self.voicecall_api.emergency_dial(self.__fake_emergency_phone_number)

                # Store various information
                self.__em_meas_tab.add_measurement(
                    [("REGISTRATION", self.modem_api.get_network_registration_status()),
                     ("VOICE_CALL", str(call_state))])

                # Detect if battery is charging, if it is the case stop usecase later
                if not capa_inc_detect_once:
                    if self.batt_capacity <= last_capacity:
                        start_time_capacity_increase = time.time()
                        capa_inc_detect_once = False
                    else:
                        capa_inc_detect_once = True

                else:
                    if self.batt_capacity < last_capacity:
                        start_time_capacity_increase = time.time()
                        capa_inc_detect_once = False

                last_capacity = self.batt_capacity

                # stop usecase if board take too long to reach battery capacity
                if time.time() - start_time_capacity_increase > capacity_increase_timeout:
                    tmp_txt = "board capacity keep increasing or not change during %ss instead of decreasing; abort usecase" % capacity_increase_timeout
                    self._logger.error(tmp_txt)
                    raise DeviceException(DeviceException.TIMEOUT_REACHED, tmp_txt)

            except AcsBaseException as e:
                # try to catch why uecmd may fail
                if not self.is_board_and_acs_ok():
                    txt = "connection with board lost during discharge to reach 0% of capacity"
                else:
                    txt = "error happened during discharge to reach 0% : " + str(e)
                self._logger.error(txt)
                raise DeviceException(DeviceException.OPERATION_FAILED, txt)
            finally:
                # Store various information
                self.__em_meas_tab.add_measurement([self.get_time_tuple(),
                                        (self._em_cst.COMMENTS, "RUNTEST:Discharging while emergency call is normally active")])
                # switch meas to next meas
                self.__em_meas_tab.switch_to_next_meas()

            # stop usecase if board take too long to reach battery capacity
            if time.time() > end_time:
                tmp_txt = "Phone failed to reach 0%% of capacity before %ss" % (timeout)
                self._logger.error(tmp_txt)
                raise DeviceException(DeviceException.TIMEOUT_REACHED, tmp_txt)

        # if we are here it means that capacity is at 0%, now checking if the call is still active
        call_state = self.voicecall_api.get_state()
        # pylint: disable=E1101
        if call_state != VOICE_CALL_STATE.ACTIVE:
            txt = "The call is not ACTIVE after reaching %s%% => Objective of UC is not reached => FAIL" % self.batt_capacity
            self._logger.error(txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, txt)

        # now is call last when we are at critical level
        self._logger.info("Battery capacity is equal to %s%%" % self.batt_capacity)
        # remove all potential cable that may supply the board
        self._device.disconnect_board()
        # disconnect usb
        self._io_card.usb_connector(False)
        self._logger.info("test if the emergency call can hold %ss at battery critical level" % self.__call_timeout)
        time.sleep(self.__call_timeout)
        # connect to board
        self._io_card.usb_host_pc_connector(True)
        # connect board
        self._device.connect_board()
        # Check the board status

        if not self.is_board_and_acs_ok():
            text_cause = "The board status is not more seen at the end of the call lasting test"
            self._logger.error(text_cause)
            raise DeviceException(DeviceException.OPERATION_FAILED, text_cause)
        else:
            # get the call state------------------------------------------------------------------------
            self._logger.info("Check the call state !")
            call_state = self.voicecall_api.get_state()
            # pylint: disable=E1101
            if call_state != VOICE_CALL_STATE.ACTIVE:
                txt = "the call is not in ACTIVE state but in %s state after spending %ss at critical battery level " % (str(call_state), self.__call_timeout)
                self._logger.error(txt)
                raise DeviceException(DeviceException.OPERATION_FAILED, txt)
            self.update_battery_info()

        # if we reach here it means that the test passed
        return Global.SUCCESS, "The call has been held in state %s during %ss" % (str(call_state), self.__call_timeout)
