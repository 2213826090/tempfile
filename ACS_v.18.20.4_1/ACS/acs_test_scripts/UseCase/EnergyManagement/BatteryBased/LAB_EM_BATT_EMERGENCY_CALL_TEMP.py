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
:summary: EM - Call an emergency call with different temperatures
Test emergency call with a temperature equal to -5, 25 and 45
:author: jvauchex, vgomberx
:since: 21/03/2013
"""
import os
import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.EMUtilities import XMLMeasurementFile
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from acs_test_scripts.UseCase.EnergyManagement.UcModule.ThermalChamberModule import ThermalChamberModule
from acs_test_scripts.Device.UECmd import UECmdTypes
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.AcsBaseException import AcsBaseException
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.UseCase.EnergyManagement.UcModule.TelephonyModule3G import TelephonyModule3g
from acs_test_scripts.UseCase.EnergyManagement.UcModule.TelephonyModule2G import TelephonyModule2g
from acs_test_scripts.Device.UECmd.UECmdTypes import VOICE_CALL_STATE


class LabEmBattEmergencyCallTemp(EmUsecaseBase):

    """
    Live Energy Management class.
    """
    DEDICATED_BENCH = "BATTERY_BENCH"

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LAB_EM_BASE Init function
        EmUsecaseBase.__init__(self, tc_name, global_config)

        # Read parameters from TC parameters
        self.__fake_emergency_phone_number = self._tc_parameters.get_param_value("FAKE_EMERGENCY_PHONE_NUMBER", "012345")
        self.__call_timeout = self._tc_parameters.get_param_value("CALL_HOLD_TIME", default_cast_type=int)
        self.__temperature_change_timeout = self._tc_parameters.get_param_value("TEMPERATURE_CHANGE_TIMEOUT", default_cast_type=int)

        # Call Type
        call_type = self._tc_parameters.get_param_value("CALL_TYPE")

        # Read TEMPERATURE from test case xml file
        self.tc_module = None
        self.__temperature = self._tc_parameters.get_param_value("TEMPERATURE")
        if self.__temperature != "ROOM":
            # get temperature chamber equipment if not room temperature
            self.__temperature = int(self.__temperature)
            self.tc_module = ThermalChamberModule(self._ambient_temperature)
            # inform module that this is not a default thermal test
            self.tc_module.set_test_type("SPECIFIC")

        # Call ConfigsParser to parse Energy_Management
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
        meas_file_name = os.path.join(self._saving_directory,
                                      "EM_meas_report.xml")
        self.__em_meas_tab = XMLMeasurementFile(meas_file_name)
        # Store the initial list of emergency number from UECmdTypes
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
        # init temperature chamber
        if self.tc_module is not None:
            self.tc_module.set_up_eq()
            self.tc_module.adjust_temp_according_to_test_value(True, 5)

        # Brightness Configuration
        self.phonesystem_api.set_phone_lock(0)
        self.phonesystem_api.set_screen_timeout(900)
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

        if self.__fake_emergency_phone_number not in self.modem_api.get_emergency_numbers():
            txt = "The fake emergency number is not in the list !"
            self._logger.error(txt)
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, txt)

        # Dial using Phone_number parameter
        self.voicecall_api.emergency_dial(self.__fake_emergency_phone_number)

        # heat the board
        self._logger.info("Try to heat the board until DUT critical temperature level is reached")
        last_batt_state = self.update_battery_info()["BATTERY"]["HEALTH"][0].upper()
        if self.tc_module is not None:
            exceed_timeout = True
            self.tc_module.get_eq().set_temperature(self.__temperature)
            self.tc_module.get_eq().wait_for_temperature(self.__temperature)

            timeout = time.time() + self.__temperature_change_timeout
            while time.time() < timeout:
                time.sleep(60)
                try:
                    # try to read measurement
                    msic_reg = self.update_battery_info()
                    thermal_conf = self.em_api.get_thermal_sensor_info()
                    # store result on xml
                    self.__em_meas_tab.add_dict_measurement(msic_reg)
                    self.__em_meas_tab.add_dict_measurement(thermal_conf)
                    last_batt_state = msic_reg["BATTERY"]["HEALTH"][0].upper()
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

                except AcsBaseException as e:
                    # try to catch why uecmd may fail
                    if not self.is_board_and_acs_ok():
                        txt = "connection with board lost during temperature change to reach CRITICAL level"
                    else:
                        txt = "error happened during temperature change to reach CRITICAL level : " + str(e)
                    self._logger.error(txt)
                    raise DeviceException(DeviceException.OPERATION_FAILED, txt)
                finally:
                    # Store various information
                    self.__em_meas_tab.add_measurement([self.get_time_tuple(),
                                            (self._em_cst.COMMENTS, "RUNTEST:heating board while emergency call is normally active")])
                    # switch meas to next meas
                    self.__em_meas_tab.switch_to_next_meas()
                if last_batt_state == "OVERHEAT":
                    self._logger.info("The board is in OVERHEAT status !")
                    exceed_timeout = False
                    break

            if exceed_timeout:
                txt = "timeout exceeded (%ss) to heat up the board to CRITICAL temperature level" % str(self.__temperature_change_timeout)
                self._logger.error(txt)
                raise DeviceException(DeviceException.OPERATION_FAILED, txt)

        # check that the call is still alive
        call_state = self.voicecall_api.get_state()
        # pylint: disable=E1101
        if call_state != VOICE_CALL_STATE.ACTIVE:
            txt = "The call is in %s state after reaching battery %s state" % (str(call_state), last_batt_state)
            self._logger.error(txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, txt)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        Check the end of the main battery charge
        """
        # Call LAB_EM_BASE Run function
        EmUsecaseBase.run_test_body(self)

        # ********** First part, Hold the emergency call **********
        # Check that the call can last
        self._logger.info("test if the emergency call can hold %ss at battery thermal critical level" % self.__call_timeout)
        time.sleep(self.__call_timeout)

        if not self.is_board_and_acs_ok():
            text_cause = "The board is not more seen at the end of the emergency call lasting test"
            self._logger.error(text_cause)
            raise DeviceException(DeviceException.OPERATION_FAILED, text_cause)
        else:
            # get the call state------------------------------------------------------------------------
            self._logger.info("Check the call state !")
            call_state = self.voicecall_api.get_state()
            # pylint: disable=E1101
            if call_state != VOICE_CALL_STATE.ACTIVE:
                txt = "the call is not in ACTIVE state but in %s state after spending %ss at critical temperature level" % (str(call_state), self.__call_timeout)
                self._logger.error(txt)
                raise DeviceException(DeviceException.OPERATION_FAILED, txt)
            else:
                txt = "the call is still in %s state after spending %ss at critical temperature level" % (str(call_state), self.__call_timeout)
                self._logger.info(txt)

            self.update_battery_info()

        # ********** Cool down the board to return at a normal level **********
        self.__cool_down_board()

        return Global.SUCCESS, "the emergency call was able to be hold during the CRITICAL battery temperature level"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        # call tear down after some operations
        EmUsecaseBase.tear_down(self)

        # release equipment
        if self.tc_module is not None:
            self.tc_module.get_eq().set_temperature(self._ambient_temperature)
            self.tc_module.get_eq().wait_for_temperature(self._ambient_temperature, margin=10)
            self.tc_module.release_eq()

        # release equipment
        if self.__telmodule is not None:
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

        return Global.SUCCESS, "No errors"

    def __cool_down_board(self):
        """
        try to make the board exit the overheat state to see
        if the board stay on after we terminate the call
        """
        self._logger.info("Try to cool down the board until DUT normal temperature level is reached")
        last_batt_state = self.update_battery_info()["BATTERY"]["HEALTH"][0].upper()
        # Go to the high or low temperature
        if self.tc_module is not None:
            exceed_timeout = True
            self.tc_module.get_eq().set_temperature(self._ambient_temperature)
            self.tc_module.get_eq().wait_for_temperature(self._ambient_temperature)
            timeout = time.time() + self.__temperature_change_timeout
            while time.time() < timeout:
                time.sleep(60)
                try:
                    # try to read measurement
                    msic_reg = self.update_battery_info()
                    thermal_conf = self.em_api.get_thermal_sensor_info()
                    # store result on xml
                    self.__em_meas_tab.add_dict_measurement(msic_reg)
                    self.__em_meas_tab.add_dict_measurement(thermal_conf)
                    last_batt_state = msic_reg["BATTERY"]["HEALTH"][0].upper()

                except AcsBaseException as e:
                    # try to catch why uecmd may fail
                    if not self.is_board_and_acs_ok():
                        txt = "connection with board lost during temperature change to exit CRITICAL level"
                    else:
                        txt = "error happened during temperature change to exit CRITICAL level : " + str(e)
                    self._logger.error(txt)
                    raise DeviceException(DeviceException.OPERATION_FAILED, txt)
                finally:
                    # Store various information
                    self.__em_meas_tab.add_measurement([self.get_time_tuple(),
                                            (self._em_cst.COMMENTS, "RUNTEST:cooldown board while emergency call is normally active")])
                    # switch meas to next meas
                    self.__em_meas_tab.switch_to_next_meas()
                if last_batt_state != "OVERHEAT":
                    self._logger.info("The board has left OVERHEAT state and now is in %s state !" % last_batt_state)
                    exceed_timeout = False
                    break

            if exceed_timeout:
                txt = "Although the EMERGENCY CALL has held during CRITICAL temperature, we fail to cooldown the board before (%ss) to a level below CRITICAL temperature" % str(self.__temperature_change_timeout)
                self._logger.error(txt)
                raise DeviceException(DeviceException.OPERATION_FAILED, txt)

        # check that the call is still alive
        call_state = self.voicecall_api.get_state()
        # pylint: disable=E1101
        if call_state != VOICE_CALL_STATE.ACTIVE:
            txt = "Although the EMERGENCY CALL has held during CRITICAL temperature, The call is in %s state instead of ACTIVE after reaching battery %s state" % (str(call_state), last_batt_state)
            self._logger.error(txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, txt)
        else:
            self._logger.info("wait 120s to see if the board turn off or not after hanging up the call")
            self.voicecall_api.release()
            time.sleep(120)

        # cool down board and check if when we hang up the call, the board does not turn off
        if not self.is_board_and_acs_ok():
            text_cause = "Although the EMERGENCY CALL has held during CRITICAL temperature, The board is not more seen after having cool it down and hanging up the EMERGENCY CALL"
            self._logger.error(text_cause)
            raise DeviceException(DeviceException.OPERATION_FAILED, text_cause)
        else:
            # get the call state------------------------------------------------------------------------
            self.update_battery_info()
