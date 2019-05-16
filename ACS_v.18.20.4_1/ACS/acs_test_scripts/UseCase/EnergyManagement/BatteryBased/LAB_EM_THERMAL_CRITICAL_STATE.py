"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
:summary: EM - test what happen at thermal critical state on different sensors
:author: vgomberx
:since: 01/10/2014
"""
import os
import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.EMUtilities import XMLMeasurementFile
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from acs_test_scripts.UseCase.EnergyManagement.UcModule.ThermalChamberModule import ThermalChamberModule
from ErrorHandling.AcsBaseException import AcsBaseException
from ErrorHandling.DeviceException import DeviceException


class LabEmThermalCriticalState(EmUsecaseBase):

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
        self.__sensor = self._tc_parameters.get_param_value("SENSOR_TO_TEST").upper()

        self.__temperature_change_timeout = self._tc_parameters.get_param_value("TEMPERATURE_CHANGE_TIMEOUT",
                                                                                default_cast_type=int)
        # TODO: add charge type choice to allow shutdown without cable

        # Read TEMPERATURE from test case xml file
        self.tc_module = None
        # set a temperature or use SMART to use data from thermal conf file
        self.__temperature = self._tc_parameters.get_param_value("TEMPERATURE")
        # we can authorize to increase at most 10 more degree on the test temp
        # if board temperature does not move
        self.__max_temp_rising = self._tc_parameters.get_param_value("TEMP_MAX_INCREASE_MARGIN", 10,
                                                                     default_cast_type=int)
        if self.__temperature != "ROOM":
            # get temperature chamber equipment if not room temperature
            if self.__temperature != "SMART":
                self.__temperature = int(self.__temperature)
            self.tc_module = ThermalChamberModule(self._ambient_temperature)
            # inform module that this is not a default thermal test
            self.tc_module.set_test_type("SPECIFIC")

        # Call ConfigsParser to parse Energy_Management
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_THERMAL_CRITICAL_STATE", self._tc_parameters.get_params_as_dict(), self._device.get_phone_model())
        self._em_meas_verdict.load_target(self._em_targets, self.tcd_to_test)
        # measurement file
        meas_file_name = os.path.join(self._saving_directory,
                                      "EM_meas_report.xml")
        self.__em_meas_tab = XMLMeasurementFile(meas_file_name)

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # Call the UseCaseBase Setup function
        EmUsecaseBase.set_up(self)

        # check that the sensor exist in readable value
        thermal_info = self.em_api.get_thermal_sensor_info()
        if self.__sensor not in thermal_info.keys():
            txt = "the %s sensor you want to test at CRITICAL level cannot be found" % self.__sensor
            self._logger.error(txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, txt)

        if self._em_cst.VALUE not in thermal_info[self.__sensor].keys():
            txt = "the %s sensor temperature cant be read" % self.__sensor
            self._logger.error(txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, txt)

        # increase all sensor values to isolate the sensor you want to test
        self.em_api.load_thermal_file(load_from_orig=True)
        # force a themal restoration to avoid reading info on the bad file
        self.em_api.restore_thermal_file()
        # check that the temperature you want to use is above or equal to critical threshold
        threshold_value = thermal_info[self.__sensor].get(self._em_cst.THRESHOLD_ALERT)
        if threshold_value is not None:
            if self.__temperature == "SMART":
                # adjust the whole threshold limit to easily reach the critical state
                self._logger.info("This test will try to smartly lower sensor temperature to reach the CRITICAL state quickly")
                # get critical temperature from sensor
                temp_value = thermal_info[self.__sensor][self._em_cst.VALUE][0]
                temp_delta = (threshold_value[0] - temp_value) / 2
                # if there is more than 5 degree between current sensor temp and critical threshold then we can lower the sensor value
                if temp_delta > 5:
                    self._logger.info("Sensor %s thresholds will be lower by %s" % (self.__sensor, temp_delta))
                    self.em_api.modify_thermal_threshold((temp_delta * 1000), self.__sensor, "all", operation="-")
                    self.__temperature = int(threshold_value[0] - temp_delta)
                else:
                    self.__temperature = int(threshold_value[0])

            elif self.__temperature < threshold_value[0]:
                txt = "the temperature you choose (%s) is below %s CRITICAL level at %s" % (self.__temperature,
                                                                                       self.__sensor, threshold_value[0])
                self._logger.error(txt)
                raise DeviceException(DeviceException.OPERATION_FAILED, txt)
        else:
            self._logger.warning("cannot read the threshold value for sensor %s" % self.__sensor)

        # increase all sensor values to isolate the sensor you want to test
        self.em_api.modify_thermal_threshold(100000, "all", "all", operation="+")

        # decrease the value of the sensor we want to test
        self.em_api.modify_thermal_threshold(100000, self.__sensor, "all", operation="-")

        self.em_api.update_thermal_file()
        # reactivate thermal manager if it was not
        self.em_api.set_thermal_management(True)
        # reboot the board to take into account the change
        self._device.reboot()

        # check that no corruption happen during the change process
        thermal_info = self.em_api.get_thermal_sensor_info()
        if self.__sensor not in thermal_info.keys():
            txt = "the %s sensor you want to test at CRITICAL level cannot be found, check if there was a file corruption during thermal threshold change" % self.__sensor
            self._logger.error(txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, txt)

        if self._em_cst.VALUE not in thermal_info[self.__sensor].keys():
            txt = "the %s sensor temperature cant be read, check if there was a file corruption during thermal threshold change" % self.__sensor
            self._logger.error(txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, txt)

        if self.tc_module is not None:
            self.tc_module.set_up_eq()
            self.tc_module.get_eq().set_regulation(True)
            self.tc_module.get_eq().set_temperature(self.__temperature)

        return Global.SUCCESS, "No errors"

    def __heat_board(self):
        """
        heat the board until critical state is seen.
        some sensor may not cause a shutdown buta change on battery/charger behavior.
        """
        self._logger.info("Try to heat the board until DUT critical temperature level is reached")
        chamber_temperature = self.__temperature
        thermal_info = self.em_api.get_thermal_sensor_info()
        last_temp = thermal_info[self.__sensor]["VALUE"][0]
        timeout_to_see_temp_move = time.time()
        shutdhown_counter = 10

        if self.tc_module is not None:
            exceed_timeout = True
            chb_temp = self.tc_module.get_eq().get_temperature()
            if chb_temp < chamber_temperature or chb_temp > (chamber_temperature + self.__max_temp_rising):
                self.tc_module.get_eq().set_temperature(chamber_temperature)
                self.tc_module.get_eq().wait_for_temperature(chamber_temperature)

            timeout = time.time() + self.__temperature_change_timeout
            while time.time() < timeout:
                current_mode = self._device.get_boot_mode()
                if current_mode == "MOS":
                    # reconnect and restore the shutdown counter if connection was lost then seen again
                    if shutdhown_counter < 10:
                        self._device.connect_board()
                        shutdhown_counter = 10
                    try:
                        # try to read measurement
                        em_info = self.em_api.get_msic_registers()
                        thermal_info = self.em_api.get_thermal_sensor_info()
                        self.__em_meas_tab.add_dict_measurement(em_info)
                        self.__em_meas_tab.add_dict_measurement(thermal_info)
                        temperature = thermal_info[self.__sensor]["VALUE"][0]

                        # if we are above the test temperature it means that we should be at critical level
                        # this is for sensor that may no have a critical shutdown
                        if temperature > self.__temperature:
                            self._meas_list.add_dict("EM_INFO_AT_CRITICAL_LEVEL", em_info)
                            self._meas_list.add("SHUTDOWN_HAPPEN", False, "")
                            exceed_timeout = False

                        # compare only the integer part
                        if int(last_temp) < int(temperature):
                            timeout_to_see_temp_move = time.time()
                        elif time.time() - timeout_to_see_temp_move > 60:
                            # Limit the temperature increasing
                            next_temp_rising = min(chamber_temperature + 1, self.__temperature + self.__max_temp_rising)
                            if (chamber_temperature != next_temp_rising):
                                self._logger.info("adjust equipment temperature to %s" % next_temp_rising)
                                chamber_temperature = next_temp_rising
                                self.tc_module.get_eq().set_temperature(chamber_temperature)
                            timeout_to_see_temp_move = time.time()
                        last_temp = temperature

                    except AcsBaseException as e:
                        # try to catch why uecmd may fail
                        if not self.is_board_and_acs_ok():
                            # if it is due to a shutdown , then exit here
                            txt = "connection with board lost during temperature change to reach CRITICAL level"
                            self._logger.info(txt)
                        else:
                            txt = "error happened during temperature change to reach CRITICAL level : " + str(e)
                            self._logger.error(txt)
                            raise DeviceException(DeviceException.OPERATION_FAILED, txt)
                    finally:
                        # Store various information
                        self.__em_meas_tab.add_measurement([self.get_time_tuple(),
                                                (self._em_cst.COMMENTS, "RUNTEST:heating board to reach CRITICAL state")])
                        # switch meas to next meas
                        self.__em_meas_tab.switch_to_next_meas()
                else:
                    shutdhown_counter -= 1

                if shutdhown_counter <= 0:
                    txt = "the board has been seen booted in another state than MOS 10 times"
                    self._logger.info(txt)
                    self._meas_list.add("SHUTDOWN_HAPPEN", True, "")
                    self._device.disconnect_board()
                    exceed_timeout = False
                    break

            # Get the last boot mode for debugging purpose
            self._meas_list.add("LAST_BOOT_MODE_SEEN", self._device.get_boot_mode(), "")
            self._em_meas_verdict.compare_list(self._meas_list, self._em_targets, clean_meas_list=True)

            if exceed_timeout:
                txt = "timeout exceeded (%ss) to heat up the board to CRITICAL temperature level" % str(self.__temperature_change_timeout)
                self._logger.error(txt)
                raise DeviceException(DeviceException.OPERATION_FAILED, txt)

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        Check the end of the main battery charge
        """
        # Call LAB_EM_BASE Run function
        EmUsecaseBase.run_test_body(self)
        self.__heat_board()
        self._em_meas_verdict.judge()

        return self._em_meas_verdict.get_current_result_v2()

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        # call tear down after some operations
        EmUsecaseBase.tear_down(self)

        # release equipment
        if self.tc_module is not None:
            self.tc_module.release_eq(temp_margin=0)

        thermal_restored = False
        try:
            # after a thermal shutdown , board may be stuck, we need to help it to boot
            if self._device.get_boot_mode() == "UNKNOWN":
                self._device.disconnect_board()
                # first try to see if we can boot in COS, it will be easier to restore thermal file from this state
                self._io_card.simulate_insertion(self._io_card.USB_HOST_PC)
                start_time = time.time()
                time_to_wait = 300
                while time.time() - start_time < time_to_wait:
                    mode = self._device.get_boot_mode()
                    if mode != "UNKNOWN":
                        self._logger.info("Board seen booted in %s after %ss" % (mode, time.time() - start_time))
                        if mode == "MOS":
                            self._device.connect_board()
                        thermal_restored = self.em_api.restore_thermal_file()
                        if thermal_restored:
                            break
                # try to boot in MOS to restore thermal
                if not thermal_restored:
                    # if cos is not seen then try to go in MOS
                    self._io_card.press_power_button(self.pwr_btn_boot)
                    # wait for a while to let the board boot
                    self._logger.info("Waiting at most %ss to see the boot mode" % time_to_wait)
                    start_time = time.time()
                    while time.time() - start_time < time_to_wait:
                        mode = self._device.get_boot_mode()
                        if mode != "UNKNOWN":
                            self._logger.info("Board seen booted in %s after %ss" % (mode, time.time() - start_time))
                            if mode == "MOS":
                                self._device.connect_board()
                            thermal_restored = self.em_api.restore_thermal_file()
                            if thermal_restored:
                                break
                # reboot to apply thermal change
                if thermal_restored:
                    self._device.reboot()

        except AcsBaseException as e:
            txt = "an error happen during first try to restore thermal file configuration: " + str(e)
            self._logger.info(txt)

        # clean the board state and retrieve logs
        self.em_core_module.clean_up()

        if not thermal_restored:
            # clean the board state and retrieve logs
            self.em_api.restore_thermal_file()
            self._device.reboot()

        return Global.SUCCESS, "No errors"
