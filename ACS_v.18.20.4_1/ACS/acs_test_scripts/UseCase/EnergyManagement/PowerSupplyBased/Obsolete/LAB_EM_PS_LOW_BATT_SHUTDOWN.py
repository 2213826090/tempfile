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
:summary: EM - when the battery is very low, the phone shall initiate a graceful shutdown
:author: dbatutx
:since: 12/11/2012
"""
import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.AcsBaseException import AcsBaseException


class LabEmPsLowBattShutdown(EmUsecaseBase):

    """
    Lab Energy Management class.
    """
    DEDICATED_BENCH = "POWER_SUPPLY_BENCH"

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LAB_EM_BASE_PS Init function
        EmUsecaseBase.__init__(self, tc_name, global_config)

        # Read the start VBATT from TC parameters
        self.__start_vbatt = float(self._tc_parameters.get_param_value("START_VBATT"))

        # Read the lower VBATT from TC parameters
        self.__low_vbatt = float(self._tc_parameters.get_param_value("LOW_VBATT"))

        # Read the VBATT board extinction threshold from TC parameters
        self.__vbatt_extinction = float(self._tc_parameters.get_param_value("VBATT_EXTINCTION"))

        # Read the timeout from TC parameters
        self.__timeout = int(self._tc_parameters.get_param_value("TIMEOUT"))

        # get targets
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_PS_LOW_BATT_SHUTDOWN", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

        # Redefine initial value for setting USBDIO:
        self.em_core_module.io_card_init_state["BatteryType"] = self.phone_info["BATTERY"]["BATTID_TYPE"]
        self.em_core_module.io_card_init_state["Battery"] = False
        self.em_core_module.io_card_init_state["Platform"] = "OFF"
        self.em_core_module.io_card_init_state["USBChargerType"] = self._io_card.USB_HOST_PC
        self.em_core_module.io_card_init_state["USBCharger"] = False
        self.em_core_module.io_card_init_state["BatteryTemperature"] = 25

        # Set initial value for setting Power Supply VBATT:
        # - VoltageLevel = VBATT
        self.em_core_module.eqp_init_state["BATT"]["VoltageLevel"] = self.__start_vbatt

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # Call the LabEmBasePS Setup function
        EmUsecaseBase.set_up(self)

        self._logger.info("Boot the board with vbatt = %s." % self.__start_vbatt)

        # Insert battery
        self._io_card.battery_connector(True)

        # set power supply resistor to 0 in order to get a better accuracy on the voltage
        self.em_core_module.pwrs_vbatt.set_resistance(0)

        # boot the board
        self.em_core_module.boot_board()

        # wake up the screen
        self.phonesystem_api.wake_screen()

        # unplug the SDP
        self._device.disconnect_board()
        self.em_core_module.unplug_charger("SDP")

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        """
        # Call LAB_EM_BASE_PS Run function
        EmUsecaseBase.run_test_body(self)

        #
        # put the board in very low battery state, juste before extinction
        #

        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()

        self._logger.info("Initial state : IBATT = %s %s" % (meas_ibatt[0], meas_ibatt[1]))

        # store value in list for later comparison
        self._meas_list.add("IBATT1", meas_ibatt)

        # change vbatt to the test value
        self.em_core_module.pwrs_vbatt.set_current_voltage(self.__low_vbatt,
                                             self.em_core_module.ps_properties["BATT"]["PortNumber"])
        # wait some seconds
        time.sleep(5)

        # plug the SDP
        self.em_core_module.plug_charger("SDP")
        self.em_core_module.check_board_connection(only_reconnect=True)

        # dicharge the board to reach VBATT_EXTINCTION
        self._discharge_board(self.__vbatt_extinction + 0.06)

        # put the board in idle state
        if self.phonesystem_api.get_screen_status():
            self._io_card.press_power_button(0.3)

        # unplug the SDP
        self._device.disconnect_board()
        self.em_core_module.unplug_charger("SDP")
        time.sleep(10)

        #
        # monitor the board in order to capture the extinction event
        #
        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()

        # init of the while loop
        timeout = self.__timeout + time.time()
        if "IBATT2" in self._em_targets:
            ibatt_idle_max = float(self._em_targets["IBATT2"]["lo_lim"])
        else:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                   "The parameter does not trig the good target parameters, " +
                                   "IBATT2 are not present")
        # the board can wake up alone for battery pop up message
        if meas_ibatt[0] > ibatt_idle_max / 2:
            # put the board in idle state
            self._io_card.press_power_button(0.3)
            time.sleep(2)
        # change vbatt to the test value
        self.em_core_module.pwrs_vbatt.set_current_voltage(self.__vbatt_extinction - 0.05,
                                             self.em_core_module.ps_properties["BATT"]["PortNumber"])
        time.sleep(20)
        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()
        # wait for ibatt rising during extinction
        self._logger.info("Waiting for board wake up (ibatt>%s)" % str(ibatt_idle_max) +
                          "  in order to initiate a graceful shutdown")
        time_before_shutdown = time.time()
        while (meas_ibatt[0] < ibatt_idle_max) and (time.time() < timeout):
            # Measure current from Vbatt
            # wait 0.5s
            time.sleep(0.5)
            meas_ibatt = self.em_core_module.get_battery_current()

        # check why we exit the loop
        if meas_ibatt[0] > ibatt_idle_max:
            self._logger.info("after wainting %s s, " % str(time.time() - (timeout - self.__timeout)) +
                              " a current peak occured at ibatt = %s %s" % (str(meas_ibatt[0]), str(meas_ibatt[1])))
            # store value in list for later comparison
            self._meas_list.add("IBATT2", max(self._monitor_current_peak(60), meas_ibatt[0]), "A")
        elif time.time() > timeout:
            self._logger.info("the timeout was reached")
            # store value in list for later comparison
            self._meas_list.add("IBATT2", meas_ibatt)
        time_after_shutdown = time.time()
        #
        # try to reboot the board to get measure
        #

        # change vbatt to the start value
        self.em_core_module.pwrs_vbatt.set_current_voltage(self.em_core_module.vbatt,
                                             self.em_core_module.ps_properties["BATT"]["PortNumber"])

        # wait 5s
        time.sleep(5)

        # boot the board
        self.em_core_module.boot_board()

        # put the board in idle state
        if self.phonesystem_api.get_screen_status():
            self._io_card.press_power_button(0.3)

        # Measure current from Vbatt
        time.sleep(20)
        meas_ibatt = self.em_core_module.get_battery_current()
        self._logger.info("reboot state : IBATT = %s %s" % (meas_ibatt[0], meas_ibatt[1]))

        # store value in list for later comparison
        self._meas_list.add("IBATT3", meas_ibatt)

        # check for shutdown  reason
        shutdown_reason = self.phonesystem_api.check_message_in_log("SHUTDOWN_REASON",
                                                                     0, check_result=True)

        # store value in dict for later comparison
        self._meas_list.add("SHUTDOWN_REASON", shutdown_reason[0], "none")

        # check for graceful shutdown event
        graceful_shutdown = self.phonesystem_api.check_message_in_log("GRACEFUL_SHUTDOWN",
                                                                       time_before_shutdown, time_after_shutdown, check_result=False)

        # store value in dict for later comparison
        self._meas_list.add("GRACEFUL_SHUTDOWN", graceful_shutdown[0], "none")

        # compare values with targets
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
        self._meas_list.clean()

        # Save data report in xml file
        self._error.Code = self._em_meas_verdict.get_current_result()
        self._error.Msg = self._em_meas_verdict.save_data_report_file()

        return self._error.Code, self._error.Msg

#--------------------------------------------------------------------------

    def _discharge_board(self, vbatt):
        """
        discharge the board to the vbatt aimed
        :type vbatt: float
        :param vbatt: vbatt to reached
        """
        # init capacity
        msic = self.em_api.get_msic_registers()
        batt_voltage = msic["BATTERY"]["VOLTAGE"][0]
        batt_capacity = msic["BATTERY"]["CAPACITY"][0]

        # Charge battery
        if batt_voltage > vbatt:
            self._logger.info("Start to discharge battery at %s V" % vbatt)

            # launch uecmd to help the discharge
            self.phonesystem_api.set_vibration("on")
            self.phonesystem_api.set_torchlight("on")
            timer = time.time()
            retry = 3
            time_limit = time.time() + 60 * 60
            while batt_voltage > vbatt and batt_capacity > 1:
                try:
                    # try to read measurement
                    self.em_api.set_usb_charging("off")
                    self.phonesystem_api.set_vibration("on")
                    msic_reg = self.em_api.get_msic_registers()
                    time.sleep(1)
                    batt_voltage = msic_reg["BATTERY"]["VOLTAGE"][0]
                    batt_capacity = msic_reg["BATTERY"]["CAPACITY"][0]
                    if time_limit < time.time():
                        self._logger.error("The battery capacity is 0 with vbatt = %sV" % batt_voltage)
                        break
                    if batt_capacity == 1:
                        break
                except AcsBaseException as e:
                    # try to reconnect to the board if uecmd failed
                    self._logger.error("fail to get measurement: " + str(e))
                    if retry < 1:
                        self._logger.error("exit from discharge function")
                        return
                    retry -= 1

            # stop uecmd
            self.phonesystem_api.set_vibration("off")
            self.phonesystem_api.set_torchlight("off")

            # restore the usb charging
            self.em_api.set_usb_charging("on")

            self._logger.info("The board was discharged in %s min" %
                              str((time.time() - timer) / 60))
            self._logger.info("battery voltage = %s | capacity = %s" %
                             (str(batt_voltage), str(batt_capacity)))

#---------------------------------------------------------------------------

    def _monitor_current_peak(self, timeout):
        """
        Monitor the battery current in order to catch the  max current value during timeout
        :type timeout: float
        :param timeout: time to monitor in second

        :rtype: tuple ( int, String)
        :return: return a tuple containing I, Unit(mA, A...)
        """
        self._logger.info("Monitor current peak during %s second" % timeout)
        timer = time.time() + timeout
        # Measure current from Vbatt
        max_ibatt = self.em_core_module.get_battery_current()[0]
        meas_ibatt = (0, "A")
        while timer > time.time():
            # Measure current from Vbatt
            meas_ibatt = self.em_core_module.get_battery_current()
            # replace max value if this current is upper
            max_ibatt = max(meas_ibatt[0], max_ibatt)
        self._logger.info("The Max battery current peak is : %f %s" % (max_ibatt, meas_ibatt[1]))
        return max_ibatt

    def tear_down(self):
        """
        End and dispose the test
        """
        EmUsecaseBase.tear_down(self)

        # restore power supply resistance
        self.em_core_module.pwrs_vbatt.set_resistance(self.em_core_module.resistance)

        return Global.SUCCESS, "No errors"
