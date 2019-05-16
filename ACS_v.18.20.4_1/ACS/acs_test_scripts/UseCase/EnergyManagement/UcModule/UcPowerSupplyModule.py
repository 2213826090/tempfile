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
:summary: Power supply module for EM usecase: io card and Power supply as battery
:author: vgombert
:since: September 17th 2013
"""

import os
import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.EnergyManagement.UcModule.UcBatteryModule import UcBatteryModule
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.TestEquipmentException import TestEquipmentException


class UcPowerSupplyModule(UcBatteryModule):
#--- POWER SUPPLY MODULE
    """
    Lab Energy Management base class.
    """
    __LOG_TAG = "[POWER_SUPPLY_BASE]\t"
    TYPE = "POWER_SUPPLY_BENCH"

    def __init__(self, uc_base):
        """
        Constructor
        """
        UcBatteryModule.__init__(self, uc_base)
        # Var common to all usecase, these warning are known
        # and cant be remove without changing framework var
        global_config = self._uc_base._global_conf
        self._em = self._uc_base._em
        self._uc_id_logged = self._uc_base._uc_id_logged
        self._execution_config_path = self._uc_base._execution_config_path

        # Get the path of execution config folder
        self._patlib = None
        self.eqp_init_state = {}
        self.ps_properties = {}
        # Get pwr supply config file folder from BenchConfig.xml
        self.vbatt = None
        self.vusb = None
        self.vac = None
        self.resistance = None

        # Get pwr supply config file folder from BenchConfig.xml
        # Read POWER_SUPPLY attributes
        # (Vbatt, Vusb, Vacc...) from BenchConfig.xml:
        #     - Instance of the service
        #     - PortNumber
        #     - CurrentLevel
        #     - VoltageLevel
        #     - VoltageProtectLevel

        for equipment_name in global_config.benchConfig.get_parameters_name():
            if "POWER_SUPPLY" not in equipment_name:
                continue

            pwrs = global_config.benchConfig.get_parameters(equipment_name)
            ps_type = pwrs.get_param_value("Model")
            # Get power supply properties if its type is not NONE
            if ps_type in ["NONE", None]:  # no output to configure
                continue

            for attr_name in pwrs.get_parameters_name():
                if "OUTPUT" not in attr_name:
                    continue

                out_p = pwrs.get_parameters(attr_name)
                out_t = out_p.get_param_value("Type")

                if "ACCHG" in out_t:
                    continue

                if out_t == "BATT":
                    self.vbatt = float(str(out_p.get_param_value("Voltage")))
                    self.resistance = float(pwrs.get_param_value("Resistance"))
                if out_t == "USB":
                    self.vusb = float(str(out_p.get_param_value("Voltage")))
                if out_t == "AC":
                    self.vac = float(str(out_p.get_param_value("Voltage")))

                # Get output properties if its name is not NONE
                if out_t == "NONE":
                    continue

                # For each output create an entry of ps_properties dictionary
                self.ps_properties[out_t] = {
                    "PortNumber": int(out_p.get_param_value("PortNumber")),
                    "CurrentLevel": float(str(out_p.get_param_value("MaxCurrent"))),
                    "VoltageLevel": float(str(out_p.get_param_value("Voltage"))),
                    "VoltageProtectLevel":
                    float(str(pwrs.get_param_value("VoltageProtectLevel")))}

                self.eqp_init_state[out_t] = {
                    "PortNumber": int(out_p.get_param_value("PortNumber")),
                    "CurrentLevel": float(str(out_p.get_param_value("MaxCurrent"))),
                    "VoltageLevel": float(str(out_p.get_param_value("Voltage"))),
                    "VoltageProtectLevel": float(str(pwrs.get_param_value("VoltageProtectLevel")))}
        # Set initial value for setting IO_CARD:
        self.io_card_init_state = {
            # the one from device model
            "BatteryType": self._uc_base.phone_info["BATTERY"]["BATTID_TYPE"],
            # battery plug platform ON
            "Battery": True,
            "Platform": "ON",
            "BatteryTemperature": 25}

        # GET THE EQUIPMENT DRIVER
        # use PAT if enable rather than power supply to measure
        if "POWER_ANALYZER_TOOL" in global_config.benchConfig.get_parameters_name():
            try:
                self._patlib = self._em.get_power_analyzer_tool("POWER_ANALYZER_TOOL")
                pat_node = global_config.benchConfig.get_parameters("POWER_ANALYZER_TOOL")
                self._pat_file = os.path.join(self._execution_config_path,
                                              pat_node.get_param_value("ConfFile"))

            except ImportError as error:
                self._logger.error(self.__LOG_TAG + " PAT is declared in your benchconfig but" +
                                   " can't be imported: %s" % str(error))

        # VBATT and VUSB VAC power supplies

        self.__pwrs_vusb = None
        self.__pwrs_vac = None
        self.__pwrs_vbatt = self._em.get_power_supply("BATT")
        if "USB" in self.eqp_init_state.keys():
            self.__pwrs_vusb = self._em.get_power_supply("USB")
        if "AC" in self.eqp_init_state.keys():
            self.__pwrs_vac = self._em.get_power_supply("AC")

        # # allow overriding of some parameters
        self.vbatt = self._tc_parameters.get_param_value("VBATT", self.vbatt, default_cast_type=float)
        self.vusb = self._tc_parameters.get_param_value("VUSB", self.vusb, default_cast_type=float)
        self.battery_type = self._tc_parameters.get_param_value("BATTERY_TYPE", self._uc_base.phone_info["BATTERY"]["BATTID_TYPE"])
        self.battery_temperature = int(self._tc_parameters.get_param_value("BATTERY_TEMPERATURE", 25))

        # set a default value for platform state
        # below dictionary is used to set the start condition
        # if you want to override start condition , you need to edit this entry on your usecase
        # some of these condition can be override from test case params
        self.io_card_init_state["BatteryType"] = self.battery_type
        self.io_card_init_state["BatteryTemperature"] = self.battery_temperature
        self.io_card_init_state["Battery"] = True
        # Platform ON mean board in MOS with data cable plugged
        # Platform OFF mean board OFF and no cable plugged
        self.io_card_init_state["Platform"] = "ON"

        # init common em parameters:
        self._boot_tries = 3
        self.wait_voltage_time = 120

    #--- POWER SUPPLY CONTROL

    def get_eq_emulated_battery(self):
        """
        for usecase used, allow to control the power supply dedicated to battery
        """
        return self.__pwrs_vbatt

    def get_eq_emulated_charger(self):
        """
        for usecase used, allow to control the power supply dedicated to charger
        """
        return self.__pwrs_vusb

    def _configure_power_supply(self, equipment_properties):
        """
        Configure power supplies (voltage, current)
        """
        # Configure settings for all power supplies (VBATT, VACC, VUSB...)
        for ps_name in self.ps_properties.keys():

            pwrs = self._em.get_power_supply(ps_name)
            self._logger.info(self.__LOG_TAG + " Configure Power Supply %s parameters", ps_name)

            if self.eqp_init_state[ps_name]["VoltageProtectLevel"] is not None:
                pwrs.set_voltage_protection_level(
                    equipment_properties[ps_name]["VoltageProtectLevel"])

            # Set the max current using CurrrentLevel and PortNumber
            if self.eqp_init_state[ps_name]["CurrentLevel"] is not None:
                pwrs.set_max_current(
                    equipment_properties[ps_name]["CurrentLevel"],
                    equipment_properties[ps_name]["PortNumber"])

            # Set Voltage level using VoltageLevel parameter
            if self.eqp_init_state[ps_name]["VoltageLevel"] is not None:
                pwrs.set_current_voltage(
                    equipment_properties[ps_name]["VoltageLevel"],
                    equipment_properties[ps_name]["PortNumber"])
            # switch on the power supply
            pwrs.power_on()

    def get_charger_current(self, charger_type, instrument="power_supply"):
        """
        get Iusb from DCP  or I ACcharger from pat or power supply
        depending on available equipment and charger plugged
        :type charger_type: str
        :param charger_type:  the charger current to measure
                              can be SDP or CDP or DCP
                              or AC_CHARGER or WWALL_CHARGER
        :type instrument: str
        :param instrument: the instrument to use
                           can be 'power_supply' or 'nidac'

        :rtype: tuple ( int, String)
        :return: return a tuple containing I, Unit(mA, A...)
                return a None tuple if i charger can't be measured
        """
        meas_icharger = ('None', 'A')
        # get WALL CHARGER definition
        if charger_type == self._io_card.WALL_CHARGER:
            charger_type = self._io_card.get_default_wall_charger()
        # check charger type : USB or AC
        if charger_type == self._io_card.AC_CHGR:
            # check the measurement instrument used
            if instrument == "power_supply":
                meas_icharger = (self.__pwrs_vac.get_current_meas_average(
                    self.ps_properties["AC"]["PortNumber"], "DC", 5), "A")
            elif instrument == "nidac":
                # get value from pat
                self._patlib.start_acquisition(True)
                # wait acquisition is done
                time.sleep(1)
                self._patlib.stop_acquisition()
                meas = self._patlib.get_measurements()[2]
                # store average value from second element
                # pat unit for I is mA
                meas_icharger = (float(meas[1]) / 1000, "A")
            else:
                raise AcsConfigException(AcsConfigException.UNKNOWN_EQT,
                                       "instrument %s is not supported" % instrument)
            self._logger.info(self.__LOG_TAG + " icharger = %s A (from %s)"
                              % (str(meas_icharger[0]), instrument))
        elif charger_type in [self._io_card.DCP, self._io_card.USB_HOST_PC, self._io_card.ACA,
                              self._io_card.CDP, self._io_card.SDP, self._io_card.OTG]:
            # check the measurement instrument used
            if instrument == "power_supply":
                meas_icharger = (self.get_eq_emulated_charger().get_current_meas_average(
                    self.ps_properties["USB"]["PortNumber"], "DC", 5), "A")
            elif instrument == "nidac":
                # get value from pat
                self._patlib.start_acquisition(True)
                # wait acquisition is done
                time.sleep(1)
                self._patlib.stop_acquisition()
                meas = self._patlib.get_measurements()[1]
                # store average value from second element
                # pat unit for I is mA
                meas_icharger = (float(meas[1]) / 1000, "A")
            else:
                raise AcsConfigException(AcsConfigException.UNKNOWN_EQT,
                                       "instrument %s is not supported" % instrument)
            self._logger.info(self.__LOG_TAG + " icharger = %s A (from %s)"
                              % (str(meas_icharger[0]), instrument))
        else:
            meas_icharger = ('None', 'A')
        return meas_icharger

    def get_battery_current(self, average=True, iteration=10,
                             instrument="power_supply"):
        """
        get Ibatt from power supply

        :type average: boolean
        :param average: active the average mode
        :type iteration: int
        :param iteration: number of iteration for the average computing
        :type instrument: str
        :param instrument: the instrument to use
                           can be 'power_supply' or 'nidac'

        :rtype: tuple ( int, String)
        :return: return a tuple containing I, Unit(mA, A...)
        """
        # check measurement instrument
        if instrument == "power_supply":
            # check average mode
            if average:
                meas_ibatt = (self.get_eq_emulated_battery().get_current_meas_average(
                    self.ps_properties["BATT"]["PortNumber"], "DC",
                              iteration), "A")
            else:
                meas_ibatt = (self.get_eq_emulated_battery().get_current_meas(
                    self.ps_properties["BATT"]["PortNumber"], "DC"), "A")
        elif instrument == "nidac":
            # get value from pat
            self._patlib.start_acquisition(True)
            # wait acquisition is done
            time.sleep(1)
            self._patlib.stop_acquisition()
            meas = self._patlib.get_measurements()[1]
            # store average value from second element
            # pat unit for I is mA
            meas_ibatt = (float(meas[0]) / 1000, meas[1])
        else:
            raise AcsConfigException(AcsConfigException.UNKNOWN_EQT,
                                   "instrument %s is not supported" % instrument)

        self._logger.info(self.__LOG_TAG + " ibatt = %s %s (from %s)"
                          % (str(meas_ibatt[0]), meas_ibatt[1], instrument))
        return meas_ibatt

    def set_battery_voltage(self, value):
        """
        Set Vbatt on power supply

        :type value: float
        :param value: value to set for vbatt
        """
        pwrs_vbatt = self._em.get_power_supply("BATT")
        pwrs_vbatt.set_current_voltage(value, self.ps_properties["BATT"]["PortNumber"])

    def __battery_connector(self, action, battery_type="DEFAULT"):
        """
        plug or unplug the battery and look for eventual power supply protection state.

        :type action: boolean
        :param action: True for plug action, False for unplug action

        :type battery_type: string
        :param battery_type: the battery type to plug
        """
        self._logger.info(self.__LOG_TAG + "[EM_BATTERY] current status before connecting/disconnecting the battery, battery current is : %s, charger current is : %s"\
                                % (str(self.get_battery_current()[0]), str(self.get_charger_current(self._io_card.USB_HOST_PC)[0])))
        self._io_card.battery_connector(action, battery_type)
        time.sleep(1)
        if self.__check_and_clean_ps_error(self.get_eq_emulated_battery(), False):
            self._io_card.battery_connector(action, battery_type)
            time.sleep(1)
            self.__check_and_clean_ps_error(self.get_eq_emulated_battery())

    def __check_and_clean_ps_error(self, powerSupply, raise_error=True):
        """
        check the status of the power supply
        if the status is not good,
        remove the protection and re-initialize the power supply

        :type powerSupply: power supply instance
        :param powerSupply: the power supply to check

        :type raise_error: boolean
        :param raise_error: specify the need to raise an error

        :rtype:boolean
        :return: True if error is still seen, False otherwise
        """
        result = False
        status = powerSupply.check_protection_state()
        self._logger.info(self.__LOG_TAG + " The power supply status is :%s" % status)
        if (status != "good") and not ("unknown" in status):
            self._logger.warning(self.__LOG_TAG + " Try to remove protection and restart the power supply")
            # Disconnect the USB and the battery
            self._io_card.usb_connector(False)
            time.sleep(1)
            # Configure power supply parameters
            powerSupply.reset_protection()
            if raise_error :
                raise TestEquipmentException(TestEquipmentException.PROHIBITIVE_BEHAVIOR,
                                   "the power supply health is %s" % status)
            else:
                status = powerSupply.check_protection_state()
                if (status != "good") and not ("unknown" in status):
                    result = True

        return result

    def __check_battery_is_valid(self):
        """
        check that battery is valid comparing the type we wanted
        invalid battery = invalid in sysfs
        valid battery = valid in sysfs
        """
        result = False
        msic = self._uc_base.em_api.get_msic_registers()
        batt_identification = msic["BATTERY"]["MODEL_NAME"][0].upper()
        self._logger.info("battery identification from DUT side is : %s" % str(batt_identification))

        # TODO : UNKNOWNB is an hardcoded value link to intel platform, it should be moved
        if  ((self.io_card_init_state["BatteryType"] in [self._io_card.BAT_DIG_INVALID , self._io_card.BAT_INVALID]) and (batt_identification == "UNKNOWNB")) or \
                ((self.io_card_init_state["BatteryType"] not in [self._io_card.BAT_DIG_INVALID , self._io_card.BAT_INVALID]) and (batt_identification != "UNKNOWNB")):
            result = True

        return result

    def __shutdown(self):
        """
        abstract layer for shutdown
        shutdown a board without keep a charger active
        this only used for this file and not dedicated
        to use outside
        """
        if self._uc_base.is_board_and_acs_ok():
            self._device.soft_shutdown(wait_for_board_off=True)
            self._io_card.remove_cable("ALL")
        else:
            # hard shutdown else
            self._device.disconnect_board()
            self._io_card.remove_cable("ALL")
            self._io_card.press_power_button(self._uc_base.pwr_btn_off)
            time.sleep(2)

    def __switch_battery_type(self):
        """
        switch the battery type the more naturally way
        """
        # TODO: if board was in MOS then perform a software shutdown first
        self.__shutdown()

        # disconnect the battery
        self.__battery_connector(False)
        time.sleep(2)
        # replug the battery only if we want it
        if self.io_card_init_state["Battery"] == True:
            # connect the battery
            self.__battery_connector(self.io_card_init_state["Battery"], self.battery_type)
            # TODO: need a check on battery current to search for a boot action at battery insertion
        # battery removal should be handled directly in the usecase and not here
        else:
            self._io_card.set_battery_type(self.battery_type)

    def __stablized_inserted_batt(self):
        """
        this function wait until stabilization of an inserted battery WITHOUT A CHARGER ONLY
        """
        required_iter = 5
        ibatt_stabilized = False
        timeout = 30
        ibatt = 9999
        last_ibatt = 9999
        unit = "A"
        self._logger.debug("try to see Ibatt stabilization after plugging battery before %ss" % str(timeout))
        end_time = time.time() + timeout
        while  time.time() < end_time and not ibatt_stabilized:
            ibatt, unit = self.get_battery_current()
            #  ibatt off should be < 100mA
            if ibatt < 0.100 and last_ibatt < 0.100:
                required_iter -= 1

            if required_iter <= 0:
                break


            last_ibatt = ibatt

        if required_iter < 0:
            txt = "IBATT has been seen stabilized around %s%s" % (ibatt, unit)
        else:
            txt = "IBATT has kept changing during %ss , last ibatt seen %s%s" % (timeout, ibatt, unit)

        self._logger.debug(txt)

    #--- BOOT FUNCTION

    def __boot_board(self, tries="default"):
        """
        try to boot board in MOS until it responds.

        :type mode: str
        :param mode: mode of boot. it can be MOS, POS, or ROS
        :type tries: int
        :param tries: number of tries for retrieving msic registers
        """
        self._logger.info(self.__LOG_TAG + "[EM_BOOT] try to boot board in MOS")
        failed_tries = 0
        board_booted = False
        board_on_reached = False
        if tries == "default":
            tries = self._boot_tries

        if self._device.get_boot_mode() == "MOS":
            self._device.connect_board()

        while self._uc_base.is_board_and_acs_ok() == False:

            # END OF TRIES
            if failed_tries > tries:
                tmp_txt = self.__LOG_TAG + "[EM_BOOT] Board has failed to boot or seen booted in MOS after %s tries, abort usecase" % failed_tries
                self._logger.error(tmp_txt)
                raise DeviceException(DeviceException.DUT_BOOT_ERROR, tmp_txt)

            if not board_on_reached:
                # if boot fail , we try to hard shutdown board
                if 0 < failed_tries < tries:
                    self._logger.error(self.__LOG_TAG + "[EM_BOOT] Board has failed to boot after %s tries, force shutdown and retry..." % failed_tries)
                    self._io_card.usb_host_pc_connector(False)
                    # turn board OFF
                    self._io_card.press_power_button(self._uc_base.pwr_btn_off)
                # if fail to boot again , try to remove battery and plug it
                elif failed_tries == tries and not board_on_reached:
                    self._logger.error(self.__LOG_TAG + "[EM_BOOT] Board has failed to boot after %s tries, remove battery and retry.." % failed_tries)
                    # disconnect usb
                    self._io_card.usb_host_pc_connector(False)
                    # disconnect battery
                    self.__battery_connector(False)
                    time.sleep(5)
                    # power cycling power supply
                    self._logger.info(self.__LOG_TAG + "[EM_BOOT] Power cycling board ...")
                    self.get_eq_emulated_battery().power_cycle()
                    # connect battery
                    self.__battery_connector(True)
                    self.__stablized_inserted_batt()

                # BOOT ACTION
                # turn board ON
                self._io_card.press_power_button(self._uc_base.pwr_btn_boot)
                time.sleep(2)
                # connect USB host pc
                self._io_card.usb_host_pc_connector(True)

                #  DETECT THE MOS BOOT
                start_time = time.time()
                # wait phone catalog boot time seconds
                self._logger.info(self.__LOG_TAG + "[EM_BOOT] Waiting for the board ready signal")
                timeout = self._device.get_boot_timeout()
                while(time.time() - start_time) < timeout:
                    if self._device.get_boot_mode() == "MOS":
                        self._logger.info(self.__LOG_TAG + "[EM_BOOT] board has booted in %s seconds" % str((time.time() - start_time)))
                        # connect board
                        self._device.connect_board()
                        # wait some sec to settle down
                        if self._device.is_available():
                            time.sleep(self._device._settledown_duration)
                            # launch acs agent if it was not the case
                            self._device.init_acs_agent()
                            board_booted = True
                        break
                    elif self._device.get_boot_mode() in ["COS", "ROS", "POS"]:
                        board_booted = self._device.reboot("MOS", wait_for_transition=True, skip_failure=False, wait_settledown_duration=True)
                        break
                # if board is booted but fail to go in MOS with acs connected, just reboot
            else:
                board_booted = self._device.reboot("MOS", wait_for_transition=True, skip_failure=False, wait_settledown_duration=True)

            if board_booted:
                break
            else:
                failed_tries += 1
                self._logger.error(self.__LOG_TAG + "[EM_BOOT] boot failed, battery current is : %s, charger current is : %s" \
                                  % (str(self.get_battery_current()[0]), str(self.get_charger_current(self._io_card.USB_HOST_PC)[0])))

        self._logger.info(self.__LOG_TAG + "[EM_BOOT] board is ready")

    #---- FUEL GAUGING RESTORATION

    def control_battery_capacity(self, capacity=15, max_wait_time=1000, vbatt=4):
        """
        try to charge/boot board until the battery capacity is correct.

        :type capacity: int
        :param capacity: battery capacity read in msic registers

        :type max_wait_time: int
        :param max_wait_time: max time to wait

        :type vbatt: float
        :param vbatt: vbatt for the charge
        """
        self._logger.info(self.__LOG_TAG + "[CAP_CONTROL] charge the board at the capacity of " +
                          "%s until %s second" % (str(capacity), str(max_wait_time)))

        batt_cap = 0
        start_time = time.time()
        # set vbatt
        self.get_eq_emulated_battery().set_current_voltage(vbatt, self.ps_properties["BATT"]["PortNumber"])

        # connect usb host pc, the board may be ON but without SDP
        if self._device.get_state() != "alive":
            # connect USB host pc
            self.plug_charger(self._io_card.USB_HOST_PC)

        while time.time() - start_time < max_wait_time:
            if self._device.get_state() != "alive":
                self._logger.info(self.__LOG_TAG + "[CAP_CONTROL] boot the board and try to" +
                                  " check battery capacity")
                # disconnect usb host pc in order to avoid COS boot
                self.unplug_charger(self._io_card.USB_HOST_PC)
                # switch off the board
                self._io_card.press_power_button(10)
                # switch on the board (and connect SDP)
                self.__boot_board()
            # try to connect board
            if self._device.get_state() == "alive":
                self._device.connect_board()
                msic = self._uc_base.em_api.get_msic_registers()
                batt_cap = msic["BATTERY"]["CAPACITY"][0]
                self._logger.info(self.__LOG_TAG + "[CAP_CONTROL] battery capacity is %s"
                                  % str(batt_cap))
                if batt_cap >= capacity:
                    self._logger.info(self.__LOG_TAG + "[CAP_CONTROL] battery capacity is" +
                                      " reached for the next step")
                    break
            else:
                self._logger.debug(self.__LOG_TAG + "[CAP_CONTROL] device not started in MOS")

            self._logger.debug(self.__LOG_TAG + "[CAP_CONTROL] reboot the board in" +
                               " COS in order to charge")

            # disconnect board
            self._device.disconnect_board()

            # disconnect usb
            self.unplug_charger(self._io_card.USB_HOST_PC)

            # disconnect battery
            self.__battery_connector(False)
            time.sleep(3)

            # connect battery
            self.__battery_connector(True)
            time.sleep(10)

            # Set USB charger type using WALL_CHARGER
            self.plug_charger(self._io_card.WALL_CHARGER, ext_ps=False)

            # wait 2 min
            time.sleep(self.wait_voltage_time)

            # disconnect charger
            self.unplug_charger(self._io_card.WALL_CHARGER)

        self._logger.info(self.__LOG_TAG + "[CAP_CONTROL] total waiting time during fake " +
                          "charge: %s min" % str((time.time() - start_time) / 60))

    #---- UC FUNCTION

    def set_up(self):
        """
        Initialize the test
        """
        # Call the setup from inheritance
        UcBatteryModule.set_up(self)
        action_on_battery = False

        if self.ps_properties == {}:
            raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG,
                "Energy Management use cases are not functional without Power Supply !")

        if self._patlib is not None:
            self._patlib.configure(self._pat_file)

        # check the power supply health
        self.__check_and_clean_ps_error(self.get_eq_emulated_battery(), False)

        # Configure power supply parameters
        # TODO: this redundant action should be done once
        self._configure_power_supply(self.eqp_init_state)

        # Set Battery temperature using BatteryTemperature parameter
        self._io_card.set_battery_temperature(self.io_card_init_state["BatteryTemperature"])

        # TODO: need to check if there is a delta between init value and current value

        # SET INITIAL STATE ON IO CARD
        # Insert/Remove battery using Battery parameter
        self._logger.info("set up: batt_state = %s ; batt_expected = %s" % (str(self._io_card.get_battery_state()),
                                                                            self.io_card_init_state["Battery"]))
        self._logger.info("set up: current battery type = %s ; type_expected = %s" % (str(self._io_card.get_battery_type()),
                                                                                      self.io_card_init_state["BatteryType"]))
        if (self._io_card.get_battery_state() != self.io_card_init_state["Battery"]) or \
                (self.io_card_init_state["BatteryType"] != self._io_card.get_battery_type()):

            self._logger.info("set up: wanted battery type %s - wanted state %s" % (self.io_card_init_state["BatteryType"],
                                                                                  self.io_card_init_state["Battery"]))
            self.__switch_battery_type()
            if self.io_card_init_state["Battery"] == True:
                self.__stablized_inserted_batt()
            action_on_battery = True

        # Platform CANT be ON without a battery in most of case but some DUT can accept that.
        if self.io_card_init_state["Platform"] == "ON":
            # Initialize Variable

            # boot board
            self.__boot_board()

            # CHECK BATTERY IDENTIFICATION
            if self._uc_base.is_board_and_acs_ok():
                is_valid_batt = self.__check_battery_is_valid()
                if not is_valid_batt:
                        txt = "Battery is not recognized as expected %s by the DUT, cant do the test" % self.battery_type
                        self._logger.error(self.__LOG_TAG + txt)
                        raise AcsConfigException(AcsConfigException.OPERATION_FAILED, txt)
                else:
                    self._logger.info("Expected battery type seen")
                    # Log test id to logcat to retrieve application log from here
                    if not self._uc_id_logged:
                        self._uc_id_logged = self._device.inject_device_log("i", self._em_cst.EM_INJECT_TAG, self._uc_base.em_stamp)
                        self._device.inject_device_log("i", "ACS_TESTCASE", "SETUP")

        # Else turn off board
        elif self.io_card_init_state["Platform"] == "OFF":
            boot_mode = self._device.get_boot_mode()
            if boot_mode != "UNKNOWN":
                if not self._uc_id_logged:
                    # log testcase tag
                    self._uc_id_logged = self._device.inject_device_log("i", self._em_cst.EM_INJECT_TAG, self._uc_base.em_stamp)

                self.__shutdown()
            else:
                # TODO: we know that an action on battery mean to have perform a shutdown and remove the charger
                if not action_on_battery:
                    self.__shutdown()

        return Global.SUCCESS, "No errors"

    def tear_down(self):
        """
        End and dispose the test
        """
        # check the power supply health
        if  self.__check_and_clean_ps_error(self.get_eq_emulated_battery(), True):
            self.get_eq_emulated_battery().configure_basic_parameters()

        # Set Battery temperature using BatteryTemperature parameter
        self._io_card.set_battery_temperature(25)

        # call tear down of the common base
        UcBatteryModule.tear_down(self)

        if self._device.get_boot_mode() != "MOS":
            # connect USB host pc
            self._logger.info(self.__LOG_TAG + "Connect usb host and wait for board state")
            self._io_card.usb_host_pc_connector(True)
            # wait some second to check if there is a usb connection
            timeout = 40
            start_time = time.time()
            while(time.time() - start_time) < timeout:
                if self._device.get_boot_mode() == "MOS":
                    # wait some sec to settle down
                    time.sleep(self._uc_base.usb_sleep)
                    # connect board
                    self._device.connect_board()
                    break

        if self._device.get_boot_mode() == "MOS":
            self._device.connect_board()
            # Call Control battery capacity to avoid OCV effect
            # self.control_battery_capacity(20, 1000)
        else:
            # Configure power supply parameters
            self._configure_power_supply(self.ps_properties)
            self._io_card.usb_connector(False)
            # disconnect the battery
            self.__battery_connector(False)
            time.sleep(3)
            # connect the battery
            self.__battery_connector(True)
            time.sleep(10)
            self.__boot_board()

        self._logger.info(self.__LOG_TAG + " Cleaning created files on DUT...")
        # clean daemon files
        if self._uc_base.is_board_and_acs_ok():
            self._uc_base.phonesystem_api.clean_daemon_files()
            # get application logs
            self._uc_base.get_application_logs()
        return Global.SUCCESS, "No errors"

    #---- WORK ARROUND AGAINST FREEZE

    def _freeze_during_boot_verification(self):
        """
        Kept but no more used
        """

        # Start of Boot Security
        # Before connexion, Check if the board is not freezed during the boot without charger pluggin
        # Max acquisition value
        max_measure_number = 15
        # Acquisition number
        measured_value_number = 0
        # Current offset value
        offset_value = 0.015
        # New measured value
        actual_value = 0.0
        # Previous measured value
        previous_value = 0.0
        # Board freeze cpt
        same_value_cpt = 0

        # Do "max_measure_number" acquisition
        while (measured_value_number < max_measure_number):
            actual_value = self.get_battery_current()[0]
            # Compare if the battery current is the same
            if (measured_value_number != 0):
                if ((previous_value - offset_value) < actual_value < (previous_value + offset_value)):
                    same_value_cpt += 1
            # If 7 value are the same => Board is freezed
            if (same_value_cpt >= 7):
                self._logger.info(self.__LOG_TAG + "[EM_BOOT] Board is freezed... Hardware Shutdown...")
                self._io_card.press_power_button(10)
                time.sleep(1)
                self._logger.info(self.__LOG_TAG + "[EM_BOOT] Press the power button...")
                self._io_card.press_power_button(self._uc_base.pwr_btn_boot)
                time.sleep(5)
                previous_value = 0
                same_value_cpt = 0
                measured_value_number = 0
            previous_value = actual_value
            measured_value_number += 1
            time.sleep(1)
        self._logger.info(self.__LOG_TAG + "[EM_BOOT] Battery current is correct : Can check the driver problem...")

    def _charger_driver_verification(self):
        """
        kept but no more used
        """
        # Verify if the charger driver problem is present
        # Max acquisition value
        max_measure_number = 15
        # Acquisition number
        measured_value_number = 0
        # New measured value
        actual_value = 0.0
        # Driver problem cpt
        problem_cpt = 0

        # Do "max_measure_number" acquisition
        while (measured_value_number < max_measure_number):
            actual_value = self.get_battery_current()[0]
            # If boot mode is UNKNOWN and battery current negative => Freeze with charger
            if ((self._device.get_boot_mode()  not in ["MOS", "COS", "ROS", "POS"]) and (actual_value < 0.0)):
                self._logger.info(self.__LOG_TAG + "[EM_BOOT] Freeze during the boot with charger pluggin...")
                problem_cpt += 1
            # If board booted, try to read uevent => If problem, charger driver issue
            elif (self._device.get_boot_mode()  in ["MOS", "COS", "ROS", "POS"]):
                self._device.connect_board()
                if (self.io_card_init_state["BatteryType"] != "DIG_INVALID"):
                    try:
                        msic = self._uc_base.em_api.get_msic_registers()
                        batt_identification = msic["BATTERY"]["MODEL_NAME"][0]
                        battery_status = msic["BATTERY"]["STATUS"][0]
                        if self.io_card_init_state["BatteryType"] != "INVALID":
                            if ((batt_identification != "UNKNOWNB") and (battery_status != "NOT CHARGING")):
                                self._logger.info(self.__LOG_TAG + "[EM_BOOT] Boot is OK... Continue the test...")
                                break
                            else:
                                self._logger.info(self.__LOG_TAG + "[EM_BOOT] Problem with the driver of the charger... Hardware Shutdown")
                                problem_cpt = 7
                        else:
                            self._logger.info(self.__LOG_TAG + "[EM_BOOT] Boot is OK... Continue the test...")
                            break
                    except:
                        self._logger.info(self.__LOG_TAG + "[EM_BOOT] Problem with the driver of the charger... Hardware Shutdown")
                        problem_cpt = 7
                else:
                    break

            # If board is freezed during boot with charger, Hard shutdown
            if (problem_cpt >= 7):
                self._logger.info(self.__LOG_TAG + "[EM_BOOT] Driver problem is seen... Hardware Shutdown...")
                self._device.disconnect_board()
                self._io_card.press_power_button(10)
                time.sleep(1)
                self._logger.info(self.__LOG_TAG + "[EM_BOOT] Press the power button...")
                self._io_card.press_power_button(self._uc_base.pwr_btn_boot)
                time.sleep(5)
                problem_cpt = 0
                measured_value_number = 0
            measured_value_number += 1
            time.sleep(1)
        self._logger.info(self.__LOG_TAG + "[EM_BOOT] Charger driver is OK : Can check the boot mode...")
        # End of boot security
