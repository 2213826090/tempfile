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
:summary: PUPDR -  usecase that allow user to create a sequence of action.
These action can touch the following battery type and insertion, charger type and insertion,
button or button combo press, and sleep.
for example by matching the right action, you can perform a fastboot boot with power button.
:author: vgomberx
:since: 08/04/2013(April)
"""
from time import sleep
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from ErrorHandling.AcsConfigException import AcsConfigException


class LabPupdrPsActionScheduler(EmUsecaseBase):

    """
    Lab Energy Management class.
    """
    # HIGH LEVEL ACTION
    DEDICATED_BENCH = "POWER_SUPPLY_BENCH"
    BATT_ACTION = "BATTERY"
    CHGR_ACTION = "CHARGER"
    BTN_ACTION = "BUTTON_PRESS"
    WAIT_ACTION = "WAIT"
    LOOP_BEGIN_ACTION = "LOOP_START"
    LOOP_END_ACTION = "LOOP_STOP"
    # LOW LEVEL ACTION
    PLUG_ACTION = "PLUG"
    UNPLUG_ACTION = "UNPLUG"
    # ACTION THAT INCLUDE VERDICT
    GET_ACTION = "GET"
    SET_ACTION = "SET"
    BOOT_MODE = "BOOT_MODE"
    GET_LIST = ["CURRENT", "VOLTAGE"]
    SET_LIST = ["CURRENT", "VOLTAGE"]

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call  Init from base
        EmUsecaseBase.__init__(self, tc_name, global_config)
        # get TC parameters
        self.__key_combo = self._tc_parameters.get_param_value("KEY_COMBO", "").upper()
        action_order = self._tc_parameters.get_param_value("RUN_TEST_ACTION", "").upper()
        action_for_setup_only = self._tc_parameters.get_param_value("SETUP_ACTION", "").upper()
        self.__test_id = self._tc_parameters.get_param_value("TEST_ID", "").upper()

        # switch to a list type
        self.__key_combo = self.__key_combo.replace(" ", "").split(";")
        action_order = action_order.replace(" ", "").replace("\n", "").split(";")
        action_for_setup_only = action_for_setup_only.replace(" ", "").replace("\n", "").split(";")
        self.__curr_vbatt = None

        if self.em_core_module.TYPE == "POWER_SUPPLY_BENCH":
            # test wil always start board OFF but battery state can change
            self.em_core_module.io_card_init_state["BatteryType"] = self.phone_info["BATTERY"]["BATTID_TYPE"]
            self.em_core_module.io_card_init_state["Battery"] = True
            self.em_core_module.io_card_init_state["Platform"] = "ON"
            self.em_core_module.io_card_init_state["USBChargerType"] = self._io_card.SDP
            self.em_core_module.io_card_init_state["USBCharger"] = True
            self.em_core_module.io_card_init_state["BatteryTemperature"] = 25
            # Set initial value for setting Power Supply VBATT:
            # - VoltageLevel = VBATT
            self.em_core_module.eqp_init_state["BATT"]["VoltageLevel"] = self.em_core_module.vbatt
            self.__current_vbatt = self.em_core_module.vbatt

        self.__vbatt_thresold = {
              "BOOT_MOS": self.phone_info["BATTERY"].get("VBATT_MOS_BOOT"),
              "BOOT_COS": self.phone_info["BATTERY"].get("VBATT_COS_BOOT"),
              "CRITICAL_MOS": self.phone_info["BATTERY"].get("VBATT_MOS_SHUTDOWN"),
              "FLASH": self.phone_info["BATTERY"].get("VBATT_FLASH"),
              "FULL": self.phone_info["BATTERY"].get("VBATT_FULL"),
              "OVERVOLTAGE": self.phone_info["BATTERY"].get("VBATT_OVERVOLTAGE")}

        self.__pwr_button_keyword = {
              "HARD_SHUTDOWN": self.phone_info["GENERAL"].get("PRESS_HARD_SHUTDOWN"),
              "SOFT_SHUTDOWN": self.phone_info["GENERAL"].get("PRESS_SOFT_SHUTDOWN"),
              "BOOT": self.phone_info["GENERAL"].get("PRESS_BOOT")}

        self.__last_battery_state = None
        self.__last_plug_charger = None

        self.__action_to_perform_run_test = self.__compute_action(action_order)
        self.__action_to_perform_set_up = self.__compute_action(action_for_setup_only)

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # Call the uc base Setup function to allow ACS tag to be injected in board logs
        EmUsecaseBase.set_up(self)
        # moved var for B2B test
        self.__last_plug_charger = self._io_card.SDP
        self.__last_battery_state = (True, self.phone_info["BATTERY"]["BATTID_TYPE"])
        # get targets
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_PUPDR_PS_ACTION_SCHEDULER", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())
        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)
        # disconnect acs parser at this time
        self._device.disconnect_board()
        # perform action to bring board in wanted state
        for action in self.__action_to_perform_set_up:
            action["FCT"](*action["PARAMS"])
            sleep(1)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        """
        # Call LAB_EM_BASE_PS Run function
        EmUsecaseBase.run_test_body(self)
        # sleep for a while to let last modification be taken into account
        sleep(5)
        for action in self.__action_to_perform_run_test:
            action["FCT"](*action["PARAMS"])
            sleep(1)

        # compare values with targets
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets, True)
        self._em_meas_verdict.judge(True)

        # Save data report in xml file
        self._error.Code = self._em_meas_verdict.get_current_result()
        self._error.Msg = self._em_meas_verdict.save_data_report_file()

        return self._error.Code, self._error.Msg

    def tear_down(self):
        """
        End and dispose the test
        """
        # try to reboot board if it is not in MOS
        if self._device.get_state() != "alive":
            self._io_card.usb_host_pc_connector(True)
            sleep(self.usb_sleep)
        # restore battery type and reboot
        if self.em_core_module.TYPE == "POWER_SUPPLY_BENCH":
            # check battery voltage and change it at least to be able to reboot board if it was off
            if self.__current_vbatt is not None and self.__current_vbatt < self.__vbatt_thresold["BOOT_MOS"]:
                self.em_core_module.set_battery_voltage(self.__vbatt_thresold["BOOT_MOS"] + 0.1)
            # check battery type
            if self.__last_battery_state is not None and (not self.__last_battery_state[0] or
                                                                  self.__last_battery_state[1] != self._io_card.get_default_battery_type()):
                # if battery is plugged, perform an hard shutdown
                if self.__last_battery_state[0]:
                    self._io_card.press_power_button(10)
                self._io_card.battery_connector(False)
                self._io_card.battery_connector(True)
                self.em_core_module.reboot_board()
        else:
            # else try to reboot using soft reboot
            boot_mode = self._device.get_boot_mode()
            if boot_mode in ["POS", "ROS", "COS"]:
                self._device.reboot(skip_failure=True)
                # then execute base tear down
        EmUsecaseBase.tear_down(self)
        return Global.SUCCESS, "No errors"

    def __compute_action(self, action_list):
        """
        map action from testcase to function to execute

        :type action_list: list of str
        :param action_list: a list of action to perform on the board

        :rtype: list
        :return: list of (fct, params) to execute
        """
        result = []
        index_list_for_looping = []
        for action in action_list:
            error_txt = "Unknown action: %s" % action
            fct_to_execute = {}
            params = []
            fct = None
            action = action.strip()
            #  compute action
            if action.startswith(self.BATT_ACTION):
                fct, params = self.__decode_battery(action)
            elif action.startswith(self.CHGR_ACTION):
                fct, params = self.__decode_charger(action)
            elif action.startswith(self.BTN_ACTION):
                fct, params = self.__decode_button(action)
            elif action.startswith(self.WAIT_ACTION):
                fct, params = self.__decode_wait(action)
            elif action.startswith(self.GET_ACTION):
                fct, params = self.__decode_global_get(action)
            elif action.startswith(self.LOOP_BEGIN_ACTION):
                # Get the first index element where iteration begin
                # which correspond to the last element inside result list
                index_list_for_looping.append(max(0, len(result) - 1))
                continue
            elif action.startswith(self.LOOP_END_ACTION):
                # if first loop index is set we compute the end of loopings
                if len(index_list_for_looping) > 0:
                    # get the more recent index where to start looping
                    result = self.__decode_action_loop(index_list_for_looping.pop(), result, action)
                else:
                    txt = "Missing start key %s" % self.LOOP_BEGIN_ACTION
                    self._logger.error(txt)
                    raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, txt)
                continue
            elif action == "":
                self._logger.warning("detected empty action")
                continue
            else:
                self._logger.error(error_txt)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_txt)

            if fct is not None:
                fct_to_execute["FCT"] = fct
                fct_to_execute["PARAMS"] = params
                result.append(fct_to_execute)
            else:
                self._logger.error(error_txt)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_txt)

        return result

    def __action_charger(self, plug, charger):
        """
        plug charger action

        :type charger: str
        :param charger: Charger type defined in iocard type for plug or USB and AC_CHGR for unplug

        :type plug: boolean
        :param plug: plug or unplug charger.
        """
        self._logger.info("performing CHARGER action: charger type %s" % charger)
        # plug the wanted charger
        if charger != "NONE" and plug:
            self.em_core_module.plug_charger(charger)
            if charger != self._io_card.AC_CHGR:
                self.__last_plug_charger = charger
        else:
            if charger == "USB":
                self._io_card.usb_connector(False)
            elif charger == "ALL":
                self._io_card.usb_connector(False)
                self._io_card.ac_charger_connector(False)
            elif charger == "WALL_CHARGER":
                self._io_card.remove_cable(charger)

    def __get_charger(self, charger_type, what_to_check, result_id):
        """
        get info from given charger
        due to the fact that ac charger and usb charger may be connected
        at the same time, we enable the possibility to specify which one to read.
        if USB is chosen, the last plugged usb will be the one target by this function

        CHARGER_GET_USB_CURRENT_IUSB1
        CHARGER_GET_AC_CHGR_CURRENT_IUSB2

        :type charger_type: str
        :param charger_type: type of charger, can be USB or AC_CHGR

        :type what_to_check: str
        :param what_to_check: info we want to read from charger

        :type result_id: str
        :param result_id: result used on target file
        """
        self._logger.info("Checking CHARGER %s" % what_to_check)
        result = None
        if what_to_check == "CURRENT":
            # If we want the current from usb then get last plugged charger
            if charger_type == "USB":
                charger_type = self.__last_plug_charger
            result = self.em_core_module.get_charger_current(self.__last_plug_charger)
            self._logger.info("IUSB = %s " % result)
        elif what_to_check == "VOLTAGE":
            # TODO: to be implemented
            pass
        else:
            txt = "Unknown CHARGER element to check: %s" % what_to_check
            self._logger.error(txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, txt)

        # store result for further comparison
        self._meas_list.add(result_id, result)

    def __action_button(self, button, duration):
        """
        press key combo

        :type button: str
        :param button: name of button to press on

        :type duration: int
        :param duration: time to keep button on.
        """
        if button == "FASTBOOT_COMBO":
            button = self.phone_info["GENERAL"]["FASTBOOT_COMBO"]
        elif button == "KEY_COMBO":
            button = self.__key_combo
        else:
            button = [button]

        # check if we want to use a special keyword to call values from device catalogs
        for keyword in self.__pwr_button_keyword.keys():
            if keyword in duration:
                duration = self.__pwr_button_keyword[keyword]
                break

        if str(duration).replace(".", "").isdigit():
            duration = float(duration)

        self._logger.info("performing BUTTON action : button to press %s during %ss" % (button, duration))
        self._io_card.press_key_combo(button, duration)

    def __action_wait(self, seconds):
        """
        sleep for a while

        :type seconds: int
        :param seconds: time to sleep in seconds.
        """
        self._logger.info("performing WAIT action: waiting %ss " % seconds)
        sleep(seconds)

    def __action_battery(self, plug, state):
        """
        Handles battery insertion / removal
        :type plug: boolean
        :param plug: action to be done:
            - True  => insert battery
            - False => remove battery

        :type battery_type: str
        :param battery_type: type of battery to plug
        """
        txt = "performing BATTERY action: type %s - connection state %s " % (state, plug)
        # if we remove battery , we dont care about the battery type
        last_state = state
        if not plug:
            txt = "performing BATTERY action: connection state %s " % plug
            last_state = self._io_card.BAT_REMOVED
        self._logger.info(txt)

        if plug:
            # remove battery before changing is type
            self._io_card.battery_connector(False)
        self._io_card.battery_connector(plug, state)
        self.__last_battery_state = (plug, last_state)

    def __set_battery(self, value):
        """
        Set battery value to given level.
        limited to voltage setting for now and in V unit.
        BATTERY_SET_VOLTAGE_ABOVE_BOOT_CHARGING

        :type value: float or str
        :param value: if pure numeric then set battery at this value else evaluate special keywords
        """
        self._logger.info("Setting BATTERY voltage to %s " % value)
        voltage = None
        # evaluate if there is keywords
        for keyword in self.__vbatt_thresold.keys():
            if keyword in value:
                keyword_value = self.__vbatt_thresold[keyword]
                value = value.replace(keyword, str(keyword_value), 1).strip()
                # evaluate action
                value = eval(value)
                break

        # first evaluate numeric value
        if str(value) != "" and str(value).replace(".", "").isdigit():
            voltage = float(value)
        else:
            txt = "Unknown BATTERY value to set: %s" % value
            self._logger.error(txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, txt)

        self.em_core_module.set_battery_voltage(voltage)
        self.__current_vbatt = voltage

    def __get_battery(self, what_to_check, result_id):
        """
        get info from battery
        BATTERY_GET_CURRENT_IBATT1

        :type result_id: str
        :param result_id: result used on target file

        :type what_to_check: str
        :param what_to_check: info we want to read from battery
        """
        self._logger.info("Checking BATTERY %s" % what_to_check)
        result = None
        if what_to_check == "CURRENT":
            result = self.em_core_module.get_battery_current(iteration=30)
            self._logger.info("IBATT = %s %s" % (result[0], result[1]))
        elif what_to_check == "VOLTAGE":
            # TODO: to be implemented
            pass
        else:
            txt = "Unknown BATTERY element to check: %s" % what_to_check
            self._logger.error(txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, txt)

        # store result for further comparison
        self._meas_list.add(result_id, result)

    def __get_boot_mode(self, result_id):
        """
        Try to get the boot mode.

        :type result_id: str
        :param result_id: result used on target file
        """
        self._logger.info("Get the boot mode")

        for _ in range(4):
            mode = self._device.get_boot_mode()
            sleep(1)

        # store result for further comparison
        self._meas_list.add(result_id, mode, "")

    def __decode_action_loop(self, start_index, result, action):
        """
        add iterative action to the action list.
        Iterative action  are describe between following key
        LOOP_START;....;LOOP_STOP_100

        internal iteration action
        LOOP_START: mark the beginning of a loop , all action after this will be looped
        LOOP_STOP_x: mark the end of a loop all action before this tag will be include in the loop and looped x times
        e.g., BATTERY_PLUG_ANALOG;LOOP_START;CHARGER_PLUG_SDP;LOOP_STOP_10 will plug SDP 10 times
        a loop with a value of 1 has no effect.
        LOOP_START;LOOP_START;CHARGER_PLUG_SDP;LOOP_STOP_2;LOOP_STOP_5 will do sdp PLUG 2*5 time

        :type start_index: int
        :param start_index: the first element of the list to duplicate

        :type result: list
        :param result: list to extend

        :type action: str
        :param action: action to be done

        :rtype: list
        :return: list of action extended with the iterative parts
        """
        loop_nb = None
        error_happen = False
        action = action.replace(self.LOOP_END_ACTION, "", 1).strip("_").strip()

        # catch the loop number to do which must be an int, not a float
        if action != "" and action.isdigit():
            # we want to loop n-1 time
            loop_nb = int(action)
            if loop_nb < 1:
                txt = "ACTION %s take only a integer value superior than 0" % action
                error_happen = True
            else:
                # looping 1 time does not have any effect
                loop_nb -= 1
        else:
            error_happen = True

        if error_happen:
            txt = "Unknown action: %s" % action
            self._logger.error(txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, txt)

        part_to_loop = result[start_index:len(result)]
        # increment the whole run list with the list part to loop
        for _ in range(loop_nb):
            result += part_to_loop

        return result

    def __decode_charger(self, action):
        """
        decode action link to charger

        charger action
        CHARGER_"plug/unplug"_"type" : "plug/unplug" the charger "type".
        if action is plug then charger type is defined by your io card supported charger
        CHARGER_PLUG_DCP
        CHARGER_PLUG_DCP
        CHARGER_PLUG_CDP
        CHARGER_PLUG_AC_CHGR
        CHARGER_PLUG_WALL_CHGR
        else it can only be USB to unplug an usb cable and AC_CHGR for ACDC connector
        CHARGER_UNPLUG_USB
        CHARGER_UNPLUG_AC_CHGR
        CHARGER_UNPLUG_ALL

        CHARGER_GET_USB_CURRENT_IUSB1
        CHARGER_GET_AC_CHGR_CURRENT_IUSB1
        CHARGER_GET_USB_VOLTAGE_IUSB1
        CHARGER_GET_AC_CHGR_VOLTAGE_IUSB1

        :type action: str
        :param action: action to be done

        :rtype: tuple
        :return: tuple containing fct , params to add to the execution list
        """
        # remove found key from txt
        action = action.replace(self.CHGR_ACTION, "", 1).strip("_").strip()
        local_params = []
        sub_action_name = ""
        fct = None

        # second level ACTION
        if action.startswith(self.PLUG_ACTION):
            sub_action_name = self.PLUG_ACTION
            fct = self.__action_charger
            local_params.append(True)
        elif action.startswith(self.UNPLUG_ACTION):
            sub_action_name = self.UNPLUG_ACTION
            fct = self.__action_charger
            local_params.append(False)
        # getter action
        elif action.startswith(self.GET_ACTION):
            action = action.replace(self.GET_ACTION, "", 1).strip("_").strip()
            fct = self.__get_charger
            # get the charger where to apply the info reading
            chgr_type = ""
            for element in ["USB", self._io_card.AC_CHGR]:
                if element in action:
                    local_params.append(element)
                    chgr_type = element
                    break
            # remove the charger name
            action = action.replace(chgr_type, "", 1).strip("_").strip()
            # get if we want current or voltage for this charger
            for element in self.GET_LIST:
                if element in action:
                    local_params.append(element)
                    sub_action_name = element
                    break
        else:
            txt = "Unknown action: %s" % action
            self._logger.error(txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, txt)

        # try to get the type of charger to plug/unplug
        action = action.replace(sub_action_name, "", 1).strip("_").strip()
        # LAST value CHARGER is str
        if action != "":
            local_params.append(action)
        else:
            txt = "Unknown or incomplete action: %s" % action
            self._logger.error(txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, txt)

        return fct, local_params

    def __decode_global_get(self, action):
        """
        decode action link to a getter.
        GET_BOOT_MODE_BOOtMOD1

        :type action: str
        :param action: action to be done
        :rtype: tuple
        :return: tuple containing fct , params to add to the execution list
        """
        # remove found key from txt
        action = action.replace(self.GET_ACTION, "", 1).strip("_").strip()
        local_params = []
        sub_action_name = ""
        fct = None
        # second level ACTION
        if action.startswith(self.BOOT_MODE):
            action = action.replace(self.BOOT_MODE, "", 1).strip("_").strip()
            fct = self.__get_boot_mode
            sub_action_name = self.BOOT_MODE
        else:
            txt = "Unknown action: %s" % action
            self._logger.error(txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, txt)

        # try to get the last element of the action
        action = action.replace(sub_action_name, "", 1).strip("_").strip()
        # LAST is the id where to store result
        if action != "":
            local_params.append(action)
        else:
            txt = "for global getter %s action, a var is necessary to store result" % sub_action_name
            self._logger.error(txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, txt)

        return fct, local_params

    def __decode_battery(self, action):
        """
        decode action link to battery

        battery action
        BATTERY_"plug/unplug"_"type": "plug/unplug" the battery "type" that can support your battery.
        BATTERY_PLUG_ANALOG
        BATTERY_PLUG_INVALID
        BATTERY_PLUG_EMULATOR
        BATTERY_PLUG_DEFAULT
        BATTERY_UNPLUG

        setting element
        BATTERY_SET_VOLTAGE_BOOT_MOS
        BATTERY_SET_VOLTAGE_3.5
        BATTERY_SET_VOLTAGE_BOOT_MOS-1

        :type action: str
        :param action: action to be done
        :rtype: tuple
        :return: tuple containing fct , params to add to the execution list
        """
        # remove found key from txt
        action = action.replace(self.BATT_ACTION, "", 1).strip("_").strip()
        local_params = []
        sub_action_name = ""
        fct = None
        # second level ACTION
        if action.startswith(self.PLUG_ACTION):
            sub_action_name = self.PLUG_ACTION
            fct = self.__action_battery
            local_params.append(True)
        # if unplug , battery type will be default
        elif action.startswith(self.UNPLUG_ACTION):
            sub_action_name = self.UNPLUG_ACTION
            fct = self.__action_battery
            local_params.append(False)
            local_params.append("default")
            # exit here
            return fct, local_params
        # getter
        elif action.startswith(self.GET_ACTION):
            action = action.replace(self.GET_ACTION, "", 1).strip("_").strip()
            fct = self.__get_battery
            # get if we want current or voltage
            for element in self.GET_LIST:
                if element in action:
                    local_params.append(element)
                    sub_action_name = element
                    break
        # setter
        elif action.startswith(self.SET_ACTION):
            action = action.replace(self.SET_ACTION, "", 1).strip("_").strip()
            fct = self.__set_battery
            # check voltage
            if "VOLTAGE" in action:
                sub_action_name = "VOLTAGE"
            else:
                txt = "Unsupported sub action: %s" % action
                self._logger.error(txt)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, txt)
        else:
            txt = "Unknown action: %s" % action
            self._logger.error(txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, txt)

        # try to get the last element of the action
        action = action.replace(sub_action_name, "", 1).strip("_").strip()
        # LAST value BATTERY is str
        if action != "":
            local_params.append(action)
        else:
            txt = "Unknown action: %s" % action
            self._logger.error(txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, txt)

        return fct, local_params

    def __decode_button(self, action):
        """
        decode action link to button interaction

        action that press on button
        BUTTON_PRESS_"button_name"_x : press x second(s) on the combo "button_name" which is a button controllable from your io card.
        e.g, BUTTON_PRESS_POWER_BUTTON_5
        BUTTON_PRESS_FASTBOOT_COMBO_x : press x second(s) on the fastBoot combo define in Device_Catalog.xml.
        BUTTON_PRESS_KEY_COMBO_x : press x second(s) on the combo define by KEY_COMBO TC parameter
        x can also be one of the following list:HARD_SHUTDOWN, SOFT_SHUTDOWN, BOOT
        in that case the value behind will be taken from device catalog.
        You should use this keywords only when you push power button only.

        :type action: str
        :param action: action to be done
        :rtype: tuple
        :return: tuple containing fct , params to add to the execution list
        """
        # remove found key from txt
        action = action.replace(self.BTN_ACTION, "", 1).strip("_").strip()
        sub_action_name = None
        local_params = []
        if "KEY_COMBO" in action:
            sub_action_name = "KEY_COMBO"
        elif "FASTBOOT_COMBO" in action:
            sub_action_name = "FASTBOOT_COMBO"
        else:
            # here check for specific button
            for button in self._io_card.BUTTON_LIST:
                if button in action:
                    sub_action_name = button
                    break

        if sub_action_name is None:
            txt = "unknown button to interact with: %s" % action
            self._logger.error(txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, txt)

        # happen the button to press on
        local_params.append(sub_action_name)
        # remove button name from text
        action = action.replace(sub_action_name, "", 1).strip("_").strip()
        # LAST value for button press is a numeric duration or a keyword
        if action != "":
            local_params.append(action)
        else:
            txt = "Not complete action, missing press duration: %s" % action
            self._logger.error(txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, txt)

        return self.__action_button, local_params

    def __decode_wait(self, action):
        """
        decode action link to button interaction

        sleep action
        WAIT_x : where x is a number in second to wait.
        e.g , WAIT_10

        :type action: str
        :param action: action to be done

        :rtype: tuple
        :return: tuple containing fct , params to add to the execution list
        """
        # remove found key from txt
        action = action.replace(self.WAIT_ACTION, "", 1).strip("_").strip()
        local_params = []
        # numeric or str last value for action
        if action != "":
            if action.replace(".", "").isdigit():
                action = float(action)
            local_params.append(action)
        else:
            txt = "Unknown or incomplete action: %s" % action
            self._logger.error(txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, txt)

        return self.__action_wait, local_params
