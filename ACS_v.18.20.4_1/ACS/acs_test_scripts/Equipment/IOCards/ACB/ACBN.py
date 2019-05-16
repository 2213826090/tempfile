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
:summary: ACBN implementation of normal Ariane Control Board
:author: ymorel, vgombert
:since: 16/03/2011
"""

import UtilitiesFWK.Utilities as Util
from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Equipment.IOCards.ACB.Common import WUsbDio as W
from acs_test_scripts.Equipment.IOCards.Interface.IIOCard import IIOCard
from acs_test_scripts.Equipment.IEquipment import DllLoader
import time

# pylint: disable=E1101


class ACBN(IIOCard, DllLoader):

    """
    Class ACBN: implementation of normal ACB
    """
    # Defining possible usb devices (Usb host, DCP, ...)
    SUPPORTED_DEVICE_TYPE = [IIOCard.USB_HOST_PC, IIOCard.SDP,
                             IIOCard.DCP, IIOCard.WALL_CHARGER]

    LINES = Util.enum('battery',  # ctrl00 on=inserted / off=removed
                      'usb_5v_gnd',  # ctrl01
                      'usb_dp_dm_id',  # ctrl02
                      'batt_id_ctrl3',  # ctrl03
                      'batt_id_ctrl4',  # ctrl04
                      'batt_id_ctrl5',  # ctrl05
                      'temp_value_ctrl6',  # ctrl06
                      'temp_value_ctrl7',  # ctrl07
                      'usb_charger',  # ctrl08 on=CP / off=DP
                      'batt_id_glitch',  # ctrl09 on=start the glitch
                                        #       off= rearm
                      'batt_id_glitch_duration',  # ctrl10 on=5p2ms / off=165us
                      'sim_card',  # ctrl11 on=insertion
                                        #       off=removal
                      'button_ctrl12',  # ctrl12 on=close connection
                                        #       off=open connection
                      'usb_charger_select',  # ctrl13 on=DC source / off=USBPC source
                      'digital_battery_protocol',  # ctrl14 on=logical 0 / off=logical 1
                      'supply1_switch',  # ctrl15 on=SDcard Vss1 / off=AC charger
                      'temp_value_ctrl16',  # ctrl16
                      'sd_card',  # ctrl17 on=insertion / off=removal
                      'usb_switch_select',  # ctrl18 on=USB accessories | USB Host PC
                                        #       off=USB charger
                      'usb_switch_select2',  # ctrl19 on=USB Host PC
                                        #       off=USB accessories
                                        # (depends on ',usb_switch_select',)
                      'button_ctrl20',  # ctrl20 volume down
                      'button_ctrl21',  # ctrl21 volume up

                      'button_ctrl22',  # ctrl22 on=close connection
                                        #       off=open connection
                      'button_ctrl23',  # ctrl23 on=close connection
                                        #       off=open connection
                      'button_ctrl24',  # ctrl24 on=close connection
                                        #       off=open connection
                      'button_ctrl25',  # ctrl25 on=close connection
                                        #       off=open connection
                      'button_ctrl26',  # ctrl26 on=close connection
                                        #       off=open connection
                      'power_supply1_ac_charger',  # ctrl27 depends on 'supply1_switch'
                      'OTG_ctrl28',  # ctrl28
                      'OTG_ctrl29',  # ctrl29
                      'OTG_ctrl30'  # ctrl30
                      )

    ACB_TO_DUT = {
        IIOCard.USB_ON: [
            (True, LINES.usb_5v_gnd),
            (True, LINES.usb_dp_dm_id)],
        IIOCard.USB_OFF: [
            (False, LINES.usb_dp_dm_id),
            (False, LINES.usb_5v_gnd)]
        }

    ACCESSORY_TO_ACB = {
        IIOCard.SDP: [  # SDP
            (False, LINES.usb_charger),  # ctrl08
            (False, LINES.usb_switch_select),  # ctrl18
            (False, LINES.usb_switch_select2)],  # ctrl19
        IIOCard.DCP: [  # DCP
            (True, LINES.usb_charger),  # ctrl08
            (True, LINES.usb_switch_select),  # ctrl18
            (False, LINES.usb_switch_select2)],  # ctrl19
        IIOCard.EXTERNAL_PS_ON: [
            # connect power supply
            (True, LINES.usb_charger_select)],  # ctrl01
        IIOCard.EXTERNAL_PS_OFF: [
            (False, LINES.usb_charger_select)]  # ctrl01
        }

    BATTERY_TYPE = {
        IIOCard.BAT_ANALOG: [
            (False, LINES.batt_id_ctrl3),  # ctrl03
            (False, LINES.batt_id_ctrl4),  # ctrl04
            (False, LINES.batt_id_ctrl5),  # ctrl05
            (True, LINES.batt_id_glitch),  # ctrl09
            (True, LINES.batt_id_glitch_duration),  # ctrl10
            (False, LINES.digital_battery_protocol)],  # ctrl14
        IIOCard.BAT_INVALID: [
            (True, LINES.batt_id_ctrl3),  # ctrl03
            (True, LINES.batt_id_ctrl4),  # ctrl04
            (True, LINES.batt_id_ctrl5),  # ctrl05
            (True, LINES.batt_id_glitch),  # ctrl09
            (True, LINES.batt_id_glitch_duration),  # ctrl10
            (False, LINES.digital_battery_protocol)],  # ctrl14
        IIOCard.BAT_EMULATOR: [
            (True, LINES.batt_id_ctrl3),  # ctrl03
            (True, LINES.batt_id_ctrl4),  # ctrl04
            (False, LINES.batt_id_ctrl5),  # ctrl05
            (True, LINES.batt_id_glitch),  # ctrl09
            (True, LINES.batt_id_glitch_duration),  # ctrl10
            (False, LINES.digital_battery_protocol)]  # ctrl14
            }

    def __init__(self, name, model, eqt_params, bench_params):
        """
        Constructor
        :type name: str
        :param name: the bench configuration name of the equipment
        :type model: str
        :param model: the model of the equipment
        :type eqt_params: dict
        :param eqt_params: the dictionary containing equipment parameters
        :type bench_params: dict
        :param bench_params: the dictionary containing equipment bench parameters
        """
        IIOCard.__init__(self)
        DllLoader.__init__(self, name, model, eqt_params)
        self.__bench_params = bench_params
        self.__device_index = -1
        self.__wall_charger = None
        self._use_ext_ps = None
        self.__default_battery = None
        self.__ext_ps_plug = False
        # MAP PHONE BUTTON here
        self.PHONE_BUTTON = {
            ACBN.VOLUME_UP: self.LINES.button_ctrl21,
            ACBN.VOLUME_DOWN: self.LINES.button_ctrl20,
            ACBN.PWR_BUTTON: self.LINES.button_ctrl12}
        # define what is usb host pc
        self.ACCESSORY_TO_ACB[self.USB_HOST_PC] = self.ACCESSORY_TO_ACB[self.SDP]
        self.ACCESSORY_TO_ACB[self.WALL_CHARGER] = []

    def get_bench_params(self):
        """
        :rtype: bench configuration
        :return: the bench configuration dictionary of the equipment
        """
        return self.__bench_params

    def get_dev_idx(self):
        """
        :rtype: integer
        :return: the index of the device
        """
        return self.__device_index

    def init(self):
        """
        Initializes the equipment. The equipment is ready to use.
            - Load equipment driver
            - Connection to the equipment is established
            - Show equipment informations
            - Reset equipment
        """
        self.get_logger().info("Initialization")
        # Loads the driver
        self.load_driver()

        serial_number = self.get_bench_params().get_param_value("serialNumber", "")
        if  serial_number not in [None, ""]:
            serial_number = int(self.get_bench_params().get_param_value("serialNumber"))

        # get if we use external power supply as charger
        self._use_ext_ps = self.get_bench_params().get_param_value("ExtPowerSupplyAsCharger", False)
        if type(self._use_ext_ps) is str:
            self._use_ext_ps = Util.str_to_bool(self._use_ext_ps)

        # Tries to connect to equipment
        self.__device_index = W.Connect(self, serial_number)
        if self.__device_index == -1:
            raise TestEquipmentException(
                TestEquipmentException.CONNECTION_ERROR,
                "Failed to connect to %s" % self.get_name())
        W.ShowInfo(self)
        W.Reset(self)
        self.battery_connector(False, self.BAT_INVALID)

    def release(self):
        """
        Releases all resources allocated to equipment
        """
        self.get_logger().info("Release")
        self.unload_driver()
        self.__device_index = -1

    def reset(self):
        """
        Reset the IO card to default states
        """
        self.set_battery_temperature(25)
        self.usb_connector(False)
        self.ext_supply_connector(False)
        self.battery_connector(False)

    def usb_connector(self, plug):
        """
        Handles USB connector connection and disconnection
        :type plug: boolean
        :param plug: action to be done:
            - True  => plug currently selected USB device
            - False => unplug currently selected USB device
        """
        if plug:
            self.get_logger().info("Plug USB")
            if not self._acb_com_cmd_list(self.ACB_TO_DUT[self.USB_ON], True):
                self._logger.debug("usb_connector: USB already plug")
        else:
            self.get_logger().info("Unplug USB")
            if self._acb_com_cmd_list(self.ACB_TO_DUT[self.USB_OFF], True):
                if self._use_ext_ps and self.__ext_ps_plug:
                    self.ext_supply_connector(False)
            else:
                self._logger.debug("usb_connector: USB already unplug")

    def usb_host_pc_connector(self, plug):
        """
        Handles USB connector connection and disconnection of USB host PC device
        :type plug: boolean
        :param plug: action to be done
            - True  => plug USB to host PC
            - False => unplug USB from host PC
        """
        # Plug or unplug USB
        if plug:
            self.get_logger().info("Plug DUT to PC")
            # check usb exist in case of plug
            if not self._is_relay_already_positioned(self.ACCESSORY_TO_ACB[self.USB_HOST_PC]):
                self.usb_connector(False)
                self.get_logger().info("switch core relay to PC HOST")
                self._acb_com_cmd_list(self.ACCESSORY_TO_ACB[self.USB_HOST_PC], True)

            self.usb_connector(plug)
        else:
            self.get_logger().info("Unplug DUT to PC")
            self.usb_connector(plug)
        return True

    def usb_device_selector(self, usb_device):
        """
        Handles USB device selection
        :param usb_device: USB device to select. Possible values:
            - "USB_HOST_PC": USB Host PC (ACS, FW/SW Updates)
            - "DCP": Dedicated Charging Port, like a wall charger that can supply 500mA-1.5A by default.
            - "SDP": like a PC or laptop that can supply 100mA by default or 500mA after negotiation.
        """
        self.get_logger().info("Select USB device [%s]", usb_device)

        if usb_device in self.ACCESSORY_TO_ACB.keys():
            self._acb_com_cmd_list(self.ACCESSORY_TO_ACB[usb_device])
        else:
            raise TestEquipmentException(
                TestEquipmentException.INVALID_PARAMETER,
                "Unknown USB device : [%s]!" % usb_device)

    def simulate_insertion(self, device_type):
        """
        Do a real insertion change by removing previous usb if there was any then plug the wanted one.
        If the same usb is inserted twice then it will unplug then plug again instead of doing nothing.
        check the SUPPORTED_DEVICE_TYPE of this class to know which one is supported.

        :type usb_device: str
        :param usb_device: USB device to select. Possible values:
            - "USB_HOST_PC": USB Host PC (ACS, FW/SW Updates)
            - "DCP": Dedicated Charging Port, like a wall charger that can supply 500mA-1.5A by default.
            - "SDP": like a PC or laptop that can supply 100mA by default or 500mA after negotiation.
            - "CDP": like a PC or laptop that can supply 500mA at all times but may be able to support up to 1.5A.
            - "OTG": USB stick
        """
        self.get_logger().info("Insert cable type %s", device_type)

        if device_type not in self.SUPPORTED_DEVICE_TYPE:
            raise TestEquipmentException(
                TestEquipmentException.INVALID_PARAMETER,
                "Unknown device type: [%s]!" % device_type)

        if device_type == self.AC_CHGR:
            # available ac charger
            self.ac_charger_connector(True)
        elif device_type == self.WALL_CHARGER:
            # call wall charger function
            self.wall_charger_connector(True)
        else:
            # Disconnect USB
            self.usb_connector(False)
            time.sleep(2)
            # change the usb type
            self.usb_device_selector(device_type)
            # USB connect
            self.usb_connector(True)
        # a wall charger connector will call twice this command that why we should
        # not play with the external power supply connector
        if device_type in [self.DCP, self.AC_CHGR] and self._use_ext_ps:
            self.ext_supply_connector(True)

    def remove_cable(self, cable_type):
        """
        disconnect given cable or ALL cable.
        This super function will call the right function depending of the cable you want to remove.

        :type cable_type: str
        :param cable_type: cable supported by your EMT and device or ALL
        """
        if cable_type not in self.SUPPORTED_DEVICE_TYPE and cable_type != "ALL":
            raise TestEquipmentException(
                TestEquipmentException.INVALID_PARAMETER,
                "Unsupported cable type : %s!" % cable_type)

        if cable_type in [self.DCP, self.SDP, self.CDP, self.USB_HOST_PC, "ALL"]:
            self.usb_connector(False)

        if cable_type in [self.AC_CHGR, "ALL"]:
            self.ac_charger_connector(False)
        elif cable_type == self.WALL_CHARGER:
            self.wall_charger_connector(False)

    def press_power_button(self, duration):
        """
        Presses power button.
        Allow to simulate special behavior on the board like S3 mode.
        :type duration: float
        :param duration: time while the power button is pressed
            The value should be superior than 0 seconds
        """
        self.get_logger().info("Press power button during %s second(s)",
                               str(duration))
        W.Enable(self, self.LINES.button_ctrl12)
        time.sleep(duration)
        W.Disable(self, self.LINES.button_ctrl12)

    def battery_connector(self, plug, battery_type="DEFAULT"):
        """
        Handles battery insertion / removal
        :type plug: boolean
        :param plug: action to be done:
            - True  => insert battery
            - False => remove battery
        :type battery_type: str
        :param battery_type: battery to plug
        """
        if plug:
            self.get_logger().info("Battery insertion")
            # work around for bench batt id bug
            self.set_battery_type(self.BAT_INVALID)
            # Insert battery
            W.Enable(self, self.LINES.battery)  # ctrl00
            W.Enable(self, self.LINES.supply1_switch)  # ctrl15
            # work around for bench batt id bug
            self.set_battery_type(battery_type)
        else:
            self.get_logger().info("Battery removal")
            # remove battery
            W.Disable(self, self.LINES.battery)  # ctrl00
            W.Enable(self, self.LINES.supply1_switch)  # ctrl15
            # work around for bench batt id bug
            self.set_battery_type(self.BAT_INVALID)

    def sim_card_connector(self, plug):
        """
        Handles SIM card insertion / removal
        :type plug: boolean
        :param plug: action to be done:
            - True  => insert SIM card
            - False => remove SIM card
        """
        if plug:
            self.get_logger().info("SIM card insertion")
            # Insert battery
            W.Enable(self, self.LINES.sim_card)  # ctrl11
        else:
            self.get_logger().info("SIM card removal")
            # remove battery
            W.Disable(self, self.LINES.sim_card)  # ctrl11

    def ext_supply_connector(self, plug):
        """
        Handles external power supply connection and disconnection
        :type plug: boolean
        :param plug: action to be done:
            - True  => plug external power supply
            - False => unplug external power supply
        """
        if plug:
            self.get_logger().info("Plug external power supply")
            # connect power supply
            if not self._acb_com_cmd_list(self.ACCESSORY_TO_ACB[self.EXTERNAL_PS_ON], True):
                self._logger.debug("ext_supply_connector: EXT PS already connected")
            self.__ext_ps_plug = True

        else:
            self.get_logger().info("Unplug external power supply")
            if not self._acb_com_cmd_list(self.ACCESSORY_TO_ACB[self.EXTERNAL_PS_OFF], True):
                self._logger.debug("ext_supply_connector: EXT PS already disconnected")
            self.__ext_ps_plug = False

    def set_battery_type(self, batt_type):
        """
        Sets Battery type.
        :type batt_type: str
        :param batt_type: batt id to select:
            - "ANALOG"
            - "INVALID"
            - "BATTERY_EMULATOR"
            - "TEST2"
            - "DIGITAL"
            - "REMOVED"
            - "DEFAULT"
        """
        # avoid cast problem
        batt_type = str(batt_type).upper()
        if "DEFAULT" in batt_type:
            batt_type = self.get_default_battery_type()
        self.get_logger().info("Set battery type: [%s]", batt_type)

        if batt_type not in self.BATTERY_TYPE.keys():
            raise TestEquipmentException(
                TestEquipmentException.INVALID_PARAMETER,
                "Unsupported battery type: [%s]!" % batt_type)

        self._acb_com_cmd_list(self.BATTERY_TYPE[batt_type])

    def set_battery_temperature(self, temperature):
        """
        Sets battery temperature
        :type temperature: integer
        :param temperature: temperature in C to set, possible values:
        {95; 70; 50; 25; 10; 5; 0; -15}
        """
        temperature = int(temperature)
        self.get_logger().info("Set battery temperature: %d C", temperature)

        codes = {95: [False, False, True],
                 70: [True, False, True],
                 50: [False, True, True],
                 25: [True, True, True],
                 10: [False, False, False],
                 5: [True, False, False],
                 0: [False, True, False],
                 - 15: [True, True, False]}

        lines = [self.LINES.temp_value_ctrl6,
                 self.LINES.temp_value_ctrl7,
                 self.LINES.temp_value_ctrl16]

        if temperature in codes.keys():
            code = codes[temperature]
            for num in range(len(code)):
                if code[num]:
                    W.Enable(self, lines[num])
                else:
                    W.Disable(self, lines[num])
        else:
            raise TestEquipmentException(
                TestEquipmentException.INVALID_PARAMETER,
                "Unsupported battery temperature: %d!" % temperature)

    def set_usb_otg_type(self, otg_type):
        """
        Sets the OTG type
        :type otg_type: str
        :param otg_type: OTG type to select:
            - "NORMAL"
            - "DUT_DEVICE"
            - "DUT_HOST"
        """
        self.get_logger().info("Set OTG type to : %s", otg_type)

        if otg_type == "NORMAL":
            W.Disable(self, self.LINES.OTG_ctrl28)  # ctrl28
            W.Enable(self, self.LINES.OTG_ctrl29)  # ctrl29
            W.Enable(self, self.LINES.OTG_ctrl30)  # ctrl30

        elif otg_type == "DUT_DEVICE":
            W.Enable(self, self.LINES.OTG_ctrl28)  # ctrl28
            W.Disable(self, self.LINES.OTG_ctrl29)  # ctrl29
            W.Enable(self, self.LINES.OTG_ctrl30)  # ctrl30

        elif otg_type == "DUT_HOST":
            W.Enable(self, self.LINES.OTG_ctrl28)  # ctrl28
            W.Disable(self, self.LINES.OTG_ctrl29)  # ctrl29
            W.Disable(self, self.LINES.OTG_ctrl30)  # ctrl30

        else:
            raise TestEquipmentException(
                TestEquipmentException.INVALID_PARAMETER,
                "Unknown OTG type : %s!" % otg_type)

    def wall_charger_connector(self, plug):
        """
        handle wall charger insertion depending of available one on io card
        and supported one by device.
        :type plug: boolean
        :param plug: action to be done:
            - True  => insert charger
            - False => remove charger
        """
        device_default_charger = self.get_default_wall_charger()

        if device_default_charger is None:
            txt = "Default charger not set, please set it on your test using set_default_wall_charger function"
            self.get_logger().error(txt)
            raise TestEquipmentException(
                TestEquipmentException.OPERATION_FAILED, txt)
        if plug:
            self.get_logger().info("Plug Wall charger")
            self.simulate_insertion(device_default_charger)
        else:
            self.get_logger().info("Remove Wall charger")
            if device_default_charger == self.AC_CHGR:
                self.ac_charger_connector(False)
            elif device_default_charger in self.SUPPORTED_DEVICE_TYPE:
                self.usb_connector(False)
            else:
                raise TestEquipmentException(
                    TestEquipmentException.INVALID_PARAMETER,
                    "Unknown cable type : %s!" % device_default_charger)

    def set_default_wall_charger(self, device_default_charger):
        """
        set default wall charger.

        :type device_default_charger: str
        :param device_default_charger: default wall charger supported by the device
        """
        self.get_logger().info("setting default wall charger %s" % device_default_charger)

        if device_default_charger not in self.SUPPORTED_DEVICE_TYPE:
            txt = "your io card does not support %s charger type" % device_default_charger
            self.get_logger().warning(txt)

        self.__wall_charger = device_default_charger
        self.ACCESSORY_TO_ACB[self.WALL_CHARGER] = self.ACCESSORY_TO_ACB[device_default_charger]

    def get_default_wall_charger(self):
        """
        get default wall charger.

        :rtype: str
        :return: default wall charger supported by the device
                  return None if not set
        """
        return self.__wall_charger

    def set_default_battery_type(self, batt_type):
        """
        set the default battery type.
        all function that play with battery type will take this value if the type is omitted.

        :type batt_type: str
        :param batt_type: default battery type supported by the device
        """
        self.get_logger().info("setting default battery type to %s" % batt_type)
        if batt_type not in self.BATTERY_TYPE.keys():
            txt = "your io card does not support %s battery type" % batt_type
            self.get_logger().warning(txt)

        self.__default_battery = batt_type

    def get_default_battery_type(self):
        """
        get default battery type.

        :rtype: str
        :return: get the default battery type
                  return None if not set
        """
        return self.__default_battery

    # pylint: disable=W0613
    def set_battid_value(self, battery_type, value):
        """
        Constructor
        :type name: str
        :param name: the bench configuration name of the equipment
        :type model: str
        :param model: the model of the equipment
        """
        self.get_logger().debug("Set bptherm only available after EMT340 model")

    def set_bp_therm(self, factor_a, factor_b):
        """
        Constructor of the bp_therm expression
        this  expression has the forme of 'R(t)=a*exp(b*t)'
        with 'R' as bptherm resistor and 't' as battery temperature
        :type factor_a: float
        :param factor_a: the factor a in the exponential bptherm expression
        :type factor_b: float
        :param factor_b: the factor a in the exponential bptherm expression
        """
        self.get_logger().debug("Set bptherm only available after EMT340 model")

    def set_provisioning_mode(self, enable):
        """
        activate provisioning mode on the device by enabling specific
        button (volume up/down, home, ...)

        :type enable: boolean
        :param enable: action to be done:
            - True  => enable provisioning mode
            - False => disable provisioning mode

        """
        # Need to be redesigned, return False to keep compatibility
        return False

    def press_key_combo(self, button_list, press_duration):
        """
        push on the following button

        :type button_list: list of str
        :param button_list: list of button to press on. defined by  PHONE_BUTTON key

        :type press_duration: int
        :param press_duration: time to keep button on.
        """
        missing_btn = set(button_list) - set(self.PHONE_BUTTON)

        if len(missing_btn) > 0:
            self.get_logger().error("missing unlinked to relay button found :" + str(missing_btn))
            raise TestEquipmentException(
                TestEquipmentException.INVALID_PARAMETER,
                "Unknown or not configured button : %s" % str(missing_btn))

        self.get_logger().info("pressing button %s during %ss" % (str(button_list), press_duration))
        # push button
        for button_name in button_list:
            W.Enable(self, self.PHONE_BUTTON[button_name])  # press button
            time.sleep(0.2)
        # wait a while
        time.sleep(press_duration)
        # release button
        for button_name in button_list:
            W.Disable(self, self.PHONE_BUTTON[button_name])  # release button
            time.sleep(0.2)

    def load_specific_dut_config(self, dut_name):
        """
        Configure different setting on your io card related to dut name,
        This is useful in multi device campaign.
        The setting will depending of your current dut name and what you declared on
        benchconfig.

        :type dut_name: str
        :param dut_name: phone name

        .. warning:: not functional for this io card, put only for compatibility reason
        """
        pass

    def _is_relay_already_positioned(self, cmd_list):
        """
        Check if an array of relay is already well positioned
        according of expecting state given by the array.

        :type cmd_list: list
        :param cmd_list: list of (action,relay where to apply action)
                        or you can add a 3rd element like following (action, relay,time_to_wait)

        :rtype: boolean
        :return: True if relays are well positioned, False otherwise
        """
        is_relay_position_ok = True
        # check all relay in given command
        for element in cmd_list:
            action = element[0]
            relay = element[1]
            # exit if at least one relay is not in the wanted position
            if action != W.GetState(self, relay):
                is_relay_position_ok = False
                break

        return is_relay_position_ok

    def _acb_com_cmd_list(self, cmd_list, check_relay_position=False):
        """
        execute sequentially a list of action on relays

        :type cmd_list: list
        :param cmd_list: list of (action,relay where to apply action)
                        or you can add a 3rd element like following (action, relay,time_to_wait)

        :type check_relay_position: boolean
        :param check_relay_position: if True then will check all the relay state before doing
                                    the relay activation and if all is already in the wanted state
                                    then no action will be taken

        :rtype: boolean
        :return: True if actions have been taken on relay, False otherwise
        """
        if check_relay_position:
            if self._is_relay_already_positioned(cmd_list):
                # no action will be taken leave here
                return False

        for element in cmd_list:
            action = element[0]
            relay = element[1]
            self._acb_com_cmd(action, relay)
            # check if there is a time sleep represented by a 3rd parameter
            if len(element) == 3:
                time.sleep(element[2])

        return True

    def _acb_com_cmd(self, plug, relay):
        """
        execute an action on given relay

        :type plug: boolean
        :param plug: False to unplug or True to plug the relay

        :type relay: enum element
        :param relay: the relay where to apply the action.
        """
        # check relay
        error_msg = ""
        # work around to avoid problem in comparing different enum
        # we compare by text here
        if str(relay) not in str(self.LINES):
            error_msg = "unknown relay %s" % str(relay)
        # check with action to perform
        if not plug:
            cmd = W.Disable
        elif plug:
            cmd = W.Enable
        else:
            error_msg += " unknown action %s for relay must be True to plug or False to unplug" % str(plug)

        if error_msg != "":
            self.get_logger().info("Error happen during relay operation :" + error_msg)
            raise TestEquipmentException(
                TestEquipmentException.INVALID_PARAMETER, error_msg)
        else:
            cmd(self, relay)
