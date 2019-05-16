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
:summary: Reverse ACB implementation to control several DUT with 1 PC
:author: vgomberx
:since: 03/09/2013(August)
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException
import UtilitiesFWK.Utilities as Util
from acs_test_scripts.Equipment.IOCards.ACB.Common import WUsbDio as W
from acs_test_scripts.Equipment.IOCards.Interface.IIOCard import IIOCard
from acs_test_scripts.Equipment.IEquipment import DllLoader
import time

# pylint: disable=E1101
# pylint: disable=E1103


class ACBMD(IIOCard, DllLoader):

    """
    Class EMT311: implementation of enhanced ACB for tablet and phone
    """

    # Defining possible usb devices (Usb host, DCP, ...)
    SUPPORTED_DEVICE_TYPE = [IIOCard.USB_HOST_PC, IIOCard.SDP]
    # enum name are taken from normal ACB to ease understanding
    LINES = Util.enum('battery',  # ctrl00 on=inserted / off=removed
                      'usb_5v_gnd',  # ctrl01
                      'usb_dp_dm_id',  # ctrl02
                      'batt_id_1',  # ctrl03
                      'batt_id_2',  # ctrl04
                      'batt_id_3',  # ctrl05
                      'temp_value_ctrl6',  # ctrl06
                      'bptherm_1m',  # ctrl07
                      'prog_res_rdy',  # ctrl08 on=CP / off=DP
                      'batt_id_glitch',  # ctrl09 on=start the glitch off= rearm
                      'batt_id_glitch_duration',  # ctrl10 on=5p2ms / off=165us
                      'prog_res_sdo',  # ctrl11 on=insertion off=removal
                      'button_ctrl12',  # ctrl12 on=close connection off=open connection
                      'usb_supply_or_ext',  # ctrl13
                      'digital_battery_protocol',  # ctrl14 on=logical 0 / off=logical 1
                      'supply1_switch',  # ctrl15 on=SDcard Vss1 / off=AC charger
                      'bptherm_connect',  # ctrl16
                      'sd_card',  # ctrl17 on=insertion / off=removal
                      'usb_switch_stage2',  # ctrl18
                      'usb_switch_stage1',  # ctrl19
                      'button_ctrl20',  # ctrl20 on=close connection off=open connection
                      'button_ctrl21',  # ctrl21 on=close connection off=open connection
                      'button_ctrl22',  # ctrl22 on=close connection off=open connection
                      'button_ctrl23',  # ctrl23 on=close connection off=open connection
                      'button_ctrl24',  # ctrl24 on=close connection off=open connection
                      'button_ctrl25',  # ctrl25 on=close connection off=open connection
                      'button_ctrl26',  # ctrl26 on=close connection  off=open connection
                      'vusb_force',  # ctrl27
                      'vbatt_force_sens',  # ctrl28
                      'OTG_microAB_1',  # ctrl29
                      'OTG_microAB_2'  # ctrl30
                      )

    USB_PC_TO_ACB = {
        "ENABLE": [
            (False, LINES.usb_5v_gnd),
            (True, LINES.usb_dp_dm_id)],
        "DISABLE": [
            (False, LINES.usb_dp_dm_id),
            (True, LINES.usb_5v_gnd)]
        }

    USB_ACB_TO_DUT = {
        "USB_1": [  # SDP
            # Disconnect exp power supply
            # first element is if this relay is ON, second element the relay name
            (True, LINES.usb_supply_or_ext),  # ctrl13
            # Charger
            (True, LINES.usb_switch_stage2),  # ctrl18
            (True, LINES.usb_switch_stage1),  # ctrl19
            # ID direct path
            (False, LINES.OTG_microAB_1)],  # ctrl29
        "USB_2": [  # DCP
            # Disconnect exp power supply
            (True, LINES.usb_supply_or_ext),  # ctrl13
            # Charger
            (False, LINES.usb_switch_stage2),  # ctrl18
            (True, LINES.usb_switch_stage1),  # ctrl19
            # ID direct path
            (False, LINES.OTG_microAB_1)],  # ctrl29
        "USB_3": [  # CDP
            # Disconnect exp power supply
            (True, LINES.usb_supply_or_ext),  # ctrl13
            # Charger
            (True, LINES.usb_switch_stage2),  # ctrl18
            (False, LINES.usb_switch_stage1),  # ctrl19
            # ID direct path
            (False, LINES.OTG_microAB_1)],  # ctrl29
        "USB_4": [  # OTG
            (True, LINES.usb_supply_or_ext),  # ctrl13
            # Charger
            (False, LINES.usb_switch_stage2),  # ctrl18
            (False, LINES.usb_switch_stage1),  # ctrl19
            # ID direct path
            (False, LINES.OTG_microAB_1)]  # ctrl29
        }

    PWR_BTN_ACB_TO_DUT = {
        "POWER_BUTTON_1": LINES.button_ctrl12,
        "POWER_BUTTON_2": LINES.button_ctrl20,
        "POWER_BUTTON_3": LINES.button_ctrl21,
        "POWER_BUTTON_4": LINES.button_ctrl22
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
        self.__default_pwr_btn = bench_params.get_param_value("DEFAULT_POWER_BUTTON", "")
        self.__default_usb_no = bench_params.get_param_value("DEFAULT_USB", "")
        self.__bench_params = bench_params
        self.__device_index = -1
        self.__wall_charger = None

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
        # get serial number
        serial_number = self.get_bench_params().get_param_value("serialNumber", "")
        if  serial_number not in [None, ""]:
            serial_number = int(self.get_bench_params().get_param_value("serialNumber"))
        # Tries to connect to equipment
        self.__device_index = W.Connect(self, serial_number)
        if self.__device_index == -1:
            raise TestEquipmentException(
                TestEquipmentException.CONNECTION_ERROR,
                "Failed to connect to %s" % self.get_name())
        W.ShowInfo(self)
        W.Reset(self)

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

    def __configure_default_usb(self, usb_no):
        """
        Configure the default usb to use with common io card interface command
        when usb No is not a parameter of this class functions.

        :type usb_no:str
        :param usb_no:name of the usb to use like 'USB_1'
        """
        self.__default_usb_no = usb_no

    def __configure_power_button(self, pwr_btn_no):
        """
        Configure the default power button to use with common io card interface command
        when power button No is not a parameter of these functions.

        :type pwr_btn_no:str
        :param pwr_btn_no:name of the power button like 'POWER_BUTTON_1'
        """
        self.__default_pwr_btn = pwr_btn_no

    def __press_specific_power_button(self, pwr_btn_no, duration):
        """
        press on a given power button.
        Starting by the "otg" port

        :type pwr_btn_no:str
        :param pwr_btn_no:name of the power button like 'POWER_BUTTON_1'

        :type duration: float
        :param duration: time while the power button is pressed
                        The value should be superior than 0 seconds
        """
        if pwr_btn_no not in self.PWR_BTN_ACB_TO_DUT:
            msg = "Unknown Power button No %s" % pwr_btn_no
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)
        self.get_logger().info("Press power button during %s second(s)",
                               str(duration))
        self.__acb_com_cmd(True, self.PWR_BTN_ACB_TO_DUT[pwr_btn_no])
        time.sleep(duration)
        self.__acb_com_cmd(False, self.PWR_BTN_ACB_TO_DUT[pwr_btn_no])

    def __acb_com_cmd_list(self, cmd_list, check_relay_position=False):
        """
        execute sequentially a list of action on relays

        :type cmd_list: list
        :param cmd_list: list of (action,relay where to apply action)

        :type check_relay_position: boolean
        :param check_relay_position: if True then will check all the relay state before doing
                                    the relay activation and if all is already in the wanted state
                                    then no action will be taken

        :rtype: boolean
        :return: True if actions have been taken on relay, False otherwise
        """
        if check_relay_position:
            is_relay_position_ok = True
            # check all relay in given command
            for action, relay in cmd_list:
                # exit if at least one relay is not in the wanted position
                if action != W.GetState(self, relay):
                    is_relay_position_ok = False
                    break

            # if all relay are already positioned then skip any action
            if is_relay_position_ok:
                return False

        for action, relay in cmd_list:
            self.__acb_com_cmd(action, relay)

        return True

    def __acb_com_cmd(self, plug, relay):
        """
        execute an action on given relay

        :type plug: boolean
        :param plug: False to unplug or True to plug the relay

        :type relay: enum element
        :param relay: the relay where to apply the action.
        """
        # check relay
        error_msg = ""

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

    def load_specific_dut_config(self, dut_name):
        """
        Configure different setting on your io card related to dut name,
        This is useful in multi device campaign.
        The setting will depending of your current dut name and what you declared on
        benchconfig.

        :type dut_name: str
        :param dut_name: phone name
        """
        # first check the right usb
        is_usb_found = False
        for usb_name in sorted(self.USB_ACB_TO_DUT.keys()):
            value = self.get_bench_params().get_param_value(usb_name, "")
            if dut_name.upper() == value.upper() != "":
                self.__configure_default_usb(usb_name)
                is_usb_found = True
                self._logger.debug("DUT [%s] will now use USB port [%s] on io card" % (dut_name, usb_name))
                break

        if not is_usb_found:
            self._logger.warning("No configuration on USB port to use for DUT [%s] found in your BenchConfig" % dut_name)
            if self.__default_usb_no != "":
                self._logger.warning("Default setting will be used : [%s]" % self.__default_usb_no)
            else:
                self._logger.warning("Any further actions on USB connection will be ignored")

        # then check the right power button
        is_pwr_found = False
        for pwr_btn_name in sorted(self.PWR_BTN_ACB_TO_DUT.keys()):
            value = self.get_bench_params().get_param_value(pwr_btn_name, "")
            if dut_name.upper() == value.upper() != "":
                self.__configure_power_button(pwr_btn_name)
                is_pwr_found = True
                self._logger.debug("DUT [%s] will now use Power Button [%s] on io card" % (dut_name, pwr_btn_name))
                break

        if not is_pwr_found:
            self._logger.warning("No configuration on Power Button to use for DUT [%s] found in your BenchConfig" % dut_name)
            if self.__default_usb_no != "":
                self._logger.warning("Default setting will be used : [%s]" % self.__default_pwr_btn)
            else:
                self._logger.warning("Any further actions on POWER BUTTON will be ignored")

    def press_power_button(self, duration):
        """
        Presses power button.
        Allow to simulate special behavior on the board like S3 mode.
        :type duration: float
        :param duration: time while the power button is pressed
            The value should be superior than 0 seconds
        """
        if self.__default_pwr_btn != "":
            self.__press_specific_power_button(self.__default_pwr_btn, duration)
        else:
            self._logger.warning("press_power_button: No power button configured, skipped!")

    def usb_connector(self, plug):
        """
        Handles USB connector connection and disconnection
        :type plug: boolean
        :param plug: action to be done:
            - True  => plug currently selected USB device
            - False => unplug currently selected USB device
        """
        if self.__default_usb_no != "":
            if plug:
                self.get_logger().info("Plug USB")
                if self.__acb_com_cmd_list(self.USB_PC_TO_ACB["ENABLE"], True):
                    # Waiting for enumeration
                    time.sleep(3)
                else:
                    self._logger.debug("usb_connector: USB already plug")
            else:
                self.get_logger().info("Unplug USB")
                if self.__acb_com_cmd_list(self.USB_PC_TO_ACB["DISABLE"], True):
                    time.sleep(1)
                else:
                    self._logger.debug("usb_connector: USB already unplug")
        else:
            self._logger.warning("usb_connector: No USB configured, skipped!")

    def usb_host_pc_connector(self, plug):
        """
        Handles USB connector connection and disconnection of USB host PC device
        :type plug: boolean
        :param plug: action to be done
            - True  => plug USB to host PC
            - False => unplug USB from host PC
        """
        # Plug or unplug USB
        if self.__default_usb_no != "":
            if plug:
                # check usb exist in case of plug
                if self.__default_usb_no not in self.USB_ACB_TO_DUT:
                    msg = "usb_host_pc_connector: Unknown USB No %s" % self.__default_usb_no
                    self.get_logger().error(msg)
                    raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)
                # cosmetic log if no action is taken
                if not self.__acb_com_cmd_list(self.USB_ACB_TO_DUT[self.__default_usb_no], True):
                    self._logger.debug("usb_host_pc_connector: USB %s is already chosen" % self.__default_usb_no)
                self.usb_connector(plug)
            else:
                self.usb_connector(plug)
            return True
        else:
            self._logger.warning("usb_host_pc_connector: No USB configured, skipped!")
            return False

    def simulate_insertion(self, device_type):
        """
        Do a cable insertion (usb or other)
        If this function is called twice then it will unplug then plug again instead of doing nothing.
        :type device_type: str
        :param device_type: cable device to select. Possible values:
            - "USB_HOST_PC": USB Host PC (ACS, FW/SW Updates)
            - "SDP": like a PC or laptop that can supply 100mA by default or 500mA after negotiation.
        """
        self.get_logger().info("Select USB device: %s", device_type)

        if device_type not in self.SUPPORTED_DEVICE_TYPE:
            err_msg = "USB device %s is not supported !" % device_type
            self.get_logger().error(err_msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, err_msg)
        self.usb_connector(False)
        self.usb_host_pc_connector(True)

    def remove_cable(self, cable_type):
        """
        disconnect given cable.
        This super function will call the right function depending the cable you want to remove.

        :type cable_type: str
        :param cable_type: cable supported by your EMT and device
        """
        if cable_type not in self.SUPPORTED_DEVICE_TYPE:
            raise TestEquipmentException(
                TestEquipmentException.INVALID_PARAMETER,
                "Unsupported cable type : %s!" % cable_type)
        self.usb_connector(False)

    def reset(self):
        """
        Reset the IO card to default states
        """
        self.usb_connector(False)

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

    def get_default_wall_charger(self):
        """
        get default wall charger.

        :rtype: str
        :return: default wall charger supported by the device
                  return None if not set
        """
        return self.__wall_charger

    def wall_charger_connector(self, plug):
        """
        Handle wall charger insertion depending of available one on io card
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
            if device_default_charger in self.SUPPORTED_DEVICE_TYPE:
                self.usb_connector(False)
            else:
                raise TestEquipmentException(
                    TestEquipmentException.INVALID_PARAMETER,
                    "Unknown cable type : %s!" % device_default_charger)

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
        self.get_logger().warning("battery_connector not functional in ACBMD")

    def set_provisioning_mode(self, enable):
        """
        activate provisioning mode on the device by enabling specific
        button (volume up/down, home, ...)

        :type enable: boolean
        :param enable: action to be done:
            - True  => enable provisioning mode
            - False => disable provisioning mode

        """
        # return False to keep compatibility
        return False

    def set_default_battery_type(self, batt_type):
        """
        set default battery type.
        all function that play with battery type will take this value if the type is omitted.

        :type batt_type: str
        :param batt_type: default battery type supported by the device
        """
        self.get_logger().warning("battery_connector not functional in ACBMD")
