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
:summary: relay card implementation
:since: 06/09/2011
:author: ymorel
"""
from acs_test_scripts.Equipment.IEquipment import EquipmentBase
from acs_test_scripts.Equipment.IOCards.Interface.IIOCard import IIOCard
from ErrorHandling.TestEquipmentException import TestEquipmentException
from serial.serialutil import SerialException
import signal
import sys
import time
import serial


class USBRly08(EquipmentBase, IIOCard):
    """
    Class that implements USB Relay Card 08 equipment
    """

    # Defining possible usb devices (Usb host, DCP, ...)
    SUPPORTED_DEVICE_TYPE = [IIOCard.USB_HOST_PC, IIOCard.SDP, IIOCard.DCP, IIOCard.CDP]

    __DEFAULT_STATE_TABLE = 0
    __DEFAULT_WIRING_TABLE = 0

    _ALL_LINES = 0

    RELAY_PUSH_DELAY = 0.2
    RELAY_RELEASE_DELAY = 0.2

    __CMDS = {
        "get_software_version": 90,
        "get_relay_states": 91,
        "set_relay_states": 92,
        "set_all_relay_on": 100,
        "set_all_relay_off": 110
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
        EquipmentBase.__init__(self, name, model, eqt_params)
        IIOCard.__init__(self)
        self.__bench_params = bench_params
        self.__handle = None
        self.__default_states = self.__DEFAULT_STATE_TABLE
        self.__wiring_table = self.__DEFAULT_WIRING_TABLE
        self.__usb_host_pc_connect_relay = None
        self.__usb_host_pc_power_minus = None
        self.__usb_host_pc_data_plus = None
        self.__usb_host_pc_data_minus = None
        self.__power_supply_relay = None
        self.__ps_sense_shunt_relay = None
        self.__switch_on_off_relay = None
        self.__wall_charger_relay = None
        self.__wireless_charger_relay = None
        self.__wall_charger = None
        self.__selected_usb_device = self.USB_HOST_PC
        self.__test_status_display_connect_relay = None
        self.__provisioning_mode = None
        self.__usb_otg_type_relay = None
        self.__serial = None
        self.original_sigint = signal.getsignal(signal.SIGINT)
        signal.signal(signal.SIGINT, self.exit_gracefully)

    def exit_gracefully(self, signum, frame):
        # restore the original signal handler as otherwise evil things will happen
        # in raw_input when CTRL+C is pressed, and our signal handler is not re-entrant
        signal.signal(signal.SIGINT, self.original_sigint)

        if self.__serial is not None:
            self.dispose_serial()

        self.reset()
        sys.exit(1)

        # restore the exit gracefully handler here
        signal.signal(signal.SIGINT, self.exit_gracefully)


    def openComPort(self):
        # Open COM port
        try:
            self.__serial = serial.Serial(
                self.get_bench_params().get_param_value("ComPort"),
                19200,
                serial.EIGHTBITS,
                serial.PARITY_NONE,
                serial.STOPBITS_ONE,
                None)
        except:
            print "Failed to open COM port"

    def closeComPort(self):
        try:
            self.__serial.close()
        except:
            print "Failed to close COM port"

    def get_bench_params(self):
        """
        Returns the bench parameters of the equipment
        """
        return self.__bench_params

    def init(self):
        """
        Initializes the equipment. The equipment is ready to use
        """
        self.get_logger().info("USBRly08 Initialization")

        # In case of multi IOcards connected to one PC,
        # best practise is to serialize IOCard. the 1st IOCard controls
        # USB power of the 2nd.
        # This architecture ensures comport numbers are fixed

        # if that IOCard is slave of a previous one
        # First is to power on USB using relay from Master IOCard
        if ((self.__bench_params.has_parameter("IOCard_master")) and
                (self.__bench_params.get_param_value("IOCard_master") != "")):
            mcard_name = \
                str(self.__bench_params.get_param_value("IOCard_master"))

            # Get the ID of the Master card's relay to control
            # power of slave card
            if ((self.__bench_params.has_parameter("RelayOnOffMasterCard")) and
                    (self.__bench_params.get_param_value("RelayOnOffMasterCard") != "")):
                rel_power_card = \
                    int(self.__bench_params.get_param_value("RelayOnOffMasterCard"))
            else:
                msg = "RelayOnOffMasterCard is not defined."
                self.get_logger().error(msg)
                raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, msg)

            # instantiate master IOCard
            # NOTE: import here to avoid circular dependency on
            # EquipmentManager if imported at top level
            from acs_test_scripts.Equipment.EquipmentManager import EquipmentManager

            eqm = EquipmentManager()
            mcard = eqm.get_io_card(mcard_name)

            # Power ON USB wire of that IOCard
            mcard.enable_line(rel_power_card)

        # Be careful to get wiring table before setting default relay states
        if ((self.get_bench_params().has_parameter("WiringTable")) and
                (self.__bench_params.get_param_value("WiringTable") != "")):
            self.__wiring_table = \
                int(self.get_bench_params().get_param_value("WiringTable"), 2)

            self.get_logger().info("Set Wiring Table settings to %s",
                                   self.__wiring_table)

        # Get the ID of the relay for USB host PC plug/unplug if exists or filled
        if ((self.get_bench_params().has_parameter("UsbHostPcConnect")) and
                (self.__bench_params.get_param_value("UsbHostPcConnect") != "")):
            self.__usb_host_pc_connect_relay = \
                int(self.__bench_params.get_param_value("UsbHostPcConnect"))

        # Get the ID of the relay for USB host PC plug/unplug if exists or filled
        if ((self.get_bench_params().has_parameter("UsbHostPcPowerMinus")) and
                (self.__bench_params.get_param_value("UsbHostPcPowerMinus") != "")):
            self.__usb_host_pc_power_minus = \
                int(self.__bench_params.get_param_value("UsbHostPcPowerMinus"))

        # Get the ID of the relay for USB host PC plug/unplug if exists or filled
        if ((self.get_bench_params().has_parameter("UsbHostPcDataPlus")) and
                (self.__bench_params.get_param_value("UsbHostPcDataPlus") != "")):
            self.__usb_host_pc_data_plus = \
                int(self.__bench_params.get_param_value("UsbHostPcDataPlus"))

            # Get the ID of the relay for USB host PC plug/unplug if exists or filled
        if ((self.get_bench_params().has_parameter("UsbHostPcDataMinus")) and
                (self.__bench_params.get_param_value("UsbHostPcDataMinus") != "")):
            self.__usb_host_pc_data_minus = \
                int(self.__bench_params.get_param_value("UsbHostPcDataMinus"))

        # Get the ID of the relay for wall charger plug/unplug if exists or filled
        if ((self.get_bench_params().has_parameter("WallCharger")) and
                (self.__bench_params.get_param_value("WallCharger") != "")):
            self.__wall_charger_relay = \
                int(self.__bench_params.get_param_value("WallCharger"))

        # Get the ID of the relay for wireless charger plug/unplug if exists or filled
        if ((self.get_bench_params().has_parameter("WirelessCharger")) and
                (self.__bench_params.get_param_value("WirelessCharger") != "")):
            self.__wireless_charger_relay = \
                int(self.__bench_params.get_param_value("WirelessCharger"))

        # Get the ID of the relay for switching ON/OFF the board if exits or filled
        if ((self.get_bench_params().has_parameter("SwitchOnOff")) and
                (self.__bench_params.get_param_value("SwitchOnOff") != "")):
            self.__switch_on_off_relay = \
                int(self.__bench_params.get_param_value("SwitchOnOff"))
            self.PHONE_BUTTON[self.PWR_BUTTON] = self.__switch_on_off_relay

        # Get the ID of the relay for linking power supply to the device
        if ((self.get_bench_params().has_parameter("PowerSupply")) and
                (self.__bench_params.get_param_value("PowerSupply") != "")):
            self.__power_supply_relay = \
                int(self.__bench_params.get_param_value("PowerSupply"))
            if ((self.get_bench_params().has_parameter("PowerSupplySenseShunt")) and
                    (self.__bench_params.get_param_value("PowerSupplySenseShunt") != "")):
                self.__ps_sense_shunt_relay = \
                    int(self.__bench_params.get_param_value("PowerSupplySenseShunt"))

        # Get the ID of the relay linked to the button to enable provisioning
        # mode on the device (may be volum up/down, home, ...)
        if ((self.get_bench_params().has_parameter("ProvisioningMode")) and
                (self.__bench_params.get_param_value("ProvisioningMode") != "")):
            self.__provisioning_mode = \
                int(self.__bench_params.get_param_value("ProvisioningMode"))

        # Get the ID of the relay for volume up, volume down
        for key in self.BUTTON_LIST:
            # Convert Button key to known key from IO Card.
            # i.e. : VOLUME_UP => VolumeUp
            relay_key = str(key).title().replace('_', '')

            if self.get_bench_params().has_parameter(relay_key):
                relay_id = int(self.__bench_params.get_param_value(relay_key))
                if relay_id:
                    self.PHONE_BUTTON[key] = relay_id

        if self.get_bench_params().has_parameter("DefaultStates"):
            if self.__bench_params.get_param_value("DefaultStates") != "":
                self.__default_states = int(self.get_bench_params().get_param_value("DefaultStates"), 2)
                self.get_logger().info("Set Default State settings to %s", self.__default_states)
                self.set_relay_states(self.__default_states)
            else:
                self.get_logger().info("io_card relay status remains unchanged")
        else:
            self.get_logger().info("Default State: %s" % self.__default_states)
            self.set_relay_states(self.__default_states)

        # Get the ID of the relay for the status display and connect it if exist
        if ((self.get_bench_params().has_parameter("TestStatusDisplayConnect")) and
                (self.__bench_params.get_param_value("TestStatusDisplayConnect") != "")):
            self.__test_status_display_connect_relay = \
                int(self.__bench_params.get_param_value("TestStatusDisplayConnect"))
            # enable display status because we found the parameter in the bench config
            self.get_logger().info("Setting bench state display to RUNNING...")
            self.enable_line(self.__test_status_display_connect_relay)

        # Get the ID of the relay for USB OTG switch (USB device / USB Host mode)
        if ((self.get_bench_params().has_parameter("UsbOtgSwitch")) and
                (self.__bench_params.get_param_value("UsbOtgSwitch") != "")):
            self.get_logger().info("UsbOtgSwitch found in bench params")
            self.__usb_otg_type_relay = \
                int(self.__bench_params.get_param_value("UsbOtgSwitch"))

    def enable_line(self, line):
        """
        Enables input/output line.
        :type line: integer
        :param line: the number of line to enable. 0 means all
        """
        if line:
            self.set_relay_on(line)
        else:
            self.set_all_relay_on()

    def disable_line(self, line):
        """
        Disables input/output line.
        :type line: integer
        :param line: the number of line to disable. 0 means all
        """
        if line:
            self.set_relay_off(line)
        else:
            self.set_all_relay_off()

    def usb_connector(self, plug):
        """
        Handles USB connector connection and disconnection
        :type plug: boolean
        :param plug: action to be done:
            - True  => plug currently selected USB device
            - False => unplug currently selected USB device
        """
        if self.__selected_usb_device == self.USB_HOST_PC:
            # Call specific method to connect/disconnect the usb PC Host
            self.usb_host_pc_connector(plug)
        else:
            err_msg = "USB device %s is not supported !" % self.__selected_usb_device
            self.get_logger().error(err_msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, err_msg)

    def usb_device_selector(self, usb_device):
        """
        Handles USB device selection
        :type usb_device: str
        :param usb_device: USB device to select. Possible values:
            - "USB_HOST_PC": USB Host PC (ACS, FW/SW Updates)
            - "DCP": Dedicated Charging Port, like a wall charger that can supply 500mA-1.5A by default.
            - "SDP": like a PC or laptop that can supply 100mA by default or 500mA after negotiation.
            - "CDP": like a PC or laptop that can supply 500mA at all times but may be able to support up to 1.5A.
            - "OTG": USB stick
        .. warning:: The values DCP, SDP, CDP, OTG are not supported for the moment. See ACB equipments to use
                     those properties
        """
        self.get_logger().info("Select USB device: %s", usb_device)

        if usb_device in self.SUPPORTED_DEVICE_TYPE:
            relay_mapping = {self.USB_HOST_PC: self.__usb_host_pc_connect_relay,
                             self.SDP: self.__usb_host_pc_connect_relay}
            self.disable_line(relay_mapping[usb_device])
            time.sleep(2)
            self.enable_line(relay_mapping[usb_device])

            self.__selected_usb_device = usb_device
        else:
            err_msg = "USB device %s is not supported !" % usb_device
            self.get_logger().error(err_msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, err_msg)

    def simulate_insertion(self, device_type):
        """
        Do a cable insertion (usb or other)
        If this function is called twice then it will unplug then plug again instead of doing nothing.
        :type device_type: str
        :param device_type: cable device to select. Possible values:
            - "USB_HOST_PC": USB Host PC (ACS, FW/SW Updates)
            - "DCP": Dedicated Charging Port, like a wall charger that can supply 500mA-1.5A by default.
            - "SDP": like a PC or laptop that can supply 100mA by default or 500mA after negotiation.
            - "CDP": like a PC or laptop that can supply 500mA at all times but may be able to support up to 1.5A.
            - "OTG": USB stick
        .. warning:: The values DCP, SDP, CDP, OTG are not supported for the moment. See ACB equipments to use
                     those properties
        """
        self.get_logger().info("Select USB device: %s", device_type)

        if device_type in self.SUPPORTED_DEVICE_TYPE:
            relay_mapping = {self.USB_HOST_PC: self.__usb_host_pc_connect_relay,
                             device_type: self.__usb_host_pc_connect_relay}
            self.disable_line(relay_mapping[device_type])
            time.sleep(2)
            self.enable_line(relay_mapping[device_type])
        else:
            err_msg = "USB device %s is not supported !" % device_type
            self.get_logger().error(err_msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, err_msg)

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

        if cable_type in [IIOCard.DCP, IIOCard.SDP, IIOCard.CDP, IIOCard.USB_HOST_PC]:
            self.usb_connector(False)
        elif cable_type == IIOCard.AC_CHGR:
            self.ac_charger_connector(False)
        elif cable_type == IIOCard.WALL_CHARGER:
            self.wall_charger_connector(False)
        elif cable_type == IIOCard.WIRELESS_CHARGER:
            self.wireless_charger_connector(False)

    def usb_host_pc_connector(self, plug):
        """
        Handles USB connector connection and disconnection of USB host PC device
        :type plug: boolean
        :param plug: action to be done true = plug, false = unplug
        """
        if self.__usb_host_pc_connect_relay is not None:
            if plug:
                self.get_logger().info("Plug USB to host PC")
                if self.__usb_host_pc_power_minus is not None and \
                                self.__usb_host_pc_data_plus is not None:

                    if self.__usb_host_pc_data_minus is not None:
                        self.enable_line("%d%d" % (self.__usb_host_pc_connect_relay,
                                                   self.__usb_host_pc_power_minus))
                        time.sleep(0.1)
                        self.enable_line("%d%d" % (self.__usb_host_pc_data_plus,
                                                   self.__usb_host_pc_data_minus))
                    else:
                        self.enable_line("%d" % self.__usb_host_pc_connect_relay)
                        self.enable_line("%d" % self.__usb_host_pc_data_plus)
                else:
                    self.enable_line(self.__usb_host_pc_connect_relay)
            else:
                self.get_logger().info("Unplug USB from host PC")
                if self.__usb_host_pc_power_minus is not None and \
                                self.__usb_host_pc_data_plus is not None:

                    if self.__usb_host_pc_data_minus is not None:
                        self.disable_line("%d%d" % (self.__usb_host_pc_data_plus,
                                                    self.__usb_host_pc_data_minus))
                        time.sleep(0.1)
                        self.disable_line("%d%d" % (self.__usb_host_pc_power_minus,
                                                    self.__usb_host_pc_connect_relay))
                    else:
                        self.disable_line("%d" % self.__usb_host_pc_data_plus)
                        self.disable_line("%d" % self.__usb_host_pc_power_minus)
                else:
                    self.disable_line(self.__usb_host_pc_connect_relay)
            return True
        else:
            msg = "USB Host PC Connect relay not configured; No action taken!"
            self.get_logger().warning(msg)
            return False

    def battery_connector(self, plug, battery_type="RELAY"):
        """
        Handles battery insertion / removal

        :type plug: boolean
        :param plug: action to be done:
            - True  => insert battery
            - False => remove battery
        :type battery_type: str
        :param battery_type: battery to plug
        """
        if battery_type == "RELAY":
            if self.__power_supply_relay is not None:
                if plug:
                    self.get_logger().info("Plug PowerSupply")
                    self.enable_line(self.__power_supply_relay)
                    if self.__ps_sense_shunt_relay is not None:
                        time.sleep(0.5)
                        self.enable_line(self.__ps_sense_shunt_relay)
                else:
                    self.get_logger().info("Unplug PowerSupply")
                    if self.__ps_sense_shunt_relay is not None:
                        self.disable_line(self.__ps_sense_shunt_relay)
                        time.sleep(0.5)
                    self.disable_line(self.__power_supply_relay)
                return True
            else:
                self.get_logger().info("Power supply relay not present / configured. No action taken!")
                return False
        elif battery_type == "ALWAYS_ON":
            self.get_logger().debug("DUT always powered. No action taken!")
            return True
        else:
            self.get_logger().warning("Battery type %s not supported" % battery_type)
            return False

    def press_power_button(self, duration):
        """
        Presses power button.
        Allow to simulate special behavior on the board like idle mode.
        :type duration: float
        :param duration: time while the power button is pressed
            The value should be superior than 0 seconds
        """
        if self.__switch_on_off_relay is not None:
            self.press_relay(self.__switch_on_off_relay, duration)
        else:
            msg = "Power button relay not configured; No action taken!"
            self.get_logger().warning(msg)

    def push_power_button(self):
        """
        Presses power button.
        """
        if self.__switch_on_off_relay is not None:
            self.enable_line(self.__power_supply_relay)
        else:
            msg = "Power button relay not configured; No action taken!"
            self.get_logger().warning(msg)

    def release_power_button(self):
        """
        Presses power button.
        """
        if self.__switch_on_off_relay is not None:
            self.enable_line(self.__power_supply_relay)
        else:
            msg = "Power button relay not configured; No action taken!"
            self.get_logger().warning(msg)

    def press_relay_button(self, duration, relay_nb):
        """
        Presses relay during time duration like a button.
        :type duration: float
        :param relay_nb: relay number value to be pressed
        :param duration: time while the relay is pressed
            The value should be superior than 0 seconds
        """
        self.press_relay(relay_nb, duration)

    def reset(self):
        """
        Reset the IO card to default states
        """
        self._logger.info("Reset to default states")

        self.get_logger().info("Set Default State settings to %s",
                               self.__default_states)
        self.set_relay_states(self.__default_states)

    def set_provisioning_mode(self, enable):
        """
        activate provisioning mode on the device by enabling specific
        button (volume up/down, home, ...) if bench is configured and user requests it

        :type enable: boolean
        :param enable: action to be done:
            - True  => enable provisioning mode
            - False => disable provisioning mode

        """
        if self.__provisioning_mode:
            if enable:
                self.get_logger().info("Activate provisioning mode.")
                self.enable_line(self.__provisioning_mode)
            else:
                self.get_logger().info("Disable provisioning mode.")
                self.disable_line(self.__provisioning_mode)
            result = True
        else:
            result = False

        return result

    def wall_charger_connector(self, plug):
        """
        handle wall charger insertion depending of available one on io card
        and supported one by device.
        :type plug: boolean
        :param plug: action to be done:
            - True  => insert charger
            - False => remove charger

        """
        if self.__wall_charger_relay is not None:
            if plug:
                self.get_logger().info("Plug wall charger")
                self.enable_line(self.__wall_charger_relay)
            else:
                self.get_logger().info("Unplug wall charger")
                self.disable_line(self.__wall_charger_relay)
            return True
        else:
            msg = "Wall charger relay not configured; No action taken!"
            self.get_logger().warning(msg)
            return False

    def set_default_wall_charger(self, device_default_charger):
        """
        set default wall charger.

        :type device_default_charger: str
        :param device_default_charger: default wall charger supported by the device

        .. warning:: function not used for usb rely
        """
        self.get_logger().info("setting default wall charger %s" % device_default_charger)
        self.__wall_charger = device_default_charger

    def get_default_wall_charger(self):
        """
        get default wall charger.

        :rtype: str
        :return: default wall charger supported by the device
                  return None if not set
        """
        return self.__wall_charger

    def wireless_charger_connector(self, plug):
        """
        handle wireless charger insertion depending of available one on io card
        and supported one by device.
        :type plug: boolean
        :param plug: action to be done:
            - True  => insert charger
            - False => remove charger

        """
        if self.__wireless_charger_relay is None:
            self.get_logger().warning("Wireless charger relay not configured. No action taken!")
            return False

        if plug:
            self.get_logger().info("Plug wireless charger")
            self.enable_line(self.__wireless_charger_relay)
        else:
            self.get_logger().info("Unplug wireless charger")
            self.disable_line(self.__wireless_charger_relay)
        return True

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

    def set_default_battery_type(self, batt_type):
        """
        set default battery type.
        all function that play with battery type will take this value if the type is omitted.

        :type batt_type: str
        :param batt_type: default battery type supported by the device
        """
        # put for compatibility reason
        pass

    def get_default_battery_type(self):
        """
        get default battery type.

        :rtype: str
        :return: default battery type
                  return None if not set
        """
        # put for compatibility reason
        pass

    def release(self):
        """
         Release resources if any
        """
        pass

    def press_key_combo(self, button_list, press_duration, push_delay=RELAY_PUSH_DELAY,
                        release_delay=RELAY_RELEASE_DELAY):
        """
        push on the following button

        :type button_list: list of str
        :param button_list: list of button to press on. defined by PHONE_BUTTON key

        :type press_duration: float
        :param press_duration: time to keep button on

        :type push_delay: float
        :param push_delay: delay between button push

        :type release_delay: float
        :param release_delay: delay between button release
        """

        if not set(button_list).issubset(set(self.PHONE_BUTTON)):
            self.get_logger().error("missing unlinked to relay button found :" + str(button_list))
            raise TestEquipmentException(
                TestEquipmentException.INVALID_PARAMETER,
                "Unknown or not configured button : %s" % str(button_list))

        self.get_logger().info("pressing button %s during %s s" % (str(button_list), press_duration))
        # push button
        for button_name in button_list:
            self.enable_line(self.PHONE_BUTTON[button_name])  # press button
            time.sleep(push_delay)
        # wait a while
        time.sleep(press_duration)
        # release button
        for button_name in button_list:
            self.disable_line(self.PHONE_BUTTON[button_name])  # release button
            time.sleep(release_delay)

    def set_usb_otg_type(self, otg_type):
        """
        Sets the OTG type
        :type otg_type: str
        :param otg_type: OTG type to select. Possible values:
            - "DUT_DEVICE"
            - "DUT_HOST"
        """
        self.get_logger().info("Set OTG type to : %s" % otg_type)

        if otg_type == "DUT_HOST":
            self.enable_line(self.__usb_otg_type_relay)
        elif otg_type in ["DUT_DEVICE", "NORMAL"]:
            self.disable_line(self.__usb_otg_type_relay)
        else:
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                         "Unknown OTG type : %s" % str(otg_type))

    def get_relay_states(self):
        """
        Sends a single byte back to the controller, bit high meaning
        the corresponding relay is powered.
        :rtype: integer
        :return: an integer representing the states of the relays
        An integer from 0 to 255
        """
        self.get_logger().info("Get relay states")
        cmd = chr(self.__CMDS["get_relay_states"])
        # get serial before read/write to avoid a connection closed
        serial_port = self.get_serial()
        serial_port.write(cmd)
        relay_states = serial_port.read()
        if relay_states is None:
            msg = "Failed to read relay states"
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.READ_PARAMETER_ERROR, msg)
        elif len(relay_states) == 0:
            msg = "Failed to read relay states"
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.READ_PARAMETER_ERROR, msg)
        else:
            self.get_logger().info("Relays states: %s", str(ord(relay_states)))
        return ord(relay_states)

    def set_relay_states(self, states):
        """
        Sends a single byte that represents the desired relay states.
        All on: 255, all off: 0.
        :type states: integer
        :param states: the desired states far all relays. An integer
        from 0 to 255.
        """
        if states not in range(0, 256):
            msg = "Invalid relay states (%s)" % (str(states))
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        serial_port = self.get_serial()
        serial_port.write(chr(self.__CMDS["set_relay_states"]))
        cmd = chr(states ^ self.__wiring_table)

        serial_port.write(cmd)

    def set_all_relay_on(self):
        """
        Sets all relay ON.
        """
        self.get_logger().info("Set all relay to ON")
        self.set_relay_states(255)

    def set_all_relay_off(self):
        """
        Sets all relay OFF.
        """
        self.get_logger().info("Set all relay to OFF")
        self.set_relay_states(0)

    def set_relay_on(self, relay_id, serial_port=None):
        """
        Enables relay of relay_id.
        :type relay_id: integer
        :param relay_id: the number of relay
        """
        relay_ids = str(relay_id)
        cmd = list()
        for relay in relay_ids:
            if int(relay) not in range(1, 9):
                error_msg = "Relay number %s is not supported" % relay
                self.get_logger().error(error_msg)
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, error_msg)
            if (pow(2, int(relay) - 1) & self.__wiring_table) != 0:
                # Relay is normally closed
                self.get_logger().debug("Set relay %s to OFF", str(relay_id))
                cmd.append(chr(self.__CMDS["set_all_relay_off"] + int(relay)))
            else:  # Relay is normally opened
                self.get_logger().debug("Set relay %s to ON", str(relay_id))
                cmd.append(chr(self.__CMDS["set_all_relay_on"] + int(relay)))
        self.get_logger().debug("CMD sent to set relay on : %s" % cmd)

        self.write(cmd, serial_port)

    def set_relay_off(self, relay_id, serial_port=None):
        """
        Disables relay of relay_id.
        :type relay_id: integer
        :param relay_id: the number of relay
        """
        relay_ids = str(relay_id)
        cmd = list()
        for relay in relay_ids:
            if int(relay) not in range(1, 9):
                error_msg = "Relay number %s is not supported" % relay
                self.get_logger().error(error_msg)
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, error_msg)
            if (pow(2, int(relay) - 1) & self.__wiring_table) != 0:
                # Relay is normally closed
                self.get_logger().debug("Set relay %s to ON", str(relay_id))
                cmd.append(chr(self.__CMDS["set_all_relay_on"] + int(relay)))
            else:  # Relay is normally opened
                self.get_logger().debug("Set relay %s to OFF", str(relay_id))
                cmd.append(chr(self.__CMDS["set_all_relay_off"] + int(relay)))
        self.get_logger().debug("CMD sent to set relay off : %s" % cmd)

        self.write(cmd, serial_port)

    def press_relay(self, relay_id, delay):
        """
        Press (enable/disable) relay of relay_id
        during a delay.
        :type relay_id: integer
        :param relay_id: the number of relay
        :type delay: integer
        :param delay: delay
        """
        self.get_logger().info("Press button %d during %3.2f second(s)" % (relay_id, delay))

        serial_port = self.get_serial()

        self.set_relay_on(relay_id, serial_port)

        while delay > 0:
            t0 = time.time()
            time.sleep(0.2)
            t1 = time.time()
            delay -= (t1 - t0)

        self.set_relay_off(relay_id, serial_port)

        self.dispose_serial()

    def write(self, cmd, serial_port=None):
        """
        Writes the given command to the configured COM port.
        :type cmd: str
        :param cmd: the command to write
        """
        result = ""
        do_dispose = False
        if serial_port is None:
            serial_port = self.get_serial()
            do_dispose = True

        for single in cmd:
            previous_status = self.get_relay_states()

            result += str(serial_port.write(single))

            current_status = self.get_relay_states()

            if ord(single) in range(101, 108):
                num_relay = ord(single) - self.__CMDS["set_all_relay_on"]
                status_fails = (current_status != (previous_status | (255 & pow(2, num_relay-1))))
            elif ord(single) > 110:
                num_relay = ord(single) - self.__CMDS["set_all_relay_off"]
                status_fails = (current_status != (previous_status & (255 ^ pow(2, num_relay-1))))

            if status_fails:
                raise TestEquipmentException(TestEquipmentException.CONNECTIVITY_ERROR,
                                             "Serial Port command fails")

        if do_dispose:
            self.dispose_serial()
        return result

    def read(self, byte_count, serial_port=None):
        """
        Read the given number of bytes from the configured COM port.
        :type byte_count: int
        :param byte_count: the number of bytes to read

        :rtype: str
        :return: a one character-long str
        """
        result = ""
        do_dispose = False
        if serial_port is None:
            serial_port = self.get_serial()
            do_dispose = True

        if serial_port is not None:
            result = serial_port.read(byte_count)
        else:
            raise TestEquipmentException(TestEquipmentException.CONNECTIVITY_ERROR,
                                         "Serial Port is not initialized, check USBRly08 connection")

        if do_dispose:
            self.dispose_serial()
        return result

    def get_serial(self):
        """
        Returns the serial port instance to use.

        :rtype: serial.Serial
        :return: the serial port
        """
        if not self.__serial:
            try_index = 0
            retry_nb = 3

            # Access to serial port is not reliable
            # Try several times
            while try_index < retry_nb:
                try:
                    self.__serial = serial.Serial(
                        str(self.get_bench_params().get_param_value("ComPort")),
                        19200,
                        serial.EIGHTBITS,
                        serial.PARITY_NONE,
                        serial.STOPBITS_TWO,
                        None)
                    break
                except SerialException as error:
                    if try_index < retry_nb:
                        self.get_logger().error(str(error))
                        self.get_logger().info("Retry the connection...")
                        try_index += 1
                        time.sleep(1)
                    else:
                        raise error

        return self.__serial

    def dispose_serial(self):
        """
        Disposes of this object's current I{serial} instance if any.
        :rtype: None
        """
        if self.__serial is not None:
            self.__serial.close()
            self.__serial = None