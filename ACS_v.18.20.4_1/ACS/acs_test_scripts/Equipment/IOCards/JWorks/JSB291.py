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
:summary: relay card implementation for JSB291
:since: 11/12/2013
:author: vdechefd
"""

import sys

from acs_test_scripts.Equipment.IEquipment import DllLoader
from acs_test_scripts.Equipment.IOCards.Interface.IIOCard import IIOCard

# Vendor requests (change depending on the product type)
PRODUCT_COMMANDS = {"PRODUCT_ID": 0x0291,
                    "SERIAL_NUM_INDEX": 5,
                    "REQUEST_NUM_RELAY": 0xb2,
                    "REQUEST_RELAY_STATE": 0xb7,
                    "REQUEST_SET_RELAY": 0xb8,
                    "REQUEST_CLEAR_RELAY": 0xb9}


class JSB291(DllLoader, IIOCard):
    """
    Class that implements J-Works JSB921 Relay Card equipment
    """

    # Defining possible usb devices
    SUPPORTED_DEVICE_TYPE = [IIOCard.USB_HOST_PC]

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
        DllLoader.__init__(self, name, model, eqt_params)
        IIOCard.__init__(self)
        self.__bench_params = bench_params
        self.__wiring_table = 0
        self.__usb_host_pc_connect_relay = 0
        self.__switch_on_off_relay = 0
        self.__jwork_wrapper = None

    def init(self):
        """
        Initializes the equipment. The equipment is ready to use
        """
        self.get_logger().info("Initialization")

        # import the relay card wrapper, depending on OS
        if 'posix' in sys.builtin_module_names:
            from acs_test_scripts.Equipment.IOCards.JWorks.Wrapper.JWorksWrapperLinux import JWorksWrapperLinux
            self.__jwork_wrapper = JWorksWrapperLinux(PRODUCT_COMMANDS, self._logger)
        else:
            from acs_test_scripts.Equipment.IOCards.JWorks.Wrapper.JWorksWrapperWindows import JWorksWrapperWindows
            self.__jwork_wrapper = JWorksWrapperWindows(self, self._logger)

        # get wiring table (describing relays that are NC or NO), used to initialize the relay card
        if (self.__bench_params.has_parameter("WiringTable") and
                    self.__bench_params.get_param_value("WiringTable") != ""):
            self.__wiring_table = int(self.__bench_params.get_param_value("WiringTable"), 2)
            self.get_logger().info("Wiring Table is set to %s", self.__wiring_table)

        # Get the ID of the relay for USB host PC plug/unplug if exists or filled
        if (self.__bench_params.has_parameter("UsbHostPcConnect") and
                    self.__bench_params.get_param_value("UsbHostPcConnect") != ""):
            self.__usb_host_pc_connect_relay = int(self.__bench_params.get_param_value("UsbHostPcConnect"))

        # Get the ID of the relay for switching ON/OFF the board if exits or filled
        if (self.__bench_params.has_parameter("SwitchOnOff") and
                    self.__bench_params.get_param_value("SwitchOnOff") != ""):
            self.__switch_on_off_relay = int(self.__bench_params.get_param_value("SwitchOnOff"))

        # disconnect all cables plugged in relay card, using wiring table
        self.reset()
        self.__jwork_wrapper.set_wiring_table(self.__wiring_table)

    def enable_line(self, line):
        """
        Enables input/output line.
        :type line: integer
        :param line: the number of line to enable. 0 means all
        """
        self.__jwork_wrapper.set_relay_off(line)

    def disable_line(self, line):
        """
        Disables input/output line.
        :type line: integer
        :param line: the number of line to disable. 0 means all
        """
        self.__jwork_wrapper.set_relay_on(line)

    def usb_connector(self, plug):
        """
        Handles USB connector connection and disconnection
        :type plug: boolean
        :param plug: action to be done:
            - True  => plug currently selected USB device
            - False => unplug currently selected USB device
        """
        self.usb_host_pc_connector(plug)

    def usb_host_pc_connector(self, plug):
        """
        Handles USB connector connection and disconnection of USB host PC device
        :type plug: boolean
        :param plug: action to be done true = plug, false = unplug
        """
        if self.__usb_host_pc_connect_relay is not None:
            if plug:
                self.get_logger().info("Plug USB to host PC")
                self.__jwork_wrapper.set_relay_off(self.__usb_host_pc_connect_relay)
            else:
                self.get_logger().info("Unplug USB from host PC")
                self.__jwork_wrapper.set_relay_on(self.__usb_host_pc_connect_relay)
            return True
        else:
            msg = "USB Host PC Connect relay not configured; No action taken!"
            self.get_logger().warning(msg)
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
            self._logger.info("Press power button during %3.2f second(s)", duration)
            is_pressed = self.press_relay_button(duration, self.__switch_on_off_relay)
            self._logger.info("Press power button %s", "succeeded" if is_pressed else "failed")
        else:
            msg = "Power button relay not configured; No action taken!"
            self.get_logger().warning(msg)

    def press_relay_button(self, duration, relay_nb):
        """
        Presses relay during time duration, like a button.
        :type duration: float
        :param duration: time while the relay is pressed
            The value should be superior than 0 seconds
        :type relay_nb: 1
        :param relay_nb: relay number value to be pressed
        """
        return self.__jwork_wrapper.press_relay(relay_nb, duration)

    def reset(self):
        """
        Reset the IO card, setting all relays to OFF
        """
        self._logger.info("Reset all relays")
        return self.__jwork_wrapper.clear_all_relays()

    # ----------------- EMPTY METHODS : defined only for compatibility reason -----------------------

    def load_specific_dut_config(self, dut_name):
        """
        .. warning:: not functional for this io card, defined only for compatibility reason
        """
        self._logger.debug("load_specific_dut_config is not supported by this relay card")
        pass

    def set_default_wall_charger(self, device_default_charger):
        """
        .. warning:: not functional for this io card, defined only for compatibility reason
        """
        self._logger.debug("set_default_wall_charger is not supported by this relay card")
        pass

    def set_default_battery_type(self, batt_type):
        """
        .. warning:: not functional for this io card, defined only for compatibility reason
        """
        self._logger.debug("set_default_battery_type is not supported by this relay card")
        pass

    def battery_connector(self, plug, battery_type=""):
        """
        .. warning:: not functional for this io card, defined only for compatibility reason
        """
        self._logger.debug("battery_connector is not supported by this relay card")
        pass

    def set_provisioning_mode(self, enable):
        """
        .. warning:: not functional for this io card, defined only for compatibility reason
        """
        self._logger.debug("set_provisioning_mode is not supported by this relay card")
        return False
