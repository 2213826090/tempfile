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
:summary: implementation for EMT350 equipment
         it support battery and charger(SDP, CDP, DCP, AC) emulation, tablet and otg
:author: vgomberx
:since: 15/01/2014
"""
from acs_test_scripts.Equipment.IEquipment import EquipmentBase
from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Equipment.IOCards.ACB.Common.EMT350Wrapper import EMT350Wrapper
from acs_test_scripts.Equipment.IOCards.ACB.Common.EMT350Cards import SUPPORTED_DAUGHTER_CARD
from acs_test_scripts.Equipment.IOCards.Interface.IIOCard import IIOCard
from UtilitiesFWK.Utilities import str_to_bool
import serial.tools.list_ports
import re
from time import sleep
import sys


class EMT350(EquipmentBase, IIOCard):

    # Defining possible usb devices (Usb host, DCP, ...)
    SUPPORTED_DEVICE_TYPE = [IIOCard.USB_HOST_PC, IIOCard.SDP,
                             IIOCard.DCP, IIOCard.AC_CHGR, IIOCard.OTG, IIOCard.WALL_CHARGER, IIOCard.CDP, IIOCard.ACA]
    CONNECT = "CONNECT"
    DISCONNECT = "DISCONNECT"

    BATTERY_TYPE = [
        IIOCard.BAT_ANALOG,
        IIOCard.BAT_DIGITAL,
        IIOCard.BAT_INVALID,
        IIOCard.BAT_DIG_INVALID]
    __SLOT = 4

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
        # what is the wall charger for th dut
        self.__wall_charger = None
        # do we use ext ps to supply usb
        self._usb_supply_path = "USB_SUPPLY"
        # default battery taken from device catalog
        self.__default_battery = None
        # current battery
        self.__current_battery = None
        # current battery state
        self.__current_battery_state = None
        # Type of Digital Battery
        self.__digital_battery_type = None
        # default battid value taken from device catalog
        self.__default_batt_id_value = None
        # current battid
        self.__current_batt_id_value = None
        # default bptherm value taken from device catalog
        self.__default_bptherm_value = None
        # is external power supply plug ?
        self.__ext_ps_plug = False
        # Client/Server things
        self.__func_wrapper = EMT350Wrapper()
        self.__io_card_conf = {}
        self.__automatic_serial_com_port_detection = None
        self.__serial_com_port = None
        self.__serial_baud_rate = None
        self.__retry_nb = None
        self.__server_ip = None
        self.__server_port = None
        self.__multi_exec_needed = None
        self.__usb_host = self.SDP

    def get_bench_params(self):
        """
        :rtype: bench configuration
        :return: the bench configuration dictionary of the equipment
        """
        return self.__bench_params

    def init(self):
        """
        Initializes the equipment. The equipment is ready to use.
            - Load equipment driver
            - Connection to the equipment is established
            - Show equipment informations
            - Reset equipment
        """
        self.get_logger().info("Initialization")

        #----------------------read info from bench config in xml way-----------------
        if str_to_bool(self.get_bench_params().get_param_value("ExtPowerSupplyAsCharger", False)):
            self._usb_supply_path = "EXT_SUPPLY"
        else:
            self._usb_supply_path = "USB_SUPPLY"

        self.__automatic_serial_com_port_detection = str_to_bool(self.__bench_params.get_param_value("AutomaticComPortDetection"))
        self.__serial_baud_rate = int(self.__bench_params.get_param_value("BaudRate"))

        if self.__automatic_serial_com_port_detection is True:
            # trying to get com port
            port_com_scan = self.auto_configure_com_port()
            if port_com_scan is not None:
                self.__serial_com_port = port_com_scan
            else:
                msg = "No able to found EMT 350, maybe your version of pyserial is too old (%s)" % serial.VERSION
                self.logger.warning(msg)
                self.__serial_com_port = int(self.__bench_params.get_param_value("ComPort"))
        else:
            self.__serial_com_port = int(self.__bench_params.get_param_value("ComPort"))

        self.__retry_nb = int(self.__bench_params.get_param_value("ConnectionRetry"))
        self.__usb_host = str(self.__bench_params.get_param_value("usbHost", self.SDP)).upper().strip()
        if self.__usb_host not in self.SUPPORTED_DEVICE_TYPE:
            raise TestEquipmentException(
                TestEquipmentException.INVALID_PARAMETER,
                "Unknown usbHost type on your benchconfig for EMT350 : %s" % self.__usb_host)

        # if multi campaign execution is needed, parameter will be handle here
        self.__multi_exec_needed = self.__bench_params.get_param_value("HandleMultiExecution", False)

        if type(self.__multi_exec_needed) is str:
            self.__multi_exec_needed = str_to_bool(self.__multi_exec_needed)

        if self.__multi_exec_needed is True:
            self.__server_ip = str(self.__bench_params.get_param_value("ServerIp"))
            self.__server_port = int(self.__bench_params.get_param_value("ServerPort"))
        # else use simple serial communication
        else:
            self.__func_wrapper.init_local_execution(self.__serial_com_port, self.__serial_baud_rate, self.__retry_nb)

    def load_specific_dut_config(self, dut_name):
        """
        Configure different setting on your io card related to dut name,
        This is useful in multi device campaign.
        The setting will depending of your current dut name and what you declared on
        benchconfig.
        for EMT350 this also start init the function wrapper.

        :type dut_name: str
        :param dut_name: phone name
        """
        # first check the right usb
        error_happen = False
        super_error_msg = "ERROR happen in your benchconfig for io card EMT350 declaration:\n"

        # parse the bench config to construct a dictionary on the bio card configuration
        # if any error is met then we will finish to parse all the file before raising to tell
        # the user about all bad configuration he/she has on the xml file.
        for parameter_name in self.get_bench_params().get_parameters_name():
            card_slot = None
            card_conf = {}

            for daughter_card in SUPPORTED_DAUGHTER_CARD.keys():
                # search for the card type
                # limit card name that will pass next checking to parameter name that start
                # begin with daughter card name prefixed by an _
                if not parameter_name.upper().startswith(daughter_card + "_"):
                    continue

                # isolate the slot of the card and check the format
                card_slot = parameter_name.split("_")[-1].strip()
                if not card_slot.isdigit():
                    super_error_msg += "card slot must be named with the [CARD_TYPE]_ and a digit that represent the slot like USB_1, found %s\n" % parameter_name
                    error_happen = True
                    continue
                # check that we don't go to a unknown slot declarations
                if int(card_slot) > self.__SLOT:
                    super_error_msg += "Mother card cant have more than %s slots affectation, found slot %s affectation for %s card\n" % (self.__SLOT,
                                                                                      card_slot, daughter_card)
                    error_happen = True
                    continue

                card_slot = "S" + card_slot
                # check if card slot is already declared or not
                if card_slot in self.__io_card_conf.keys():
                    super_error_msg += "it cant have more than one card per slot, detected more than 1 occurrence of %s\n" % card_slot
                    error_happen = True
                    continue

                # get daughter card parameters
                sub_parameter = self.get_bench_params().get_parameters(parameter_name)
                error, error_msg, card_conf = SUPPORTED_DAUGHTER_CARD[daughter_card]().configuration_validation(sub_parameter, dut_name)

                if error:
                    error_happen = True
                    super_error_msg += error_msg

                # store value only if a there is output configuration for a given card
                if card_slot is not None and len(card_conf) > 1:
                    self.__io_card_conf[card_slot] = card_conf

        if error_happen:
            self.get_logger().error(super_error_msg)
            raise TestEquipmentException(
                TestEquipmentException.INVALID_PARAMETER,
                "Error happen in you io card declaration on Benchconfig file, check test logs for more details")

        #  init the wrapper
        self.__func_wrapper.init(self.__io_card_conf)
        # use client/server mode
        if self.__multi_exec_needed is True:
            self.__func_wrapper.init_multi_execution(self.__server_ip, self.__server_port,
                           self.__serial_com_port, self.__serial_baud_rate, self.__retry_nb)

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
            self.__func_wrapper.BoardFunctionUsbSelect("through", self._usb_supply_path, self.__usb_host)

        else:
            self.get_logger().info("Unplug USB")
            self.__func_wrapper.BoardFunctionUsbDisconnect()

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
            if self.__func_wrapper.get_last_usb_plugged() not in [self.__usb_host, None]:
                self.get_logger().info("Unplug any USB first then Plug USB PC host")
                self.__func_wrapper.BoardFunctionUsbDisconnect()
            else:
                self.get_logger().info("Plug USB PC host")
            self.__func_wrapper.BoardFunctionUsbSelect("through", self._usb_supply_path, self.__usb_host)
        else:
            self.get_logger().info("Unplug USB PC host")
            self.__func_wrapper.BoardFunctionUsbDisconnect()

        return True

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

        self.__func_wrapper.BoardFunctionUsbDisconnect()
        sleep(0.5)

        if device_type == IIOCard.USB_HOST_PC:
            device_type = self.__usb_host

        self.__func_wrapper.BoardFunctionUsbSelect("through", self._usb_supply_path, device_type)

    def remove_cable(self, cable_type):
        """
        disconnect given cable or ALL cables.
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

    def get_battery_state(self):
        """
        get the current battery state.

        :rtype: bool
        :return: the battery state of the device
                  return None if not set
        """
        return self.__current_battery_state

    def get_battery_type(self):
        """
        get the battery type.

        :rtype: str
        :return: the battery_type of the device
                  return None if not set
        """
        return self.__current_battery

    def set_default_battery_type(self, batt_type):
        """
        set the default battery type.
        all function that play with battery type will take this value if the type is omitted.

        :type batt_type: str
        :param batt_type: default battery type supported by the device
        """
        self.get_logger().info("setting default battery type to %s" % batt_type)
        if batt_type not in self.BATTERY_TYPE:
            txt = "your io card does not support %s battery type" % batt_type
            self.get_logger().warning(txt)

        self.__default_battery = batt_type
        if self.__current_battery is None:
            self.__current_battery = self.__default_battery

    def set_default_batt_id_value(self, batt_id_value):
        """
        set the default batt_id value.
        all function that play with battid  will take this value if the type is omitted.

        :type batt_id_value: int
        :param batt_id_value: default battery id value supported by the device
        """
        self.get_logger().info("setting default battid value to %d" % batt_id_value)
        if batt_id_value < 0 or batt_id_value > 500000:
            txt = "your io card does not support this battid value (%d) " % batt_id_value
            self.get_logger().warning(txt)

        self.__default_batt_id_value = batt_id_value

        if self.__current_batt_id_value is None:
            self.__current_batt_id_value = self.__default_batt_id_value

    def set_default_bptherm_value(self, bptherm_value):
        """
        set the default bptherm value.
        all function that play with bptherm will take this value if the type is omitted.

        :type bptherm_value: int
        :param bptherm_value: default bptherm value supported by the device
        """
        self.get_logger().info("setting default battid value to %d" % bptherm_value)
        if bptherm_value < 0 or bptherm_value > 1000000:
            txt = "your io card does not support this bptherm value (%d) " % bptherm_value
            self.get_logger().warning(txt)

        self.__default_bptherm_value = bptherm_value

    def get_default_battery_type(self):
        """
        get default battery type.

        :rtype: str
        :return: get the default battery type
                  return None if not set
        """
        return self.__default_battery

    def get_default_batt_id_value(self):
        """
        get default batt id value.

        :rtype: str
        :return: get the default batt id value
                  return None if not set
        """
        return self.__default_batt_id_value

    def get_default_bptherm_value(self):
        """
        get default bptherm  value.

        :rtype: str
        :return: get the default bptherm value
                  return None if not set
        """
        return self.__default_bptherm_value

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

    def battery_connector(self, plug, battery_type="DEFAULT"):
        """
        Handles battery insertion / removal
        :type plug: boolean
        :param plug: action to be done:
            - True  => insert battery
            - False => remove battery
        :type battery_type: str
        :param battery_type: ANALOG or INVALID or DIGITAL or DIG_INVALID
        """

        if plug:
            if battery_type != "DEFAULT":
                self.set_battery_type(battery_type)
            self.get_logger().info(" CONNECT a battery %s" % self.__current_battery)
            self.__func_wrapper.BoardFunctionBatteryConnect(self.CONNECT, self.__default_bptherm_value,
                                                self.__current_batt_id_value, self.__current_battery)
        elif not plug:
            self.get_logger().info(" DISCONNECT a battery ")
            self.__func_wrapper.BoardFunctionBatteryConnect(self.DISCONNECT, 0, 0, "NONE")
        self.__current_battery_state = plug

    def press_power_button(self, duration):
        """
        Presses power button.
        Allow to simulate special behavior on the board like S3 mode.
        :type duration: float
        :param duration: time while the power button is pressed
            The value should be superior to 0 seconds
        """
        self.get_logger().info("Press power button during %s second(s)" % str(duration))

        # Put in milliseconds the duration value
        duration *= 1000
        self.__func_wrapper.HwControlPressPowerButton(duration)

    def set_battery_type(self, batt_type):
        """
        Sets Battery type.Unknown
        :type batt_type: str
        :param batt_type: batt id to select. Possible values:
            - "ANALOG"
            - "INVALID"
            - "DIGITAL"
            - "DIG_INVALID"
        """
        if batt_type == IIOCard.BAT_INVALID:
            # this only for analog batt for now
            self.__current_batt_id_value = 14000
            self.__current_battery = IIOCard.BAT_INVALID
        else:
            self.__current_battery = batt_type
            self.__current_batt_id_value = self.__default_batt_id_value


    def set_battery_temperature(self, temperature):
        """
        Sets battery temperature
        :type temperature: integer
        :param temperature: temperature in C to set, set of possible values:
        {95; 70; 50; 25; 10; 5; 0; -15}
        """
        return

    def ac_charger_connector(self, plug):
        """
        handle AC charger insertion.
        :type plug: boolean
        :param plug: action to be done:
            - True  => insert charger
            - False => remove charger
        """
        if plug:
            self.__func_wrapper.BoardFunctionAcConnect("CONNECT")
        elif not plug:
            self.__func_wrapper.BoardFunctionAcConnect("DISCONNECT")

    def release(self):
        self.get_logger().info("Release")
        if self.__func_wrapper is not None:
            self.__func_wrapper.release()

    def reset(self):
        """
        this function reset all the EMT350
        """
        self.__func_wrapper.GlobalCardReset()
        # init some status variable:
        self.__current_battery = self.__default_battery
        self.__current_battery_state = False
        self.__current_batt_id_value = self.__default_batt_id_value

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

    def auto_configure_com_port(self):
        """
        Try to autoconfigure com port
        :rtype int or None
        :return port number:
        if port number is None no EMT found on COM port
        """
        serial_test = serial.Serial()
        # get the list of avaible com port
        # this function is not sure until version 2.7 of pyserial
        com_list = list(serial.tools.list_ports.comports())
        self.logger.info("Start scanning com port to find an EMT 350")
        for port_com in com_list:

            try:
                serial_test.port = port_com[0]
                serial_test.baudrate = int(self.__serial_baud_rate)
                serial_test.open()

            except serial.SerialException:
                msg = "COM port %s seems to be in use, this is not an EMT 350 " % port_com[0]
                self.logger.info(msg)
                continue

            # send a get configuration command
            cmd = "S0?"
            serial_test.flush()
            serial_test.write(cmd)
            sleep(1)
            serial_return = serial_test.read(serial_test.inWaiting())

            # if there is SLOT in the return you can be sure you are talking to an EMT350
            if "SLOT" in serial_return:
                # EMT is found, now we have to return port number
                # this return is for exemple COM5 on windows or /dev/ttyUSB5 on linux
                real_port = int(re.findall('\d+', port_com[0])[0])
                msg = "EMT 350 found on port %s" % port_com[0]
                self.logger.info(msg)
                serial_test.close()
                if 'win' in sys.platform:
                    real_port -= 1
                return real_port
            else:
                msg = "No EMT 350 on COM port %s" % port_com[0]
                self.logger.info(msg)
                serial_test.close()

        return None
