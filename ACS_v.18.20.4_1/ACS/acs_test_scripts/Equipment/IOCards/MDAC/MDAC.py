"""
@copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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

@organization: INTEL MCG PSI
@summary: relay card implementation
@since: 07/07/2014
@author: jlmathew
"""

from Equipment.IEquipment import EquipmentBase
from Equipment.IOCards.Interface.IIOCard import IIOCard
#from ErrorHandling.Equipment.EqIOException import EqIOException
from ErrorHandling.TestEquipmentException import TestEquipmentException
import time
import socket
import serial


class MDAC(EquipmentBase, IIOCard):

    """
    Class that implements USB Relay Card 08 equipment
    """

    RCP_SERVER_IP = 'localhost'
    RCP_SERVER_PORT = 8002

    # Defining possible usb devices (Usb host, DCP, ...)
    SUPPORTED_DEVICE_TYPE = [IIOCard.USB_HOST_PC, IIOCard.SDP, IIOCard.DCP]

    __DEFAULT_STATE_TABLE = 0
    __DEFAULT_WIRING_TABLE = 0

    _ALL_LINES = 0

    def __init__(self, name, model, eqt_params, bench_params):
        """
        Constructor
        @type name: string
        @param name: the bench configuration name of the equipment
        @type model: string
        @param model: the model of the equipment
        @type eqt_params: dict
        @param eqt_params: the dictionary containing equipment parameters
        @type bench_params: dict
        @param bench_params: the dictionary containing equipment bench parameters
        """
        print 'Name: %s' % name
        print 'Model: %s' % model
        print 'EQT: %s' % eqt_params
        print 'Bench: %s' % bench_params
        EquipmentBase.__init__(self, name, model, eqt_params)
        IIOCard.__init__(self)
        self.get_logger().debug("Starting INIT")
        self.__bench_params = bench_params
        self.__handle = None
        self.__default_states = MDAC.__DEFAULT_STATE_TABLE
        self.__wiring_table = MDAC.__DEFAULT_WIRING_TABLE
        self.__usb_host_pc_connect_relay = None
        self.__power_supply_relay = None
        self.__hdmi_connect_relay = None
        self.__ext_supply_connect_relay = None
        self.__switch_on_off_relay = None
        self.__wall_charger_relay = None
        self.__rpc_server_port = None
        self.__wall_charger = None
        self.__selected_usb_device = MDAC.USB_HOST_PC
        self.__test_status_display_connect_relay = None
        self.__provisioning_mode = None
        self.serial = None

        self.get_logger().debug("Finished INIT")


    def __del__(self):
        """
        Destructor: release automatically the connection
        """
        self.get_logger().debug("Delete %s", str(self.get_name()))
        if self.serial():
            self.close_serial()


    def get_bench_params(self):
        """
        Returns the bench parameters of the equipment
        """
        return self.__bench_params

    def init(self):
        """
        Initializes the equipment. The equipment is ready to use
        """
        self.get_logger().info("Initialization")

        # In case of multi IOcards connected to one PC,
        # best practise is to serialize IOCard. the 1st IOCard controls
        # USB power of the 2nd.
        # This architecture ensures comport numbers are fixed

        # instantiate master IOCard
        # NOTE: import here to avoid circular dependency on
        # EquipmentManager if imported at top level
        from Equipment.EquipmentManager import EquipmentManager
        eqm = EquipmentManager()

        com_port = self.get_bench_params().get_param_value("ComPort")

        self.serial = serial.Serial()
        self.serial.port = com_port
        self.serial.baudrate = 9600
        self.serial.bytesize = serial.EIGHTBITS
        self.serial.stopbits = serial.STOPBITS_ONE
        self.serial.parity = serial.PARITY_NONE
        self.serial.xonxoff = False
        self.serial.timeout = 0
        self.serial.writeTimeout = 0
        self.serial.rtscts = False
        self.serial.dsrdtr = False

        # Get the ID of the relay for USB host PC plug/unplug if exists or filled
        if ((self.get_bench_params().has_parameter("UsbHostPcConnect")) and
        (self.__bench_params.get_param_value("UsbHostPcConnect") != "")):
            self.__usb_host_pc_connect_relay = \
            int(self.__bench_params.get_param_value("UsbHostPcConnect"))

        # Get the ID of the relay for USB host PC plug/unplug if exists or filled
        if ((self.get_bench_params().has_parameter("SwitchOnOff")) and
        (self.__bench_params.get_param_value("SwitchOnOff") != "")):
            self.__switch_on_off_relay = \
            int(self.__bench_params.get_param_value("SwitchOnOff"))

        # Get the ID of the relay for HDMI plug/unplug if exists or filled
        if ((self.get_bench_params().has_parameter("HdmiConnect")) and
        (self.__bench_params.get_param_value("HdmiConnect") != "")):
            self.__hdmi_connect_relay = \
            int(self.__bench_params.get_param_value("HdmiConnect"))

        # Get the ID of the relay for Ext Supply plug/unplug if exists or filled
        if ((self.get_bench_params().has_parameter("ExtSupplyConnect")) and
        (self.__bench_params.get_param_value("ExtSupplyConnect") != "")):
            self.__ext_supply_connect_relay = \
            int(self.__bench_params.get_param_value("ExtSupplyConnect"))

        return

    def open_serial(self):
        if(self.serial.isOpen() == False):
            try:
                self.get_logger().info('Try to open MDAC serial port');
                self.serial.open()
                time.sleep(0.3)
                return True;
            except serial.serialutil.SerialException:
                self.get_logger().info('Warning: Port not configured. Cannot open port.');
                self.get_logger().info('MDAC serial: %s' % self.serial, 'debug');
                return False;
        else:
            self.get_logger().info('MDAC serial port is already opened.');
            return True;

    def close_serial(self):
        if(self.serial.isOpen() == True):
            self.get_logger().info('Try to close MDAC serial port');
            self.serial.close()
            time.sleep(0.3)
        else:
            self.get_logger().info('MDAC serial port is already closed.');

    def usb_connector(self, plug):
        """
        Handles USB connector connection and disconnection
        @type plug: boolean
        @param plug: action to be done:
        - True  => plug currently selected USB device
        - False => unplug currently selected USB device
        """
        if self.__selected_usb_device == MDAC.USB_HOST_PC:
            # Call specific method to connect/disconnect the usb PC Host
            self.usb_host_pc_connector(plug)
        else:
            err_msg = \
            "USB MDAC device %s is not supported !" % self.__selected_usb_device
            self.get_logger().error(err_msg)
            #raise EqIOException(EqIOException.INVALID_PARAMETER, err_msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, err_msg)

        return

    def usb_device_selector(self, usb_device):

        self.get_logger().info("Select USB device: %s", usb_device)

        if usb_device == self.USB_HOST_PC:

            self.usb_host_pc_connector(True)
            self.__selected_usb_device = usb_device
        else:
            err_msg = "USB device %s is not supported !" % usb_device
            self.get_logger().error(err_msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, err_msg)

        return

    def reset(self):

        self.get_logger().info("Set Default State settings")
        return

    def remove_cable(self, cable_type):
        """
        disconnect given cable.
        This super function will call the right function depending the cable you want to remove.

        @type cable_type: str
        @param cable_type: cable supported by your EMT and device
        """
        if cable_type is not IIOCard.USB_HOST_PC:
            raise TestEquipmentException(
                TestEquipmentException.INVALID_PARAMETER,
                "Unsupported cable type : %s!" % cable_type)

        if cable_type in [IIOCard.USB_HOST_PC]:
            self.usb_connector(False)

        return

    def battery_connector(self, plug, battery_type="ANALOG"):

        if self.__power_supply_relay is not None:
            if plug:
                self.get_logger().info("Plug PowerSupply")
                self.ext_supply_connector(plug=True)
            else:
                self.get_logger().info("Unplug PowerSupply")
                self.ext_supply_connector(plug=False)
            return True
        else:
            # Power supply relay not present / configured; No action taken!
            msg = "Power supply MDAC not present / configured; No action taken!"
            self.get_logger().warning(msg)
            return False

        return

    def usb_host_pc_connector(self, plug):
        """
        Handles USB connector connection and disconnection of USB host PC device
        @type plug: boolean
        @param plug: action to be done true = plug, false = unplug
        """
        if (self.__usb_host_pc_connect_relay is not None):
            if plug:
                self.get_logger().info("Plug USB MDAC to host PC")
                self.open_serial()
                self.serial.write( "USB_%s ON\r\n" % self.__usb_host_pc_connect_relay )
                self.close_serial()
            else:
                self.get_logger().info("Unplug USB MDAC to host PC")
                self.open_serial()
                self.serial.write( "USB_%s OFF\r\n" % self.__usb_host_pc_connect_relay )
                self.close_serial()
        else:
            msg = "USB Host PC Connect MDAC not configured; No action taken!"
            self.get_logger().warning(msg)
            return False

        return True

    def press_relay(self, relay_number, duration):

        self.open_serial()
        self.serial.write( "GPO_3 ON\r\n" )
        time.sleep(0.3)
        self.serial.write( "LED3 ON\r\n" )
        time.sleep(0.1)
        self.close_serial()

        time.sleep(duration)

        self.open_serial()
        self.serial.write( "GPO_3 OFF\r\n" )
        time.sleep(0.3)
        self.serial.write( "LED3 OFF\r\n" )
        time.sleep(0.1)
        self.close_serial()

        return

    def press_power_button(self, duration):
        """
        Presses power button.
        Allow to simulate special behavior on the board like idle mode.
        :type duration: float
        :param duration: time while the power button is pressed
            The value should be superior than 0 seconds
        """
        if self.__switch_on_off_relay is not None:
            if duration:
                self.get_logger().info("Hold MDAC Power Button for %s seconds" % duration)
                self.press_relay(self.__switch_on_off_relay, duration)
            else:
                msg = "Power Button MDAC not configured; No action taken!"
                self.get_logger().warning(msg)
                return False
        else:
            msg = "Power Button MDAC not configured; No action taken!"
            self.get_logger().warning(msg)
            return False

        return True

    def press_relay_button(self, duration, relay_nb):
        """
        Presses relay during time duration like a button.
        :type duration: float
        :param relay_nb: relay number value to be pressed
        :param duration: time while the relay is pressed
            The value should be superior than 0 seconds
        """

        self.press_relay(relay_nb, duration)

        return True

    def set_default_wall_charger(self, device_default_charger):
        """
        set default wall charger.

        @type device_default_charger: str
        @param device_default_charger: default wall charger supported by the device

        @warning: function not used for usb rely
        """
        self.get_logger().info("setting default wall charger %s" % device_default_charger)
        self.__wall_charger = device_default_charger

        return

    def get_default_wall_charger(self):
        """
        get default wall charger.

        @rtype : str
        @return : default wall charger supported by the device
                return None if not set
        """
        return self.__wall_charger

    def load_specific_dut_config(self, dut_name):


        return

    def set_default_battery_type(self, batt_type):
        """
        set default battery type.
        all function that play with battery type will take this value if the type is omitted.

        @type batt_type: str
        @param batt_type: default battery type supported by the device
        """
        # put for compatibility reason
        pass

    def get_default_battery_type(self):
        """
        get default battery type.

        @rtype : str
        @return : default battery type
                return None if not set
        """
        # put for compatibility reason
        pass

    def release(self):
        """
        Release resources if any
        """
        self.get_logger().info("Release MDAC")
        pass

    def hdmi_connector(self, plug):
        """
        Handles HDMI Cable connection and disconnection
        :type plug: boolean
        :param plug: action to be done
            - True  => plug HDMI Cable
            - False => unplug HDMI Cable
        """

        if (self.__hdmi_connect_relay is not None):
            if plug:
                self.get_logger().info("Plug HDMI Cable MDAC")
                self.open_serial()
                self.serial.write( "Hdmi_%s DISCONNECT\r\n" % self.__hdmi_connect_relay )
                self.close_serial()
            else:
                self.get_logger().info("Unplug HDMI Cable MDAC")
                self.open_serial()
                self.serial.write( "Hdmi_%s CONNECT\r\n" % self.__hdmi_connect_relay )
                self.close_serial()
        else:
            msg = "HDMI Connect MDAC not configured; No action taken!"
            self.get_logger().warning(msg)
            return False

        return True

    def ext_supply_connector(self, plug):
        """
        Handles external power supply connection and disconnection
        :type plug: boolean
        :param plug: action to be done
            - True  => plug external power supply
            - False => unplug external power supply
        """

        if (self.__ext_supply_connect_relay is not None):
            if plug:
                self.get_logger().info("Plug Ext Supply MDAC")
                self.open_serial()
                self.serial.write( "VP_SEL_%s CONNECT\r\n" % self.__ext_supply_connect_relay )
                self.close_serial()
            else:
                self.get_logger().info("Unplug Ext Supply MDAC")
                self.open_serial()
                self.serial.write( "VP_SEL_%s DISCONNECT\r\n" % self.__ext_supply_connect_relay )
                self.close_serial()
        else:
            msg = "Ext Supply Connect MDAC not configured; No action taken!"
            self.get_logger().warning(msg)
            return False

        return True

