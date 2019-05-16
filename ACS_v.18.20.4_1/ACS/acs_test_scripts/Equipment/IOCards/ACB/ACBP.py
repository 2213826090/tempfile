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
:summary: ACBP implementation of digital IO
:author: ymorel
:since: 16/03/2011
"""

import time
import UtilitiesFWK.Utilities as Util
from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Equipment.IEquipment import DllLoader
from acs_test_scripts.Equipment.IOCards.ACB.Common import WUsbDio as W
from acs_test_scripts.Equipment.IOCards.Interface.IIOCard import IIOCard

# pylint: disable=E1101


class ACBP(IIOCard, DllLoader):

    """
    Class ACBP: implementation of UsbDIO prototype
    """
    # Defining possible usb devices (Usb host, DCP, ...)
    SUPPORTED_DEVICE_TYPE = [IIOCard.USB_HOST_PC, IIOCard.SDP]

    LINES = Util.enum(
        'battery',  # ctrl00 on=inserted / off=removed
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
        'button_ctrl20',  # ctrl20 on=close connection
        #       off=open connection
        'button_ctrl21',  # ctrl21 on=close connection
        #       off=open connection
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
        'power_supply1_ac_charger'  # ctrl27 depends on 'supply1_switch'
    )

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

        serial_number = None
        if self.get_bench_params().has_parameter("serialNumber"):
            serial_number = \
                int(self.get_bench_params().get_param_value("serialNumber"))

        # Tries to connect to equipment
        self.__device_index = W.Connect(self, serial_number)
        if self.__device_index == -1:
            raise TestEquipmentException(TestEquipmentException.CONNECTION_ERROR,
                                "Failed to connect to %s" % self.get_name())
        W.ShowInfo(self)
        W.Reset(self)

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
        self._logger.warning("reset method is not implemented !")

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
            W.Enable(self, self.LINES.usb_5v_gnd)
        else:
            self.get_logger().info("Unplug USB")
            W.Disable(self, self.LINES.usb_dp_dm_id)

        time.sleep(0.2)

        if plug:
            W.Enable(self, self.LINES.usb_dp_dm_id)
            # Waiting for enumeration
            time.sleep(3)
        else:
            W.Disable(self, self.LINES.usb_5v_gnd)

    def usb_device_selector(self, usb_device):
        """
        Handles USB device selection
        :param usb_device: USB device to select:
            - "USB_HOST_PC" -> USB Host PC (ACS, FW/SW Updates)
        """
        self.get_logger().info("Select USB device: %s", usb_device)

        if usb_device == ACBP.USB_HOST_PC:
            W.Enable(self, self.LINES.usb_switch_select)
            W.Disable(self, self.LINES.usb_switch_select2)
        else:
            raise TestEquipmentException(
                TestEquipmentException.INVALID_PARAMETER,
                "Unknown USB device: %s!" % usb_device)

    def simulate_insertion(self, device_type):
        """
        Do a cable insertion (usb or other)
        If this function is called twice then it will unplug then plug again instead of doing nothing.
        :type device_type: str
        :param device_type: cable device to select. Possible values:
            - "USB_HOST_PC": USB Host PC (ACS, FW/SW Updates)
        """
        self.get_logger().info("Select cable type: %s", device_type)

        if device_type == self.USB_HOST_PC:
            # remove usb
            self.usb_connector(False)
            # switch to wanted usb
            self.usb_device_selector(device_type)
            # plug usb
            self.usb_connector(True)
        else:
            raise TestEquipmentException(
                TestEquipmentException.INVALID_PARAMETER,
                "Unknown USB device: %s!" % device_type)

    def usb_host_pc_connector(self, plug):
        """
        Handles USB connector connection and disconnection of USB host PC device
        :type plug: boolean
        :param plug: action to be done
            - True  => plug USB to host PC
            - False => unplug USB from host PC

        :rtype:None
        """
        # Select USB host PC device
        self.usb_device_selector(ACBP.USB_HOST_PC)
        # Plug or unplug USB
        self.usb_connector(plug)
        return True

    def press_power_button(self, duration):
        """
        Presses power button.
        Allow to simulate special behavior on the board like S3 mode.
        :type duration: float
        :param duration: time while the power button is pressed
            The value should be superior than 0 seconds
        """
        self.get_logger().info("Press power button during %f second(s)",
                               duration)

        W.Enable(self, self.LINES.button_ctrl12)
        time.sleep(duration)
        W.Disable(self, self.LINES.button_ctrl12)

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
        self.get_logger().warning("load_specific_dut_config: NOT implemented for ACBP")

    def set_default_battery_type(self, batt_type):
        """
        set default battery type.
        all function that play with battery type will take this value if the type is omitted.

        :type batt_type: str
        :param batt_type: default battery type supported by the device
        """
        self.get_logger().warning("set_default_battery_type: NOT implemented for ACBP")

    def get_default_battery_type(self):
        """
        get default battery type.

        :rtype: str
        :return: default battery type
                  return None if not set
        """
        self.get_logger().warning("get_default_battery_type: NOT implemented for ACBP")

    def set_default_wall_charger(self, device_default_charger):
        """
        set default wall charger.

        :type device_default_charger: str
        :param device_default_charger: default wall charger supported by the device
        """
        self.get_logger().warning("set_default_wall_charger: NOT implemented for ACBP")

    def get_default_wall_charger(self):
        """
        get default wall charger.

        :rtype: str
        :return: default wall charger supported by the device
                  return None if not set
        """
        self.get_logger().warning("get_default_wall_charger: NOT implemented for ACBP")
