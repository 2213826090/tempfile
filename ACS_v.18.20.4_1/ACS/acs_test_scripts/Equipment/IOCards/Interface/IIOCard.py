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
:summary: virtual interface for input/output cards like Ariane Control Board
or relay cards
:author: ssavrimoutou
:since: 15/03/2011
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException

# pylint: disable=W0613


class IIOCard(object):

    """
    Class IIOCard: virtual interface for input/output cards
    """

    # Defining possible existing cable type (Usb host, DCP, ...)
    # and what is supported and is not supported.
    USB_HOST_PC = "USB_HOST_PC"
    SDP = "SDP"
    DCP = "DCP"
    CDP = "CDP"
    OTG = "OTG"
    ACA = "ACA"
    AC_CHGR = "AC_CHGR"
    WALL_CHARGER = "WALL_CHARGER"
    WIRELESS_CHARGER = "WIRELESS_CHARGER"
    SUPPORTED_DEVICE_TYPE = []
    # describe constant button name to be used outside
    VOLUME_UP = "VOLUME_UP"
    VOLUME_DOWN = "VOLUME_DOWN"
    PWR_BUTTON = "POWER_BUTTON"
    CAMERA_BUTTON = "CAMERA_BUTTON"
    HOME_BUTTON = "HOME_BUTTON"
    BUTTON_LIST = [HOME_BUTTON, CAMERA_BUTTON, PWR_BUTTON, VOLUME_UP, VOLUME_DOWN]
    # describe here for each button, the associated relay
    PHONE_BUTTON = {}
    # different constant used especially on ACB board
    EXTERNAL_PS_ON = "EXTERNAL_PS_ON"
    EXTERNAL_PS_OFF = "EXTERNAL_PS_OFF"
    USB_ON = "USB_ON"
    USB_OFF = "USB_OFF"
    AC_CHGR_ON = "AC_CHGR_ON"
    AC_CHGR_OFF = "AC_CHGR_OFF"
    # battery constant
    BAT_ANALOG = "ANALOG"
    BAT_DIGITAL = "DIGITAL"
    BAT_INVALID = "INVALID"
    BAT_EMULATOR = "BATTERY_EMULATOR"
    BAT_REMOVED = "REMOVED"
    BAT_DIG_INVALID = "DIG_INVALID"

    def init(self):
        """
        Initializes the equipment. The equipment is ready to use
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def release(self):
        """
        Releases all resources allocated to equipment
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def usb_connector(self, plug):
        """
        Handles USB connector connection and disconnection
        :type plug: boolean
        :param plug: action to be done:
            - True  => plug currently selected USB device
            - False => unplug currently selected USB device
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def usb_host_pc_connector(self, plug):
        """
        Handles USB connector connection and disconnection of USB host PC device
        :type plug: boolean
        :param plug: action to be done:
            - True  => plug USB to host PC
            - False => unplug USB from host PC
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def remove_cable(self, cable_type):
        """
        disconnect given cable.
        This super function will call the right function depending the cable you want to remove.

        :type cable_type: str
        :param cable_type: cable supported by your EMT and device
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def battery_connector(self, plug, battery_type="ANALOG"):
        """
        Handles battery insertion / removal
        :type plug: boolean
        :param plug: action to be done
            - True  => insert battery
            - False => remove battery
        :type battery_type: str
        :param battery_type: battery to plug
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def sim_card_connector(self, plug):
        """
        Handles SIM card insertion / removal
        :type plug: boolean
        :param plug: action to be done
            - True  => insert SIM card
            - False => remove SIM card
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def ext_supply_connector(self, plug):
        """
        Handles external power supply connection and disconnection
        :type plug: boolean
        :param plug: action to be done
            - True  => plug external power supply
            - False => unplug external power supply
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_battery_type(self, batt_type):
        """
        Sets Battery type.
        :type batt_type: str
        :param batt_type: batt id to select. Possible values:
            - "ANALOG"
            - "INVALID"
            - "TEST1"
            - "TEST2"
            - "DIGITAL"
            - "REMOVED"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_battery_temperature(self, temperature):
        """
        Sets battery temperature
        :type temperature: integer
        :param temperature: temperature in C to set, set of possible values:
        {95; 70; 50; 25; 10; 5; 0; -15}
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_usb_otg_type(self, otg_type):
        """
        Sets the OTG type
        :type otg_type: str
        :param otg_type: OTG type to select. Possible values:
            - "NORMAL"
            - "DUT_DEVICE"
            - "DUT_HOST"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def press_power_button(self, duration):
        """
        Presses power button.
        Allow to simulate special behavior on the board like S3 mode.
        :type duration: float
        :param duration: time while the power button is pressed
            The value should be superior to 0 seconds
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def push_power_button(self):
        """
        Presses power button.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def release_power_button(self):
        """
        Release power button.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def press_relay_button(self, duration, relay_nb):
        """
        Presses relay during time duration like a button.
        :type duration: float
        :param relay_nb: relay number value to be pressed
        :param duration: time while the relay is pressed
            The value should be superior than 0 seconds
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def reset(self):
        """
        Reset the IO card to default states
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def ac_charger_connector(self, plug):
        """
        handle AC charger insertion.
        :type plug: boolean
        :param plug: action to be done:
            - True  => insert charger
            - False => remove charger
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def wall_charger_connector(self, plug):
        """
        handle wall charger insertion depending of available one on io card
        and supported one by device.
        :type plug: boolean
        :param plug: action to be done:
            - True  => insert charger
            - False => remove charger
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_default_wall_charger(self, device_default_charger):
        """
        set default wall charger.

        :type device_default_charger: str
        :param device_default_charger: default wall charger supported by the device
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_default_wall_charger(self):
        """
        get default wall charger.

        :rtype: str
        :return: default wall charger supported by the device
                  return None if not set
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_default_battery_type(self, batt_type):
        """
        set default battery type.
        all function that play with battery type will take this value if the type is omitted.

        :type batt_type: str
        :param batt_type: default battery type supported by the device
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_default_batt_id_value(self, batt_id_value):
        """
        set the default batt_id value.
        all function that play with battid  will take this value if the type is omitted.

        :type batt_id_value: int
        :param batt_id_value: default battery id value supported by the device
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_default_bptherm_value(self, bptherm_value):
        """
        set the default bptherm value.
        all function that play with bptherm will take this value if the type is omitted.

        :type bptherm_value: int
        :param bptherm_value: default bptherm value supported by the device
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_default_battery_type(self):
        """
        get default battery type.

        :rtype: str
        :return: default battery type
                  return None if not set
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_default_batt_id_value(self):
        """
        get default batt id value.

        :rtype: str
        :return: get the default batt id value
                  return None if not set
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_default_bptherm_value(self):
        """
        get default bptherm  value.

        :rtype: str
        :return: get the default bptherm value
                  return None if not set
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def usb_connector_data(self, plug):
        """
        Handles USB connector connection and disconnection
        :type plug: boolean
        :param plug: action to be done:
            - True  => plug data cable
            - False => unplug data cable
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_provisioning_mode(self, enable):
        """
        activate provisioning mode on the device by enabling specific
        button (volume up/down, home, ...)

        :type enable: boolean
        :param enable: action to be done:
            - True  => enable provisioning mode
            - False => disable provisioning mode

        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def press_key_combo(self, button_list, press_duration, push_delay, release_delay):
        """
        push on the following button

        :type button_list: list of str
        :param button_list: list of button to press on. defined by PHONE_BUTTON key

        :type press_duration: int
        :param press_duration: time to keep button on.

        :type push_delay: float
        :param push_delay: delay between button push

        :type release_delay: float
        :param release_delay: delay between button release
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def load_specific_dut_config(self, dut_name):
        """
        Configure different setting on your io card related to dut name,
        This is useful in multi device campaign.
        The setting will depending of your current dut name and what you declared on
        benchconfig.

        :type dut_name: str
        :param dut_name: phone name
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)
