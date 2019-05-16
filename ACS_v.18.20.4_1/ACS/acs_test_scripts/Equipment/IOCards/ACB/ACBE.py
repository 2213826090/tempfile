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
:summary: ACBE implementation of digital IO
:author: ymorel, apairex
:since: 15/03/2011
"""
from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Equipment.IOCards.ACB.Common import WUsbDio as W
from acs_test_scripts.Equipment.IOCards.ACB.ACBN import ACBN
import UtilitiesFWK.Utilities as Util
import time

# pylint: disable=E1101
# pylint: disable=E1103


class ACBE(ACBN):

    """
    Class ACBE: implementation of enhanced ACB
    """

    # Defining possible usb devices (Usb host, DCP, ...)
    SUPPORTED_DEVICE_TYPE = [ACBN.USB_HOST_PC, ACBN.WALL_CHARGER,
                             ACBN.SDP, ACBN.DCP, ACBN.CDP, ACBN.OTG]

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

    ACCESSORY_TO_ACB = {
        ACBN.SDP: [  # SDP
            (True, LINES.usb_charger),  # ctrl08
            (True, LINES.usb_charger_select),  # ctrl13
            (True, LINES.usb_switch_select),  # ctrl18
            (False, LINES.usb_switch_select2),  # ctrl19
            (False, LINES.button_ctrl20)],  # ctrl20 open ID pin
        ACBN.DCP: [  # DCP
            (False, LINES.usb_charger),  # ctrl08
            (False, LINES.usb_charger_select),  # ctrl13
            (False, LINES.usb_switch_select),  # ctrl18
            (False, LINES.usb_switch_select2),  # ctrl19
            (False, LINES.button_ctrl20)],  # ctrl20 open ID pin
        ACBN.CDP: [  # CDP
            (True, LINES.usb_charger),  # ctrl08
            (True, LINES.usb_charger_select),  # ctrl13
            (True, LINES.usb_switch_select),  # ctrl18
            (True, LINES.usb_switch_select2),  # ctrl19
            (True, LINES.button_ctrl20),  # ctrl20 ground ID pin
            # ID in normal mode
            (False, LINES.OTG_ctrl28),  # ctrl28
            (True, LINES.OTG_ctrl29),  # ctrl29
            (False, LINES.OTG_ctrl30)],  # ctrl30
        ACBN.OTG: [  # OTG
            (False, LINES.usb_charger),  # ctrl08
            (True, LINES.usb_switch_select),  # ctrl18
            (True, LINES.usb_switch_select2),  # ctrl19
            (False, LINES.OTG_ctrl28),  # ctrl28
            (True, LINES.OTG_ctrl29),  # ctrl29
            (False, LINES.OTG_ctrl30)],  # ctrl30
        ACBN.EXTERNAL_PS_ON: [
            (True, LINES.usb_charger_select)],  # ctrl01
        ACBN.EXTERNAL_PS_OFF: [
            (False, LINES.usb_charger_select)]  # ctrl01
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
        ACBN.__init__(self, name, model, eqt_params, bench_params)

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
            # Insert battery
            W.Disable(self, self.LINES.power_supply1_ac_charger)  # ctrl27
            time.sleep(0.3)
            W.Enable(self, self.LINES.battery)  # ctrl00
            W.Enable(self, self.LINES.supply1_switch)  # ctrl15
            time.sleep(0.3)
            W.Enable(self, self.LINES.power_supply1_ac_charger)  # ctrl27
            W.Disable(self, self.LINES.button_ctrl12)  # ctrl12
            # work around for bench batt id bug
            self.set_battery_type(battery_type)
        else:
            self.get_logger().info("Battery removal")
            # remove battery
            W.Disable(self, self.LINES.power_supply1_ac_charger)  # ctrl27
            time.sleep(0.3)
            W.Disable(self, self.LINES.battery)  # ctrl00
            W.Enable(self, self.LINES.supply1_switch)  # ctrl15
            # work around for bench batt id bug
            self.set_battery_type(self.BAT_INVALID)

    def set_usb_otg_type(self, otg_type):
        """
        Sets the OTG type
        :type otg_type: str
        :param otg_type: OTG type to select. Possible values:
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
            W.Disable(self, self.LINES.OTG_ctrl30)  # ctrl30

        elif otg_type == "DUT_HOST":
            W.Enable(self, self.LINES.OTG_ctrl28)  # ctrl28
            W.Disable(self, self.LINES.OTG_ctrl29)  # ctrl29
            W.Enable(self, self.LINES.OTG_ctrl30)  # ctrl30

        else:
            raise TestEquipmentException(
                TestEquipmentException.INVALID_PARAMETER,
                "Unknown OTG type : %s !" % otg_type)

    def ac_charger_connector(self, plug):
        """
        handle AC charger insertion.
        :type plug: boolean
        :param plug: action to be done:
            - True  => insert charger
            - False => remove charger
        """
        self.get_logger().warning("AC charger connector NOT implemented for ACBE")
