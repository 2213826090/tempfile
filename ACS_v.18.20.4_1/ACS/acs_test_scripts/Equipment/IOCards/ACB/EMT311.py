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
:summary: ACB implementation for EMT311 equipment
:author: vgomberx
:since: 04/08/2012(August)
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Equipment.IOCards.ACB.Common import WUsbDio as W
from acs_test_scripts.Equipment.IOCards.ACB.ACBN import ACBN
import UtilitiesFWK.Utilities as Util
import time

# pylint: disable=E1101
# pylint: disable=E1103


class EMT311(ACBN):

    """
    Class EMT311: implementation of enhanced ACB for tablet and phone
    """

    # Defining possible usb devices (Usb host, DCP, ...)
    SUPPORTED_DEVICE_TYPE = [ACBN.DCP, ACBN.CDP, ACBN.USB_HOST_PC,
                             ACBN.SDP, ACBN.AC_CHGR, ACBN.WALL_CHARGER, ACBN.OTG]

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
                      'prog_res_cs',  # ctrl23 on=close connection off=open connection
                      'prog_res_clk',  # ctrl24 on=close connection off=open connection
                      'prog_res_sdi',  # ctrl25 on=close connection off=open connection
                      'vusb_force_sens',  # ctrl26 on=close connection  off=open connection
                      'vusb_force',  # ctrl27
                      'vbatt_force_sens',  # ctrl28
                      'OTG_microAB_1',  # ctrl29
                      'OTG_microAB_2'  # ctrl30
                      )

    # describe here element that connect ACB to DUT
    ACB_TO_DUT = {
        ACBN.USB_ON: [
            (False, LINES.usb_5v_gnd),
            (True, LINES.usb_dp_dm_id, 3)],
        ACBN.USB_OFF: [
            (False, LINES.usb_dp_dm_id),
            (True, LINES.usb_5v_gnd, 1)]
        }

    # describe here element that connect ACCESSORY to ACB
    ACCESSORY_TO_ACB = {
        ACBN.SDP: [  # SDP
            # Disconnect exp power supply
            # first element is if this relay is ON, second element the relay name
            (True, LINES.usb_supply_or_ext),  # ctrl13
            # Charger
            (True, LINES.usb_switch_stage2),  # ctrl18
            (True, LINES.usb_switch_stage1),  # ctrl19
            # ID direct path
            (False, LINES.OTG_microAB_1)],  # ctrl29
        ACBN.DCP: [  # DCP
            # Disconnect exp power supply
            (True, LINES.usb_supply_or_ext),  # ctrl13
            # Charger
            (False, LINES.usb_switch_stage2),  # ctrl18
            (True, LINES.usb_switch_stage1),  # ctrl19
            # ID direct path
            (False, LINES.OTG_microAB_1)],  # ctrl29
        ACBN.CDP: [  # CDP
            # Disconnect exp power supply
            (True, LINES.usb_supply_or_ext),  # ctrl13
            # Charger
            (True, LINES.usb_switch_stage2),  # ctrl18
            (False, LINES.usb_switch_stage1),  # ctrl19
            # ID direct path
            (False, LINES.OTG_microAB_1)],  # ctrl29
        ACBN.OTG: [  # OTG
            (True, LINES.usb_supply_or_ext),  # ctrl13
            # Charger
            (False, LINES.usb_switch_stage2),  # ctrl18
            (False, LINES.usb_switch_stage1),  # ctrl19
            # ID direct path
            (False, LINES.OTG_microAB_1)],  # ctrl29
        ACBN.EXTERNAL_PS_ON: [
            # connect power supply
            (True, LINES.vusb_force_sens, 0.3),  # ctrl26
            (True, LINES.vusb_force, 0.3),  # ctrl27
            (False, LINES.vusb_force_sens),  # ctrl26
            (False, LINES.usb_supply_or_ext),  # ctrl13
            (False, LINES.usb_5v_gnd)],  # ctrl01
        ACBN.EXTERNAL_PS_OFF: [
            (True, LINES.vusb_force_sens, 0.3),  # ctrl26
            (False, LINES.vusb_force),  # ctrl27
            # disconnect exp power supply
            (True, LINES.usb_supply_or_ext),  # ctrl13
            # disconnect USB
            (True, LINES.usb_5v_gnd)],  # ctrl01
        ACBN.AC_CHGR_ON: [
            (True, LINES.vbatt_force_sens, 0.3),  # ctrl28
            (True, LINES.battery, 0.3),  # ctrl00
            (False, LINES.vbatt_force_sens)],  # ctrl28
        ACBN.AC_CHGR_OFF: [
            (True, LINES.vbatt_force_sens, 0.3),  # ctrl28
            (False, LINES.battery)]  # ctrl00
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
        self.get_logger().warning("battery_connector:not functional on EMT311")

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
        if plug:
            self.get_logger().info("Plug AC CHGR")
            if not self._acb_com_cmd_list(self.ACCESSORY_TO_ACB[self.AC_CHGR_ON]):
                self._logger.debug("ac_charger_connector: already plug")
        else:
            self.get_logger().info("Unplug AC CHGR")
            if self._acb_com_cmd_list(self.ACCESSORY_TO_ACB[self.AC_CHGR_OFF]):
                if self._use_ext_ps:
                    self.ext_supply_connector(False)
            else:
                self._logger.debug("ac_charger_connector: already unplug")

    def reset(self):
        """
        Reset the IO card to default states
        """
        ACBN.reset(self)
        self.ac_charger_connector(False)

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
        self.get_logger().warning("set_battery_type: not functional on EMT311")

    def usb_connector_data(self, plug):
        """
        Handles USB connector connection and disconnection
        :type plug: boolean
        :param plug: action to be done:
            - True  => plug data cable
            - False => unplug data cable
        """
        if plug:
            self.get_logger().info("Plug Datas USB")
            W.Enable(self, self.LINES.usb_dp_dm_id)
            # Waiting for enumeration
            time.sleep(3)
        else:
            self.get_logger().info("Unplug datas USB")
            W.Disable(self, self.LINES.usb_dp_dm_id)
            time.sleep(1)
