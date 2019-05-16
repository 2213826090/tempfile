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
:summary: ACB implementation for EMT340 equipment
         it support battery and charger(SDP, CDP, DCP, AC) emulation, tablet and otg
:author: dbatutx
:since: 11/12/2012
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Equipment.IOCards.ACB.Common import WUsbDio as W
from acs_test_scripts.Equipment.IOCards.ACB.Common import ProgResistance as ProgRes
from acs_test_scripts.Equipment.IOCards.ACB.ACBN import ACBN
import UtilitiesFWK.Utilities as Util
import time
import math

# pylint: disable=E1101
# pylint: disable=E1103


class EMT340(ACBN):

    """
    Class EMT340: implementation of enhanced ACB for tablet and phone
    """

    # Defining possible usb devices (Usb host, DCP, ...)
    SUPPORTED_DEVICE_TYPE = [ACBN.DCP, ACBN.CDP, ACBN.USB_HOST_PC, ACBN.OTG,
                             ACBN.SDP, ACBN.AC_CHGR, ACBN.WALL_CHARGER]

    LINES = Util.enum('battery',  # ctrl00 on=inserted / off=removed
                      'usb_5v_gnd',  # ctrl01
                      'usb_dp_dm_id',  # ctrl02
                      'batt_id_1',  # ctrl03
                      'batt_id_2',  # ctrl04
                      'batt_id_3',  # ctrl05
                      'temp_value_ctrl6',  # ctrl06
                      'bptherm_1m',  # ctrl07
                      'prog_res_rdy',  # ctrl08 on=CP / off=DP
                      'batt_id_glitch',  # ctrl09 on=start the glitch
                      #       off= rearm
                      'batt_id_glitch_duration',  # ctrl10 on=5p2ms / off=165us
                      'prog_res_sdo',  # ctrl11 on=insertion
                      #       off=removal
                      'button_ctrl12',  # ctrl12 on=close connection
                      #       off=open connection
                      'usb_supply_or_ext',  # ctrl13
                      'digital_battery_protocol',  # ctrl14 on=logical 0 / off=logical 1
                      'AC_supply1_switch',  # ctrl15
                      'bptherm_connect',  # ctrl16
                      'AC_supply2_switch',  # ctrl17 on=insertion / off=removal
                      'usb_switch_stage2',  # ctrl18
                      'usb_switch_stage1',  # ctrl19
                      'button_ctrl20',  # ctrl20 on=close connection
                      #       off=open connection
                      'button_ctrl21',  # ctrl21 on=close connection
                      #       off=open connection
                      'button_ctrl22',  # ctrl22 on=close connection
                      #       off=open connection
                      'prog_res_cs',  # ctrl23 on=close connection
                      #       off=open connection
                      'prog_res_clk',  # ctrl24 on=close connection
                      #       off=open connection
                      'prog_res_sdi',  # ctrl25 on=close connection
                      #       off=open connection
                      'vusb_force_sens',  # ctrl26 on=close connection
                      #       off=open connection
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
            (True, LINES.usb_switch_stage2),  # ctrl18
            (False, LINES.usb_switch_stage1),  # ctrl19
            # ID direct path
            (False, LINES.OTG_microAB_1)],  # ctrl29
        ACBN.CDP: [  # CDP
            # Disconnect exp power supply
            (True, LINES.usb_supply_or_ext),  # ctrl13
            # Charger
            (False, LINES.usb_switch_stage2),  # ctrl18
            (True, LINES.usb_switch_stage1),  # ctrl19
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
            (True, LINES.AC_supply2_switch, 0.3),  # ctrl17
            (True, LINES.AC_supply1_switch, 0.3),  # ctrl15
            (False, LINES.AC_supply2_switch)],  # ctrl17]
        ACBN.AC_CHGR_OFF: [
            (True, LINES.AC_supply2_switch, 0.3),  # ctrl17
            (False, LINES.AC_supply1_switch)]  # ctrl15
        }

    # define supported battery type
    BATTERY_TYPE = {
        ACBN.BAT_ANALOG: [
            (False, LINES.batt_id_1),  # ctrl03
            (False, LINES.batt_id_2),  # ctrl04
            (False, LINES.batt_id_3)],  # ctrl05
        ACBN.BAT_DIGITAL: [
            (True, LINES.button_ctrl20),  # ctrl20
            (True, LINES.button_ctrl21),  # ctrl21
            (False, LINES.batt_id_1),  # ctrl03
            (False, LINES.batt_id_2),  # ctrl04
            (True, LINES.batt_id_3)],  # ctrl05
        ACBN.BAT_INVALID: [
            (False, LINES.button_ctrl20),  # ctrl20
            (True, LINES.button_ctrl21),  # ctrl21
            (False, LINES.batt_id_1),  # ctrl03
            (False, LINES.batt_id_2),  # ctrl04
            (True, LINES.batt_id_3)],  # ctrl05
        ACBN.BAT_EMULATOR: [
            (True, LINES.batt_id_1),  # ctrl03
            (True, LINES.batt_id_2),  # ctrl04
            (False, LINES.batt_id_3)],  # ctrl05
        ACBN.BAT_REMOVED: [
            (True, LINES.button_ctrl20),  # ctrl20
            (False, LINES.button_ctrl21),  # ctrl21
            (True, LINES.batt_id_1),  # ctrl03
            (False, LINES.batt_id_2),  # ctrl04
            (True, LINES.batt_id_3)]  # ctrl05
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
        self.__wall_charger = None
        self._batt_id_dict = {self.BAT_ANALOG: 115000,
                              self.BAT_INVALID: 300000,
                              self.BAT_EMULATOR: 7000}
        self._factor_a = float(170897)
        self._factor_b = -0.049

    def init(self):
        """
        Initializes the equipment. The equipment is ready to use.
            - connect sense & force of the alim in order to avoid
            overprotection
        """
        ACBN.init(self)
        self.reset()

    def set_battid_value(self, battery_type, value):
        """
        Constructor
        :type name: str
        :param name: the bench configuration name of the equipment
        :type model: str
        :param model: the model of the equipment
        """
        self.get_logger().info("Set battid value %s on battery type type: %s" %
                               (battery_type, str(value)))
        # check battery type
        if battery_type in self.BATTERY_TYPE.keys():
            self._batt_id_dict[battery_type] = value
        else:
            raise TestEquipmentException(
                TestEquipmentException.INVALID_PARAMETER,
                "Unsupported battery type : %s!" % battery_type)

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
        self.get_logger().info("Set bptherm expression as R(t)=%s*exp(%s*t)" %
                               (factor_a, str(factor_b)))
        # check battery type
        if not isinstance(factor_a, float) or not isinstance(factor_b, float):
            raise TestEquipmentException(
                TestEquipmentException.INVALID_PARAMETER,
                "the bp therm factor must be float!")
        else:
            self._factor_a = factor_a
            self._factor_b = factor_b

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
            W.Enable(self, self.LINES.vbatt_force_sens)  # ctrl28
            time.sleep(0.5)
            W.Enable(self, self.LINES.battery)  # ctrl00
            time.sleep(0.5)
            W.Disable(self, self.LINES.vbatt_force_sens)  # ctrl28
            # insert batt id action
            self.set_battery_type(battery_type)

        else:
            self.get_logger().info("Battery removal")
            # remove batt id action
            self.set_battery_type(self.BAT_REMOVED)
            W.Enable(self, self.LINES.vbatt_force_sens)  # ctrl28
            time.sleep(0.5)
            W.Disable(self, self.LINES.battery)  # ctrl00

    def reset(self):
        """
        Reset the IO card to default states
        """
        ACBN.reset(self)
        self.ac_charger_connector(False)
        self.glitch(False, "short")
        W.Disable(self, self.LINES.prog_res_rdy)  # ctrl08
        W.Enable(self, self.LINES.prog_res_cs)  # ctrl23
        W.Disable(self, self.LINES.prog_res_clk)  # ctrl24
        W.Disable(self, self.LINES.prog_res_sdi)  # ctrl25

    def usb_connector(self, plug):
        """
        Handles USB connector connection and disconnection
        :type plug: boolean
        :param plug: action to be done:
            - True  => plug currently selected USB device
            - False => unplug currently selected USB device

        .. warning:: on EMT340 ext power supply is always connected
        """
        if plug:
            #self.ext_supply_connector(True)
            self.get_logger().info("Plug USB")
            if not self._acb_com_cmd_list(self.ACB_TO_DUT[self.USB_ON], True):
                self._logger.debug("usb_connector: USB already plug")
        else:
            #self.ext_supply_connector(False)
            self.get_logger().info("Unplug USB")
            if not self._acb_com_cmd_list(self.ACB_TO_DUT[self.USB_OFF], True):
                self._logger.debug("usb_connector: USB already unplug")

    def set_battery_temperature(self, temperature):
        """
        Sets battery temperature
        :type temperature: integer
        :param temperature: temperature in C to set, possible values:
        {95; 70; 50; 25; 10; 5; 0; -15}
        """
        temperature = int(temperature)
        self.get_logger().info("Set battery temperature: %d C", temperature)

        resistor = int(self._factor_a * math.exp(self._factor_b * temperature))

        if 0 < resistor < 400000:
            ProgRes.select_resistance(resistor, "bptherm", self)
        else:
            raise TestEquipmentException(
                TestEquipmentException.INVALID_PARAMETER,
                "Unsupported battery temperature: %d/" % temperature +
                "resistor:%s" % str(resistor))

    def glitch(self, start, behavior):
        """
        Start or stop a short or a long Glitch
        :type start: boolean
        :param start: 'True' for starting or 'False' for stopping a glitch
        :type behavior: str
        :param behavior: can be "short" or "long
        """
        if start:
            self.get_logger().info("Start a %s Glitch", behavior)
            if behavior == "short":
                # long glitch
                W.Disable(self, self.LINES.batt_id_glitch)  # ctrl09
                W.Enable(self, self.LINES.batt_id_glitch_duration)  # ctrl10
                W.Disable(self, self.LINES.digital_battery_protocol)  # ctrl14
            elif behavior == "long":
                # short glitch
                W.Disable(self, self.LINES.batt_id_glitch)  # ctrl09
                W.Disable(self, self.LINES.batt_id_glitch_duration)  # ctrl10
                W.Disable(self, self.LINES.digital_battery_protocol)  # ctrl14
            else:
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                    "Unsupported behavior: %d!" % behavior)
        else:
            self.get_logger().info("Stop a %s Glitch", behavior)
            if behavior == "short":
                # long glitch
                W.Enable(self, self.LINES.batt_id_glitch)  # ctrl09
                W.Enable(self, self.LINES.batt_id_glitch_duration)  # ctrl10
                W.Disable(self, self.LINES.digital_battery_protocol)  # ctrl14
            elif behavior == "long":
                # short glitch
                W.Enable(self, self.LINES.batt_id_glitch)  # ctrl09
                W.Disable(self, self.LINES.batt_id_glitch_duration)  # ctrl10
                W.Disable(self, self.LINES.digital_battery_protocol)  # ctrl14
            else:
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                    "Unsupported behavior: %d!" % behavior)

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
