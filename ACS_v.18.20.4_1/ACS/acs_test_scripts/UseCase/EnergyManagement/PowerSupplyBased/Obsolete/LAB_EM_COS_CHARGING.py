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
:summary: Energy Management hardware charging Use case
:author: apairex
:since: 08/31/2011
"""
import time
from UtilitiesFWK.Utilities import str_to_bool
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase


class LabEmCosCharging(EmUsecaseBase):

    """
    Lab Energy Management class.
    """
    DEDICATED_BENCH = "POWER_SUPPLY_BENCH"

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LAB_EM_BASE Init function
        EmUsecaseBase.__init__(self, tc_name, global_config)

        self._scu_charging_hi_voltage = float(self._tc_parameters.get_param_value(
            "SCU_CHARGING_HI_VOLTAGE"))
        self._cos_charging_hi_voltage = float(self._tc_parameters.get_param_value(
            "COS_CHARGING_HI_VOLTAGE"))
        self._mos_charging_lo_voltage = float(self._tc_parameters.get_param_value(
            "MOS_CHARGING_LO_VOLTAGE"))
        self._ext_ps = str_to_bool(str(self._tc_parameters.get_param_value("EXT_PS")))

        # Read VBATT from TC parameters
        self._charger = self._tc_parameters.get_param_value("CHARGER_TYPE")

        # Call ConfigsParser to parse Energy_Management
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_COS_CHARGING", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

        # Redefine initial value for setting USBDIO:
        # - BatteryType = ANALOG
        self.em_core_module.io_card_init_state["BatteryType"] = self.phone_info["BATTERY"]["BATTID_TYPE"]
        # - Battery  = False (removed)
        self.em_core_module.io_card_init_state["Battery"] = True
        # - Platform = False (OFF)
        self.em_core_module.io_card_init_state["Platform"] = "OFF"
        # - USBChargerType = DCP
        self.em_core_module.io_card_init_state["USBChargerType"] = self._charger
        # - USBCharger = True (inserted)
        self.em_core_module.io_card_init_state["USBCharger"] = False
        # - BatteryTemperature = 25
        self.em_core_module.io_card_init_state["BatteryTemperature"] = 25

        # Set initial value for setting Power Supply VBATT:
        # - VoltageLevel = VBATT
        self.em_core_module.eqp_init_state["BATT"]["VoltageLevel"] = self._scu_charging_hi_voltage

        # VBATT and VUSB power supplies
        self.em_core_module.pwrs_vbatt = self._em.get_power_supply("BATT")
        if self.em_core_module.ps_properties["USB"] is not None:
            self.em_core_module.pwrs_vusb = self._em.get_power_supply("USB")

#------------------------------------------------------------------------------
    def run_test_body(self):
        """
        Execute the test
        """

        # Call LAB_EM_BASE Init function
        EmUsecaseBase.run_test_body(self)

        #
        # BEGIN SCU charging TEST
        #
        # Set VBatt to scu voltage
        self._logger.info("SCU TEST")
        self.em_core_module.pwrs_vbatt.set_current_voltage(self._scu_charging_hi_voltage,
                                             self.em_core_module.ps_properties["BATT"]["PortNumber"])
        time.sleep(15)
        # Power Supply Current control: Control that the board is switched off
        meas_ibatt = self.em_core_module.get_battery_current()
        self._meas_list.add("IBATT_OFF1", meas_ibatt)
        meas_iusb = self.em_core_module.get_charger_current(self._charger)
        self._meas_list.add("IUSB_OFF1", meas_iusb)
        # Charger Plug
        self.em_core_module.plug_charger(self._charger, ext_ps=self._ext_ps)
        time.sleep(70)
        # PS Current control: Control that the SCU is charging the battery
        meas_ibatt2 = self.em_core_module.get_battery_current()
        self._meas_list.add("IBATT_SCU", meas_ibatt2)
        meas_iusb2 = self.em_core_module.get_charger_current(self._charger)
        self._meas_list.add("IUSB_SCU", meas_iusb2)
        # Unplug all
        self.em_core_module.unplug_charger(self._charger)
        time.sleep(20)
        # END OF SCU Charging TEST

        #
        # BEGIN OF COS TEST
        #
        # Set VBatt to cos voltage
        self._logger.info("COS TEST")
        self.em_core_module.pwrs_vbatt.set_current_voltage(self._cos_charging_hi_voltage,
                                             self.em_core_module.ps_properties["BATT"]["PortNumber"])
        time.sleep(30)
        # Power Supply Current control: Control that the board is switched off
        meas_ibatt3 = self.em_core_module.get_battery_current()
        self._meas_list.add("IBATT_OFF2", meas_ibatt3)
        meas_iusb3 = self.em_core_module.get_charger_current(self._charger)
        self._meas_list.add("IUSB_OFF2", meas_iusb3)
        # Charger Plug
        self.em_core_module.plug_charger(self._charger, ext_ps=self._ext_ps)
        # Start the 120s timer
        origin = time.time()
        current = origin
        # time out should be 120 seconds
        try:
            timeout = float(self._em_targets["TIMER1"]["hi_lim"])
        except (ValueError, TypeError):
            timeout = 120
        try:
            criteria = float(self._em_targets["IBATT_COS"]["hi_lim"])
        except (ValueError, TypeError):
            criteria = -.4
        while origin + timeout > current:
            # Power Supply current control
            meas_ibatt4 = self.em_core_module.get_battery_current()
            meas_iusb4 = self.em_core_module.get_charger_current(self._charger)

            self._logger.info("COS starting up... Ibatt=%f" % (meas_ibatt4[0]))

            if meas_ibatt4[0] < criteria:
                break

            time.sleep(5)
            current = time.time()
        # PowerSupply current control and Timer control
        self._meas_list.add("IBATT_COS", meas_ibatt4)
        self._meas_list.add("IUSB_COS", meas_iusb4)
        self._meas_list.add("TIMER1", (current - origin, "s"))
        # END OF COS TEST

        #
        # BEGIN OF MOS TEST
        #
        time.sleep(20)
        self._logger.info("MOS TEST")
        self.em_core_module.pwrs_vbatt.set_current_voltage(self._mos_charging_lo_voltage,
                                             self.em_core_module.ps_properties["BATT"]["PortNumber"])
        # Unplug all
        self.em_core_module.unplug_charger(self._charger)
        # Board should switch off
        time.sleep(40)
        # init for loop test
        msg = "board has failed to boot"
        connection_state = ("LOST", "none")
        # Power button key press
        self._io_card.press_power_button(self.pwr_btn_boot)
        self.em_core_module.plug_charger(self._io_card.SDP)
        # Start the 120s timer
        origin = time.time()
        current = origin
        # time out should be 120 seconds
        try:
            timeout = float(self._em_targets["TIMER2"]["hi_lim"])
        except ValueError:
            timeout = 120
        # Check the board is booting
        timer2 = 0
        while origin + 2 * timeout > current:
            time.sleep(1)
            if self._device.get_state() == "alive":
                connection_state = ("ALIVE", "none")
                timer2 = time.time() - origin
                msg = "board has booted in %s seconds" % \
                    str(timer2)
                break
            current = time.time()
        self._logger.info(msg)
        self.em_core_module.unplug_charger(self._io_card.SDP)
        time.sleep(10)
        # Control that the board starts up
        meas_ibatt = (self.em_core_module.pwrs_vbatt.get_current_meas(
            self.em_core_module.ps_properties["BATT"]["PortNumber"], "DC"), "A")

        self._meas_list.add("IBATT_MOS", meas_ibatt)
        self._meas_list.add("TIMER2", (timer2, "s"))
        self._meas_list.add("ADB_CONNECTION", connection_state)
        # END OF MOS TEST

        # generate em verdict
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
        self._meas_list.clean()

        # Save data report in xml file
        self._error.Code = self._em_meas_verdict.get_current_result()
        self._error.Msg = self._em_meas_verdict.save_data_report_file()

        return self._error.Code, self._error.Msg
