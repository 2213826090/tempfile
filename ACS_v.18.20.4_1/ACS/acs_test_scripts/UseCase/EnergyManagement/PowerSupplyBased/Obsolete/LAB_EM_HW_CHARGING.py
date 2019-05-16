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
:author: ssavrimoutou, apairex
:since: 08/25/2010
"""
import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from ErrorHandling.AcsConfigException import AcsConfigException


class LabEmHwCharging(EmUsecaseBase):

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
        # Read CHARGER from TC parameters
        self._charger = str(self._tc_parameters.get_param_value("CHARGER_TYPE"))

        # Call ConfigsParser to parse Energy_Management
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_HW_CHARGING", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

        # Redefine initial value for setting IO_CARD:
        # - BatteryType = ANALOG
        self.em_core_module.io_card_init_state["BatteryType"] = self.phone_info["BATTERY"]["BATTID_TYPE"]
        # - Battery
        self.em_core_module.io_card_init_state["Battery"] = True
        # - Platform
        self.em_core_module.io_card_init_state["Platform"] = "OFF"
        # - USBChargerType
        self.em_core_module.io_card_init_state["USBChargerType"] = self._charger
        # - USBCharger
        self.em_core_module.io_card_init_state["USBCharger"] = False
        # - BatteryTemperature
        self.em_core_module.io_card_init_state["BatteryTemperature"] = 25

        # Set initial value for setting Power Supply VBATT:
        # - VoltageLevel = VBATT
        self.em_core_module.eqp_init_state["BATT"]["VoltageLevel"] = self.em_core_module.vbatt

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Set up the test configuration
        """
        # Call UseCase base Setup function
        EmUsecaseBase.set_up(self)
        if self._charger not in [self._io_card.DCP,
                                 self._io_card.AC_CHGR]:
            tmp_txt = "Not allowed charger type for this type :%s" % \
                self._charger
            self._logger.error(tmp_txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, tmp_txt)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        """

        # Call LAB_EM_BASE Init function
        EmUsecaseBase.run_test_body(self)

        # Measure current from Vbatt and Vcharger
        meas_icharger = self.em_core_module.get_charger_current(self._charger)
        meas_ibatt = self.em_core_module.get_battery_current()

        self._logger.info("Initial state : IBATT = %s %s - Icharger = %s %s" %
                         (meas_ibatt[0], meas_ibatt[1], meas_icharger[0], meas_icharger[1]))

        # Check IUSB and IBATT when no battery is plugged in
        self._meas_list.add("IBATT_INIT", meas_ibatt)
        self._meas_list.add("ICHARGER_INIT", meas_icharger)
        # connect wall charger
        start_time = time.time()
        self.em_core_module.plug_charger(self._charger, True)

        # Wait time between command
        time.sleep(20)

        # Loop during safety_timer (given in target file)
        safety_timer = int(self._em_targets["SAFETY_TIMER"].get("value")) + 30
        timer = 0
        self._logger.info(
            "Wait %s seconds safety timer expiration" % safety_timer)

        while timer < safety_timer:

            # Measure current from V every 20 seconds
            meas_icharger = self.em_core_module.get_charger_current(self._charger)
            # Measure current from Vbatt every 20 seconds
            meas_ibatt = self.em_core_module.get_battery_current()

            self._logger.info(
                "Battery inserted : " +
                "IBatt(%dV) = %s %s - Icharger = %s %s , remaining seconds = %.1f" %
                (self.em_core_module.vbatt,
                 meas_ibatt[0], meas_ibatt[1], meas_icharger[0], meas_icharger[1],
                             (safety_timer - timer)))

            self._meas_list.add("IBATT_CHARGING", meas_ibatt)
            self._meas_list.add("ICHARGER_CHARGING", meas_icharger)
            # Compare ibatt value with Low and High limit parameters
            if not self._em_meas_verdict.test_value(self._meas_list,
                                                    self._em_targets["IBATT_CHARGING"]):
                break

            # Compare i value with Low and High limit parameters
            if not self._em_meas_verdict.test_value(self._meas_list,
                                                    self._em_targets["ICHARGER_CHARGING"]):
                break
            # wait few seconds
            time.sleep(20)
            timer = time.time() - start_time
        # End Loop
        # Compare ibatt value with limit parameters
        self._meas_list.add("SAFETY_TIMER", timer, "s")
        time.sleep(25)

        # Measure current from V
        meas_icharger = self.em_core_module.get_charger_current(self._charger)
        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()

        self._logger.info(
            "Wall Charger plugged for " +
            "%d seconds: IBatt = %s %s - Icharger = %s %s" %
            (safety_timer, meas_ibatt[0], meas_ibatt[1],
             meas_icharger[0], meas_icharger[1]))

        # Compare ibatt value with limit parameters
        self._meas_list.add("IBATT_AFTER_15MIN", meas_ibatt)

        # Compare i value with limit parameters
        self._meas_list.add("ICHARGER_AFTER_15MIN", meas_icharger)

        self.em_core_module.unplug_charger(self._charger)
        time.sleep(2)
        # plug wall charger
        self.em_core_module.plug_charger(self._charger, True)

        # Wait time between command
        time.sleep(5)

        # Measure current from V
        meas_icharger = self.em_core_module.get_charger_current(self._charger)
        # Measure current from Vbatt
        meas_ibatt = (self.em_core_module.pwrs_vbatt.get_current_meas(
            self.em_core_module.ps_properties["BATT"]["PortNumber"], "DC"), "A")

        self._logger.info(
            "USB Wall Charger re-plug - IBatt(%dV) = %s %s Icharger= %s %s" %
            (self.em_core_module.vbatt,
             meas_ibatt[0], meas_ibatt[1], meas_icharger[0], meas_icharger[1]))

        # Compare ibatt value with limit parameters
        self._meas_list.add("IBATT_REPLUG", meas_ibatt)

        # Compare icharger value with limit parameters
        self._meas_list.add("ICHARGER_REPLUG", meas_icharger)

        # generate em verdict
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
        self._meas_list.clean()

        # Save data report in xml file
        self._error.Code = self._em_meas_verdict.get_current_result()
        self._error.Msg = self._em_meas_verdict.save_data_report_file()

        return self._error.Code, self._error.Msg
