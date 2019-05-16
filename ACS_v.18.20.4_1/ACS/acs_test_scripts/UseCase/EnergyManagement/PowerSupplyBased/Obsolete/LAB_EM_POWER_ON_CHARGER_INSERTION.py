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
:summary: Energy Management plug charger Use case
:author: apairex
:since: 11/28/2011
"""
import time

from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from UtilitiesFWK.Utilities import Global


class LabEmPowerOnChargerInsertion(EmUsecaseBase):

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

        # Read USB_CHARGER_TYPE from TC parameters
        self._charger_type = \
            str(self._tc_parameters.get_param_value("CHARGER_TYPE"))

        # Redefine initial value for setting USBDIO:
        # - BatteryType
        self.em_core_module.io_card_init_state["BatteryType"] = self.phone_info["BATTERY"]["BATTID_TYPE"]
        # - Battery
        self.em_core_module.io_card_init_state["Battery"] = True
        # - Platform
        self.em_core_module.io_card_init_state["Platform"] = "OFF"
        # - USBCharger
        self.em_core_module.io_card_init_state["USBCharger"] = False

        # Set initial value for setting Power Supply VBATT:
        # - VoltageLevel = VBATT
        self.em_core_module.eqp_init_state["BATT"]["VoltageLevel"] = self.em_core_module.vbatt
        self.em_core_module.pwrs_vbatt = self._em.get_power_supply("BATT")

    def set_up(self):
        EmUsecaseBase.set_up(self)

        self._logger.info("Wait 30s for the phone to be switched off")
        time.sleep(30)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        """
        # Call LAB_EM_BASE Run function
        EmUsecaseBase.run_test_body(self)

        if self._charger_type == "ALL":
            for charger in self._io_card.SUPPORTED_DEVICE_TYPE:
                if charger not in [self._io_card.USB_HOST_PC,
                                   self._io_card.OTG, self._io_card.WALL_CHARGER]:
                    self._run_test_for_specific_charger(charger)
        else:
            self._run_test_for_specific_charger(self._charger_type)

        # Save data report in xml file
        self._error.Code = self._em_meas_verdict.get_current_result()
        self._error.Msg = self._em_meas_verdict.save_data_report_file()

        return self._error.Code, self._error.Msg

    def _run_test_for_specific_charger(self, charger_type):
        """
        Execute the test for a specific charger
        """
        self._logger.info("Run POWER_ON_CHARGER_INSERTION for %s"
                          % charger_type)

        targets = self._tc_parameters.get_params_as_dict()
        targets["CHARGER_TYPE"] = charger_type
        # Load the reference values
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_POWER_ON_CHARGER_INSERTION",
            targets,
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

        # Check Ibatt before plug
        meas_ibatt = self.em_core_module.get_battery_current()
        self._logger.info("Phone should be OFF: IBatt = %s %s"
                          % (meas_ibatt[0], meas_ibatt[1]))
        self._meas_list.add("IBATT_BEFORE", meas_ibatt)
        # Measure current from Vcharger
        meas_icharger = self.em_core_module.get_charger_current(charger_type)
        if meas_icharger[0] != 'None':
            self._meas_list.add("ICHARGER_BEFORE", meas_icharger)

        # Plug charger
        before_cos_time = time.time()
        self.em_core_module.plug_charger(charger_type, True)
        self._logger.info("Waiting %s to check the phone boot"
                          % self._device.get_boot_timeout())
        time.sleep(self._device.get_boot_timeout())

        # Check Ibatt and Icharger after charger insertion
        meas_ibatt = self.em_core_module.get_battery_current()
        self._meas_list.add("IBATT_AFTER", meas_ibatt)

        # Measure current from Vcharger
        meas_icharger = self.em_core_module.get_charger_current(charger_type)
        if meas_icharger[0] != 'None':
            self._meas_list.add("ICHARGER_AFTER", meas_icharger)

        self._logger.info(
            "Charger %s plugged : IBatt = %s %s , ICharger = %s %s " %
            (charger_type,
             meas_ibatt[0], meas_ibatt[1], meas_icharger[0], meas_icharger[1]))

        # try to get boot mode and wakesrc in COS if available
        # check for  boot mode
        mode = self._device.get_boot_mode()

        # Remove charger and wait for the phone to switch back off
        after_cos_time = time.time()
        self.em_core_module.unplug_charger(charger_type)
        self._logger.info("Waiting 60s for the phone to switch OFF")
        time.sleep(60)

        if mode == "UNKNOWN":
            # boot the board
            self.em_core_module.boot_board("MOS", 2)
            # check for android boot mode cos
            result = self.phonesystem_api.check_message_in_log("COS_MODE",
                                                                before_cos_time, after_cos_time, check_result=False)
            if result[0].lower() == "true":
                mode = "COS"
            else:
                mode = "UNKNOWN"

            # restore board init state to off
            self._device.soft_shutdown(False)
            time.sleep(30)

        # Compare boot mode cos value with limit parameters
        self._meas_list.add("BOOT_REASON", mode, "none")

        # generate em verdict
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
        self._em_meas_verdict.judge(ignore_blocked_tc=True)
        self._meas_list.clean()
