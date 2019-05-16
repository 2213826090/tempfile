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
:summary: PUPDR - boot a board with given vbatt and power button press time.
:author: dbatutx
:since: 18/06/2012(June)
"""
import time
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase


class LabEmPsBootOnPowerButton(EmUsecaseBase):

    """
    Lab Energy Management class.
    """
    DEDICATED_BENCH = "POWER_SUPPLY_BENCH"

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LAB_EM_BASE_PS Init function
        EmUsecaseBase.__init__(self, tc_name, global_config)
        # Read power_button_time_to_on from TC parameters
        self._power_button_time_to_on = \
            float(self._tc_parameters.get_param_value("PWR_BT_TIME_TO_ON"))

        # get targets
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_PS_BOOT_ON_POWER_BUTTON", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

        # Redefine initial value for setting USBDIO:
        self.em_core_module.io_card_init_state["BatteryType"] = self.phone_info["BATTERY"]["BATTID_TYPE"]
        self.em_core_module.io_card_init_state["Battery"] = True
        self.em_core_module.io_card_init_state["Platform"] = "OFF"
        self.em_core_module.io_card_init_state["USBChargerType"] = self._io_card.USB_HOST_PC
        self.em_core_module.io_card_init_state["USBCharger"] = False
        self.em_core_module.io_card_init_state["BatteryTemperature"] = 25

        # Set initial value for setting Power Supply VBATT:
        # - VoltageLevel = VBATT
        self.em_core_module.eqp_init_state["BATT"]["VoltageLevel"] = self.em_core_module.vbatt

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        """
        # Call LAB_EM_BASE_PS Run function
        EmUsecaseBase.run_test_body(self)

        # Measure current from Vbatt
        time.sleep(10)
        meas_ibatt = self.em_core_module.get_battery_current()
        self._logger.info("Initial state : IBATT1 = %s %s" %
                         (meas_ibatt[0], meas_ibatt[1]))
        # store value in list for later comparison
        self._meas_list.add("IBATT1", meas_ibatt)

        # Power On sequence
        # PUSH ON button 1.5s
        self._logger.info("PUSH ON button 1.5s...")
        self._io_card.press_power_button(1.5)
        time.sleep(int(self._dut_config.get("bootTimeout")) / 2)
        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()

        self._logger.info("IBATT2 = %s %s" %
                         (meas_ibatt[0], meas_ibatt[1]))
        # store value in dict for later comparison
        self._meas_list.add("IBATT2", meas_ibatt)

        # Power On sequence
        # PUSH ON button 2.5s
        self._logger.info("PUSH ON button 2.5s...")
        time_before_boot = time.time()
        self._io_card.press_power_button(self._power_button_time_to_on)
        time.sleep(int(self._dut_config.get("bootTimeout")))
        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()

        self._logger.info("IBATT3 = %s %s" %
                         (meas_ibatt[0], meas_ibatt[1]))
        # store value in dict for later comparison
        self._meas_list.add("IBATT3", meas_ibatt)

        #
        # get boot info
        #
        # Connect USB PC/Host and DUT
        self.em_core_module.check_board_connection(only_reconnect=True)
        # get boot reason info
        boot_reason = self.phonesystem_api.check_message_in_log("BOOT_REASON",
                                                                 time_before_boot, check_result=True)
        # store value in dict for later comparison
        self._meas_list.add("BOOT_REASON", boot_reason[0], "none")
        # get boot mode info
        boot_mode = self.phonesystem_api.check_message_in_log("BOOT_MODE",
                                                               time_before_boot, check_result=True)
        # store value in dict for later comparison
        self._meas_list.add("BOOT_MODE", boot_mode[0], "none")

        # compare values with targets
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
        self._meas_list.clean()

        # Save data report in xml file
        self._error.Code = self._em_meas_verdict.get_current_result()
        self._error.Msg = self._em_meas_verdict.save_data_report_file()

        return self._error.Code, self._error.Msg
