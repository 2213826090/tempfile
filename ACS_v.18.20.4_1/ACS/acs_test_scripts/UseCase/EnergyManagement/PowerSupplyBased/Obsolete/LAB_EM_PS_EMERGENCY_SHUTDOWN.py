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
:summary: Energy Management Emergency shutdown test.
          The test boot the board at a normal battery voltage.
          Then it change the battery voltage to a voltage below the Emergency voltage threshold.
          After this, we reboot the board at a normal voltage in order to control the result.
:author: dbatutx
:since: 07/03/2013
"""

import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase


class LabEmPsEmergencyShutdown(EmUsecaseBase):

    """
    Lab Energy Management class.
    """
    DEDICATED_BENCH = "POWER_SUPPLY_BENCH"

    def __init__(self, tc_name, global_config):

        # Call LAB_EM_BASE Init function
        EmUsecaseBase.__init__(self, tc_name, global_config)

        # Get the emergency battery voltage
        self._emergency_voltage = float(self._tc_parameters.get_param_value(
            "EMERGENCY_BATTERY_VOLTAGE", "2.7"))

        # get the normal battery voltage
        self._normal_battery_voltage = float(self._tc_parameters.get_param_value(
            "NORMAL_BATTERY_VOLTAGE", 3.8))

        # Redefine initial value for setting USBDIO:
        # - BatteryType = ANALOG
        self.em_core_module.io_card_init_state["BatteryType"] = self.phone_info["BATTERY"]["BATTID_TYPE"]
        # - Battery
        self.em_core_module.io_card_init_state["Battery"] = True
        # - Platform
        self.em_core_module.io_card_init_state["Platform"] = "ON"
        # - USBChargerType
        self.em_core_module.io_card_init_state["USBChargerType"] = "USB_HOST_PC"
        # - USBCharger
        self.em_core_module.io_card_init_state["USBCharger"] = False
        # - BatteryTemperature
        self.em_core_module.io_card_init_state["BatteryTemperature"] = 25

        # Set initial value for setting Power Supply VBATT:
        # - VoltageLevel
        self.em_core_module.eqp_init_state["BATT"]["VoltageLevel"] = self._normal_battery_voltage

    def set_up(self):
        """
        Initialize the test:
        """
        EmUsecaseBase.set_up(self)

        # Call ConfigsParser to parse Energy_Management
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_PS_EMERGENCY_SHUTDOWN", self._tc_parameters.get_params_as_dict(), self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

        return Global.SUCCESS, "No errors"

    def run_test_body(self):
        """
        Execute the test
        """
        # Call LAB_EM_BASE Init function
        EmUsecaseBase.run_test_body(self)

        # init error Msg
        self._error.Msg = ""
        # init vbatt to the emergency battery voltage
        self._logger.info("START THE TEST BY SETTING THE EMERGENCY " +
                          "BATTERY VOLTAGE (%s V)" % self._emergency_voltage)
        self.em_core_module.pwrs_vbatt.set_current_voltage(self._emergency_voltage,
                                             self.em_core_module.ps_properties["BATT"]["PortNumber"])
        time.sleep(15)
        # measure battery current
        ibatt = self.em_core_module.get_battery_current()
        # store result in dict for later comparison
        self._meas_list.add("IBATT_AFTER_EMERGENCY", ibatt)
        # test the value in order to return a good error message
        result = self._em_meas_verdict.test_value(self._meas_list,
                                                  self._em_targets["IBATT_AFTER_EMERGENCY"])
        if result is False:
            self._error.Msg += "the board shall be OFF after a voltage drop " \
                + "to %s\n" % self._emergency_voltage
        # set a normal battery voltage
        self._logger.info("RESTORE A NORMAL BATTERY VOLTAGE AND CHECK LOG")
        self.em_core_module.pwrs_vbatt.set_current_voltage(self._normal_battery_voltage,
                                             self.em_core_module.ps_properties["BATT"]["PortNumber"])
        # record time before booting the board
        before_boot_time = time.time()
        # boot the board
        self.em_core_module.boot_board("MOS")
        # check shutdown confirmation
        shutdown_reason = self.phonesystem_api.check_message_in_log(
            "SHUTDOWN_REASON2", before_boot_time, time.time(), True)
        # store result in dict for later comparison
        self._meas_list.add("SHUTDOWN_REASON", shutdown_reason[0], "none")
        # test the value in order to return a good error message
        result = self._em_meas_verdict.test_value(self._meas_list,
                                                  self._em_targets["SHUTDOWN_REASON"])
        if result is False:
            self._error.Msg += "The shutdown reason is not good in the device logs\n"
        # compare values with targets
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
        self._em_meas_verdict.judge(ignore_blocked_tc=True)
        self._meas_list.clean()

        # Save data report in xml file
        self._error.Code = self._em_meas_verdict.get_current_result()
        self._error.Msg += self._em_meas_verdict.save_data_report_file()
        return self._error.Code, self._error.Msg
