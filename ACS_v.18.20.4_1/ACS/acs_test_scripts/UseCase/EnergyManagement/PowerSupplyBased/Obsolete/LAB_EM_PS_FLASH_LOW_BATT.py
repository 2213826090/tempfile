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
:summary: Energy Management Flash with a low batt test.
          First the test tries to flash the board under the battery voltage threshold.
          Then it tries to flash the board with a normal battery voltage.
:author: dbatutx
:since: 28/02/2013
"""

from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from ErrorHandling.AcsBaseException import AcsBaseException


class LabEmPsFlashLowBatt(EmUsecaseBase):

    """
    Lab Energy Management class.
    """
    DEDICATED_BENCH = "POWER_SUPPLY_BENCH"

    def __init__(self, tc_name, global_config):

        # Call LAB_EM_BASE Init function
        EmUsecaseBase.__init__(self, tc_name, global_config)

        # Initialize flash attributes
        self._flash_file_path = None
        self._flash_timeout = None

        # Get the flash file path value either from parameter file or
        # default value from phone catalog
        flash_file_path = \
            str(self._tc_parameters.get_param_value("FLASH_FILE_PATH", ""))
        if flash_file_path != "":
            self._flash_file_path = flash_file_path

        # Get flashTimeout value either from parameter file or
        # default value from phone catalog
        flash_timeout = \
            str(self._tc_parameters.get_param_value("FLASH_TIMEOUT"))
        if flash_timeout.isdigit():
            self._flash_timeout = int(flash_timeout)

        # get the low battery voltage to try to flash
        self._low_battery_voltage = float(self._tc_parameters.get_param_value(
            "LOW_BATTERY_VOLTAGE", 3.5))

        # get the normal battery voltage to try to flash
        self._normal_battery_voltage = float(self._tc_parameters.get_param_value(
            "NORMAL_BATTERY_VOLTAGE", 3.8))

        # Redefine initial value for setting USBDIO:
        # - BatteryType = ANALOG
        self.em_core_module.io_card_init_state["BatteryType"] = self.phone_info["BATTERY"]["BATTID_TYPE"]
        # - Battery
        self.em_core_module.io_card_init_state["Battery"] = True
        # - Platform
        self.em_core_module.io_card_init_state["Platform"] = "OFF"
        # - USBChargerType
        self.em_core_module.io_card_init_state["USBChargerType"] = "USB_HOST_PC"
        # - USBCharger
        self.em_core_module.io_card_init_state["USBCharger"] = False
        # - BatteryTemperature
        self.em_core_module.io_card_init_state["BatteryTemperature"] = 25

        # Set initial value for setting Power Supply VBATT:
        # - VoltageLevel
        self.em_core_module.eqp_init_state["BATT"]["VoltageLevel"] = self._low_battery_voltage

    def set_up(self):
        """
        Initialize the test:
        """
        EmUsecaseBase.set_up(self)

        # Call ConfigsParser to parse Energy_Management
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_PS_FLASH_LOW_BATT", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

        return Global.SUCCESS, "No errors"

    def run_test_body(self):
        """
        Execute the test
        """
        # Call LAB_EM_BASE Init function
        EmUsecaseBase.run_test_body(self)

        # reset error message for back to back mode
        self._error.Msg = ""

        # init vbatt to the low battery voltage
        self.em_core_module.pwrs_vbatt.set_current_voltage(self._low_battery_voltage,
                                             self.em_core_module.ps_properties["BATT"]["PortNumber"])

        # Call flash sequence with the low battery voltage
        self._logger.info("START FLASH SEQUENCE WITH LOW BATTERY VOLTAGE(%s)!"
                            % str(self._low_battery_voltage))

        try:
            return_code, return_msg = self._device.flash(self._flash_file_path, self._flash_timeout)
        except AcsBaseException as ex:
            return_code = Global.FAILURE
            return_msg = ex.get_error_message()

        # Check flash result
        if return_code == Global.SUCCESS:
            # store result in dict for later comparison
            self._meas_list.add("FLASH_LOW_BATT_RESULT", "PASS", "none")
            self._error.Msg += "THE FLASH SHOULD NOT PASS AT LOW BATTERY VOLTAGE\n"
        else:
            self._logger.info("FAIL TO FLASH: %s" % return_msg)
            # store result in dict for later comparison
            self._meas_list.add("FLASH_LOW_BATT_RESULT", "FAIL", "none")
            self._error.Msg += "THE FLASH FAILED AT LOW BATTERY VOLTAGE: %s\n" % return_msg

        # init vbatt to the low battery voltage
        self.em_core_module.pwrs_vbatt.set_current_voltage(self._normal_battery_voltage,
                                             self.em_core_module.ps_properties["BATT"]["PortNumber"])

        # Call flash sequence with the normal battery voltage
        self._logger.info("START FLASH SEQUENCE WITH NORMAL BATTERY VOLTAGE(%s)!"
                          % str(self._normal_battery_voltage))

        try:
            self._device.flash(self._flash_file_path, self._flash_timeout)
        except AcsBaseException as ex:
            return_code = Global.FAILURE
            return_msg = ex.get_error_message()

        # Check flash result
        if return_code == Global.SUCCESS:
            # store result in dict for later comparison
            self._meas_list.add("FLASH_NORMAL_BATT_RESULT", "PASS", "none")
        else:
            self._logger.info("FAIL TO FLASH: %s" % return_msg)
            # store result in dict for later comparison
            self._meas_list.add("FLASH_NORMAL_BATT_RESULT", "FAIL", "none")
            self._error.Msg += "THE FLASH FAILED AT NORMAL BATTERY VOLTAGE: %s\n" % return_msg

        # compare values with targets
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
        self._em_meas_verdict.judge(ignore_blocked_tc=True)
        self._meas_list.clean()

        # Save data report in xml file
        self._error.Code = self._em_meas_verdict.get_current_result()
        self._error.Msg += self._em_meas_verdict.save_data_report_file()
        self._logger.debug("[THERMAL] Thermal test end")
        return self._error.Code, self._error.Msg

    def tear_down(self):
        """
        End and dispose the test
        """
        # switch on the board
        self._device.switch_on()
        # install acs apk embedded DEPREACTED CODE TO BE REPLACED
        # self._device.setup_embedded()
        # call the base tear down
        EmUsecaseBase.tear_down(self)

        return Global.SUCCESS, "No errors"
