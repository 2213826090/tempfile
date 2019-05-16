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
:summary: Energy Management battery insertion Use case
:author: ssavrimoutou
:since: 08/25/2010
"""
import time

from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from UtilitiesFWK.Utilities import Global


class LabEmBattInsertion(EmUsecaseBase):

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

        # Read BATTERY_TYPE from TC parameters
        self._battery_type = \
            str(self._tc_parameters.get_param_value("BATTERY_TYPE", self.phone_info["BATTERY"]["BATTID_TYPE"]))
        # Read Charger_TYPE from TC parameters
        self._charger_type = \
            str(self._tc_parameters.get_param_value("CHARGER_TYPE"))
        # Read Battery temperature from TC parameters
        self._batt_temp = \
            str(self._tc_parameters.get_param_value("BATTERY_TEMP"))

        # Redefine initial value for setting USBDIO:
        # - BatteryType = ANALOG
        self.em_core_module.io_card_init_state["BatteryType"] = self._battery_type
        # - Battery  = False (removed)
        self.em_core_module.io_card_init_state["Battery"] = False
        # - Platform = False (OFF)
        self.em_core_module.io_card_init_state["Platform"] = "OFF"
        if self._charger_type.upper() not in ["NONE", ""]:
            # - ChargerType
            self.em_core_module.io_card_init_state["USBChargerType"] = self._charger_type
            # - USBCharger = False (removed)
            self.em_core_module.io_card_init_state["USBCharger"] = False
        else:
            self.em_core_module.io_card_init_state["USBCharger"] = False
        # - BatteryTemperature = 25
        self.em_core_module.io_card_init_state["BatteryTemperature"] = self._batt_temp

        # Redefine initial value for setting Power Supply VBATT:
        # - VoltageLevel = 3.8
        self.em_core_module.eqp_init_state["BATT"]["VoltageLevel"] = self.em_core_module.vbatt

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        """
        # Call LAB_EM_BASE Run function
        EmUsecaseBase.run_test_body(self)

        # run specific test for ALL charger available or for only one
        if self._charger_type == "ALL":
            for charger in self._io_card.SUPPORTED_DEVICE_TYPE:
                if charger not in [self._io_card.USB_HOST_PC,
                                   self._io_card.OTG, self._io_card.WALL_CHARGER]:
                    self.run_specific_test(charger)
                    # switch off the board
                    self._device.soft_shutdown(False)
                    time.sleep(20)
                    # Remove the battery
                    self._io_card.battery_connector(False)
            # add case where no charger is plugged
            self.run_specific_test("None")
        else:
            self.run_specific_test(self._charger_type)

        # Save data report in xml file
        self._error.Code = self._em_meas_verdict.get_current_result()
        self._error.Msg = self._em_meas_verdict.save_data_report_file()

        return self._error.Code, self._error.Msg

    def run_specific_test(self, charger_type):
        """
        Execute the test for a specific charger
        """
        self._logger.info("Run Battery insertion for charger : %s"
                          % charger_type)

        targets = self._tc_parameters.get_params_as_dict()
        targets["CHARGER_TYPE"] = charger_type
        # Load the reference values
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_BATT_INSERTION", targets,
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

        # Remove battery
        self._io_card.battery_connector(False)
        time.sleep(2)

        # plug charger
        self.em_core_module.plug_charger(charger_type, ext_ps=False)

        # Wait time between command
        time.sleep(5)

        # measure current from V battery
        meas_ibatt = self.em_core_module.get_battery_current()

        # Measure current from V charger
        meas_icharger = self.em_core_module.get_charger_current(charger_type)

        self._logger.info(
            "Battery removed : IBatt = %s %s" % (meas_ibatt[0], meas_ibatt[1]))

        # Compare value with limit parameters
        self._meas_list.add("IBATT1", meas_ibatt)

        # Compare Vcharger value with limit parameters
        if meas_icharger[0] != 'None':
            self._meas_list.add("ICHARGER1", meas_icharger)

        if self._battery_type == "BATTERY_EMULATOR":
            self._io_card.set_battery_type(self._battery_type)
        # Insert battery
        self._io_card.battery_connector(True, self._battery_type)

        # Wait time between command
        time.sleep(self._device.get_boot_timeout())

        # Measure current from Vbatt
        meas_ibatt2 = self.em_core_module.get_battery_current()

        # Measure current from V charger
        meas_icharger2 = self.em_core_module.get_charger_current(charger_type)

        self._logger.info(
            "Battery inserted : IBatt = %s %s" % (meas_ibatt2[0], meas_ibatt2[1]))

        # Compare value with limit parameters
        self._meas_list.add("IBATT2", meas_ibatt2)

        # Compare Vchargervalue with limit parameters
        if meas_icharger2[0] != 'None':
            self._meas_list.add("ICHARGER2", meas_icharger2)

        # check for  boot mode
        mode = self._device.get_boot_mode()
        if mode == "COS":

            # check for  boot mode
            wake_src = self.phonesystem_api.get_boot_wake_source()

            # Compare value with limit parameters
            self._meas_list.add("COS_MODE", mode, "none")

            # Compare value with limit parameters
            self._meas_list.add("BOOT_WAKESRC_BATT", wake_src[1], "none")
        else:
            # try to reboot the board
            # plug charger
            self.em_core_module.unplug_charger(charger_type)
            time.sleep(30)
            # try to boot in MOS
            self._io_card.press_power_button(self.pwr_btn_boot)
            time.sleep(50)
            # reconnect the board
            self.em_core_module.plug_charger("SDP")
            self.em_core_module.check_board_connection(use_exception=False, only_reconnect=True)
            # check for android boot mode cos
            # we use 0 as time stamp because the time has been reset with the battery removal
            result = self.phonesystem_api.check_message_in_log("COS_MODE",
                                                                0, check_result=False)
            mode = ""
            if result[0] == "TRUE":
                mode = "COS"
            else:
                mode = "UNKNOWN"
            # Compare value with limit parameters
            self._meas_list.add("COS_MODE", mode, "none")
            if result[0] == "TRUE":
                # check for boot reason boot mode cos
                wakesrc = self.phonesystem_api.check_message_in_log("BOOT_REASON",
                                                                     0, result[1] + 5, check_result=True)
                # Compare value with limit parameters
                self._meas_list.add("BOOT_WAKESRC_BATT", wakesrc[0], "none")

        # if the battery is not an ANALOG type, the board may boot on battery insertion
        if self._battery_type != "ANALOG":
            # check if it boots
            if self._em_meas_verdict.test_value(self._meas_list, self._em_targets["IBATT2"]):
                self._logger.info("the phone can be on, try to read msic register")
                # connect SDP and check the board connection
                self.em_core_module.plug_charger("SDP")
                self.em_core_module.check_board_connection(use_exception=False, only_reconnect=True)
                # get the msic register and check it
                msic_registers = self.em_api.get_msic_registers()
                self._meas_list.add_dict("MSIC_REGISTER", msic_registers,
                                         msic_registers["TIME_STAMP"][0])
                self._device.disconnect_board()
                self.em_core_module.unplug_charger("SDP")

        # generate em verdict
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
        self._em_meas_verdict.judge(ignore_blocked_tc=True)
        self._meas_list.clean()

    def tear_down(self):
        """
        End and dispose the test
        """
        EmUsecaseBase.tear_down(self)

        # reboot in normal mode for next test
        if self._battery_type == "BATTERY_EMULATOR":
            # Set Battery type using BATTERY_TYPE parameter
            self._io_card.set_battery_type("DEFAULT")
            self.em_core_module.reboot_board()

        return Global.SUCCESS, "No errors"
