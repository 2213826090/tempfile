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
:summary: PUPDR - boot a board with given vbatt.
:author: vgomberx
:since: 13/06/2012(June)
"""
import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase


class LabEmPsBoot(EmUsecaseBase):

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

        # Read VBATT from TC parameters
        self._modified_vbatt = float(self._tc_parameters.get_param_value("VBATT"))

        # Read INITIAL_OS_STATE from TC parameters
        self._initial_os_state = str(
            self._tc_parameters.get_param_value("INITIAL_OS_STATE")).upper()

        # Read CHARGER_TYPE from TC parameters
        self._charger_type = str(
            self._tc_parameters.get_param_value("CHARGER_TYPE")).upper()

        # get targets
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_PS_BOOT", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

        # Redefine initial value for setting USBDIO:
        self.em_core_module.io_card_init_state["BatteryType"] = self.phone_info["BATTERY"]["BATTID_TYPE"]
        self.em_core_module.io_card_init_state["Battery"] = True
        self.em_core_module.io_card_init_state["Platform"] = "ON"
        self.em_core_module.io_card_init_state["USBChargerType"] = self._io_card.SDP
        self.em_core_module.io_card_init_state["USBCharger"] = True
        self.em_core_module.io_card_init_state["BatteryTemperature"] = 25

        # Set initial value for setting Power Supply VBATT:
        # - VoltageLevel = VBATT
        self.em_core_module.eqp_init_state["BATT"]["VoltageLevel"] = self.em_core_module.vbatt

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # Call the LabEmBasePS Setup function
        EmUsecaseBase.set_up(self)

        self._logger.info("Set the board to the state %s." % self._initial_os_state)
        if self._initial_os_state == "OFF":
            self.em_core_module.unplug_charger(self._io_card.SDP)
            self._device.disconnect_board()
            self._logger.info("Switch off the board by a hard shutdown...")
            self._io_card.press_power_button(10)

        elif self._initial_os_state == "OFF_NO_BATT":
            # unplug the battery
            self._io_card.battery_connector(False)
        elif self._initial_os_state == "COS":
            # switch off the board
            self._device.soft_shutdown(False)
            # wait to switch off
            time.sleep(15)
            # reboot in cos
            self.em_core_module.plug_charger(self._io_card.SDP, True)
            # wait for reboot in COS
            time.sleep(30)
        elif self._initial_os_state == "MAIN":
            pass
        else:
            self._device.reboot(self._initial_os_state,
                                wait_for_transition=True,
                                skip_failure=True)

        # wait some seconds
        time.sleep(5)
        # init vbatt to the wanted one
        self.em_core_module.pwrs_vbatt.set_current_voltage(self._modified_vbatt,
                                             self.em_core_module.ps_properties["BATT"]["PortNumber"])

        if self._charger_type != "NONE":
            # plug the charger for the test
            self.em_core_module.plug_charger(self._charger_type)

        # wait some seconds
        time.sleep(5)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        """
        # Call LAB_EM_BASE_PS Run function
        EmUsecaseBase.run_test_body(self)
        time_before_boot = time.time()

        # Measure current from Vbatt and Vcharger
        meas_icharger = self.em_core_module.get_charger_current(self._io_card.WALL_CHARGER)
        meas_ibatt = self.em_core_module.get_battery_current()

        self._logger.info("Initial state : IBATT = %s %s - Icharger = %s %s" %
                         (meas_ibatt[0], meas_ibatt[1], meas_icharger[0], meas_icharger[1]))

        # store value in list for later comparison
        self._meas_list.add("IBATT1", meas_ibatt)
        self._meas_list.add("ICHARGER1", meas_icharger)

        # Power On sequence
        # PUSH ON button 3s
        self._logger.info(" push the power button...")
        self._io_card.press_power_button(self.pwr_btn_boot)
        time.sleep(self._device.get_boot_timeout())

        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()

        self._logger.info("reboot state : IBATT = %s %s" %
                         (meas_ibatt[0], meas_ibatt[1]))

        # store value in dict for later comparison
        self._meas_list.add("IBATT2", meas_ibatt)

        #  connect USB_HOST_PC
        self.em_core_module.plug_charger(self._io_card.SDP)
        time.sleep(10)
        if self._device.get_boot_mode() == "MOS":
            self.em_core_module.check_board_connection(use_exception=False, only_reconnect=True)
            # check for boot reason
            boot_reason = self.phonesystem_api.check_message_in_log("BOOT_REASON",
                                                                     time_before_boot, check_result=True)
            # store value in dict for later comparison
            self._meas_list.add("BOOT_REASON", boot_reason[0], "none")
            # check for boot mode
            boot_mode = self.phonesystem_api.check_message_in_log("BOOT_MODE",
                                                                   time_before_boot, check_result=True)
            # store value in dict for later comparison
            self._meas_list.add("BOOT_MODE", boot_mode[0], "none")
            # check for shutdown  reason
            shutdown_reason = self.phonesystem_api.check_message_in_log("SHUTDOWN_REASON",
                                                                         time_before_boot, check_result=True)
            # store value in dict for later comparison
            self._meas_list.add("SHUTDOWN_REASON", shutdown_reason[0], "none")
        else:
            # store value in dict for later comparison
            self._meas_list.add("BOOT_REASON", "BOARD NOT BOOTED", "none")
            # store value in dict for later comparison
            self._meas_list.add("BOOT_MODE", "BOARD NOT BOOTED", "none")
            # store value in dict for later comparison
            self._meas_list.add("SHUTDOWN_REASON", "BOARD NOT BOOTED", "none")

        # compare values with targets
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
        self._meas_list.clean()

        # Save data report in xml file
        self._error.Code = self._em_meas_verdict.get_current_result()
        self._error.Msg = self._em_meas_verdict.save_data_report_file()

        return self._error.Code, self._error.Msg

    def tear_down(self):
        """
        End and dispose the test
        """
        if self._initial_os_state.upper() == "OFF_NO_BATT":
            # plug the battery
            self._io_card.battery_connector(True)

        EmUsecaseBase.tear_down(self)

        return Global.SUCCESS, "No errors"
