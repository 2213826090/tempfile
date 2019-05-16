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
:summary: PUPDR Energy Management test watchdog mechanism.
:author: vgomberx
:since: 15/06/2012(June)
"""
import time
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase


class LabEmPsWatchdogMechanism(EmUsecaseBase):

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

        # Read FREEZE_TYPE from TC parameters
        self._freeze_type = \
            str(self._tc_parameters.get_param_value("FREEZE_TYPE"))

        # Read REBOOT_TIMEOUT from TC parameters
        self._reboot_timeout = \
            int(self._tc_parameters.get_param_value("REBOOT_TIMEOUT"))

        # get targets
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_PS_WATCHDOG_MECHANISM", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

        # Redefine initial value for setting USBDIO:
        self.em_core_module.io_card_init_state["BatteryType"] = self.phone_info["BATTERY"]["BATTID_TYPE"]
        self.em_core_module.io_card_init_state["Battery"] = True
        self.em_core_module.io_card_init_state["Platform"] = "ON"
        self.em_core_module.io_card_init_state["USBChargerType"] = self._io_card.USB_HOST_PC
        self.em_core_module.io_card_init_state["USBCharger"] = True
        self.em_core_module.io_card_init_state["BatteryTemperature"] = 25
        # Set initial value for setting Power Supply VBATT:
        # - VoltageLevel = VBATT
        self.em_core_module.eqp_init_state["BATT"]["VoltageLevel"] = self.em_core_module.vbatt

        if self._freeze_type == "BOTH":
            self._freeze_type = ["SOFT", "HARD"]
        else:
            self._freeze_type = [self._freeze_type]

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        """
        # Call LAB_EM_BASE_PS Run function
        EmUsecaseBase.run_test_body(self)

        # Measure current from Vbatt and Vcharger
        meas_ibatt = self.em_core_module.get_battery_current()

        self._logger.info("Initial state : IBATT = %s %s" %
                         (meas_ibatt[0], meas_ibatt[1]))

        # store value in dict for later comparison
        self._meas_list.add("IBATT1", meas_ibatt)

        for element in self._freeze_type:
            time_before_shutdown = time.time()
            # stop usb charging
            self.em_api.set_usb_charging("off")
            # wake up the board if necessary
            if not self.phonesystem_api.get_screen_status():
                self._io_card.press_power_button(0.3)
            # crash board
            self.phonesystem_api.freeze_board(element)
            self._device.disconnect_board()

            # Measure current from Vbatt
            meas_ibatt = self.em_core_module.get_battery_current()
            self._logger.info("freeze state : IBATT = %s %s" %
                             (meas_ibatt[0], meas_ibatt[1]))

            # store value in list for later comparison
            self._meas_list.add(element + "_IBATT", meas_ibatt)

            # check device connection is lost
            self._logger.info("wait at most %s s for the board to lost connection" % str(self._reboot_timeout))
            connection_state = ("ALIVE", "none")
            start_time = time.time()
            while time.time() - start_time < self._reboot_timeout:
                if self._device.get_state() != "alive":
                    connection_state = ("LOST", "none")
                    break

            # store value in list for later comparison
            self._meas_list.add(element + "_DEVICE_CONNECTION",
                                connection_state)

            # wait for the board to reboot
            self._logger.info("wait for the board to reboot itself")
            start_time = time.time()
            while time.time() - start_time < self._device.get_boot_timeout():
                if self._device.get_state() == "alive":
                    # establish connection
                    self._device.connect_board()
                    break

            if self._device.is_available():
                # check for boot reason
                boot_reason = self.phonesystem_api.check_message_in_log(
                    "BOOT_REASON",
                    time_before_shutdown, check_result=True)
                # store value in list for later comparison
                self._meas_list.add(element + "_BOOT_REASON",
                                    boot_reason[0], "none")
                # check for shutdown reason
                shutdown_reason = self.phonesystem_api.check_message_in_log(
                    "SHUTDOWN_REASON",
                    time_before_shutdown, check_result=True)
                # store value in list for later comparison
                self._meas_list.add(element + "_SHUTDOWN_REASON",
                                    shutdown_reason[0], "none")
                # check for boot mode
                boot_mode = self._device.get_boot_mode()
                # store value in list for later comparison
                self._meas_list.add(element + "_BOOT_MODE", boot_mode, "none")
            else:
                # store value in list for later comparison
                self._meas_list.add(element + "_SHUTDOWN_REASON",
                                    "CANT GET SHUTDOWN REASON", "none")
                self._meas_list.add(element + "_BOOT_REASON",
                                    "CANT GET BOOT REASON", "none")
                self._meas_list.add(element + "_BOOT_MODE",
                                    "CANT GET BOOT MODE", "none")

        # compare values with targets
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
        self._meas_list.clean()

        # Save data report in xml file
        self._error.Code = self._em_meas_verdict.get_current_result()
        self._error.Msg = self._em_meas_verdict.save_data_report_file()

        return self._error.Code, self._error.Msg
