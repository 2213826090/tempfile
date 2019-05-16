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
:summary: PUPDR Energy Management perform shutdown with cable plug or unplug then boot.
:author: vgomberx
:since: 06/04/2012(June)
"""
import time
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from ErrorHandling.AcsConfigException import AcsConfigException


class LabEmPsReboot(EmUsecaseBase):

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

        # Read CABLE_TYPE from TC parameters
        self._cable_type = \
            str(self._tc_parameters.get_param_value("CABLE_TYPE"))

        # Read SHUTDOWN_TYPE from TC parameters
        self._shutdown_type = \
            str(self._tc_parameters.get_param_value("SHUTDOWN_TYPE"))

        # create tag in order to retrieve the right measurement
        # get targets
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_PS_REBOOT", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

        # Redefine initial value for setting USBDIO:
        self.em_core_module.io_card_init_state["BatteryType"] = self.phone_info["BATTERY"]["BATTID_TYPE"]
        self.em_core_module.io_card_init_state["Battery"] = True
        self.em_core_module.io_card_init_state["Platform"] = "ON"
        self.em_core_module.io_card_init_state["USBChargerType"] = self._io_card.USB_HOST_PC

        if self._cable_type == "NONE":
            self.em_core_module.io_card_init_state["USBCharger"] = False
        else:
            self.em_core_module.io_card_init_state["USBChargerType"] = self._cable_type
            self.em_core_module.io_card_init_state["USBCharger"] = True

        # - BatteryTemperature = BATTERY_TEMPERATURE
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

        # Measure current from Vbatt and Vcharger
        meas_icharger = self.em_core_module.get_charger_current(self._io_card.AC_CHGR)
        meas_ibatt = self.em_core_module.get_battery_current()

        self._logger.info("Initial state : IBATT = %s %s - Icharger = %s %s" %
                         (meas_ibatt[0], meas_ibatt[1], meas_icharger[0], meas_icharger[1]))

        # store value in dict for later comparison
        self._meas_list.add("IBATT1", meas_ibatt)
        self._meas_list.add("ICHARGER1", meas_icharger)

        # shutdown sequence
        before_shutdown_time = time.time()
        self.__shutdown(self._shutdown_type.upper())
        # wait for shutdown
        self._logger.info("wait 30s for board shutdown...")
        time.sleep(30)

        # Measure current from Vbatt and Vcharger
        meas_icharger = self.em_core_module.get_charger_current(self._io_card.AC_CHGR)
        meas_ibatt = self.em_core_module.get_battery_current()

        self._logger.info("shutdown state : IBATT = %s %s - Icharger = %s %s" %
                         (meas_ibatt[0], meas_ibatt[1], meas_icharger[0], meas_icharger[1]))

        # store value in dict for later comparison
        self._meas_list.add("IBATT2", meas_ibatt)
        self._meas_list.add("ICHARGER2", meas_icharger)

        # Power On sequence
        # PUSH ON button 3s
        self._io_card.press_power_button(self.pwr_btn_boot)
        time.sleep(self._device.get_boot_timeout())

        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()

        self._logger.info("reboot state : IBATT = %s %s" %
                         (meas_ibatt[0], meas_ibatt[1]))

        # store value in dict for later comparison
        self._meas_list.add("IBATT3", meas_ibatt)

        #  connect USB_HOST_PC
        if self._cable_type in [self._io_card.AC_CHGR, "NONE"]:
            self._io_card.simulate_insertion(self._io_card.USB_HOST_PC)
        self._device.connect_board()

        if self.is_board_and_acs_ok():
            # check for boot reason
            boot_reason = self.phonesystem_api.check_message_in_log("BOOT_REASON",
                                                                     before_shutdown_time, check_result=True)
            # store value in dict for later comparison
            self._meas_list.add("BOOT_REASON", boot_reason[0], "none")
            # check for boot mode
            boot_mode = self._device.get_boot_mode()
            # store value in dict for later comparison
            self._meas_list.add("BOOT_MODE", boot_mode, "none")
        else:
            self._meas_list.add("BOOT_REASON", "CANT GET BOOT REASON", "none")
            self._meas_list.add("BOOT_MODE", "CANT GET BOOT MODE", "none")

        # compare values with targets
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
        self._meas_list.clean()

        # Save data report in xml file
        self._error.Code = self._em_meas_verdict.get_current_result()
        self._error.Msg = self._em_meas_verdict.save_data_report_file()

        return self._error.Code, self._error.Msg

    def __shutdown(self, mode):
        """
        perform shutdown depending of chosen mode.

        :type mode : str
        :param mode: can be SOFT or HARD
        """
        if mode.upper() == "HARD":
            self._logger.info("performing hard shutdown")
            self._io_card.press_power_button(10)
        elif mode.upper() == "SOFT":
            # plug host pc cable before measurement to init the shutdown
            if self._cable_type in [self._io_card.AC_CHGR, "NONE"]:
                self._io_card.simulate_insertion(self._io_card.USB_HOST_PC)
                self._device.connect_board()

            self._logger.info("performing shutdown from UI")
            self.phonesystem_api.set_phone_lock(0)
            self._io_card.press_power_button(2)
            self.phonesystem_api.select_pwoff_menu("ok")
            if self._cable_type == "NONE":
                # - remove HOST PC charger, AC CHARGER should be disconnect since init
                self._io_card.usb_connector(False)
            elif self._cable_type == self._io_card.AC_CHGR:
                # insert AC charger
                self._io_card.usb_connector(False)
                self._io_card.simulate_insertion(self._cable_type)
            # release board connection
            self._device.disconnect_board()
        else:
            tmp_txt = "Unknown mode %s" % str(mode)
            self._logger.error(tmp_txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, tmp_txt)
