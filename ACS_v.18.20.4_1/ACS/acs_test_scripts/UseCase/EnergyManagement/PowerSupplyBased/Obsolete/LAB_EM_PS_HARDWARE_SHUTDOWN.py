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
:summary: Energy Management press shutdown while phone is on Use case
:author: dbatutx
:since: 05/16/2012
"""
import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from ErrorHandling.AcsConfigException import AcsConfigException


class LabEmPsHardwareShutdown(EmUsecaseBase):

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

        # Read BATTERY_TEMPERATURE from TC parameters
        self._batt_temp = \
            int(self._tc_parameters.get_param_value("BATTERY_TEMPERATURE"))

        # Read power button time to press in a HardWare shutdown
        self._HW_shutdown_time = \
            int(self._tc_parameters.get_param_value("HW_SHUTDOWN_PRESS_TIME"))

        # Read the os to be tested
        self._test_os = \
            str(self._tc_parameters.get_param_value("TEST_OS")).upper()

        # Call ConfigsParser to parse Energy_Management
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_PS_HARDWARE_SHUTDOWN", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

        # Redefine initial value for setting USBDIO:
        # - BatteryType = ANALOG
        self.em_core_module.io_card_init_state["BatteryType"] = self.phone_info["BATTERY"]["BATTID_TYPE"]
        # - Battery  = True (inserted)
        self.em_core_module.io_card_init_state["Battery"] = True
        # - Platform = True (ON)
        self.em_core_module.io_card_init_state["Platform"] = "OFF"
        # - USBChargerType = USB_HOST_PC
        self.em_core_module.io_card_init_state["USBChargerType"] = "USB_HOST_PC"
        # - USBCharger = False (removed)
        self.em_core_module.io_card_init_state["USBCharger"] = False
        # - BatteryTemperature = BATTERY_TEMPERATURE
        self.em_core_module.io_card_init_state["BatteryTemperature"] = self._batt_temp

        # Set initial value for setting Power Supply VBATT:
        # - VoltageLevel = VBATT
        self.em_core_module.eqp_init_state["BATT"]["VoltageLevel"] = self.em_core_module.vbatt

        self._MOS_boot_time = 0
        self._before_pos_boot_time = 0

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the tests
        """
        # Call LAB_EM_BASE Run function
        EmUsecaseBase.run_test_body(self)

        if "MOS" in self._test_os:
            # boot the board in MOS
            self._MOS_boot_time = time.time()
            self.em_core_module.boot_board("MOS")
            # Set screen timeout to 30 minutes
            self.phonesystem_api.set_screen_timeout(60 * 60)
            # run mos sequence
            self._run_mos_sequence()

        if "POS" in self._test_os:
            # boot the board in POS
            self._before_pos_boot_time = time.time()
            self.em_core_module.boot_board("POS")
            # run pos sequence
            self._run_pos_sequence()

        if "ROS" in self._test_os:
            # boot the board in POS
            self.em_core_module.boot_board("ROS")
            # run pos sequence
            self._run_ros_sequence()

        if not (("MOS" in self._test_os) or ("POS" in self._test_os) or ("ROS" in self._test_os)):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                   "parameter test_os %s is invalid" % self._test_os)

        # generate em verdict
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
        self._meas_list.clean()

        # Save data report in xml file
        self._error.Code = self._em_meas_verdict.get_current_result()
        self._error.Msg = self._em_meas_verdict.save_data_report_file()

        return self._error.Code, self._error.Msg

    def _run_mos_sequence(self):
        """
        Execute the MOS Hardware Shutdown Sequence
        """
        self._logger.info("MOS HARDWARE shutdown sequence")
        #
        # checking boot sequence
        #
        # get the phone device
        device = self.phonesystem_api.read_phone_device("MOS")
        self._meas_list.add("MOS_DEVICE_MODE", (device, "none"))

        # check for android boot mode
        mode = self._device.get_boot_mode()
        # Compare boot mode value with limit parameters
        self._meas_list.add("MOS_BOOT_MODE", (mode, "none"))

        # wake up the board if necessary
        if not self.phonesystem_api.get_screen_status():
            self._io_card.press_power_button(0.3)

        # unplug USB
        self._device.disconnect_board()
        self._io_card.usb_host_pc_connector(False)
        time.sleep(30)

        # Measure current from Vbatt
        meas_ibatt = (self.em_core_module.pwrs_vbatt.get_current_meas(
            self.em_core_module.ps_properties["BATT"]["PortNumber"], "DC"), "A")
        self._logger.info("        initial state : IBatt1 MOS = %s %s " %
                         (meas_ibatt[0], meas_ibatt[1]))
        # Compare ibatt value with limit parameters
        self._meas_list.add("MOS_IBATT1", meas_ibatt)

        #
        # push 5 s sequence
        #
        # PUSH power button 5s for shutdown the board
        self._logger.info("        press on 5s")
        graceful_sh_time = time.time()
        self._io_card.press_power_button(5)
        time.sleep(40)

        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()
        self._logger.info("        board should be off : IBatt2 MOS = %s %s " %
                         (meas_ibatt[0], meas_ibatt[1]))
        # Compare ibatt value with limit parameters
        self._meas_list.add("MOS_IBATT2", meas_ibatt)
        result = self._em_meas_verdict.test_value(self._meas_list,
                                                  self._em_targets["MOS_IBATT2"])
        # boot the board if the result is true
        if result:
            # boot the board
            self.em_core_module.boot_board()

            # check shutdown confirmation
            result = self.phonesystem_api.check_message_in_log("SHUTDOWN_REASON",
                                                                graceful_sh_time, check_result=True)
            # Compare shutdown reason with limit parameters
            self._meas_list.add("MOS_HARDWARE_SHUTDOWN_0", (result[0], 'none'))
            # UNPLUG usb
            self._device.disconnect_board()
            self._io_card.usb_host_pc_connector(False)

        #
        # push 7.2s sequence
        #
        # PUSH power button 7.2s for doing nothing
        # freeze the board before in order to avoid a graceful shutdown betwwen 4s and 7s
        self.em_core_module.plug_charger("SDP")
        self._device.connect_board()
        self.phonesystem_api.freeze_board("SOFT")
        time.sleep(5)
        self._device.disconnect_board()
        self.em_core_module.unplug_charger("SDP")
        # do the Hardware shutdown
        self._logger.info("        press on 7.2s")
        self._io_card.press_power_button(7.2)
        time.sleep(2)

        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()
        self._logger.info("        board off : IBatt4 MOS = %s %s " %
                         (meas_ibatt[0], meas_ibatt[1]))
        # Compare ibatt value with limit parameters
        self._meas_list.add("MOS_IBATT4", meas_ibatt)
        result = self._em_meas_verdict.test_value(self._meas_list,
                                                  self._em_targets["MOS_IBATT4"])
        if not result:
            # disconnect battery
            self._io_card.battery_connector(False)
            time.sleep(3)
            # connect battery
            self._io_card.battery_connector(True)
            time.sleep(3)

        #
        # Power on sequence
        #
        # boot the board
        before_boot_time = time.time()
        self._logger.info("        press on %ss" % self.pwr_btn_boot)
        self._io_card.press_power_button(self.pwr_btn_boot)
        time.sleep(60)
        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()
        self._logger.info("        state after 'HW shutdown' : IBatt5 = %s %s " %
                         (meas_ibatt[0], meas_ibatt[1]))
        # Compare ibatt value with limit parameters
        self._meas_list.add("MOS_IBATT5", meas_ibatt)
        result = self._em_meas_verdict.test_value(self._meas_list,
                                                  self._em_targets["MOS_IBATT5"])
        after_boot_time = time.time()
        if not result:
            self.em_core_module.reboot_board()

        #  connect USB SDP
        self.em_core_module.check_board_connection(only_reconnect=True)
        time.sleep(5)
        # check shutdown confirmation
        result = self.phonesystem_api.check_message_in_log("SHUTDOWN_REASON",
                                                            before_boot_time, after_boot_time, True)
        # Compare shutdown confirmation value with limit parameters
        self._meas_list.add("MOS_HARDWARE_SHUTDOWN", (result[0], "none"))
        # check for android boot wake src
        result = self.phonesystem_api.check_message_in_log("BOOT_REASON",
                                                            before_boot_time, after_boot_time, True)
        # Compare Vbatt value with limit parameters
        self._meas_list.add("MOS_BOOT_REASON", (result[0], "none"))
        # check for android boot mode
        result = self.phonesystem_api.check_message_in_log("BOOT_MODE",
                                                            before_boot_time, after_boot_time, True)
        # Compare boot mode value with limit parameters
        self._meas_list.add("MOS_BOOT_MODE", (result[0], "none"))

        #
        # push 11s sequence
        #
        # wake up the board
        time.sleep(2)
        self.em_core_module.unplug_charger("SDP")
        # PUSH power button 11s for shutdown the board
        self._logger.info("        press on 11s")
        self._io_card.press_power_button(11)
        time.sleep(40)

        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()
        self._logger.info("        board should be off : IBatt6 MOS = %s %s " %
                         (meas_ibatt[0], meas_ibatt[1]))
        # Compare ibatt value with limit parameters
        self._meas_list.add("MOS_IBATT6", meas_ibatt)

        # reboot the board in MOS for the next test
        self.em_core_module.reboot_board()

    def _run_pos_sequence(self):
        """
        Execute the POS Hardware Shutdown Sequence
        """
        self._logger.info("POS HARDWARE shutdown sequence")
        #
        # checking boot sequence
        #
        # get the phone device
        device = self.phonesystem_api.read_phone_device("POS")

        self._meas_list.add("POS_DEVICE_MODE", (device, "none"))

        # unplug USB
        self._device.disconnect_board()
        self._io_card.usb_host_pc_connector(False)

        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()
        self._logger.info("        initial state : IBatt1 POS = %s %s " %
                         (meas_ibatt[0], meas_ibatt[1]))
        # Compare ibatt value with limit parameters
        self._meas_list.add("POS_IBATT1", meas_ibatt)
        after_pos_boot_time = time.time()

        #
        # push 5 s sequence
        #
        # PUSH power button 5s for doing nothing
        self._logger.info("        press on 5s")
        self._io_card.press_power_button(5)
        time.sleep(40)

        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()
        self._logger.info("        board should be on : IBatt2 POS = %s %s " %
                         (meas_ibatt[0], meas_ibatt[1]))
        # Compare ibatt value with limit parameters
        self._meas_list.add("POS_IBATT2", meas_ibatt)
        result = self._em_meas_verdict.test_value(self._meas_list,
                                                  self._em_targets["POS_IBATT2"])
        # boot the board if the board is not on
        if not result:
            self.em_core_module.boot_board("POS")
            self._device.disconnect_board()
            self._io_card.usb_host_pc_connector(False)
        #
        # push 6s sequence
        #
        # PUSH power button 6s for doing nothing
        self._logger.info("        press on 6s")
        self._io_card.press_power_button(6)
        time.sleep(40)

        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()
        self._logger.info("       board should be on : IBatt3 POS = %s %s " %
                         (meas_ibatt[0], meas_ibatt[1]))
        # Compare ibatt value with limit parameters
        self._meas_list.add("POS_IBATT3", meas_ibatt)
        result = self._em_meas_verdict.test_value(self._meas_list,
                                                  self._em_targets["POS_IBATT3"])
        # boot the board if the board is not on
        if not result:
            self.em_core_module.boot_board("POS")
            self._device.disconnect_board()
            self._io_card.usb_host_pc_connector(False)

        #
        # push 7.2s sequence
        #
        # PUSH power button 7.2s for doing nothing
        self._logger.info("        press on 7.2s")
        self._io_card.press_power_button(7.2)
        time.sleep(2)

        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()
        self._logger.info("        board should be off : IBatt POS = %s %s " %
                         (meas_ibatt[0], meas_ibatt[1]))
        # Compare ibatt value with limit parameters
        self._meas_list.add("POS_IBATT4", meas_ibatt)
        result = self._em_meas_verdict.test_value(self._meas_list,
                                                  self._em_targets["POS_IBATT4"])
        if not result:
            # disconnect battery
            self._io_card.battery_connector(False)
            time.sleep(3)
            # connect battery
            self._io_card.battery_connector(True)
            time.sleep(3)

        #
        # Power on sequence
        #
        # boot the board
        before_boot_time = time.time()
        self._logger.info("        press on %ss" % self.pwr_btn_boot)
        self._io_card.press_power_button(3)
        time.sleep(60)
        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()
        self._logger.info("        state after 'HW shutdown' : IBatt5 POS = %s %s " %
                         (meas_ibatt[0], meas_ibatt[1]))
        # Compare ibatt value with limit parameters
        self._meas_list.add("POS_IBATT5", meas_ibatt)
        result = self._em_meas_verdict.test_value(self._meas_list,
                                                  self._em_targets["POS_IBATT5"])
        after_boot_time = time.time()
        if not result:
            self.em_core_module.reboot_board()

        #  connect USB SDP
        self.em_core_module.check_board_connection(only_reconnect=True)
        time.sleep(5)
        # check shutdown confirmation
        result = self.phonesystem_api.check_message_in_log("SHUTDOWN_REASON",
                                                            before_boot_time, after_boot_time, True)
        # Compare shutdown confirmation value with limit parameters
        self._meas_list.add("POS_HARDWARE_SHUTDOWN", (result[0], "none"))
        # check for android boot wake src
        result = self.phonesystem_api.check_message_in_log("BOOT_REASON",
                                                            before_boot_time, after_boot_time, True)
        # Compare Vbatt value with limit parameters
        self._meas_list.add("POS_BOOT_REASON", (result[0], "none"))
        # check for android boot mode
        result = self.phonesystem_api.check_message_in_log("BOOT_MODE",
                                                            self._before_pos_boot_time, after_pos_boot_time, True)
        # Compare boot mode value with limit parameters
        self._meas_list.add("POS_BOOT_MODE1", (result[0], "none"))
        # check for android boot mode
        result = self.phonesystem_api.check_message_in_log("BOOT_MODE",
                                                            before_boot_time, after_boot_time, True)
        # Compare boot mode value with limit parameters
        self._meas_list.add("POS_BOOT_MODE2", (result[0], "none"))

        #
        # push 11s sequence
        #
        # wake up the board
        self.phonesystem_api.wake_screen()
        time.sleep(2)

        # unplug the SDP
        self.em_core_module.unplug_charger("SDP")
        # PUSH power button 11s for doing nothing
        self._logger.info("        press on 11s")
        self._io_card.press_power_button(11)
        time.sleep(40)

        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()
        self._logger.info("        board should be on : IBatt6 POS = %s %s " %
                         (meas_ibatt[0], meas_ibatt[1]))
        # Compare ibatt value with limit parameters
        self._meas_list.add("POS_IBATT6", meas_ibatt)

        # reboot the board in MOS for the next test
        self.em_core_module.reboot_board()

    def _run_ros_sequence(self):
        """
        Execute the ROS Hardware Shutdown Sequence
        """
        self._logger.info("ROS HARDWARE shutdown sequence")
        #
        # checking boot sequence
        #
        time.sleep(70)
        # get the phone device
        device = self.phonesystem_api.read_phone_device("ROS")

        self._meas_list.add("ROS_DEVICE_MODE", (device, "none"))
        # Compare boot mode value with limit parameters
        self._meas_list.add("ROS_BOOT_MODE1", (self._device.get_boot_mode(), "none"))
        # unplug USB
        self._device.disconnect_board()
        self._io_card.usb_host_pc_connector(False)

        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()
        self._logger.info("        initial state : IBatt1 ROS = %s %s " %
                         (meas_ibatt[0], meas_ibatt[1]))
        # Compare ibatt value with limit parameters
        self._meas_list.add("ROS_IBATT1", meas_ibatt)

        #
        # push 5 s sequence
        #
        # PUSH power button 5s for doing nothing
        self._logger.info("        press on 5s")
        self._io_card.press_power_button(5)
        time.sleep(40)

        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()
        self._logger.info("        board should be on : IBatt2 ROS = %s %s " %
                         (meas_ibatt[0], meas_ibatt[1]))
        # Compare ibatt value with limit parameters
        self._meas_list.add("ROS_IBATT2", meas_ibatt)
        result = self._em_meas_verdict.test_value(self._meas_list,
                                                  self._em_targets["ROS_IBATT2"])
        # boot the board if the board is not on
        if not result:
            self._device.reboot("ROS")
            self._device.disconnect_board()
            self._io_card.usb_host_pc_connector(False)
        #
        # push 6s sequence
        #
        # PUSH power button 6s for doing nothing
        self._logger.info("        press on 6s")
        self._io_card.press_power_button(6)
        time.sleep(40)

        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()
        self._logger.info("       board should be on : IBatt3 ROS = %s %s " %
                         (meas_ibatt[0], meas_ibatt[1]))
        # Compare ibatt value with limit parameters
        self._meas_list.add("ROS_IBATT3", meas_ibatt)
        result = self._em_meas_verdict.test_value(self._meas_list,
                                                  self._em_targets["ROS_IBATT3"])
        # boot the board if the board is not on
        if not result:
            self._device.reboot("ROS")
            self._device.disconnect_board()
            self._io_card.usb_host_pc_connector(False)

        #
        # push 7.2s sequence
        #
        # PUSH power button 7.2s for an hardware shutdown
        self._logger.info("        press on 7.2s")
        self._io_card.press_power_button(7.2)
        time.sleep(2)

        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()
        self._logger.info("        state after 'HW shutdown'  : IBatt4 ROS = %s %s " %
                         (meas_ibatt[0], meas_ibatt[1]))
        # Compare ibatt value with limit parameters
        self._meas_list.add("ROS_IBATT4", meas_ibatt)
        result = self._em_meas_verdict.test_value(self._meas_list,
                                                  self._em_targets["ROS_IBATT4"])
        if not result:
            # disconnect battery
            self._io_card.battery_connector(False)
            time.sleep(3)
            # connect battery
            self._io_card.battery_connector(True)
            time.sleep(3)

        #
        # Power on sequence
        #
        # boot the board
        before_boot_time = time.time()
        self._logger.info("        press on 3s")
        self._io_card.press_power_button(3)
        time.sleep(30)
        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()
        self._logger.info("        board should be on: IBatt5 ROS = %s %s " %
                         (meas_ibatt[0], meas_ibatt[1]))
        # Compare ibatt value with limit parameters
        self._meas_list.add("ROS_IBATT5", meas_ibatt)
        after_boot_time = time.time()
        # plug SDP
        self.em_core_module.plug_charger("SDP")
        time.sleep(70)
        # Compare boot mode value with limit parameters
        self._meas_list.add("ROS_BOOT_MODE2", (self._device.get_boot_mode(), "none"))
        # unplug SDP
        self.em_core_module.unplug_charger("SDP")

        #
        # push 11s sequence
        #
        time.sleep(2)
        # PUSH power button 11s for doing nothing
        self._logger.info("        press on 11s")
        self._io_card.press_power_button(11)
        time.sleep(40)

        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()
        self._logger.info("        board should be off : IBatt6 ROS = %s %s " %
                         (meas_ibatt[0], meas_ibatt[1]))
        # Compare ibatt value with limit parameters
        self._meas_list.add("ROS_IBATT6", meas_ibatt)

        # reboot the board in MOS for the next test
        self._io_card.press_power_button(3)
        time.sleep(30)
        # plug SDP
        self.em_core_module.plug_charger("SDP")
        time.sleep(70)
        # reboot in MOS
        self._device.reboot("MOS")
        # Compare boot reason value with limit parameters
        result = self.phonesystem_api.check_message_in_log("BOOT_REASON_IRQ",
                                                            before_boot_time, after_boot_time, True)
        self._meas_list.add("ROS_BOOT_REASON", (result[0], "none"))
        # check shutdown confirmation
        result = self.phonesystem_api.check_message_in_log("SHUTDOWN_REASON",
                                                            before_boot_time, after_boot_time, True)
        # Compare shutdown confirmation value with limit parameters
        self._meas_list.add("ROS_HARDWARE_SHUTDOWN", (result[0], "none"))

    def tear_down(self):
        """
        End and dispose the test
        """
        self._logger.info("[HARDWARE_SHUTDOWN] Tear Down")

        if self._device.get_boot_mode() != "MOS":
            # disconnect battery
            self._io_card.battery_connector(False)
            time.sleep(5)

            # connect battery
            self._io_card.battery_connector(True)
            time.sleep(0.3)

            self._logger.info("        press on 3s")
            self._io_card.press_power_button(3)
            time.sleep(60)

            # plug SDP
            self.em_core_module.plug_charger("SDP")

            # reboot in MOS
            self._device.reboot("MOS")

        # execute base tear down
        EmUsecaseBase.tear_down(self)

        return Global.SUCCESS, "No errors"
