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
:summary: Energy Management Thermal Temperature protection Use case
:author: apairex / dbatutx
:since: 08/31/2011
"""
import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase


class LabEmThermalTempProtection(EmUsecaseBase):

    """
    Lab Energy Management class.
    """
    DEDICATED_BENCH = "POWER_SUPPLY_BENCH"

    def __init__(self, tc_name, global_config):

        # Call LAB_EM_BASE Init function
        EmUsecaseBase.__init__(self, tc_name, global_config)

        # get thermal poll delay
        self._poll_delay = self._tc_parameters.get_param_value(
            "POLL_DELAY")

        if self._poll_delay is not None and self._poll_delay != "":
            self._poll_delay = float(self._poll_delay)
        else:
            self._poll_delay = 30

        # Call ConfigsParser to parse Energy_Management
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_THERMAL_TEMP_PROTECTION", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

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
        self.em_core_module.io_card_init_state["USBCharger"] = True
        # - BatteryTemperature
        self.em_core_module.io_card_init_state["BatteryTemperature"] = 25

        # Set initial value for setting Power Supply VBATT:
        # - VoltageLevel
        self.em_core_module.eqp_init_state["BATT"]["VoltageLevel"] = self.em_core_module.vbatt

        # get the power supply once
        self.em_core_module.pwrs_vbatt = self._em.get_power_supply("BATT")

        self._thermal_dict = {}

    def set_up(self):
        """
        Initialize the test:
        Create 3 txt files for the 3 temperature zone value
        Change the thermal configuration file to point on these 3 files
        """
        EmUsecaseBase.set_up(self)

        # build a fake thermal conf
        self.em_api.build_fake_thermal_conf()

        # Set the brightness mode to manual
        self.phonesystem_api.set_brightness_mode("manual")
        time.sleep(5)

        # set brightness to 100%
        self.phonesystem_api.set_display_brightness(100)
        time.sleep(5)

        # Set screen timeout to 30 minutes
        self.phonesystem_api.set_screen_timeout(60 * 60)
        time.sleep(20)

        # Reboots the phone
        self.em_core_module.reboot_board()

        # replug usb
        self.em_core_module.unplug_charger("SDP")
        self.em_core_module.plug_charger("SDP")
        self.em_core_module.check_board_connection(use_exception=False,
                                    only_reconnect=True)

        # get thermal info
        self._thermal_dict = self.em_api.get_thermal_sensor_info()

        return Global.SUCCESS, "No errors"

    def run_test_body(self):
        """
        Execute the test
        """
        # Call LAB_EM_BASE Init function
        EmUsecaseBase.run_test_body(self)

        #
        # BACKSKIN -> 75 deg
        # ------------
        self._logger.debug("[THERMAL] TEST BACKSKIN 75deg")
        # set zone1 to 75000
        temp = self._thermal_dict[self._em_cst.BACKSKIN][self._em_cst.THRESHOLD_ALERT][0] * 1000
        self.em_api.thermal_test_set_temp(self._em_cst.BACKSKIN, temp)
        # Disconnect cable
        self._device.disconnect_board()
        self._io_card.usb_connector(False)
        time.sleep(2)
        self.__wait_for_shut_down("BACKSKIN set to %s millideg" % temp,
                                  "IBATT1")
        # Reboots the phone
        self.em_core_module.reset_thermal_critical_state(self._em_cst.BACKSKIN)
        time.sleep(10)
        self._device.connect_board()

        #
        # FRONTSKIN -> 30deg / Z2 -> 64deg
        # -------------------------
        self._logger.debug("[THERMAL] TEST FRONTSKIN 30deg - Z2 64deg")
        temp = self._thermal_dict[self._em_cst.FRONTSKIN][self._em_cst.THRESHOLD_ALERT][0] * 1000
        self.em_api.thermal_test_set_temp(self._em_cst.FRONTSKIN, temp)
        # Disconnect cable
        self._device.disconnect_board()
        self._io_card.usb_connector(False)
        time.sleep(2)
        self.__wait_for_shut_down("FRONTSKIN set to %s - Z2 set to %s" % (30000, temp),
                                  "IBATT2")
        # Reboots the phone
        self.em_core_module.reset_thermal_critical_state(self._em_cst.FRONTSKIN)
        time.sleep(10)
        self._device.connect_board()

        #
        # DTS max frequency
        # -----------------
        self._logger.debug("[THERMAL] TEST DTS max frequency")
        self.__wait_for_DTS_freq("Sensors back to 30 deg", "CPUFREQ1")
        temp = self._thermal_dict[self._em_cst.DTS][self._em_cst.THRESHOLD_ALERT][0] * 1000
        self.em_api.thermal_test_set_temp(self._em_cst.DTS, temp)
        self.__wait_for_DTS_freq("DTS set to %s" % temp, "CPUFREQ2")
        self.em_api.thermal_test_set_temp(self._em_cst.DTS, 30000)
        time.sleep(2)
        # Disconnect cable
        self._device.disconnect_board()
        self._io_card.usb_connector(False)
        time.sleep(2)

        #
        # Battery temperature
        # -------------------
        temperature = "70"
        self._logger.debug("[THERMAL] TEST Battery temperature")
        self._io_card.set_battery_temperature(temperature)

        self.__wait_for_shut_down("Batt temp set to %s" % temperature,
                                  "IBATT3")
        temperature = "25"
        self._io_card.set_battery_temperature(temperature)

        # compare values with targets
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
        self._meas_list.clean()

        # Save data report in xml file
        self._error.Code = self._em_meas_verdict.get_current_result()
        self._error.Msg = self._em_meas_verdict.save_data_report_file()
        self._logger.debug("[THERMAL] Thermal test end")
        return self._error.Code, self._error.Msg

    def tear_down(self):
        """
        End and dispose the test
        """
        self._logger.info("[THERMAL] Tear Down")

        try:
            # Reboots the phone
            self.em_core_module.reboot_board()
            # restore the real thermal conf
            self._device.set_filesystem_rw()
            self.em_api.restore_thermal_file()

            # connect board
            self.em_core_module.reboot_board()
            # Set the brightness mode to manual
            self.phonesystem_api.set_brightness_mode("automatic")
            time.sleep(20)
        finally:
            EmUsecaseBase.tear_down(self)

        return Global.SUCCESS, "No errors"

# ---------------------------------------
    def __wait_for_shut_down(self, log_info, target_ref):
        """
        Function that checks the board to shutdown.
        It is used to determine that the Critical state has been reached
        After 120 seconds timeout the process returns fail

        :type  log_info: str
        :param log_info: str that will be written in the periodic log
        :type  log_info: str
        :param log_info: Index of the _em_target array to get the specific value
                         to reach for the DTS
        """
        # Start 120s timer
        origin = time.time()
        current = origin
        timeout = self._poll_delay * 4
        criteria = 0.006

        while origin + timeout > current:
            # Power Supply current control
            meas_ibatt = (self.em_core_module.pwrs_vbatt.get_current_meas(
                self.em_core_module.ps_properties["BATT"]["PortNumber"], "DC"), "A")

            self._logger.info("[THERMAL] %s, %s sec ago, IBatt = %s %s" %
                              (log_info, current - origin,
                               meas_ibatt[0], meas_ibatt[1]))

            if meas_ibatt[0] < criteria:
                break

            time.sleep(10)
            current = time.time()

        # PowerSupply current control and Timer control
        self._meas_list.add(target_ref, meas_ibatt)

    def __wait_for_DTS_freq(self, log_info, target_ref):
        """
        Function that checks the DTS freq value to reach a specific value
        After 120 seconds timeout the process returns fail

        :type  log_info: str
        :param log_info: str that will be written in the periodic log
        :type  log_info: str
        :param log_info: Index of the _em_target array to get the specific value
                         to reach for the DTS
        """
        # Start 120s timer
        origin = time.time()
        current = origin
        timeout = self._poll_delay * 6

        criteria = float(self._em_targets[target_ref]["value"])

        while origin + timeout > current:

            max_DTS_freq = self.phonesystem_api.get_max_cpu_freq(0)

            self._logger.info("[THERMAL] %s, %s sec ago, FREQ = %s Hz" %
                              (log_info, str(current - origin), max_DTS_freq))

            if float(max_DTS_freq) == float(criteria):
                # wait more than on poll_delay in order to detect a lower case
                time.sleep(self._poll_delay + 5)
                max_DTS_freq = self.phonesystem_api.get_max_cpu_freq(0)
                break

            time.sleep(10)
            current = time.time()

        # PowerSupply current control and Timer control
        self._meas_list.add(target_ref, (max_DTS_freq, "Hz"))
