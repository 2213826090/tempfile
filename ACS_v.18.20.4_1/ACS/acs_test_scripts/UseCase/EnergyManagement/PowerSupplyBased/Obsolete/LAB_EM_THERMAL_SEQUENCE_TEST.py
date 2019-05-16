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
:summary: Energy Management Thermal Sequence Test UseCase.
          The user can define a sequence of critical shutdown test on different
          zones and a sequence of threshold/zones to test
:author: dbatutx
:since: 12/02/2013
"""
import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from acs_test_scripts.Utilities.EMUtilities import EMConstant as CST


class LabEmThermalSequenceTest(EmUsecaseBase):

    """
    Lab Energy Management class.
    """
    DEDICATED_BENCH = "POWER_SUPPLY_BENCH"

    def __init__(self, tc_name, global_config):

        # Call LAB_EM_BASE Init function
        EmUsecaseBase.__init__(self, tc_name, global_config)

        # get thermal poll delay
        self._poll_delay = int(self._tc_parameters.get_param_value(
            "POLL_DELAY", 30))

        # Read OS from TC parameters
        self._os = self._tc_parameters.get_param_value("OS", "MOS")

        # additionnal degrees after threshold
        self._add_degrees = \
            int(self._tc_parameters.get_param_value("ADD_DEGREES", "1")) * 1000

        # Read zone to test on critical threshold from TC parameters
        self._zone_on_critical_list = str(
            self._tc_parameters.get_param_value("ZONE_ON_CRITICAL")).split(";")

        # Read ZONE_THRESHOLD from TC parameters
        self._zone_threshold_list = str(
            self._tc_parameters.get_param_value("ZONE_ON_THRESHOLD")).split(";")

        self._logger.info("[THERMAL] critical thermal shutdown will be test " +
                          "on this ZONE: %s" % str(self._zone_on_critical_list))
        self._logger.info("[THERMAL]  thermal test will be done on these tuple "
                          + "ZONE/THRESHOLD: %s" % str(self._zone_threshold_list))

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

        self._thermal_dict = {}

    def set_up(self):
        """
        Initialize the test:
        change the thermal managament config file by a fake file.
        change the screen config
        """
        EmUsecaseBase.set_up(self)

        # Call ConfigsParser to parse Energy_Management
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_THERMAL_SEQUENCE_TEST", self._tc_parameters.get_params_as_dict(), self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

        # build a fake thermal conf
        self.em_api.build_fake_thermal_conf()
        # Set the brightness mode to manual
        self.phonesystem_api.set_brightness_mode("manual")
        time.sleep(5)
        # set brightness to 100%
        self.phonesystem_api.set_display_brightness(100)
        time.sleep(5)
        # Set screen timeout very high
        self.phonesystem_api.set_screen_timeout(60 * 60)
        time.sleep(20)
        # get thermal info
        self._thermal_dict = self.em_api.get_thermal_sensor_info()
        # Reboots the phone
        self._device.reboot(self._os)

        return Global.SUCCESS, "No errors"

    def run_test_body(self):
        """
        Execute the test
        """
        # Call LAB_EM_BASE Init function
        EmUsecaseBase.run_test_body(self)

        # run the test:
        self.__run_zone_to_threshold_test()
        self.__run_critical_shutdown_test()

        # compare values with targets
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
        self._em_meas_verdict.judge(ignore_blocked_tc=True)
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
            self._device.reboot("MOS")
            # restore the real thermal conf
            self._device.set_filesystem_rw()
            self.em_api.restore_thermal_file()
            # connect board
            self._device.reboot("MOS")
            # Set the brightness mode to manual
            self.phonesystem_api.set_brightness_mode("automatic")
            time.sleep(20)
        finally:
            EmUsecaseBase.tear_down(self)
        return Global.SUCCESS, "No errors"

    def __critical_shutdown_test(self, zone, meas_nb):
        """
        This function test the critical shutdown due to an over temparature on a zone.
        It is used to determine that the Critical threshold has been reached.

        :type  zone: str
        :param zone: the zone to test

        :type  meas_nb: str
        :param meas_nb: the number of the measure needed for the target file
        """
        # set zone1 to 75000
        key = self._em_cst.THRESHOLD_CRITICAL
        temp = self._thermal_dict[zone][key][0] * 1000 + self._add_degrees
        self.em_api.thermal_test_set_temp(zone, temp)
        self._logger.info("[THERMAL] TEST %s at %s deg (over %s threshold)"
                          % (zone, temp, str(self._em_cst.THRESHOLD_CRITICAL)))
        # launch criteria
        self._logger.info("[THERMAL] launch measure %s" % ("IBATT_CRIT_"
                                                           + self._os + "_" + zone + "_" + meas_nb))
        criteria_hi = float(self._em_targets["IBATT_CRIT_" + self._os + "_"
                                             + zone + "_" + meas_nb]["hi_lim"])
        criteria_lo = float(self._em_targets["IBATT_CRIT_" + self._os + "_"
                                             + zone + "_" + meas_nb]["lo_lim"])
        # Start 120s timer
        origin = time.time()
        current = origin
        timeout = self._poll_delay * 4
        # wait for board extinction
        while origin + timeout > current:
            # Power Supply current control
            meas_ibatt = self.em_core_module.get_battery_current()

            self._logger.info("[THERMAL] wait for critical shutdown on" +
                              "%s (%s sec ago, IBatt = %s %s)" % (zone, str(current - origin),
                                                                  str(meas_ibatt[0]), str(meas_ibatt[1])))
            # check criteria
            if criteria_hi > meas_ibatt[0] > criteria_lo:
                time.sleep(10)
                # check one more time to be sure
                if criteria_hi > meas_ibatt[0] > criteria_lo:
                    break
            time.sleep(10)
            time_before_shutdown = time.time()
            current = time.time()
        # PowerSupply current control and Timer control
        self._meas_list.add("IBATT_CRIT_" + self._os + "_" + zone + "_"
                            + meas_nb, meas_ibatt)
        # Reset the phone state
        self.em_core_module.reset_thermal_critical_state(zone, os_to_reboot=self._os)
        time.sleep(10)
        # check for shutdown  reason
        shutdown_reason = self.phonesystem_api.check_message_in_log(
            "SHUTDOWN_REASON", time_before_shutdown, time_before_shutdown +
            50, check_result=True)
        # store value in dict for later comparison
        self._meas_list.add("SHUTDOWN_REASON_" + self._os + "_" + zone + "_"
                            + meas_nb, shutdown_reason[0], "none")

    def __zone_to_threshold_test(self, zone, threshold, meas_nb):
        """
        This test put a thermal 'zone' in a determined 'threshold'
        It can measure the frequencies, the brightness and the charge level

        :type  zone: str
        :param zone: the zone to test

        :type  threshold: str
        :param threshold: the threshold to test

        :type  meas_nb: str
        :param meas_nb: the number of the measure needed for the target file
        """
        # check if we shall do specific MOS action,
        if self._device.get_boot_mode() == "MOS":
            # wake up the screen
            if not self.phonesystem_api.get_screen_status():
                self._io_card.press_power_button(0.3)
        # set zone1 to 75000
        key = "THRESHOLD_" + threshold
        temp = self._thermal_dict[zone][key][0] * 1000 + self._add_degrees
        self.em_api.thermal_test_set_temp(zone, temp)
        self._logger.info("[THERMAL] TEST %s at %sdeg (%s threshold)"
                          % (zone, temp, threshold))
        # get expected criteria
        self._logger.info("[THERMAL] launch measure %s" % ("FREQ_" +
                                                           threshold + "_" + self._os + "_" + zone + "_" + meas_nb))
        criteria = float(self._em_targets["FREQ_" + threshold + "_"
                                          + self._os + "_" + zone + "_" + meas_nb]["value"])
        # Start 120s timer
        origin = time.time()
        current = origin
        timeout = self._poll_delay * 6
        # wait for expected frequency before timeout
        while origin + timeout > current:
            # get the CPU frequency
            max_dts_freq = self.phonesystem_api.get_max_cpu_freq(0)
            self._logger.info("[THERMAL] wait for frequency %s on threshold %s"
                              % (str(criteria), threshold) +
                              " on zone %s (%s sec ago, FREQ = %s Hz)" %
                              (zone, str(current - origin), str(max_dts_freq)))
            # check criteria
            if int(max_dts_freq) == int(criteria):
                # wait more than on poll_delay in order to detect a lower case
                time.sleep(self._poll_delay + 5)
                max_dts_freq = self.phonesystem_api.get_max_cpu_freq(0)
                break
            time.sleep(10)
            current = time.time()
        # get charger level
        c_level = self.em_api.get_charger_level()
        # check if we shall do specific MOS action,
        if self._device.get_boot_mode() == "MOS":
            # switch on/off the screen
            if self.phonesystem_api.get_screen_status():
                self._io_card.press_power_button(0.3)
                time.sleep(2)
                self._io_card.press_power_button(0.3)
                time.sleep(1)
        bl_level = self.phonesystem_api.get_backlight_level()
        # measures
        self._meas_list.add("FREQ_" + threshold + "_" + self._os
                            + "_" + zone + "_" + meas_nb, (max_dts_freq, "Hz"))
        self._meas_list.add("CL_" + threshold + "_" + self._os
                            + "_" + zone + "_" + meas_nb, (c_level, "none"))
        self._meas_list.add("BL_" + threshold + "_" + self._os
                            + "_" + zone + "_" + meas_nb, (bl_level, "none"))

    def __run_zone_to_threshold_test(self):
        """
        parse an sequence all zone_to_threshold test
        """
        #
        # run  thermal general test
        #
        for zone_threshold in self._zone_threshold_list:
            if zone_threshold == '':
                continue
            # split the measure
            try:
                # special split for the threshold 'TM_OFF' because of the '_'
                if CST.TM_OFF in zone_threshold:
                    (zone, threshold_part1, threshold_part2,
                     meas_nb) = zone_threshold.split("_")
                    threshold = threshold_part1 + "_" + threshold_part2
                else:
                    (zone, threshold, meas_nb) = zone_threshold.split("_")
            except ValueError:
                # check if the measure is not empty

                self._logger.warning("[THERMAL] the zone/measure %s "
                                     % zone_threshold + "is not supported on zone/threshold test")
                continue
            # check the validity of the zone
            if zone not in [CST.BACKSKIN, CST.FRONTSKIN, CST.DTS,
                            CST.GPUSKIN, CST.BATTERY]:
                self._logger.warning("[THERMAL] the zone %s" % zone
                                     + " is not supported")
                continue
            # check the validity of the threshold
            if threshold not in [CST.TM_OFF, CST.NORMAL, CST.WARNING,
                                 CST.ALERT, CST.CRITICAL]:
                self._logger.warning("[THERMAL] the threshold %s" % threshold
                                     + " is not supported")
                continue
            # check if the test exist in the target file
            target = "FREQ_" + threshold + "_" + self._os + "_" + zone + \
                                                            "_" + meas_nb
            if target not in self._em_targets:
                self._logger.warning("[THERMAL] the measure " +
                                     "%s" % target + " is not available in the target" +
                                     " file! This test on the zone %s"
                                     % zone + " will not be executed!")
                continue
            # run the test                      )
            self.__zone_to_threshold_test(zone, threshold, meas_nb)

    def __run_critical_shutdown_test(self):
        """
        parse an sequence all zone_to_threshold test
        """
        #
        # run critical thermal shutdown test
        #
        for zone_measnb in self._zone_on_critical_list:
            if zone_measnb == '':
                continue
            # split the measure
            try:
                (zone, meas_nb) = zone_measnb.split("_")
            except ValueError:
                # check if the measure is not empty
                self._logger.warning("[THERMAL] the zone/measure %s "
                                     % zone_measnb + "is not supported on critical test")
                continue
            # check the validity of the zone
            if zone not in [CST.TM_OFF, CST.BACKSKIN, CST.FRONTSKIN,
                            CST.DTS, CST.GPUSKIN, CST.BATTERY]:
                self._logger.warning("[THERMAL] the zone %s" % zone +
                                     " is not supported")
                continue
            # check if the test exist in the target file
            if "IBATT_CRIT_" + self._os + "_" + zone + "_" + meas_nb not in self._em_targets:
                self._logger.warning("[THERMAL] the measure   is not%s"
                                     % ("IBATT_CRIT_" + self._os + "_" + zone + "_" + meas_nb)
                                     + " available in the target file! This test on the zone %s"
                                     % zone + " will not be executed!")
                continue
            self.__critical_shutdown_test(zone, meas_nb)
