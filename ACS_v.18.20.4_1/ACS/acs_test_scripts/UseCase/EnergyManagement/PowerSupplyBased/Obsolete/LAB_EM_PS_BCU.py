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
:summary: Energy Management BCU vbatt drop test
:author: vgomberx
:since: 22/10/2012
"""

import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.EMUtilities import update_conf
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase


class LabEmPsBcu(EmUsecaseBase):

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

        # Read VBATT from TC parameters
        self.__vbatt = float(self._tc_parameters.get_param_value("VBATT"))
        # Read VBATT_DROP from TC parameters
        self.__vbatt_drop = float(self._tc_parameters.get_param_value("VBATT_DROP"))
        # Read VBATT_DROP from TC parameters
        self.__drop_duration = float(self._tc_parameters.get_param_value("DROP_DURATION"))
        # we need to consider the code execution duration
        if self.__drop_duration > 0.35:
            self.__drop_duration -= 0.35
        else:
            self.__drop_duration = 0
        # Read VBATT_DROP from TC parameters
        self.__reboot_after_test = str(self._tc_parameters.
                                       get_param_value("REBOOT_AFTER_TEST"))

        # Redefine initial value for setting USBDIO:
        # - BatteryType
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
        # - VoltageLevel = VBATT
        self.em_core_module.eqp_init_state["BATT"]["VoltageLevel"] = self.__vbatt

        # init the warntype dict
        self.__warntype = {"A": self.__vbatt_drop + 0.1,
                           "B": self.__vbatt_drop + 0.1,
                           "_crit": self.__vbatt_drop + 0.1}

    def set_up(self):
        """
        Initialize the test:

        """
        EmUsecaseBase.set_up(self)

        # we need to reload each time the target because of dynamic change of the target
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_PS_BCU", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

        # Set the brightness mode to manual
        self.phonesystem_api.set_brightness_mode("manual")
        time.sleep(5)

        # set brightness to 50%
        self.phonesystem_api.set_display_brightness(50)
        time.sleep(5)

        # Set screen timeout to 30 minutes
        self.phonesystem_api.set_screen_timeout(60 * 60)
        time.sleep(20)

        # record the all warn voltage value
        # and replace by the new
        for key in self.__warntype:
            self.__warntype[key] = self.em_api.get_bcu_warn_level(key)
            self._logger.info("warn_type%s is %s" % (key, str(self.__warntype[key])))
            self.em_api.set_bcu_warn_level(self.__vbatt_drop + 0.1, key)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        """
        # Call LAB_EM_BASE Run function
        EmUsecaseBase.run_test_body(self)

        # Check the bcu_status1 = 1 (TC 60391_1)
        bcu_alive = self.em_api.get_bcu_activation_status()

        # Compare value with limit parameters
        self._meas_list.add("BCU_ALIVE_BFORE_DROP", bcu_alive, "none")

        # Check BCU interrupt1
        interrupt_1 = self.em_api.get_bcu_interrupt()
        # Set the vibra ON
        self.phonesystem_api.set_vibration("on")
        self.phonesystem_api.set_torchlight("on")

        # Unplug SDP
        self.em_core_module.unplug_charger("SDP")

        # Time_stamp_A here
        stamp_1 = time.time()

        # wait 10s second to balance ibatt
        time.sleep(10)

        # Check (with NIDAC) ibatt_vibra_on_1
        ibatt_1 = self.em_core_module.get_battery_current()
        self._logger.info("ibatt before voltage drop= %s" % str(ibatt_1))

        # adjust limit here
        if self._em_targets["IBATT_DURING_DROP"] is not None:
            update_conf(self._em_targets["IBATT_DURING_DROP"],
                        ["lo_lim", "hi_lim"], ibatt_1[0], "+")
        if self._em_targets["IBATT_AFTER_DROP"] is not None:
            update_conf(self._em_targets["IBATT_AFTER_DROP"],
                        ["lo_lim", "hi_lim"], ibatt_1[0], "*")

        # Set vbatt to 2.8V (should be parameterizable)
        self.em_core_module.pwrs_vbatt.set_current_voltage(self.__vbatt_drop,
                                             self.em_core_module.ps_properties["BATT"]["PortNumber"])
        # Wait 20ms (should be parameterizable)
        time.sleep(self.__drop_duration)

        # Check (with NIDAC) 20mA < ibatt_vibra_off < ibatt_vibra_on_1 - 50 mA (TC 60391_2)
        ibatt_off = self.em_core_module.get_battery_current(iteration=5)
        self._meas_list.add("IBATT_DURING_DROP", ibatt_off)

        # Set vbatt to 3.8 V
        self.em_core_module.pwrs_vbatt.set_current_voltage(self.__vbatt,
                                             self.em_core_module.ps_properties["BATT"]["PortNumber"])
        self._logger.info("ibatt during voltage drop= %s" % str(ibatt_off))

        # wait 10s second to balance ibatt
        time.sleep(10)

        # Check (with NIDAC) 20mA < ibatt_vibra_on_2 == ibatt_vibra_on_1 (TC 60392_1)
        ibatt_2 = self.em_core_module.get_battery_current()
        self._logger.info("ibatt after voltage drop= %s" % str(ibatt_2))
        self._meas_list.add("IBATT_AFTER_DROP", ibatt_2)

        # Plug the SDP
        self.em_core_module.plug_charger("SDP")
        # Time_stamp_B here
        stamp_2 = time.time()

        if not(self.em_core_module.check_board_connection(use_exception=False,
                                           only_reconnect=True)):
            self.em_core_module.reboot_board()

        # Set vibra OFF
        self.phonesystem_api.set_vibration("off")
        self.phonesystem_api.set_torchlight("off")

        # Check VWARNA & VCRIT (TC 60391_3 & 60391_4) in the aplog between time_stamp A and B
        vwarna = self.phonesystem_api.check_message_in_log(
            "BCU_WARNING_A", stamp_1,
            stamp_2)[0]
        self._meas_list.add("VWARNA_SEEN", (vwarna, "none"))
        vcrit = self.phonesystem_api.check_message_in_log(
            "BCU_VCRIT", stamp_1,
            stamp_2)[0]
        self._meas_list.add("VCRIT_SEEN", (vcrit, "none"))

        # Check action_status == d (TC 60391_5)
        action_status = self.em_api.get_bcu_status()
        self._meas_list.add("BCU_ACTION_STATUS_1",
                            action_status,
                            "none")
        # Check BCU interrupt1 < BCU interrupt2 < BCU interrupt1 + 4 (TC 60391_6)
        interrupt_2 = self.em_api.get_bcu_interrupt()
        self._meas_list.add("BCU_INTERRUPT_1", interrupt_2 - interrupt_1, "none")

        if self.__reboot_after_test.lower() == "True":
            # reboot the board for the next test
            self.em_core_module.reboot_board()
        # generate em verdict
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
        self._em_meas_verdict.judge()
        self._meas_list.clean()

        # Save data report in xml file
        self._error.Code = self._em_meas_verdict.get_current_result()
        self._error.Msg = self._em_meas_verdict.save_data_report_file()

        return self._error.Code, self._error.Msg

    def tear_down(self):
        """
        End and dispose the test
        """
        EmUsecaseBase.tear_down(self)

        # restore the original warn voltage value
        for key in self.__warntype:
            self.em_api.set_bcu_warn_level(self.__warntype[key], key)

        return Global.SUCCESS, "No errors"
