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
:summary: Energy Management battery temperature monitor Use case
:author: dbatutx
:since: 12/02/2011
"""
import time
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from ErrorHandling.AcsBaseException import AcsBaseException


class LabEmMonitorFuelGauging(EmUsecaseBase):

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
        self._aplog_waiting_time = int(self._tc_parameters.get_param_value("WAITING_TIME"))
        # Call ConfigsParser to parse Energy_Management
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_MONITOR_FUEL_GAUGING", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

        # Redefine initial value for setting USBDIO:
        # - BatteryType = ANALOG
        self.em_core_module.io_card_init_state["BatteryType"] = self.phone_info["BATTERY"]["BATTID_TYPE"]
        # - Battery  = True (inserted)
        self.em_core_module.io_card_init_state["Battery"] = True
        # - Platform = True (ON)
        self.em_core_module.io_card_init_state["Platform"] = "ON"
        # - USBChargerType = USB_HOST_PC
        self.em_core_module.io_card_init_state["USBChargerType"] = "USB_HOST_PC"
        # - USBCharger = False (removed)
        self.em_core_module.io_card_init_state["USBCharger"] = False
        # - BatteryTemperature = BATTERY_TEMPERATURE
        self.em_core_module.io_card_init_state["BatteryTemperature"] = 25

        # Set initial value for setting Power Supply VBATT:
        # - VoltageLevel = VBATT
        self.em_core_module.eqp_init_state["BATT"]["VoltageLevel"] = self.em_core_module.vbatt

        # - Delay to wait for scheduled commands
        self._scheduled_timer_1 = self._aplog_waiting_time

#------------------------------------------------------------------------------
    def run_test_body(self):
        """
        Execute the test
        """
        # Call LAB_EM_BASE Run function
        EmUsecaseBase.run_test_body(self)

        # Connect USB PC/Host and DUT
        self.em_core_module.check_board_connection(only_reconnect=True)

        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()

        # Compare Vbatt value with limit parameters
        self._meas_list.add("IBATT1", meas_ibatt)

        # Wait time between command
        time.sleep(self._scheduled_timer_1)

        # get the delta_time result and compare it
        result1 = self.em_api.get_fuel_gauging_monitoring_time_result()
        self._meas_list.add("DELTA_TIME1", (result1[0], "none"))

        # switch on /off the charge
        self.em_core_module.plug_charger(self._io_card.WALL_CHARGER)
        time.sleep(15)
        self.em_core_module.unplug_charger(self._io_card.WALL_CHARGER)

        # Plug SDP
        self.em_core_module.check_board_connection(only_reconnect=True)
        time.sleep(5)

        # Wait time between command
        time.sleep(self._scheduled_timer_1)

        # get the delta_time result and compare it
        result2 = self.em_api.get_fuel_gauging_monitoring_time_result()

        # check if it is a new delta time
        if result2[1] == result1[1]:
            self._logger.info(" no new delta time in aplog")
            self._meas_list.add("DELTA_TIME2", ("no new delta time in aplog", "none"))
        else:
            self._meas_list.add("DELTA_TIME2", (result2[0], "none"))

        # Wait time between command
        time.sleep(self._scheduled_timer_1)

        # Read Platform OS and compare MSIC registers with expected values
        msic_registers = self.em_api.get_msic_registers()
        self._meas_list.add_dict("MSIC_REGISTER1", msic_registers,
                                 msic_registers["TIME_STAMP"][0])

        # switch on /off the charge
        self.em_core_module.plug_charger(self._io_card.WALL_CHARGER)
        time.sleep(15)
        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()

        # Compare Vbatt value with limit parameters
        self._meas_list.add("IBATT2", meas_ibatt)
        self.em_core_module.unplug_charger(self._io_card.WALL_CHARGER)

        # Plug SDP
        self.em_core_module.check_board_connection(only_reconnect=True)
        time.sleep(5)

        # Wait time between command
        time.sleep(self._scheduled_timer_1)

        # get the delta_time result and compare it
        result3 = self.em_api.get_fuel_gauging_monitoring_time_result()
        # check if it is a new delta time
        if result3[1] == result2[1]:
            self._logger.info(" no new delta time in aplog")
            self._meas_list.add("DELTA_TIME3", ("no new delta time in aplog", "none"))
        else:
            self._meas_list.add("DELTA_TIME3", (result3[0], "none"))

        # switch on /off the charge
        self.em_core_module.plug_charger(self._io_card.WALL_CHARGER)
        time.sleep(15)
        self.em_core_module.unplug_charger(self._io_card.WALL_CHARGER)

        # Plug SDP
        self.em_core_module.check_board_connection(only_reconnect=True)
        time.sleep(5)

        # Wait time between command
        time.sleep(self._scheduled_timer_1)

        # get the delta_time result and compare it
        result4 = self.em_api.get_fuel_gauging_monitoring_time_result()

        # check if it is a new delta time
        if result4[1] == result3[1]:
            self._logger.info(" no new delta time in aplog")
            self._meas_list.add("DELTA_TIME4", ("no new delta time in aplog", "none"))
        else:
            self._meas_list.add("DELTA_TIME4", (result4[0], "none"))

        # Wait time between command
        time.sleep(self._scheduled_timer_1)

        # Read Platform OS and compare MSIC registers with expected values
        msic_registers = self.em_api.get_msic_registers()
        self._meas_list.add_dict("MSIC_REGISTER2", msic_registers,
                                 msic_registers["TIME_STAMP"][0])

        # check read only file system
        try:
            self.phonesystem_api.write_file("hello",
                                             self.em_api._batt_capacity)  # pylint:disable=W0212

            self._meas_list.add("FILE_SYSTEM", ("write on file system",
                                                "none"))
        except AcsBaseException as e:
            if str(e).find("no write permission"):
                # expected behavior
                self._meas_list.add("FILE_SYSTEM", ("READ-ONLY FILE SYSTEM",
                                                    "none"))
            else:
                self._meas_list.add("FILE_SYSTEM", ("unknown file system",
                                                    "none"))

        # generate em verdict
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
        self._meas_list.clean()

        # Save data report in xml file
        self._error.Code = self._em_meas_verdict.get_current_result()
        self._error.Msg = self._em_meas_verdict.save_data_report_file()

        return self._error.Code, self._error.Msg
