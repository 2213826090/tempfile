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
:summary: Energy Management Plug / unplug charger Use case
:author: apairex
:since: 07/07/2011
        23/04/2012(April) - vgomberx - Bug 3145 - [EM] move usecase based on power supply on a specific folder
         26/04/2012(April) - vgomberx - Bug 3137 - [EM] reorganized em usecase and base to allow same usecase to be executed on both battery and power supply bench when possible
"""

import time

from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase


class LabEmPlugUnplug(EmUsecaseBase):

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

        # Read CHARGER_TYPE from TC parameters
        self._charger_type = \
            str(self._tc_parameters.get_param_value("CHARGER_TYPE"))

        # Read BATTERY_TEMPERATURE from TC parameters
        self._batt_temp = \
            int(self._tc_parameters.get_param_value("BATTERY_TEMPERATURE"))

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
        self.em_core_module.io_card_init_state["BatteryTemperature"] = self._batt_temp

        # Set initial value for setting Power Supply VBATT:
        # - VoltageLevel = VBATT
        self.em_core_module.eqp_init_state["BATT"]["VoltageLevel"] = self.em_core_module.vbatt

        # - additional delay taken by commands to set the hardware
        #  in wanted state
        self._delay_time_1 = 12

        # - additional delay taken by commands to set the hardware
        #  in wanted state
        self._delay_time_2 = 3
        self._delay_time_3 = 3

        # Initialise _em_targets to a null value
        # (will be instatiate with the charger type as parameter)
        self._em_targets = None
        self._scheduled_timer_1 = self._delay_time_1 + 11
        self._scheduled_timer_2 = self._scheduled_timer_1 \
            + self._delay_time_2 + 11
        self._scheduled_timer_3 = self._scheduled_timer_2 \
            + self._delay_time_3 + 11

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        """
        # Call LAB_EM_BASE Run function
        EmUsecaseBase.run_test_body(self)

        if self._charger_type == "ALL":
            for charger in self._io_card.SUPPORTED_DEVICE_TYPE:
                if charger not in [self._io_card.USB_HOST_PC,
                                   self._io_card.OTG, self._io_card.WALL_CHARGER]:
                    self.__run_specific_test(charger)
        else:
            self.__run_specific_test(self._charger_type)

        # Save data report in xml file
        self._error.Code = self._em_meas_verdict.get_current_result()
        self._error.Msg = self._em_meas_verdict.save_data_report_file()

        return self._error.Code, self._error.Msg

#------------------------------------------------------------------------------
    def __run_specific_test(self, charger_type):
        """
        Run the test for a specific charger_type

        :type  charger_type: str
        :param charger_type: Name of the charger to test with
        """
        self._logger.info("  charger_type parameter = %s" % charger_type)

        # Load the _em_target piece of XML file and set the timers values
        self.__setup_timers(charger_type)

        # interrupt measurement inital status
        interrupt_pid1 = self.em_api.get_proc_interrupt("scheduled", 0)

        # Set API to start before the "plug" action
        time1_before = time.time()
        pid1 = self.em_api.get_msic_registers("scheduled",
                                               self._scheduled_timer_1)
        time1_after = time.time()

        # Set API to start after the plug action
        time2_before = time.time()
        pid2 = self.em_api.get_msic_registers("scheduled",
                                               self._scheduled_timer_2)
        time2_after = time.time()

        # Set API to start after the unplug action
        time3_before = time.time()
        pid3 = self.em_api.get_msic_registers("scheduled",
                                               self._scheduled_timer_3)

        # start interrupt measurement before next connection start
        interrupt_pid3 = self.em_api.get_proc_interrupt("scheduled",
                                                         self._scheduled_timer_2)
        time3_after = time.time()

        # INITIAL STATE: Everything unplug
        # Close connection
        self._device.disconnect_board()

        # Remove data cable
        self._io_card.usb_connector(False)

        # Wait time between command
        if (time.time() - time1_before) > self._scheduled_timer_1:
            self._logger.info("Test implementation incorrect " +
                              "(increase MSIC_REGISTER1 / scheduled_time constant)")
            return (Global.FAILURE, "Test implementation incorrect " +
                    "(increase MSIC_REGISTER1 / scheduled_time constant)")
        self._wait_smart_time(time1_after, self._scheduled_timer_1)

        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()

        # Measure current from VCharger
        meas_icharger = self.em_core_module.get_charger_current(self._charger_type)

        self._logger.info("Initial state : IBatt = %s %s , ICharger = %s %s " %
                         (meas_ibatt[0], meas_ibatt[1], meas_icharger[0], meas_icharger[1]))

        # Compare Vbatt value with limit parameters
        self._meas_list.add("IBATT1", meas_ibatt)

        # Compare Vcharger value with limit parameters
        if meas_icharger[0] is not "None":
            self._meas_list.add("ICHARGER1", meas_icharger)

        # PLUG ACTION

        # Plug CHARGER_TYPE
        self.em_core_module.plug_charger(self._charger_type, True)

        # Wait time between command
        if (time.time() - time2_before) > self._scheduled_timer_2:
            self._logger.info("Test implementation incorrect " +
                              "(increase MSIC_REGISTER2 / scheduled_time constant)")
            return (Global.FAILURE, "Test implementation incorrect " +
                    "(increase MSIC_REGISTER2 / scheduled_time constant)")
        self._wait_smart_time(time2_after, self._scheduled_timer_2)

        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()

        # Measure current from VCharger
        meas_icharger2 = self.em_core_module.get_charger_current(self._charger_type)

        self._logger.info("Charger %s plugged : IBatt = %s %s , ICharger = %s %s " %
                          (charger_type,
                           meas_ibatt[0], meas_ibatt[1],
                           meas_icharger2[0], meas_icharger2[1]))

        # Compare Vbatt value with limit parameters
        self._meas_list.add("IBATT2", meas_ibatt)

        # Compare Vcharger value with limit parameters
        if meas_icharger[0] is not "None":
            self._meas_list.add("ICHARGER2", meas_icharger2)

        # unplug the charger
        self.em_core_module.unplug_charger(self._charger_type)

        # Wait time between command
        if (time.time() - time3_before) > self._scheduled_timer_3:
            self._logger.info("Test implementation incorrect " +
                              "(increase MSIC_REGISTER3 / scheduled_time constant)")
            return (Global.FAILURE, "Test implementation incorrect " +
                    "(increase MSIC_REGISTER3 / scheduled_time constant)")
        self._wait_smart_time(time3_after, self._scheduled_timer_3)

        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()

        # Measure current from VCharger
        meas_icharger3 = self.em_core_module.get_charger_current(self._charger_type)

        self._logger.info("Charger removed : IBatt = %s %s , ICharger = %s %s " %
                         (meas_ibatt[0], meas_ibatt[1], meas_icharger3[0], meas_icharger3[1]))

        # Compare Vbatt value with limit parameters
        self._meas_list.add("IBATT3", meas_ibatt)

        # Compare Vcharger value with limit parameters
        if meas_icharger[0] is not "None":
            self._meas_list.add("ICHARGER3", meas_icharger3)

        # PLUG PC HOST to get the read the recoded result on DUT

        # Connect USB PC/Host and DUT
        self.em_core_module.check_board_connection(only_reconnect=True)

        # Read Platform OS and compare MSIC registers with expected values
        msic_registers = self.em_api.get_msic_registers("read", pid1)
        self._meas_list.add_dict("MSIC_REGISTER1", msic_registers,
                                 msic_registers["TIME_STAMP"][0])

        # Read Platform OS and compare MSIC registers with expected values
        msic_registers = self.em_api.get_msic_registers("read", pid2)
        self._meas_list.add_dict("MSIC_REGISTER2", msic_registers,
                                 msic_registers["TIME_STAMP"][0])

        # Read Platform OS and compare MSIC registers with expected values
        msic_registers = self.em_api.get_msic_registers("read", pid3)
        self._meas_list.add_dict("MSIC_REGISTER3", msic_registers,
                                 msic_registers["TIME_STAMP"][0])

        # Calculate the interrupt numbers between 2 connections
        interrupt = (
            self.em_api.get_proc_interrupt("read", interrupt_pid3) -
            self.em_api.get_proc_interrupt("read", interrupt_pid1), "none")
        self._meas_list.add("INTERRUPT1", interrupt)

        # generate em verdict
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
        self._meas_list.clean()

    def __setup_timers(self, charger_type):
        """
        Load the piece of target file associated to the charger_type to use for
        the test and set the timer value.

        :type  charger_type: str
        :param charger_type: Name of the charger to test with
        """
        targets = self._tc_parameters.get_params_as_dict()
        targets["CHARGER_TYPE"] = charger_type
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_PLUG_UNPLUG", targets,
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

        # - Delay to wait for scheduled commands
        msic_timer_1 = \
            int(self._em_targets["MSIC_REGISTER1"]["scheduled_time"])
        self._scheduled_timer_1 = msic_timer_1 + self._delay_time_1

        # - Delay to wait for scheduled commands
        msic_timer_2 = \
            int(self._em_targets["MSIC_REGISTER2"]["scheduled_time"])
        self._scheduled_timer_2 = \
            self._scheduled_timer_1 + msic_timer_2 + self._delay_time_2

        # - Delay to wait for scheduled commands
        msic_timer_3 = \
            int(self._em_targets["MSIC_REGISTER3"]["scheduled_time"])
        self._scheduled_timer_3 = \
            self._scheduled_timer_2 + msic_timer_3 + self._delay_time_3
