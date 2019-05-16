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
:summary: Energy Management plug charger Use case
:author: apairex (based from Remove charger Use case written by vgombert)
:since: 07/25/2011
"""
import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from ErrorHandling.AcsConfigException import AcsConfigException


class LabEmPlugCharger(EmUsecaseBase):

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
        self._charger_type = self._tc_parameters.get_param_value("CHARGER_TYPE", "SDP")
        # Read START_OS from TC parameters
        self._start_os = self._tc_parameters.get_param_value("START_OS", "MOS")

        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_PLUG_CHARGER", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets, self.tcd_to_test)
        # - Delay to wait for scheduled commands
        self._scheduled_timer_1 = int(self._tc_parameters.get_param_value("DELAY_TO_MEASURE_BEFORE_PLUG", 60))
        self._scheduled_timer_2 = int(self._tc_parameters.get_param_value("DELAY_TO_MEASURE_AFTER_PLUG", 90))

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        EmUsecaseBase.set_up(self)

        # now we have to set brightness to a low value to limit drawn current
        self.phonesystem_api.set_display_brightness(5)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        """
        # Call LAB_EM_BASE Run function
        EmUsecaseBase.run_test_body(self)

        # Set API to start in x seconds to read and store all MSIC registers
        time1_before = time.time()
        pid1 = self.em_api.get_msic_registers("scheduled", self._scheduled_timer_1)
        time1_after = time.time()

        # Set API to start in x seconds to read and store all MSIC registers
        time2_before = time.time()
        pid2 = self.em_api.get_msic_registers("scheduled", self._scheduled_timer_2)

        time2_after = time.time()

        # wake up the board if necessary
        if self._device.get_boot_mode() == "MOS":
            if not self.phonesystem_api.get_screen_status():
                self._io_card.press_power_button(0.3)

        # Close connection
        self._device.disconnect_board()

        # Remove data cable
        self.em_core_module.unplug_charger(self._io_card.SDP)

        # Wait time between command
        if (time.time() - time1_before) > self._scheduled_timer_1:
            msg = "Test implementation incorrect (increase MSIC_REGISTER1 / scheduled_time constant)"
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, msg)
        self._wait_smart_time(time1_after, self._scheduled_timer_1)

        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()

        # Measure current from V charger
        meas_icharger = self.em_core_module.get_charger_current(self._charger_type)

        self._logger.info(
            "Charger %s unplugged : IBatt = %s %s , Icharger = %s %s " %
            (self._charger_type,
             meas_ibatt[0], meas_ibatt[1], meas_icharger[0], meas_icharger[1]))

        # Compare Vbatt value with limit parameters
        self._meas_list.add("IBATT1", meas_ibatt)

        # Compare Vcharger value with limit parameters
        self._meas_list.add("ICHARGER1", meas_icharger)

        # Plug CHARGER_TYPE
        self.em_core_module.plug_charger(self._charger_type, ext_ps=True)

        # put the board in idle
        self._io_card.press_power_button(0.3)

        # Wait time between command
        if (time.time() - time2_before) > self._scheduled_timer_2:
            msg = "Test implementation incorrect (increase MSIC_REGISTER2 / scheduled_time constant)"
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, msg)
        self._wait_smart_time(time2_after, self._scheduled_timer_2)

        # Measure current from Vbatt
        meas_ibatt = self.em_core_module.get_battery_current()

        # Measure current from Vusb
        meas_icharger = self.em_core_module.get_charger_current(self._charger_type)

        self._logger.info("Charger plugged : IBatt = %s %s , Icharger = %s %s " %
                         (meas_ibatt[0], meas_ibatt[1], meas_icharger[0], meas_icharger[1]))

        # Compare Vbatt value with limit parameters
        self._meas_list.add("IBATT2", meas_ibatt)

        # Compare Vusb value with limit parameters
        self._meas_list.add("ICHARGER2", meas_icharger)

        # plug the SDP charger
        self.em_core_module.plug_charger(self._io_card.USB_HOST_PC)
        time.sleep(self.usb_sleep)
        # Connect USB PC/Host and DUT
        self.em_core_module.check_board_connection(only_reconnect=True)
        self._device.connect_board()

        # Read Platform OS and compare MSIC registers with expected values
        msic_registers = self.em_api.get_msic_registers("read", pid1)
        self._meas_list.add_dict("MSIC_REGISTER1", msic_registers,
                                 msic_registers["TIME_STAMP"][0])

        # Read Platform OS and compare MSIC registers with expected values
        msic_registers = self.em_api.get_msic_registers("read", pid2)
        self._meas_list.add_dict("MSIC_REGISTER2", msic_registers,
                                 msic_registers["TIME_STAMP"][0])

        # generate em verdict
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
        self._em_meas_verdict.judge()
        self._meas_list.clean()

        return self._em_meas_verdict.get_current_result_v2()

    def tear_down(self):
        """
        End and dispose the test
        """

        # call the base
        EmUsecaseBase.tear_down(self)

        # set brightness mode to auto
        self.phonesystem_api.set_brightness_mode("automatic")

        return Global.SUCCESS, "No errors"
