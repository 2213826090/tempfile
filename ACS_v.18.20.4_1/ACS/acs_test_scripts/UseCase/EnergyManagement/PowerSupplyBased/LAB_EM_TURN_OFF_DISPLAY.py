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
:summary: Energy Management battery turn off the display Use case
:author: dbatut
:since: 03/20/2012
:last update: 14/08/2014
"""
import time

from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from acs_test_scripts.UseCase.EnergyManagement.UcModule.LoadModule import LoadModule


class LabEmTurnOffDisplay(EmUsecaseBase):

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

        # Read STABILIZATION_TIME from TC parameters
        self.__stabilization_time = int(self._tc_parameters.get_param_value("STABILIZATION_TIME"))

        # prepare LOAD
        self.__load = self._tc_parameters.get_param_value("LOAD", "NO_LOAD")

        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_TURN_OFF_DISPLAY", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets, self.tcd_to_test)

        # Measure current from Vbatt
        self._pwrs = self._em.get_power_supply("BATT")
        self.__load_module = None
        self.__screen_timeout = 60 * 60

#------------------------------------------------------------------------------
    def set_up(self):

        EmUsecaseBase.set_up(self)

        if self.__load != "NO_LOAD" :
            # Activate Loadmodule instance
            self.__load_module = LoadModule()
            # Add list of LOAD to the loadmodule
            self.__load_module.add_load(self.__load)

        # Set the brightness mode to manual
        self.phonesystem_api.set_brightness_mode("manual")
        time.sleep(2)

        # set brightness to 100%
        self.phonesystem_api.set_display_brightness(100)
        time.sleep(2)

        # Set screen timeout to 30 minutes
        self.phonesystem_api.set_screen_timeout(self.__screen_timeout)
        time.sleep(1)

        # wake up the board if necessary
        if not self.phonesystem_api.get_screen_status():
            self._io_card.press_power_button(0.3)
        time.sleep(1)

        # run a load upon parameter configuration
        if self.__load != "NO_LOAD" :
            # start HIGH LOAD
            self.__load_module.start_load()
            time.sleep(10)

        return Global.SUCCESS, "No errors"

    def run_test_body(self):
        """
        Execute the test
        """

        # Call LAB_EM_BASE Run function
        EmUsecaseBase.run_test_body(self)

        # get ibatt with display on
        ibatt_screen_on = self.em_core_module.get_battery_current()
        self._logger.info("Screen On : IBatt = %s %s" % (ibatt_screen_on[0], ibatt_screen_on[1]))

        # switch off the screen by pressing power on button during 0.5 second
        self._io_card.press_power_button(0.5)

        # sleep 5 s
        time.sleep(5)

        # check_screen status
        screen_status = self.phonesystem_api.get_screen_status()
        self._meas_list.add("SCREEN_STATUS", screen_status, "")

        # now get ibatt value with display off
        ibatt_screen_off = self.em_core_module.get_battery_current()
        self._logger.info("Screen Off : IBatt = %s %s" % (ibatt_screen_off[0], ibatt_screen_off[1]))
        self._meas_list.add("SAVED_IBATT", (ibatt_screen_on[0] - ibatt_screen_off[0], ibatt_screen_off[1]))

        # Close connection
        self._device.disconnect_board()
        # Remove data cable
        self._io_card.usb_connector(False)

        # turn screen off
        time.sleep(2)
        self._io_card.press_power_button(0.5)

        # now just wait x second min and check IBATT value
        self._logger.info("Wait %ss in order to stabilize the battery current" % str(self.__stabilization_time))
        time.sleep(self.__stabilization_time)

        # now get ibatt value with display off
        ibatt_screen_off_after_waiting = self.em_core_module.get_battery_current()
        self._meas_list.add("WAIT_FOR_MIN_IBATT", (ibatt_screen_off_after_waiting[0], ibatt_screen_off_after_waiting[1]))

        # generate em verdict
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
        self._em_meas_verdict.judge()
        self._meas_list.clean()

        return self._em_meas_verdict.get_current_result_v2()

    def tear_down(self):
        """
        End and dispose the test
        """

        # connect usb host cable
        self._io_card.usb_host_pc_connector(True)
        # Close connection
        self._device.connect_board()

        # stop the laod if it was started
        if self.__load != "NO_LOAD" and self.is_board_and_acs_ok() :
            # Stop HIGH LOAD
            self.__load_module.stop_load()

        # call the base
        EmUsecaseBase.tear_down(self)

        return Global.SUCCESS, "No errors"
