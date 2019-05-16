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
:summary: Energy Management Battery Monitor - OFF ON cycle
:author: vgombert
:since: 09/29/2011
"""
import time
import os

from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase

from UtilitiesFWK.Utilities import Global
import acs_test_scripts.Utilities.EMUtilities as EMUtil

from ErrorHandling.AcsBaseException import AcsBaseException
from ErrorHandling.DeviceException import DeviceException


class LabEmBattOffOnNoVc(EmUsecaseBase):

    """
    Lab Energy Management base class.
    """
    DEDICATED_BENCH = "BATTERY_BENCH"

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call UseCase base Init function
        EmUsecaseBase.__init__(self, tc_name, global_config)

        # Read REBOOT_MODE from test case xml file
        self._reboot_mode = str(self._tc_parameters.get_param_value("REBOOT_MODE"))
        # Init fuel gauging param
        self.em_core_module.init_fg_param()

        # Initialize EM  xml object
        # measurement file
        meas_file_name = os.path.join(self._saving_directory,
                                      "EM_meas_report.xml")
        self.__em_meas_tab = EMUtil.XMLMeasurementFile(meas_file_name)
        # enable Global Measurement file
        name = os.path.join(self._campaign_folder,
                            self._em_cst.GLOBAL_MEAS_FILE)
        self.__em_meas_tab.enable_global_meas(name, self._name)

#-----------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # Call the UseCaseBase Setup function
        EmUsecaseBase.set_up(self)

        # set usb charging on
        self.em_api.set_usb_charging("on")

        # init capacity
        msic = self.update_battery_info()

        # get capability targets
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_BATT_OFF_ON_NO_VC", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

        # init verdict value
        if self._em_targets["MSIC_REGISTER_PLUG.BATTERY.CHARGE_NOW"] is not None:
            EMUtil.update_conf(
                self._em_targets["MSIC_REGISTER_PLUG.BATTERY.CHARGE_NOW"],
                "hi_lim", msic["BATTERY"]["CHARGE_FULL_DESIGN"][0], "=")

        if self.tc_module is not None:
            if self._em_targets["THERMAL_MSIC_REGISTER_PLUG.BATTERY.TEMP"] is not None:
                EMUtil.update_conf(
                    self._em_targets["THERMAL_MSIC_REGISTER_PLUG.BATTERY.TEMP"],
                    ["lo_lim", "hi_lim"], self._tct, "*")

        # Charge battery
        self.em_core_module.monitor_charging(self.em_core_module.batt_max_capacity, self.em_core_module.charge_time,
                              self.__em_meas_tab)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        EmUsecaseBase.run_test_body(self)

        self._logger.info("Start OFF ON cycle until battery capacity is at %s%%" %
                          self.em_core_module.batt_min_capacity)
        # init failed measurement counter
        measurement_fail = 0

        # OFF ON loop
        while self.batt_capacity > self.em_core_module.batt_min_capacity:

            # reboot board
            self.em_core_module.reboot_board(self._reboot_mode)
            # check board connection
            self.em_core_module.check_board_connection(1, False)

            try:
                # stop to charge through usb
                self.em_api.set_usb_charging("off")
                time.sleep(10)

                # get msic registers value after booting
                msic_reg = self.update_battery_info()

                # get thermal info
                thermal_conf = self.em_api.get_thermal_sensor_info()
                self.__em_meas_tab.add_dict_measurement(thermal_conf)

                self._meas_list.add_dict("MSIC_REGISTER_PLUG", msic_reg)

                # check thermal capabilities only if thermal chamber is used
                if self.tc_module is not None:
                    # Store various information
                    self.__em_meas_tab.add_measurement(
                        [self.tc_module.feed_meas_report()])
                    self._meas_list.add_dict("THERMAL_MSIC_REGISTER_PLUG", msic_reg)
                    self._meas_list.add_dict("THERMAL_CONF_PLUG", thermal_conf)

                measurement_fail = 0

            except AcsBaseException as e:
                # Just log error , board will be rebooted in next iteration
                self._logger.error("fail to get measurement: %s" % str(e))
                measurement_fail += 1

                # stop the usecase if measurement fail several times.
                if measurement_fail >= self._consecutive_meas_error:
                    tmp_txt = "Measurement failed after %s times, stop usecase" % \
                        self._consecutive_meas_error
                    self._logger.error(tmp_txt)
                    if self.batt_voltage > self.vbatt_mos_shutdown or \
                            self.batt_voltage == -1:
                        raise DeviceException(DeviceException.OPERATION_FAILED, tmp_txt)
                    else:
                        self._logger.info("battery must be empty, stop usecase")
                        break
            finally:
                # generate em verdict
                self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
                self._em_meas_verdict.judge(ignore_blocked_tc=True)
                self._meas_list.clean()

                # Store various information
                self.__em_meas_tab.add_measurement(
                    [self.get_time_tuple(),
                     ("COMMENTS", "OFF ON cycle discharge")])

                # switch to next meas
                self.__em_meas_tab.switch_to_next_meas()

                # reinitialize reboot variable
                self.phone_as_reboot = False

            self._logger.info(
                "waiting board discharge during  %s seconds" %
                self.em_core_module.discharge_time)
            time.sleep(self.em_core_module.discharge_time)

        return(self._em_meas_verdict.get_global_result(),
               self._em_meas_verdict.save_data_report_file())
#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        # call tear down after some operations
        EmUsecaseBase.tear_down(self)

        # retrieve measurement from test
        self.__em_meas_tab.generate_global_file()

        if self.is_board_and_acs_ok():
            self.em_api.set_usb_charging("on")

        # clean the board state and retrieve logs
        self.em_core_module.clean_up()

        return Global.SUCCESS, "No errors"
