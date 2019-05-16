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
:summary: Energy Management - Measure Ibatt leak when phone is in special mode
:author: vgombert
:since: 14/02/2012 (feb)
"""
import time
import os

from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase

from UtilitiesFWK.Utilities import Global
import acs_test_scripts.Utilities.EMUtilities as EMUtil
from ErrorHandling.AcsConfigException import AcsConfigException


class LabEmBattIbattStandbyMode(EmUsecaseBase):

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
        # init fuel gauging parameters
        self.em_core_module.init_fg_param()
        # Read STAND_DURATION from test case xml file
        self._stand_mode = \
            self._tc_parameters.get_param_value(
                "STAND_MODE")
        # Read STANDBY_DURATION from test case xml file
        self._stand_duration = \
            int(self._tc_parameters.get_param_value(
                "STAND_DURATION"))
        # Initialize EM  xml object
        # measurement file
        meas_file_name = os.path.join(self._saving_directory,
                                      "EM_meas_report.xml")
        self.__em_meas_tab = EMUtil.XMLMeasurementFile(meas_file_name)

        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_BATT_IBATT_STANDBY_MODE",
            self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

        # get network api
        self._network_api = self._device.get_uecmd("Networking")

#-----------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # Call the UseCaseBase Setup function
        EmUsecaseBase.set_up(self)

        if not self._stand_mode in ["FLIGHT_MODE", "STANDBY", "OFF"]:
            tmp_txt = "unknown STAND_MODE option value :" + str(self._stand_mode)
            self._logger.error(tmp_txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, tmp_txt)

        # check keys on dict to avoid reading empty files
        self.update_battery_info()

        # Charge battery
        self.em_core_module.monitor_charging(self.em_core_module.batt_max_capacity, self.em_core_module.charge_time,
                              self.__em_meas_tab)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        EmUsecaseBase.run_test_body(self)

        # generate msic logs
        msic = self.em_api.get_msic_registers()
        self._meas_list.add_dict("MSIC_REGISTER_START", msic)
        # compute value
        battery_charge_now_start = msic["BATTERY"]["CHARGE_NOW"][0]
        battery_capacity_start = float(battery_charge_now_start) / msic["BATTERY"]["CHARGE_FULL"][0]

        self._meas_list.add("BATTERY_CAPACITY_START",
                           (battery_capacity_start, "none"))

        self.__em_meas_tab.add_dict_measurement(msic)
        self.__em_meas_tab.add_measurement(
            [self.get_time_tuple(),
             ("COMMENTS", "Before standing in " + self._stand_mode),
             ("REBOOT", self.phone_as_reboot)])

        # switch meas to next meas
        self.__em_meas_tab.switch_to_next_meas()

        if self._stand_mode in ["FLIGHT_MODE", "STANDBY"]:

            if self._stand_mode == "STANDBY":
                self.phonesystem_api.set_screen_timeout(15)
            elif self._stand_mode == "FLIGHT_MODE":
                self._network_api.set_flight_mode("on")

            # disconnect board
            self._device.disconnect_board()
            # connect DCP
            self._io_card.usb_host_pc_connector(False)

        elif self._stand_mode == "OFF":
            self._device.soft_shutdown(False)

        self._logger.info("wait in %s mode during %s seconds" %
                         (self._stand_mode, self._stand_duration))
        time.sleep(self._stand_duration)

        if self._stand_mode == "OFF":
            self._device.switch_on()

        elif self._stand_mode in ["FLIGHT_MODE", "STANDBY"]:
            # connect USB SDP
            self._io_card.usb_host_pc_connector(True)
            # wait x seconds
            time.sleep(self.usb_sleep)
            # connect board
            self._device.connect_board()
            # check adb connection
            self.em_core_module.check_board_connection()

        # generate msic logs
        msic = self.em_api.get_msic_registers()
        self.__em_meas_tab.add_dict_measurement(msic)
        self.__em_meas_tab.add_measurement(
            [self.get_time_tuple(),
             ("COMMENTS", "After standing in " + self._stand_mode),
             ("REBOOT", self.phone_as_reboot)])

        # switch meas to next meas
        self.__em_meas_tab.switch_to_next_meas()

        # compute values
        battery_charge_now_stop = msic["BATTERY"]["CHARGE_NOW"][0]
        battery_capacity_stop = float(battery_charge_now_stop) / msic["BATTERY"]["CHARGE_FULL"][0]
        lost_capacity = (battery_capacity_start - battery_capacity_stop, "none")
        Ibatt = ((float((battery_charge_now_start - battery_charge_now_stop) * 1e-3) / float(self._stand_duration) / 3600),
                 "mA")

        # compute result
        self._meas_list.add_dict("MSIC_REGISTER_STOP", msic)
        self._meas_list.add("BATTERY_CAPACITY_STOP",
                           (battery_capacity_stop, "none"))
        self._meas_list.add("LOST_CAPACITY",
                            lost_capacity)
        self._meas_list.add("IBATT", Ibatt)

        # Save data report in xml file
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
        self._em_meas_verdict.judge(ignore_blocked_tc=True)
        self._meas_list.clean()

        return(self._em_meas_verdict.get_global_result(),
               self._em_meas_verdict.save_data_report_file())

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        EmUsecaseBase.tear_down(self)

        # clean the board state and retrieve logs
        self.em_core_module.clean_up()

        if self.is_board_and_acs_ok() and self.batt_capacity != -1:
            if self._stand_mode == "FLIGHT_MODE":
                self._network_api.set_flight_mode("off")

        return Global.SUCCESS, "No errors"
