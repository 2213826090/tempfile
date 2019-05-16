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
:summary: Energy Management boot in COS then MOS
:author: vgombert
:since: 12/09/2011
"""
import time
import os

from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
import acs_test_scripts.Utilities.EMUtilities as EMUtil


class LabEmBattBootCosMos(EmUsecaseBase):

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

        # save global config into an attribute in
        # order to retrieve value in set_up method
        self.__global_config = global_config

        # Read OFF_TIMEOUT from test case xml file
        self._sleep_time = \
            int(self._tc_parameters.get_param_value(
                "OFF_TIMEOUT"))

        # Initialize EM  xml object
        # measurement file
        meas_file_name = os.path.join(self._saving_directory,
                                      "EM_meas_report.xml")
        self.__em_meas_tab = EMUtil.XMLMeasurementFile(meas_file_name)

        # Call ConfigsParser to parse Energy_Management
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_BATT_BOOT_COS_MOS", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

        # init variables
        self._off_timeout = 300

#------------------------------------------------------------------------------

    def run_test_body(self):
        EmUsecaseBase.run_test_body(self)

        # get msic/ thermal info
        msic_reg = self.update_battery_info()
        thermal_conf = self.em_api.get_thermal_sensor_info()

        # store result on xml
        self.__em_meas_tab.add_dict_measurement(msic_reg)
        self.__em_meas_tab.add_dict_measurement(thermal_conf)

        # perform soft shutdown
        self._logger.info("software shutdown with Charger")
        self._device.disconnect_board()
        self._device.soft_shutdown_cmd()

        # boot board
        state = ""
        start_time = time.time()
        while state != "unknown":
            if (time.time() - start_time) >= self._off_timeout:
                self._logger.error("board failed to turn off before %s" %
                                   str(self._device.get_boot_off_timeout()))
                break
            state = self._device.get_state()

        # compare measurement
        if state == "alive":
            state = "UP"
        else:
            state = "DOWN"
        self._meas_list.add("DUT_CONNECTION_OFF_1", (state, "none"))

        self._logger.info("wait %s seconds" % self._sleep_time)
        time.sleep(self._sleep_time)

        # compare measurement
        if self._device.get_state() == "alive":
            state = "UP"
        else:
            state = "DOWN"
        self._meas_list.add("DUT_CONNECTION_COS", (state, "none"))

        # boot board
        self._io_card.press_power_button(self.pwr_btn_boot)
        # boot board
        state = ""
        start_time = time.time()
        while state != "alive":
            if (time.time() - start_time) >= self._device.get_boot_timeout():
                self._logger.error("board failed to turn on before %s" %
                                   str(self._device.get_boot_timeout()))
                break
            state = self._device.get_state()

        if state == "alive":
            # Wait some second to settle down
            time.sleep(self.usb_sleep)
            self._device.connect_board()
            # get msic/ thermal info
            msic_reg = self.update_battery_info()
            thermal_conf = self.em_api.get_thermal_sensor_info()

            # store result on xml
            self.__em_meas_tab.add_dict_measurement(msic_reg)
            self.__em_meas_tab.add_dict_measurement(thermal_conf)
            state = "UP"
        else:
            state = "DOWN"

        # compare measurement
        self._meas_list.add("DUT_CONNECTION_MOS", (state, "none"))

        # store result on xml
        self.__em_meas_tab.add_dict_measurement(msic_reg)
        self.__em_meas_tab.add_dict_measurement(thermal_conf)
        self.__em_meas_tab.add_measurement(
            [self.get_time_tuple(),
             ("COMMENTS", "software shutdown with Charger")])

        # switch meas to next meas
        self.__em_meas_tab.switch_to_next_meas()

        # perform soft shutdown
        self._logger.info("software shutdown without Charger")
        self._device.disconnect_board()
        self._device.soft_shutdown_cmd()
        self._io_card.usb_connector(False)

        self._logger.info("wait %s seconds" % self._sleep_time)
        time.sleep(self._sleep_time)

        # compare measurement
        if self._device.get_state() == "alive":
            state = "UP"
        else:
            state = "DOWN"

        self._meas_list.add("DUT_CONNECTION_OFF_2", (state, "none"))

        # Save data report in xml file
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
        self._em_meas_verdict.judge(ignore_blocked_tc=True)
        self._meas_list.clean()

        # boot board
        self._device.switch_on(settledown_duration=10,
                               simple_switch_mode=True)

        if self._device.get_state() == "alive":
            # get msic/ thermal info
            msic_reg = self.update_battery_info()
            thermal_conf = self.em_api.get_thermal_sensor_info()

            # store result on xml
            self.__em_meas_tab.add_dict_measurement(msic_reg)
            self.__em_meas_tab.add_dict_measurement(thermal_conf)

        self.__em_meas_tab.add_measurement(
            [self.get_time_tuple(),
             ("COMMENTS", "SW Shutdown w/o Charger")])

        # switch meas to next meas
        self.__em_meas_tab.switch_to_next_meas()

        return(self._em_meas_verdict.get_global_result(),
               self._em_meas_verdict.save_data_report_file())
