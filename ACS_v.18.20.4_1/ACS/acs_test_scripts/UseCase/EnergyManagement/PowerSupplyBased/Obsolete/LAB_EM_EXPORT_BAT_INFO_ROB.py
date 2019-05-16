"""
@copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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

:organization: INTEL QCTV
:summary: EM - test export information about battery to userapp robustness
:author: jortetx
:since: 07/01/2014
"""
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
import time


class LabEmExportBatInfoRob(EmUsecaseBase):
    """
    Lab Energy Management class.
    """
    DEDICATED_BENCH = "POWER_SUPPLY_BENCH"

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call the LAB_EM_USE_CASE init method
        EmUsecaseBase.__init__(self, tc_name, global_config)

        # Read mandatory battery info
        tmp_batt_minfo = self._tc_parameters.get_param_value("SYSFS_MANDATORY_INFO")
        self.__sysfs_mandatory_batt_info = tmp_batt_minfo.split(';')

        # Read optional battery info
        self.__number_of_pass = self._tc_parameters.get_param_value("NUMBER_PASS", default_cast_type=int)

        self.em_core_module.io_card_init_state["BatteryType"] = self.phone_info["BATTERY"]["BATTID_TYPE"]
        # - Battery
        self.em_core_module.io_card_init_state["Battery"] = True
        # - Platform
        self.em_core_module.io_card_init_state["Platform"] = "ON"
        self.em_core_module.io_card_init_state["USBChargerType"] = self._io_card.USB_HOST_PC
        self.em_core_module.io_card_init_state["USBCharger"] = True
        # get target and initialize measurement
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_EXPORT_BAT_INFO_ROB", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())
        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        """
        # st_energymngt_monitorbattery_4095_5
        # st_energymngt_monitorbattery_4095_9
        # robustness test about the battery information
        robust = True
        i = 0
        while i < self.__number_of_pass:
            self._logger.info("iteration %s" % i)
            # disconnect and reconnect a valid battery
            self._device.disconnect_board()
            self.em_core_module.unplug_charger("SDP")
            self._io_card.battery_connector(False)
            time.sleep(10)
            self._io_card.battery_connector(True)
            time.sleep(5)
            # power on the device
            self._device.switch_on()
            self.em_core_module.check_board_connection(only_reconnect=True)

            # get board state
            if self._device.get_boot_mode() != "UNKNOWN":
                self._logger.info("Robustness test %dth time" % i)
                # get register value
                rob_msic_register = self.em_api.get_msic_registers()
                # get all the battery info type contains in register
                battery_infos = rob_msic_register["BATTERY"].keys()
                # verify that all mandatory info are in register
                # st_energymngt_monitorbattery_4095_1
                for minfo in self.__sysfs_mandatory_batt_info:
                    target_name = "AFTER_BI_" + minfo
                    self._meas_list.add(target_name, rob_msic_register["BATTERY"][minfo])
                    self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
                    self._em_meas_verdict.judge(ignore_blocked_tc=True)
                    self._meas_list.clean()
                    if minfo not in battery_infos:
                        robust = False
                        break
                i += 1

        self._meas_list.add("MANDATORY_INFO_ALWAYS_HERE", robust, "")

        # generate em verdict
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
        self._em_meas_verdict.judge()
        self._meas_list.clean()

        # Save data report in xml file
        self._error.Code = self._em_meas_verdict.get_current_result()
        self._error.Msg = self._em_meas_verdict.save_data_report_file()
        return self._error.Code, self._error.Msg
