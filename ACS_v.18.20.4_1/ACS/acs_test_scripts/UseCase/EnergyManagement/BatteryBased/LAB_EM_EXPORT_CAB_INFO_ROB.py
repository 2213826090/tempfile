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
:summary: EM - test export information about cable to userapp robustness
:author: jortetx
:since: 07/01/2014
"""
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from time import sleep


class LabEmExportCabInfoRob(EmUsecaseBase):
    """
    Lab Energy Management class.
    """
    DEDICATED_BENCH = "BATTERY_BENCH"

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call the LAB_EM_USE_CASE init method
        EmUsecaseBase.__init__(self, tc_name, global_config)

        # Read mandatory battery info
        tmp_batt_minfo = self._tc_parameters.get_param_value("SYSFS_MANDATORY_INFO")
        self.__sysfs_mandatory_cable_info = tmp_batt_minfo.split(';')

        # Read optional battery info
        self.__number_of_pass = self._tc_parameters.get_param_value("NUMBER_PASS", default_cast_type=int)
        if self.em_core_module.TYPE == "POWER_SUPPLY_BENCH":
            self.em_core_module.io_card_init_state["BatteryType"] = self.phone_info["BATTERY"]["BATTID_TYPE"]
            # - Battery
            self.em_core_module.io_card_init_state["Battery"] = True
            # - Platform
            self.em_core_module.io_card_init_state["Platform"] = "ON"
            self.em_core_module.io_card_init_state["USBChargerType"] = self._io_card.USB_HOST_PC
            self.em_core_module.io_card_init_state["USBCharger"] = True
        # get target and initialiZe measurement
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_EXPORT_CAB_INFO_ROB", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())
        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets)

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        """
        robust = True
        i = 0
        while i < self.__number_of_pass:
            self._logger.info("iteration %s" % i)
            sleep(10)
            # get board state
            self._logger.info("Robustness test %dth time" % i)
            # get register value
            rob_msic_register = self.em_api.get_msic_registers()
            # get all the battery info type contains in register
            battery_infos = rob_msic_register["CHARGER"].keys()
            # verify that all mandatory info are in register
            for minfo in self.__sysfs_mandatory_cable_info:
                self._meas_list.add(minfo, rob_msic_register["CHARGER"][minfo])
                self._em_meas_verdict.compare_list(self._meas_list, self._em_targets, True)
                self._em_meas_verdict.judge(ignore_blocked_tc=True)
                if minfo not in battery_infos:
                    robust = False
                    break
            i += 1

        self._meas_list.add("MANDATORY_INFO_ALWAYS_HERE", robust, "")
        # generate em verdict
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets, True)
        self._em_meas_verdict.judge()

        # Save data report in xml file
        self._error.Code = self._em_meas_verdict.get_current_result()
        self._error.Msg = self._em_meas_verdict.save_data_report_file()
        return self._error.Code, self._error.Msg
