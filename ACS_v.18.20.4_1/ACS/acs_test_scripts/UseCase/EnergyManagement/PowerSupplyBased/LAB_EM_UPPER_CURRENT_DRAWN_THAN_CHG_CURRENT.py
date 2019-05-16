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
:summary: Energy Management current drawn than charger current
:author: jortetx
:since: 03/12/2013
"""
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from acs_test_scripts.UseCase.EnergyManagement.UcModule.LoadModule import LoadModule
from UtilitiesFWK.Utilities import Global
import time


class LabEmUpperCurrentDrawnThanChgCurrent(EmUsecaseBase):
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
        # Read CHARGER_TYPE from TC parameters
        self.__charger_type = self._tc_parameters.get_param_value("CHARGER_TYPE", "SDP")
        # Read load to activate
        self.__load = self._tc_parameters.get_param_value("LOAD")
        # Activate Loadmodule instance
        self.__load_module = LoadModule()
        # Add list of LOAD to the loadmodule
        self.__load_module.add_load(self.__load)

        # get target and initialise measurment
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_UPPER_CURRENT_DRAWN_THAN_CHG_CURRENT", self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets, self.tcd_to_test)
        self.__scheduled_timer = 60

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Initialize the test:
        """
        EmUsecaseBase.set_up(self)
        # Load module to have a high current drawn
        self.__load_module.start_load()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def run_test_body(self):
        """
        Execute the test
        """
        # Set API to start in x seconds to read and store all MSIC registers
        pid = self.em_api.get_msic_registers("scheduled", self.__scheduled_timer)

        # Disconnect board from ACS
        self._device.disconnect_board()

        # Plug charger
        self.em_core_module.plug_charger(self.__charger_type, True)

        # IBatt measurement
        meas_ibatt = self.em_core_module.get_battery_current()
        # add ibatt to measure list to target comparison
        self._meas_list.add("IBATT", meas_ibatt)

        # wait for stabilize current
        time.sleep(self.__scheduled_timer)

        # Measure current from usb
        meas_icharger = self.em_core_module.get_charger_current(self.__charger_type)
        self._meas_list.add("IUSB", meas_icharger)

        self.em_core_module.unplug_charger(self.__charger_type)
        # wait for charger to be unplugged
        time.sleep(self.usb_sleep)
        # plug data
        self._io_card.usb_host_pc_connector(True)
        # wait for data to be fully plugged
        time.sleep(self.usb_sleep)

        self._device.connect_board()

        # Read Platform OS and compare MSIC registers with expected values
        msic_registers = self.em_api.get_msic_registers("read", pid)

        # Read Platform OS and compare MSIC registers with expected values
        self._meas_list.add_dict("MSIC_REGISTER", msic_registers, msic_registers["TIME_STAMP"][0])

        # generate em verdict
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
        self._em_meas_verdict.judge()
        self._meas_list.clean()

        return self._em_meas_verdict.get_current_result_v2()

#------------------------------------------------------------------------------
    def tear_down(self):
        """
        End and dispose the test
        """
        # desactivate load
        self.__load_module.stop_load()
        EmUsecaseBase.tear_down(self)

        return Global.SUCCESS, "No errors"
