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
:summary: do a charger plug and unplug during a voicecall. Check that the plug/unplug is working well and the call does not stop.
:author: vgomberx
:since: July 09 2013
"""
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.EnergyManagement.BatteryBased.LAB_EM_BATT_PLUG_UNPLUG import LabEmBattPlugUnplug
from acs_test_scripts.UseCase.EnergyManagement.UcModule.TelephonyModule2G import TelephonyModule2g
from acs_test_scripts.UseCase.EnergyManagement.UcModule.TelephonyModule3G import TelephonyModule3g


class LabEmBattPlugUnplugDuringCall(LabEmBattPlugUnplug):
    """
    Lab Energy Management class.
    """
    DEDICATED_BENCH = "BATTERY_BENCH"

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call em plug unplug object
        LabEmBattPlugUnplug.__init__(self, tc_name, global_config)
        # Declare the type of network here like 2G/3G
        network_type = self._tc_parameters.get_param_value("NETWORK_TYPE", "").upper()
        # declare uc name here to ease inheritance
        self._uc_name = "LAB_EM_BATT_PLUG_UNPLUG_DURING_CALL"
        # init telephony module to allow call
        if network_type == "2G":
            self.__telmodule = TelephonyModule2g()
        elif network_type == "3G":
            self.__telmodule = TelephonyModule3g()

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Execute the set up
        """
        # Call the UseCaseBase Setup function
        LabEmBattPlugUnplug.set_up(self)
        # setup simulator
        self.__telmodule.set_up_eq()
        # wait for registration
        self.__telmodule.register_to_network()
        # establish voice call
        self.__telmodule.establish_call()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        """
        # Call LAB_EM_BASE Run function
        LabEmBattPlugUnplug.run_test_body(self)
        vc_state = str(self.__telmodule.is_call_active()).upper()
        # Read Platform OS and compare with expected values

        self._meas_list.add("VOICE_CALL_ON_GOING", vc_state, "")
        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
        self._em_meas_verdict.judge(ignore_blocked_tc=True)
        self._meas_list.clean()

        # Save data report in xml file
        return(self._em_meas_verdict.get_current_result(),
               self._em_meas_verdict.save_data_report_file())

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        try:
            LabEmBattPlugUnplug.tear_down(self)
            self.em_core_module.clean_up()
        finally:
            self.__telmodule.release_eq()

        return Global.SUCCESS, "No errors"
