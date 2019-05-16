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
:summary: This file is the implementation of LIVE_MODEM_SELECT_DATA_SIM use case
:since: 02/14/2013
:author: jreynaux
"""

import time
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global, str_to_bool


class LiveModemDsdsSelectDataSim(UseCaseBase):

    """
    This UC will allows user to select wich sim should be used for Data
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # Read callSetupTimeout from Phone_Catalog.xml
        self._call_setup_time = \
            int(self._dut_config.get("callSetupTimeout"))

        # Get TC Parameters
        self._target_sim = int(self._tc_parameters.
                               get_param_value("TARGET_SIM"))

        self._restore_initial = \
            str_to_bool(self._tc_parameters.get_param_value("RESTORE_INITIAL"))

        # Get UECmdLayer
        self._modem_api = self._device.get_uecmd("Modem")
        self._phone_system = self._device.get_uecmd("PhoneSystem")
        self._networking_api = self._device.get_uecmd("Networking")

        self._initial_target_sim = None
        self._initial_pdp_context_status = None

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Initialize the test
        """
        UseCaseBase.set_up(self)

        # Backup the current target sim
        self._initial_target_sim = self._device.get_target_sim()

        # Backup the initial pdp context status
        # pylint: disable=W0212
        self._initial_pdp_context_status = \
            self._networking_api._get_pdp_context_status()
        # pylint: enable=W0212

        # If deactivated, activate it
        if self._initial_pdp_context_status == "2":
            # Then deactivate PDP context status
            self._networking_api.activate_pdp_context()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """
        # Call UseCase base Run function
        UseCaseBase.run_test(self)

        self._logger.info('Run the test with SIM%s as target sim'
                          % (str(self._target_sim)))
        self._do_set_data_sim(self._target_sim)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def tear_down(self):
        """
        End and dispose the test
        """
        UseCaseBase.tear_down(self)

        output_message = "No errors"
        # Restore the current target sim
        self._device.set_target_sim(self._initial_target_sim)

        # Check whether we had to change the PDP context status
        # If initially deactivated, deactivate it
        if self._initial_pdp_context_status == "2":
            # Then deactivate PDP context status
            self._networking_api.deactivate_pdp_context()

        # If value as been set on tc parameters, restore initial data sim
        # in order to avoid disturbing next tests
        if self._restore_initial:
            self._logger.info("Restore default data sim as SIM%s "
                              % (str(self._initial_target_sim)))
            self._do_set_data_sim(self._initial_target_sim)
        else:
            # Otherwise the default data sim will stay one configured in this tc
            # even after reboot, this may not affect most of test, but should be
            # taken into account.
            self._logger.warning("Will not restore initial data SIM, next tests"
                                 " will use SIM%s as default data SIM !" % (str(self._target_sim)))
            output_message = "SIM%s remain default data SIM" \
                % (str(self._target_sim))
        return Global.SUCCESS, output_message

#------------------------------------------------------------------------------

    def _do_set_data_sim(self, target_sim):
        """
        Performs the operation to set the data sim and check data state on
        both sims.

        :type target_sim: int
        :param target_sim: The SIM to be tested as DATA SIM
        """
        # Configure target sim (used for getter)
        self._device.set_target_sim(target_sim)

        # Set data sim
        self._modem_api.set_data_sim(target_sim)

        # Wait a time
        time.sleep(self._wait_btwn_cmd)

        # Wake Up Screen to ensure Connection
        self._phone_system.wake_screen()

        # pylint: disable=E1101
        # Check data state
        self._modem_api.wait_for_data_state(
            self._uecmd_types.DATA_STATE.CONNECTED,
            self._call_setup_time)
        # pylint: enable=E1101

        time.sleep(self._wait_btwn_cmd)

        # Use the other sim than targeted one
        self._device.set_target_sim(2 if target_sim == 1 else 1)

        # pylint: disable=E1101
        # Check that other SIM is well disconnected
        self._modem_api.check_data_state(
            self._uecmd_types.DATA_STATE.DISCONNECTED)
        # pylint: enable=E1101

        # Re set the SIM targeted
        self._device.set_target_sim(target_sim)
