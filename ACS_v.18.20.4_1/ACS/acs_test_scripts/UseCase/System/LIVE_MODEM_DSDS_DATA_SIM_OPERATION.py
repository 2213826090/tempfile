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
:summary: This file is the implementation of LIVE_MODEM_DATA_SIM_OPERATION

:since: 02/27/2013
:author: jreynaux
"""

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Device.UECmd.UECmdTypes import SIM_ROLE
import time


class LiveModemDsdsDataSimOperation(UseCaseBase):

    """
    This UC will allows user to perform multiple operation on both sims:
    - Data disabled when data SIM disabled
    - Data enabled when data SIM enabled
    - Impact of other sim state on data SIM (off)
    - Verify data sim switch on/off keep settings
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

        # Check data state when data sim is switched on/off.
        self._check_data_state_sim_state(self._target_sim)

        # Wait a while
        time.sleep(self._wait_btwn_cmd)

        #  Check data state when other sim is switched on/off.
        self._check_data_sim_secondary_sim_off(self._target_sim)

        # Wait a while
        time.sleep(self._wait_btwn_cmd)

        # Check data sim status (primary sim) when data sim is switched on/off.
        self._check_data_sim_is_primary_sim(self._target_sim)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def tear_down(self):
        """
        End and dispose the test
        """
        UseCaseBase.tear_down(self)

        # Restore the current target sim
        self._device.set_target_sim(self._initial_target_sim)

        # Check whether we had to change the PDP context status
        # If initially deactivated, deactivate it
        if self._initial_pdp_context_status == "2":
            # Then deactivate PDP context status
            self._networking_api.deactivate_pdp_context()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def _check_data_state_sim_state(self, target_sim):
        """
        Check data state when data sim is switched on/off.

        .. warning:: It will raise an exception if a check is not ok.

        :type target_sim: int
        :param target_sim: The sim to be tested, usually the data sim.
        """
        self._logger.info("Check data state when data sim is switched on/off.")

        # Configure target sim (used for getter)
        self._device.set_target_sim(target_sim)

        self._logger.info("Putting SIM%d OFF ..." % target_sim)

        # Set target sim power down
        self._modem_api.set_modem_power(0)

        # Wait a while
        time.sleep(self._wait_btwn_cmd)

        self._logger.info("Check data state when data sim is switched OFF ...")

        # pylint: disable=E1101
        # Check data state is well disconnected
        self._modem_api.wait_for_data_state(
            self._uecmd_types.DATA_STATE.DISCONNECTED,
            self._call_setup_time)
        # pylint: enable=E1101

        time.sleep(self._wait_btwn_cmd)

        self._logger.info("Putting SIM%d ON ..." % target_sim)

        # Set target sim power down
        self._modem_api.set_modem_power(1)

        # Wait a while
        time.sleep(self._wait_btwn_cmd)

        self._logger.info("Check data state when data sim is switched ON ...")

        # Wake Up Screen to ensure Connection
        self._phone_system.wake_screen()

        # pylint: disable=E1101
        # Check data state is well connected
        self._modem_api.wait_for_data_state(
            self._uecmd_types.DATA_STATE.CONNECTED,
            self._call_setup_time)
        # pylint: enable=E1101
#------------------------------------------------------------------------------

    def _check_data_sim_secondary_sim_off(self, target_sim):
        """
        Check data state when other sim is switched on/off.

        .. warning:: It will raise an exception if a check is not ok.

        :type target_sim: int
        :param target_sim: The sim to be tested, usually the data sim.
        """
        self._logger.info("Check data state when other sim is switched on/off.")

        # Configure target sim (used for getter)
        self._device.set_target_sim(target_sim)

        # Wake Up Screen to ensure Connection
        self._phone_system.wake_screen()

        # Check data state
        # pylint: disable=E1101
        self._modem_api.wait_for_data_state(
            self._uecmd_types.DATA_STATE.CONNECTED,
            self._call_setup_time)
        # pylint: enable=E1101

        # Use the other sim than targeted one
        self._device.set_target_sim(2 if target_sim == 1 else 1)

        self._logger.info("Putting SIM%d OFF ..." % (self._device.get_target_sim()))

        # Set other than target (data) sim power down
        self._modem_api.set_modem_power(0)

        # Re set the SIM targeted to check data state
        self._device.set_target_sim(target_sim)

        # Wake Up Screen to ensure Connection
        self._phone_system.wake_screen()

        # pylint: disable=E1101
        # Check data is still disconnected state
        self._modem_api.wait_for_data_state(
            self._uecmd_types.DATA_STATE.CONNECTED,
            self._call_setup_time)
        # pylint: enable=E1101

        # Use the other sim than targeted one
        self._device.set_target_sim(2 if target_sim == 1 else 1)

        # Re-Set other than target (data) sim power up
        self._modem_api.set_modem_power(1)

        # Wait a while
        time.sleep(self._wait_btwn_cmd)

        # Re set the SIM targeted
        self._device.set_target_sim(target_sim)

#------------------------------------------------------------------------------
    def _check_data_sim_is_primary_sim(self, target_sim):
        """
        Check data sim status (primary sim) when data sim is switched on/off.

        .. warning:: It will raise an exception if a check is not ok.

        :type target_sim: int
        :param target_sim: The sim to be tested, usually the data sim.
        """
        self._logger.info("Check data sim status "
                          "(primary sim) when data sim is switched on/off.")

        # Configure target sim (used for getter)
        self._device.set_target_sim(target_sim)

        # pylint: disable=E1101
        # Check is target sim is primary sim
        self._modem_api.check_target_sim_role(SIM_ROLE.PRIMARY)
        # pylint: enable=E1101

        # Set target sim power down
        self._modem_api.set_modem_power(0)

        # Wait a while
        time.sleep(self._wait_btwn_cmd)

        # pylint: disable=E1101
        # Check that target sim is still primary sim
        self._modem_api.check_target_sim_role(SIM_ROLE.PRIMARY)
        # pylint: enable=E1101

        # Re Set target sim power up
        self._modem_api.set_modem_power(1)

        # Wait a while
        time.sleep(self._wait_btwn_cmd)

        # Wake Up Screen to ensure Connection
        self._phone_system.wake_screen()

        # pylint: disable=E1101
        # Check data is still disconnected state
        self._modem_api.wait_for_data_state(
            self._uecmd_types.DATA_STATE.CONNECTED,
            self._call_setup_time)
        # pylint: enable=E1101
