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
:summary: This file implements the LIVE VC MOMR UC for DUAL SIM phone
:since: 29/01/2013
:author: jreynaux
"""

import time

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global


class LiveDsdsVcMoMr(UseCaseBase):

    """
    Live DSDS Voice Call MO/MR using first sim then secondary sim.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        UseCaseBase.__init__(self, tc_name, global_config)

        # Read callSetupTimeout from Phone_Catalog.xml
        self._call_setup_time = \
            int(self._dut_config.get("callSetupTimeout"))

        # Get Test Cases Parameters
        self._numtocall = [self._tc_parameters.
                           get_param_value("PHONE_NUMBER1"),
                           self._tc_parameters.
                           get_param_value("PHONE_NUMBER2")]

        self._callduration = \
            int(self._tc_parameters.get_param_value("CALL_DURATION"))

        # Get UECmdLayer
        self._voicecall_api = self._device.get_uecmd("VoiceCall")
        self._networking_api = self._device.get_uecmd("Networking")

        self._initial_target_sim = None

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Initialize the test
        """

        UseCaseBase.set_up(self)

        # Backup the current target sim
        self._initial_target_sim = self._device.get_target_sim()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """
        UseCaseBase.run_test(self)

        # Disable flight mode
        self._networking_api.set_flight_mode("off")

        (status, output) = self._dial_from_sim(1)

        if status == Global.FAILURE:
            return status, output

        (status, output) = self._dial_from_sim(2)

        return status, output

#------------------------------------------------------------------------------
    def tear_down(self):
        """
        End and dispose the test
        """
        UseCaseBase.tear_down(self)

        # Restore the current target sim
        self._device.set_target_sim(self._initial_target_sim)

        return Global.SUCCESS, "No errors"

    def _dial_from_sim(self, sim_index):
        """
        Dial the number defined on the Use Case using the given sim index.

        :type sim_index: int
        :param sim_index: The sim index to be used to place the call

        :rtype: tuple
        :return: The ouput status and log message
        """

        self._logger.info("Dialing from SIM" + str(sim_index) + " ...")

        # Configure target sim
        self._device.set_target_sim(sim_index)

        time.sleep(self._wait_btwn_cmd)

        # Release any previous call (Robustness)
        self._logger.info("Release all calls ...")
        self._voicecall_api.release()

        self._logger.info("Dialing %s from SIM%s ..."
                          % (str(self._numtocall[sim_index - 1]), str(sim_index)))
        self._voicecall_api.dial(self._numtocall[sim_index - 1])

        self._logger.info("Wait for call duration: "
                          + str(self._callduration) + "s...")
        time.sleep(self._callduration)

        # pylint: disable=E1101
        # Because pylint can not resolve enum.
        self._voicecall_api.check_state(
            self._uecmd_types.VOICE_CALL_STATE.ACTIVE)
        # pylint: enable=E1101

        self._logger.info("Release all calls ...")
        self._voicecall_api.release()

        (number, call_type, sim) = self._voicecall_api.get_last_call_details()
        # pylint: disable=E1101
        # Because pylint can not resolve enum.
        # Checking last call is an outgoing one.
        if call_type != str(self._uecmd_types.VOICE_CALL_TYPE.OUTGOING):
            # pylint: enable=E1101
            return (Global.FAILURE,
                    "The last call is not an outgoing one. Call type is: %s"
                    % call_type)
        # If NOT right sim used to call
        if "SIM" + str(sim_index) != sim:
            return (Global.FAILURE,
                    "Wrong SIM used: %s (%s), expected SIM%s (%s)"
                    % ("SIM" + str(sim), number,
                       "SIM" + str(sim_index),
                       self._device.get_target_phone_number()))
        else:
            return Global.SUCCESS, "No errors"
