"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

:organization: SII on behalf of INTEL MCG PSI
:summary: This file implements Voice Call UECmds
:since: 11 Feb 2014
:author: vgomberx
"""
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Device.UECmd.Interface.Communication.IVoiceCall import IVoiceCall
from acs_test_scripts.Device.UECmd.Imp.Windows.Common.Base import Base
from acs_test_scripts.Device.UECmd.UECmdDecorator import need

class VoiceCall(Base, IVoiceCall):

    """
    Class that handles all voice call related operations.
    """
    @need('modem')
    def __init__(self, device):
        """
        Initializes this instance.
        """
        Base.__init__(self, device)
        IVoiceCall.__init__(self, device)

    def release(self, check_state=True):
        """
        Releases all voice calls

        :return: None
        """
        self._logger.warning("[NOT IMPLEMENTED] VoiceCall.release")
        # TODO : implements it

    def dial(self, number_to_call, check_state=True, single_dial=False):
        """
        Dials a voice call.

        :type number_to_call: str
        :param number_to_call: number to call (MSISDN)

        :type check_state: boolean
        :param check_state: check call state or not.

        :type single_dial: bool
        :param single_dial: dial a single key for DTMF generating purpose

        :return: None
        """

        self._logger.warning("[NOT IMPLEMENTED] VoiceCall.dial")
        # TODO : implements it

    def wait_for_state(self, state, timeout):
        """
        Waits to reach a voice call state until a timeout.

        :type state: UECmd.VOICE_CALL_STATE
        :param state: expected state (see UECmd.VOICE_CALL_STATE)

        :type timeout: int
        :param timeout: maximum time to wait in seconds

        :return: None
        """

        self._logger.warning("[NOT IMPLEMENTED] VoiceCall.wait_for_state")
        # TODO : implements it
