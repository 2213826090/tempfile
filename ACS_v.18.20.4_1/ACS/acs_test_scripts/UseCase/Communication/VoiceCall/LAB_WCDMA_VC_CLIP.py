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
:summary: Use Case WCDMA CLIP.
:since: 19/03/2013
:author: rbertolx
"""

# pylint: disable=C0103
# Disable this pylint error due to ACS naming convention

import time

from UtilitiesFWK.Utilities import Global

from LAB_WCDMA_VC_BASE import LabWcdmaVcBase


class LabWcdmaVcClip(LabWcdmaVcBase):

    """
    Class implementing the CLIP test.
    Setup: Build 3G connection with DUT.
    Test:
        step 1: turn the CLIP off on the network simulator.
        step 2: call the DUT from network simulator. (No need to answer the call)
        step 3: check in the call logs of the DUT that the call number has not
        been displayed.
        step 4: turn the CLIP on on the network simulator.
        step 5: call the DUT from network simulator. (No need to answer the call)
        step 6: check in the DUT call logs that the call number has been displayed.
    WARNING: does not test if the *#30# call returns the right info.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call WCDMA voice call  base Init function
        LabWcdmaVcBase.__init__(self, tc_name, global_config)

        self._phone_number = \
            str(self._tc_parameters.get_param_value("PHONE_NUMBER"))
        if self._phone_number.upper() == "[PHONE_NUMBER]":
            self._phone_number = str(self._device.get_phone_number())

#------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """
        # Call WCDMA VoiceCall base run_test function
        LabWcdmaVcBase.run_test(self)
        # Release any previous call (Robustness)
        self._voice_call_api.release()
        # Turn off the CLIP.
        self._ns_voice_call_3g.include_calling_party_number("OFF")
        # Calling the DUT from the network simulator.
        self._ns_voice_call_3g.mt_originate_call()
        # Waiting for the phone to receive the call, not need to answer the
        # call.
        # pylint: disable=E1101
        # As pylint is not able to resolve enum types
        self._voice_call_api.wait_for_state(self._uecmd_types.
                                            VOICE_CALL_STATE.INCOMING,
                                            self._call_setup_time)
        # pylint: enable=E1101
        # Once the alerting state has been reached release the call on the
        # network simulator.
        self._ns_voice_call_3g.voice_call_network_release()
        time.sleep(self._wait_btwn_cmd)
        # Getting the last call information from DUT.
        (number, call_type, _sim) = self._voice_call_api.get_last_call_details()
        # Checking the last call type is missed.
        self._logger.info("Checking last call type is MISSED.")
        # pylint: disable=E1101
        # As pylint is not able to resolve enum types
        if call_type != str(self._uecmd_types.VOICE_CALL_TYPE.MISSED):
            # pylint: enable=E1101
            return (Global.FAILURE, "Last call should be MISSED, is: %s"
                                    % call_type)
        # Checking the last call number not displayed on the DUT.
        self._logger.info("Checking last call number is private")
        # "" is the phone number returned when the number is unknown
        if number != "":
            return (Global.FAILURE, "The CLIP is not included but the phone"
                                    " number was shown. Incoming number: %s"
                                    % number)
        time.sleep(self._wait_btwn_cmd)
        # Turn the CLIP on.
        self._ns_voice_call_3g.include_calling_party_number("ON")
        self._ns_voice_call_3g.set_calling_party_pi("ALLOWED")
        # Set the phone number the network simulator will use to call the DUT.
        self._ns_voice_call_3g.set_calling_party_number(self._phone_number)
        # Calling the DUT from the network simulator.
        self._ns_voice_call_3g.mt_originate_call()
        # Waiting for the phone to receive the call, not need to answer the
        # call.
        # pylint: disable=E1101
        # As pylint is not able to resolve enum types
        self._voice_call_api.wait_for_state(self._uecmd_types.
                                            VOICE_CALL_STATE.INCOMING,
                                            self._call_setup_time)
        # pylint: enable=E1101
        self._ns_voice_call_3g.voice_call_network_release()
        time.sleep(self._wait_btwn_cmd)
        # Getting the last call information from DUT.
        (number, call_type, _sim) = self._voice_call_api.get_last_call_details()
        # Checking the last call type is missed.
        self._logger.info("Checking last call type is MISSED.")
        # pylint: disable=E1101
        # As pylint is not able to resolve enum types
        if call_type != str(self._uecmd_types.VOICE_CALL_TYPE.MISSED):
            # pylint: enable=E1101
            return (Global.FAILURE, "Last call should be MISSED, is: %s"
                                    % call_type)
        # Checking the last call number is the expected one.
        self._logger.info("Checking last call number is displayed on DUT and"
                          " is the same as the one given as parameter.")
        if number != self._phone_number:
            return (Global.FAILURE, "The CLIP is included but the phone"
                                    " number shown on the DUT is not the same"
                                    " as the  one set on the network"
                                    " simulator. Incoming number: %s" % number)
        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def tear_down(self):
        """
        Closing the test.
        """
        # Turn off the CLIP.
        self._ns_voice_call_3g.include_calling_party_number("OFF")
        # Release all possible ongoing calls.
        self._ns_voice_call_3g.voice_call_network_release()
        # Call WCDMA VoiceCall base tear_down function
        LabWcdmaVcBase.tear_down(self)
        return Global.SUCCESS, "No errors"
