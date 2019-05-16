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
:summary: This file implements the LIVE VC Line Identification UC.
This use case will need two phones
:since: 05/02/2014
:author: mmorchex
"""

from LIVE_DUAL_PHONE_VC_BASE import LiveDualPhoneVcBase
from UtilitiesFWK.Utilities import Global
import time


class LiveDualPhoneVcLineIdentification(LiveDualPhoneVcBase):

    """
    Live Voice Call Line Identification.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LiveDualPhoneVcBase.__init__(self, tc_name, global_config)

        # Get phone number of the reference phone
        if self._phone2 is not None:
            # Add #31# to phone number to have a private number as caller
            self._phone_number = "%23" + "31" + "%23" + str(self._phone2.get_phone_number())

        # Instantiate the instances for phone caller, receiver and releaser
        self._caller_phone = self._voice_call_api
        self._receiver_phone = self._voice_call_api2
        self._releaser_phone = self._voice_call_api

    def run_test(self):
        """
        Execute the test
        """
        verdict = Global.SUCCESS
        msg = "No errors"
        LiveDualPhoneVcBase.run_test(self)

        # Get last call details
        dialed_number, call_type, used_sim = self._voice_call_api2.get_last_call_details()
        time.sleep(self._wait_btwn_cmd)

        # Check if last call number is a private number
        if int(dialed_number) != -2:
            verdict = Global.FAILURE
            msg = "Line identification result is not as expected: %s" % dialed_number

        return verdict, msg
