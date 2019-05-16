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
:summary: This file implements the LIVE VC MTMR UC.
This use case will need to phones
:since: 07/11/2011
:author: ssavrimoutou
"""

from LIVE_DUAL_PHONE_VC_BASE import LiveDualPhoneVcBase


class LiveDualPhoneVcMtMr(LiveDualPhoneVcBase):

    """
    Live Voice Call MT/MR.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LiveDualPhoneVcBase.__init__(self, tc_name, global_config)

        # Get phone number of the DUT
        self._phone_number = str(self._device.get_phone_number())

        # Instantiate the instances for phone caller, receiver and releaser
        self._caller_phone = self._voice_call_api2
        self._receiver_phone = self._voice_call_api
        self._releaser_phone = self._voice_call_api
