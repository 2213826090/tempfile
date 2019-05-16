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
:summary: Use Case Live SMS checking delivery report
:since: 17/07/2013
:author: jreynaux
"""

import time
from UtilitiesFWK.Utilities import Global
from LIVE_MESSAGING_BASE import LiveMessagingBase


class LiveSmsCheckDelivery(LiveMessagingBase):
    """
       Use Case Live SMS check delivery reports status class.

       The UC will check the information send back from the
       network in order to check the delivery report.
       For that, ensure that delivery report is activated on the dut
       then send sms with specific parameter.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call UseCase base Init function
        LiveMessagingBase.__init__(self, tc_name, global_config)

        # Read SMS_TRANSFER_TIMEOUT from xml UseCase parameter file
        self._sms_transfer_timeout = \
            int(self._tc_parameters.get_param_value("SMS_TRANSFER_TIMEOUT"))

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        # Call UseCase base Run function
        LiveMessagingBase.run_test(self)

        # Send SMS to equipment using SMS parameters :
        # - SMS_TEXT
        # - DESTINATION_NUMBER
        # check_delivery parameter set to TRUE
        time.sleep(self._wait_btwn_cmd)

        self._sms_api.send_sms(self._destination_number,
                               self._message, check_delivery=True)

        return Global.SUCCESS, "No errors"
