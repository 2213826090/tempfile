"""
:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
The source code contained or described herein and all documents related
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
:summary: This file implements the LIVE NFC SCAPI APDU EXCHANGE UC
:since: 20/08/2012
:author: lpastor
"""

from LIVE_NFC_BASE import LiveNfcBase
from UtilitiesFWK.Utilities import Global


class LiveNfcScapiApduExchange(LiveNfcBase):

    """
    Live NFC select SE test
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LiveNfcBase.__init__(self, tc_name, global_config)

        # Get channel to use from test case xml file
        self._channel = \
            str(self._tc_parameters.get_param_value("CHANNEL"))
        # Get reader to use from test case xml file
        self._reader = \
            str(self._tc_parameters.get_param_value("READER"))
        # Get APDU case to use from test case xml file
        self._apdu_case = \
            str(self._tc_parameters.get_param_value("APDU_CASE"))
        # Get data length to use from test case xml file
        self._data_length = \
            int(self._tc_parameters.get_param_value("DATA_LENGTH"))
        # Get the number of commands to send to stress the system
        self._loop = \
            int(self._tc_parameters.get_param_value("LOOP"))

    def run_test(self):
        """
        Execute the test
        """
        LiveNfcBase.run_test(self)

        self._nfc_api.exchange_apdu_using_scapi(self._channel, self._reader, self._apdu_case, self._data_length, self._loop)

        return Global.SUCCESS, "No errors"
