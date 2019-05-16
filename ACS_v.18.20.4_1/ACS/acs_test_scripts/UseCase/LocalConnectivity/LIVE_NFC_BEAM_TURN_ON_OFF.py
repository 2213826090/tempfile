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
:summary: This file implements the LIVE Beam NFC enable/disable UC
:since: 11/04/2013
:author: fchagnea
"""

import time
from UtilitiesFWK.Utilities import Global
from LIVE_NFC_BASE import LiveNfcBase
from ErrorHandling.DeviceException import DeviceException


class LiveNfcBeamTurnOnOff(LiveNfcBase):

    """
    Live NFC Beam on/off test.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LiveNfcBase.__init__(self, tc_name, global_config)

    def run_test(self):
        """
        Execute the test
        """
        LiveNfcBase.run_test(self)
        # enable Beam NFC
        self._nfc_api.enable_nfc_beam()
        time.sleep(self._wait_btwn_cmd)

        # Compare beam status to the one desired
        is_beam_on = self._nfc_api.get_nfc_beam_status()
        if not is_beam_on:
            msg = "Unable to turn beam on"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # disable Beam NFC
        self._nfc_api.disable_nfc_beam()
        time.sleep(self._wait_btwn_cmd)

        # Compare beam status to the one desired
        is_beam_on = self._nfc_api.get_nfc_beam_status()
        if is_beam_on:
            msg = "Unable to turn beam off"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        return Global.SUCCESS, "No errors"
