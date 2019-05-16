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
:summary: This file implements the LIVE NFC enable/disable UC
:since: 26/06/2012
:author: lpastor
"""

import time
from UtilitiesFWK.Utilities import Global
from LIVE_NFC_BASE import LiveNfcBase
from ErrorHandling.DeviceException import DeviceException


class LiveNfcTurnOnOff(LiveNfcBase):

    """
    Live NFC on/off test.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LiveNfcBase.__init__(self, tc_name, global_config)

        # Get "Beam control check" from test case xml file (set ON, set OFF)
        self._beam_checked = self._get_beam_check_config()

    def run_test(self):
        """
        Execute the test
        """
        LiveNfcBase.run_test(self)
        # enable NFC
        self._nfc_api.nfc_enable()
        time.sleep(self._wait_btwn_cmd)

        # If Beam to be checked
        if self._beam_used and self._beam_checked:
            is_beam_on = self._nfc_api.get_nfc_beam_status()

            # Compare beam status to the one set at the begining
            if is_beam_on != self._beam_wished_value:
                msg = "Unexpected result! Read beam value is %s instead of %s" % (
                    str(is_beam_on),
                    str(self._beam_wished_value))
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # disable NFC
        self._nfc_api.nfc_disable()
        time.sleep(self._wait_btwn_cmd)

        # If Beam to be checked
        if self._beam_used and self._beam_checked:
            is_beam_on = self._nfc_api.get_nfc_beam_status()

            # Compare beam status to the one set at the begining
            if is_beam_on != self._beam_wished_value:
                msg = "Unexpected result! Read beam value is %s instead of %s" % (
                    str(is_beam_on),
                    str(self._beam_wished_value))
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        return Global.SUCCESS, "No errors"
