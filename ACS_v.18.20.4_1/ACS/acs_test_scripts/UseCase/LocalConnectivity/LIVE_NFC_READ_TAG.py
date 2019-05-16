"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
:summary: This file implements the LIVE NFC Read Tag UC
:since: 11/05/2014
:author: mmorchex
"""

import time
from UtilitiesFWK.Utilities import Global
from LIVE_NFC_BASE import LiveNfcBase


class LiveNfcReadTag(LiveNfcBase):

    """
    Live NFC Read Tag.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LiveNfcBase.__init__(self, tc_name, global_config)

        # Get URI name from TC parameters
        self.__uri = self._tc_parameters.get_param_value("URI")

        # Get UECmdLayer
        self._system_api = self._device.get_uecmd("System")
        self._key_event_api = self._device.get_uecmd("KeyEvent")

    def set_up(self):
        """
        Execute the test
        """
        LiveNfcBase.set_up(self)

        # Get device logger instance
        self._device_logger = self._device.get_device_logger()

        # Add trigger messages to the device logger
        self._device_logger.add_trigger_message(self.__uri)
        self._device_logger.add_trigger_message("android.nfc.action.NDEF_DISCOVERED")

        return Global.SUCCESS, "No errors"

    def run_test(self):
        """
        Execute the test
        """
        LiveNfcBase.run_test(self)

        msg = "reading NFC tag successfully done : %s" % self.__uri
        verdict = Global.SUCCESS

        # Check for resolver activity
        if self._system_api.check_Activity("ResolverActivity"):
            count = 0
            while self._system_api.check_Activity("ResolverActivity") and count < 2:
                self._key_event_api.enter()
                time.sleep(self._wait_btwn_cmd)
                count += 1

            if not self._system_api.check_Activity("ResolverActivity"):

            # Start to search the nfc tag discovering in the logcat log
                ndef_discovered = self._device_logger.get_message_triggered_status("android.nfc.action.NDEF_DISCOVERED")
                uri_check = self._device_logger.get_message_triggered_status(self.__uri)

                self._key_event_api.home()

                if not ndef_discovered:
                    msg = "NDEF discovering failure"
                    verdict = Global.FAILURE

                if ndef_discovered and not uri_check:
                    msg = "Bad NFC tag discovered expected : %s" % self.__uri
                    verdict = Global.FAILURE

            else:
                msg = "Can't go over the 'choose application' pop-up"
                verdict = Global.FAILURE

        else:
            msg = "No tag discovered"
            verdict = Global.FAILURE

        return verdict, msg
