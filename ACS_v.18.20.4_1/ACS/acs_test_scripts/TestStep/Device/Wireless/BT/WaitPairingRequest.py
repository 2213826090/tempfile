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

:organization: INTEL NDG SW
:summary: Implements a test step to wait for pairing request
:since:16/10/2014
:author: ymorelx
"""

from TestStep.Device.Wireless.BT.Base import BtBase, BTConst


class BtWaitPairingRequest(BtBase):
    """
    Implements the test step to wait for pairing request
    """

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """

        BtBase.run(self, context)

        # Collects pairing arguments defaulting the non passed ones
        address = self._pars.bdaddr
        reconnect = "on" if self._pars.unpair_first == True else "off"
        pairing_reply = BTConst.DFLT_PAIRING_REPLY
        key = self._pars.pairing_reply.lower()
        if key in BTConst.PAIRING_REPLIES:
            pairing_reply = BTConst.PAIRING_REPLIES[key]
            self._logger.debug("PAIRING_REPLY in TEST STEP: %s => %d" % (key, pairing_reply))
        timeout = self._pars.timeout
        pincode = self._pars.pin_code if self._pars.pin_code else "0000"
        passkey = self._pars.pass_key if self._pars.pass_key else 0

        # Wait for pairing request
        self._api.wait_for_pairing(address, reconnect, pairing_reply, pincode, passkey, timeout)
