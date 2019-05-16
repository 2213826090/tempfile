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
:summary: Implements a test step that pairs the DUT with a BT Device
:since:18/09/2014
:author: ymorelx
"""

from acs_test_scripts.Device.UECmd.UECmdTypes import BT_BOND_STATE
from TestStep.Device.Wireless.BT.Base import BtBase, BTConst


class BtPairAndCheck(BtBase):
    """
    Implements the test step to pair the DUT with another device
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

        pincode = self._pars.pin_code if self._pars.pin_code else "0000"
        passkey = self._pars.pass_key if self._pars.pass_key else 0

        # Pair the DUT with the device
        pair_result = self._api.pair_to_device(address, reconnect, pairing_reply, pincode, passkey)

        if pair_result[0] == BT_BOND_STATE.BOND_NONE:
            if self._pars.paired:
                self._raise_device_exception("Pairing with device failed")
            else:
                return_msg = "Pairing with device rejected"
                self._ts_verdict_msg = return_msg
                self._logger.info(return_msg)
                return

        if pair_result[0] == BT_BOND_STATE.BOND_BONDED:
            if self._pars.paired:
                return_msg = "Pairing with device succeeded"
                self._ts_verdict_msg = return_msg
                self._logger.info(return_msg)
                return
            else:
                self._raise_device_exception("Pairing with device succeeded but it should not")

        self._raise_device_exception("Unknown bond state returned by the device")
