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
:summary: This file implements a class holding pairing information
:since:31/12/2013
:author: fbongiax
"""


class PairingConfig(object):
    """
    Given dictionary from test step extracts pairing config info
    """

    def __init__(self, test_step_args):
        """
        Constructor
        """

        self.address = test_step_args.bdaddr
        self.reconnect = "on" if test_step_args.unpair_first == True else "off"
        self.accept = 1 if test_step_args.accept_pairing is None or test_step_args.accept_pairing == True else 0
        self.pincode = test_step_args.pin_code if test_step_args.pin_code else "0000"
        self.passkey = test_step_args.pass_key if test_step_args.pass_key else 0
