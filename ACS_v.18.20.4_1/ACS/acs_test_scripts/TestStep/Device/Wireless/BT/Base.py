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
:summary: This file implements a Test Step base class
:since:17/12/2013
:author: fbongiax
"""

from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase


class BTConst(object):
    """
    Bluetooth Test Step constants
    """

    STR_PAR_MAC_ADDRESS = "MacAddress"
    STR_BT_DEVICE = "btDevice"
    STR_LOCAL_CONN = "LocalConnectivity"

    PAIRING_REPLIES = {
        'reject': 0,
        'cancel': 0,
        'accept': 1,
        'timeout': 2
    }

    DFLT_PAIRING_REPLY = 1

class BtBase(DeviceTestStepBase):
    """
    Implements the base test step for BT

    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

        # Get BT device parameters
        self._bt_device = self._config.get(BTConst.STR_BT_DEVICE)

        # # gets handle to local connectivity API
        self._api = self._device.get_uecmd(BTConst.STR_LOCAL_CONN)
