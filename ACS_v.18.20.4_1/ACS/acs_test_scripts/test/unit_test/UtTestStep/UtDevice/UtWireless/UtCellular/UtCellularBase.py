"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related to the source code ("Material") are owned by
Intel Corporation or its suppliers or licensors. Title to the Material remains with Intel Corporation or its suppliers
and licensors. The Material contains trade secrets and proprietary and confidential information of Intel or its
suppliers and licensors.

The Material is protected by worldwide copyright and trade secret laws and treaty provisions. No part of the Material
may be used, copied, reproduced, modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual property right is granted to or conferred
upon you by disclosure or delivery of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express and approved by Intel in writing.

:organization: INTEL MCG
:summary: Common base for Cellular unit tests.
:since: 17/09/2014
:author: jfranchx

"""

from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase


class UtCellularBase(UTestTestStepBase):
    """
    Common base for Cellular unit tests.
    """

    FLIGHT_MODE_ON = 1
    FLIGHT_MODE_OFF = 0
    PREFERRED_NETWORK_2G_ONLY = "2G_ONLY"
    PREFERRED_NETWORK_3G_ONLY = "3G_ONLY"
    PREFERRED_NETWORK_4G_ONLY = "4G_ONLY"
    PREFERRED_NETWORK_3G_PREF = "3G_PREF"
    PREFERRED_NETWORK_4G_PREF = "4G_PREF"
    PREFERRED_NETWORK_2G_3G = "2G_3G"
    PREFERRED_NETWORK_CDMA_PREF = "CDMA_PREF"
    PREFERRED_NETWORK_CDMA_ONLY = "CDMA_ONLY"
    PREFERRED_NETWORK_EVDO_ONLY = "EVDO_ONLY"
    PREFERRED_NETWORK_GLOBAL = "GLOBAL"
    PREFERRED_NETWORK_4G_PREF_US = "4G_PREF_US"
    PREFERRED_NETWORK_WORLD_MODE = "WORLD_MODE"

    def setUp(self):
        """
        Set up
        """
        UTestTestStepBase.setUp(self)
