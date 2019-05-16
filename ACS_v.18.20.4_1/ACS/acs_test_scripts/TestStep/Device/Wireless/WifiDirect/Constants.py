"""
@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
:summary: This file cointains the Wifi Direct Constants
:since 21/07/2014
:author: emarchan
"""


class Constants(object):
    """
    Defines Wifi constants
    """

    FILE_NAME_SEPARATOR = ","

    STR_WIFI_DIRECT_UECMD = "WifiDirect"
    STR_NET_WORK_UECMD = "Networking"

    LIST_GO_INTENT_VALUES = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]

    LIST_US_2G_CHANNELS = [2412,2417,2422,2427,2432,2437,2442,2447,2452,2457,2462]

    LIST_US_5G_CHANNELS = [5180,5200,5220,5240,5260,5280,5300,5320,5500,5520,5540,5560,5580,
                           5660,5680,5700,5745,5765,5785,5805,5825]

    LIST_FR_2G_CHANNELS = [2412,2417,2422,2427,2432,2437,2442,2447,2452,2457,2462,2467,2472]

    LIST_FR_5G_CHANNELS = [5180,5200,5220,5240,5260,5280,5300,5320,5500,5520,5540,5560,5580,
                           5600,5620,5640,5660,5680,5700]
