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
:summary: Common base for Wifi Direct unit tests.
:since: 2014-08-05
:author: emarchan

"""
import mock
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from Core.TestStep.TestStepContext import TestStepContext



class UtWifiDirectBase(UTestTestStepBase):
    """
    Common base for Wifi Direct unit tests.
    """
    SAVE_AS_PARAM = "SAVE_AS"
    DEFAULT_GROUP_LIST = ["DIRECT-AE-ACS_DUT1", "DIRECT-1y-ACS_DUT2", "DIRECT-Ec-ACS_DUT3"]
    DEFAULT_SAVE_VAR = "save_var"
    DEFAULT_PEER_ADDR = "AA:AA:AA:AA:AA:AA"
    DEFAULT_PEER_NAME = "ACS_DUT1"
    DEFAULT_PEER_LIST = ["ACS_DUT1", "ACS_DUT2", "ACS_DUT3"]

    def setUp(self):
        """
        Set up
        """
        UTestTestStepBase.setUp(self)
