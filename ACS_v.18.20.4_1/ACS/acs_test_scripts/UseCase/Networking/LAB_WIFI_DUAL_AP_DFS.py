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
:summary: This file implements the LAB WIFI AP DUAL DFS
:since: 09/09/2013
:author: aberthex
"""

from acs_test_scripts.UseCase.Networking.LAB_WIFI_DUAL_BASE import LabWifiDualBase
from acs_test_scripts.UseCase.Networking.LAB_WIFI_DFS import LabWifiDfs
from UtilitiesFWK.Utilities import Global


class LabWifiDualApDfs(LabWifiDualBase, LabWifiDfs):

    """
    Lab Wifi Dual AP Dfs Test class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        LabWifiDualBase.__init__(self, tc_name, global_config)
        LabWifiDfs.__init__(self, tc_name, global_config)

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LabWifiDualBase.set_up(self)
        # Connect DUT to AP 1
        self._networking_api.wifi_connect(self._ssid, False)
        # call the generic LabWifiDsf._set_up() method
        self._set_up()

        return Global.SUCCESS, "no_error"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """

        # trigger dfs on ap1 and check that dut remains connected to ap1
        LabWifiDfs.run_test(self)

        return Global.SUCCESS, "no_error"
