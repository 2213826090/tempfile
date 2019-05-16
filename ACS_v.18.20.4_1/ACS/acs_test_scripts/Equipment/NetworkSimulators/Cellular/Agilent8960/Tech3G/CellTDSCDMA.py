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
:summary: cell 3G TD-SCDMA implementation for Agilent 8960
:since: 24/07/2014
:author: mbrisbax
"""
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Agilent8960.Tech3G.Cell3G import Cell3G
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Agilent8960.Tech3G.DataTDSCDMA import DataTDSCDMA
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Agilent8960.Tech3G.Messaging3G import Messaging3G
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Agilent8960.Tech3G.VoiceCall3G import VoiceCall3G
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Agilent8960.Tech3G.TestMode3G import TestMode3G
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Interface.ICell3G import ICell3G


class CellTDSCDMA(Cell3G):
    """
    Cell 3G TD-SCDMA implementation for Agilent 8960
    """
    def __init__(self, root):
        """
        Constructor
        :type root: weakref
        :param root: a weak reference on the root class (Agilent8960)
        """
        ICell3G.__init__(self)
        self._root = root
        self._messaging = Messaging3G(root)
        self._voicecall = VoiceCall3G(root)
        self._testmode = TestMode3G(root)
        # overload __data with TDSCDMA
        self._data = DataTDSCDMA(root)
