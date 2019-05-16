"""
@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

@summary: This file implements a Test Step to disable network interface
    Pre-requisite: Add Equipment COMPUTER1 to Bench_Config XML with Model, IP(Linux only) and username(Linux only).
        Model: LOCAL_COMPUTER or REMOTE_COMPUTER. Only LOCAL_COMPUTER is supported by ACS as for now(09/09/2014)
        IP: Ip address of the equipment. Only required for Linux, and localhost is a common value
        username: login used for the connection. Only required for Linux, and root is a common value
@since 25 Aug 2014
@author: Jongyoon Choi
@organization: INTEL PEG-SVE-DSV
"""
import os
from Core.TestStep.TestStepBase import TestStepBase
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.Equipment.EquipmentManager import EquipmentManager

class DisableNetworkInterfaceHost(TestStepBase):
    """
    Implements a Test Step to disable a network interface on the host pc
    """

    def run(self, context):
        """
        Disable a network interface on the host pc

        :type context: TestStepContext
        :param context: test case context
        """
        TestStepBase.run(self, context)

        # Get parameters
        self._hostpc = EquipmentManager().get_computer(eqt_name="COMPUTER1")
        self._interface = self._pars.interface

        # Bring down the specified network interface
        if 'Linux' in self._hostpc.get_os():
            primaryAddress =  str(self._hostpc.get_eqt_dict().get_param_value("IP", ""))
            username =  str(self._hostpc.get_eqt_dict().get_param_value("username", ""))

            if os.system('ssh {0}@{1} ifconfig {2} down'.format(username, primaryAddress, self._interface)) != 0:
                msg = "Failed to bring the {0} network interface up".format(self._interface)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        else:
            if os.system('netsh interface set interface name="{0}" admin="disabled"'.format(self._interface)) != 0:
                msg = "Failed to bring the {0} network interface down".format(self._interface)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)
