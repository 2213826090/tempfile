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

@organization: UMG PSI Validation
@summary: This file implements the step to set the BT visibility a given computer.

@author: mbisellx

"""

from acs_test_scripts.TestStep.Equipment.Computer.ComputerBase import ComputerBase
from ErrorHandling.TestEquipmentException import TestEquipmentException


class SetComputerBtVisibility(ComputerBase):
    """
    Gets current date and time of a given computer.
    """

    def run(self, context):
        """
        Run the test step
        """
        msg = ""
        ComputerBase.run(self, context)
        if self._check_os_linux():
            if self._check_bt_interface_availability(self._pars.hci_interface):
                msg = self._set_bt_visibility(self._pars.hci_interface, self._pars.enable_visibility)
                if msg != True:
                    msg = "check_bt_interface_availability : Failed to set the computer visible"
                    raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)
            else:
                msg = "check_bt_interface_availability : required hci_interface is not available"
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

    def _check_os_linux(self):
        if self._computer.get_os() != self._computer.LINUX:
            msg = "SetComputerBtVisibility : Not implemented for OS %s" % self._os
            raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED, msg)
        else:
            return True

    def _check_bt_interface_availability(self, hci_interface):
        return self._computer.check_bt_interface_availability(hci_interface)

    def _set_bt_visibility(self, hci_interface, enable_visibility):
        return self._computer.set_bt_visibility(hci_interface, enable_visibility)


