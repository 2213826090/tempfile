"""
:copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
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

:organization: INTEL NDG SW DEV
:summary: This file implements a Test Step to insert/remove wireless charger using io card
:since: 05/02/2015
:author: msouyrix
"""
from Core.TestStep.EquipmentTestStepBase import EquipmentTestStepBase
from ErrorHandling.TestEquipmentException import TestEquipmentException


class InsertWirelessCharger(EquipmentTestStepBase):
    """
    Insert wireless charger using io card
    """

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        EquipmentTestStepBase.run(self, context)
        io_card = self._equipment_manager.get_io_card(self._pars.eqt)
        if not io_card.wireless_charger_connector(True):
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED,
                                         "Fail to insert wireless charger using {0}".format(self._pars.eqt))


class RemoveWirelessCharger(EquipmentTestStepBase):
    """
    Remove wireless charger using io card
    """

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        EquipmentTestStepBase.run(self, context)
        io_card = self._equipment_manager.get_io_card(self._pars.eqt)
        if not io_card.wireless_charger_connector(False):
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED,
                                         "Fail to remove wireless charger using {0}".format(self._pars.eqt))
