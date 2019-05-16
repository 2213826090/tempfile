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

:organization: INTEL PEG-SVE-DSV
:summary: This test step will use the MCCI USB Connection Exerciser to set the USB speed.
             PREREQUISITES:
             MCCI USB Connection Exerciser (Model 2101) devices must be properly setup and connected to the host PC.
             mcci2101.exe should be installed at the location given from "SCRIPTS_PATH" onto host PC from Artifactory under:  acs_test_artifacts:CONCURRENCY/TESTS/usb_unplug_plug
:since: 12/10/2014
:author: sasmith2
"""
import time
import os
import sys
from Core.TestStep.EquipmentTestStepBase import EquipmentTestStepBase
from ErrorHandling.TestEquipmentException import TestEquipmentException


class MCCISetUsbSpeed(EquipmentTestStepBase):
    """
    Use the MCCI USB Connection Exerciser to set the USB speed.
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        EquipmentTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self.local_computer = None

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        self._logger.info(self._pars.id + ": Test step starting.")
        EquipmentTestStepBase.run(self, context)
        self.mcciappPath = self._pars.scripts_path + " -dev 1"

        self.local_computer = self._equipment_manager.get_computer(self._pars.eqt)
        # Let's try to set the usb speed and fail the test if hardware is improperly set up.
        if self._pars.usb_speed == 1:
            ret = self.USB_HS_On()
            if ret != 0:
                err_msg = ": Issue using MCCI USB Connection Exerciser.  Try checking whether the hardware is correctly set up.  If not HW, try checking SW usage manually or if newer version of mcci2101.exe online"
                raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, self._pars.id + err_msg)
        elif self._pars.usb_speed == 2:
            ret = self.USB_SS_On()
            if ret != 0:
                err_msg = ": Issue using MCCI USB Connection Exerciser.  Try checking whether the hardware is correctly set up.  If not HW, try checking SW usage manually or if newer version of mcci2101.exe online"
                raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, self._pars.id + err_msg)

        self._logger.info(self._pars.id + ": Test step finished.")


    def USB_SS_On(self):
        ret=os.system(self.mcciappPath + ' -ssattach')
        return ret

    def USB_HS_On(self):
        ret=os.system(self.mcciappPath + ' -hsattach')
        return ret

