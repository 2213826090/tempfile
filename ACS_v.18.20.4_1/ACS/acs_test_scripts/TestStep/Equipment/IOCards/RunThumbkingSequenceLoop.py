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
:summary: This test step will toggle Thumbking's GPIO pins for the test duration. The specific sequence that will be looped for the test duration is defined by COMMAND_LIST.
         PREREQUISITES:
         Install TTK software from \\goto\teak.  Currently using version 1.6.3.
         Install ThumbKing hardware and configure GPIO pin connections as needed for your usecase.
            Hardware guide:  http://pcevwiki.amr.corp.intel.com/ttk/docs/Thumbking%20User%20Guide.pdf
            Note:  The pin description on p.26 is not correct.  Swap CD numbers to opposite side.  i.e. CD0 -> pin 25, CD1 -> pin 26.
         TTkClientLib.dll should be installed at the location given from "SCRIPTS_PATH" onto host PC from Artifactory under:  acs_test_artifacts:CONCURRENCY/TESTS/low_speed_interrupt
         These files all put on host PC in same location as TTkClientLib.dll:
         Intel Ave Usage Statistics Client.dll
         Jayrock.Json.dll
         log4net.dll
         PinControl.dll
         PinControl.pdb
         ThumbKingAPI.dll
         ThumbKingAPI.pdb
         TTkClientLib.dll

:since: 18/09/2014
:author: sasmith2
"""
import time
from Core.TestStep.EquipmentTestStepBase import EquipmentTestStepBase
from ErrorHandling.TestEquipmentException import TestEquipmentException
import clr
import sys
import os


class RunThumbkingSequenceLoop(EquipmentTestStepBase):
    """
    Toggles GPIO pins using ThumbKing device for the test duration.
    """

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        self._logger.info(self._pars.id + ": Test step starting.")
        EquipmentTestStepBase.run(self, context)

        sys.path.append(self._pars.binaries_path)
        clr.AddReference("TTkClientLib")
        clr.AddReference("ThumbKingAPI")
        from ThumbKingAPI import ThumbKingFactory
        ttk = ThumbKingFactory().GetConnectedDevice(int(self._equipment_manager.get_global_config().benchConfig.get_parameters(self._pars.eqt).get_param_value('Device')))
        pin_list = self._pars.pin_list
        pin_list = pin_list.replace(' ','').split(',')
        index = 0
        for pin in pin_list:
            pin_list[index] = int(pin)
            # Pins are defined as inputs by default.  Make them output pins by writing to them.
            ttk.WritePin(pin_list[index], True)
            index += 1
        start_time = time.time()
        test_duration = self._pars.duration*60
        iteration = 1
        try:
            while time.time() - start_time < test_duration:
                self._logger.debug("{0}:  Starting iteration {1}".format(self._pars.id, iteration))
                for pin in pin_list:
                    ttk.WritePin(pin, False)
                    time.sleep(self._pars.pulse_length)
                    ttk.WritePin(pin, True)
                    time.sleep(self._pars.sleep_length)
                iteration += 1
        except:
            import traceback
            self._logger.error(self._pars.id + ":  Unexpected exception -> " + str(sys.exc_info()[0]))
            self._logger.error(traceback.format_exc())
            self.ts_verdict_msg = self._pars.id + ":  Unexpected exception being raised"
            raise
        self._logger.info(self._pars.id + ": Test step finished.")
