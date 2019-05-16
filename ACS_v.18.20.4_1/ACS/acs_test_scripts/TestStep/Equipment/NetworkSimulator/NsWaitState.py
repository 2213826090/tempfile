"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
otherwise. Any license under such intellectual property rights must be expressed
and approved by Intel in writing.

:summary: This file implements a Test Step for waiting a network simulator's status
:author: jduran4x
:since 12/09/2014
:organization: INTEL NDG
"""

from Core.TestStep.EquipmentTestStepBase import EquipmentTestStepBase
from ErrorHandling.TestEquipmentException import TestEquipmentException
import time


class NsWaitState(EquipmentTestStepBase):
    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """

        EquipmentTestStepBase.run(self, context)
        if self._pars.option.lower() == "none":
            self._pars.option = None

        net_sim = self._equipment_manager.get_cellular_network_simulator(self._pars.eqt, visa=True)
        db = net_sim.get_database_access()
        db.verify_parameters(self._pars.name, self._pars.option)

        command = db.get_command()
        query_function = net_sim.get_interface().query
        if self._pars.option is not None:
            command = "%s:%s?" % (command, self._pars.option)
        else:
            command += "?"

        log = True
        start_time = time.time()
        end_time = start_time + self._pars.timeout
        while time.time() <= end_time:
            state = query_function(command, log)
            # log only the first command
            if log:
                log = False
            if state.lower() == self._pars.state.lower():
                self._logger.info("Expected connection state (%s) reached" % state)
                break
        else:
            raise TestEquipmentException(TestEquipmentException.MEASUREMENT_ERROR,
                                         "expected state (%s) not reached" % self._pars.state)
