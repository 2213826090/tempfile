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

:summary: This file implements a Test Step for setting a network simulator parameter
:author: jduran4x
:since 12/09/2014
:organization: INTEL QCTV
"""

from Core.TestStep.EquipmentTestStepBase import EquipmentTestStepBase


class NsSetParam(EquipmentTestStepBase):
    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        EquipmentTestStepBase.run(self, context)

        if self._pars.option.lower() == "none":
            self._pars.option = None
        log = "Try setting parameter '%s' " % self._pars.name
        if self._pars.option is not None:
            log += "(option: %s) " % self._pars.option
        log += "to value %s" % self._pars.value
        self._logger.info(log)

        net_sim = self._equipment_manager.get_cellular_network_simulator(self._pars.eqt, visa=True)
        db = net_sim.get_database_access()
        db.verify_parameters(self._pars.name, self._pars.option, self._pars.value)

        command = db.get_command()
        if self._pars.option is not None:
            command = "%s:%s" % (command, self._pars.option)

        cmd = command
        if self._pars.value is not None:
            cmd = "%s %s" % (command, self._pars.value)
        net_sim.send_command(cmd)
