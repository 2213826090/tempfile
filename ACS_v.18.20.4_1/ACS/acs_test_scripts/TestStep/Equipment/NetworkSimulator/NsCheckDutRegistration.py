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
otherwise. Any license under such intellectual property rights must be
expressed and approved by Intel in writing.

:summary: This file implements a Test Step to check while DUT is registered on
          networ simulator
:author: pblunie
:since 25/11/2014
:organization: INTEL PNP
"""
import time
from Core.TestStep.EquipmentTestStepBase import EquipmentTestStepBase
from ErrorHandling.TestEquipmentException import TestEquipmentException


class NsCheckDutRegistration(EquipmentTestStepBase):
    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        EquipmentTestStepBase.run(self, context)

        imsi = self._pars.imsi
        timeout = int(self._pars.timeout)

        net_sim = self._equipment_manager.get_cellular_network_simulator(self._pars.eqt, visa=True)
        cell = None
        if self._pars.feature == "2G":
            cell = net_sim.get_cell_2g()
        elif self._pars.feature == "3G":
            cell = net_sim.get_cell_3g()
        elif self._pars.feature in ("TDSCDMA", "TD-SCDMA"):
            cell = net_sim.get_cell_tdscdma()
        elif self._pars.feature in ("4G", "LTE"):
            cell = net_sim.get_cell_4g()

        self._logger.info("Network simulator: Check DUT registration before %d seconds",
                          self._pars.timeout)

        is_registered = False
        start_time = time.time()
        while time.time() - start_time < timeout:
            if cell.is_dut_registered(imsi):
                is_registered = True
                break
            time.sleep(1)

        if is_registered:  # Registration success
            self._logger.info("Network simulator: Registration success!")
        else:  # Registration failure
            msg = "Network simulator: Registration failure after %d seconds!" % timeout
            raise TestEquipmentException(TestEquipmentException.TIMEOUT_REACHED, msg)
