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

:organization: CWS
:summary: Connect wired headset implementation Test Step
:since: 02/04/2015
:author: mcarriex
"""

from Core.TestStep.EquipmentTestStepBase import EquipmentTestStepBase
from ErrorHandling.TestEquipmentException import TestEquipmentException


class ConnectHeadset(EquipmentTestStepBase):
    """
    Connect wired headset using io card
    """

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        EquipmentTestStepBase.run(self, context)
        io_card = self._equipment_manager.get_io_card(self._pars.eqt)

        if "WiredHeadset" in self._global_conf.benchConfig.get_parameters_name():
            eqt_wired_headset = self._global_conf.benchConfig.get_parameters("WiredHeadset")
            ground_line = eqt_wired_headset.get_dict()[("wired_headset")]["value"]
            left_line = eqt_wired_headset.get_dict()[("left_wired_headset")]["value"]
            right_line = eqt_wired_headset.get_dict()[("right_wired_headset")]["value"]
            jack_det_n = eqt_wired_headset.get_dict()[("jack_detection")]["value"]
            io_card.enable_line(ground_line)
            io_card.enable_line(left_line)
            io_card.enable_line(right_line)
            io_card.enable_line(jack_det_n)
        else:
            self._logger.warning("WiredHeadset parameter missing !")
