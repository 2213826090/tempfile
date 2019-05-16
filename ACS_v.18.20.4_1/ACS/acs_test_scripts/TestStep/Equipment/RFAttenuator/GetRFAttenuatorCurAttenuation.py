"""
@copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
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

@organization: INTEL MCG PSI
@summary: This file implements a Test Step to get current attenuation value in dB
@since 17/04/2015
@author: thamdi
"""

from Core.TestStep.EquipmentTestStepBase import EquipmentTestStepBase

class GetRFAttenuatorCurAttenuation(EquipmentTestStepBase): 
    """
    get current attenuation value in dB
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Constructor
        """
        EquipmentTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._rf_attn= None
        
    def run(self, context):
        EquipmentTestStepBase.run(self, context)
        self._rf_attn=self._equipment_manager.get_rf_attenuator(self._pars.eqt)
        cur_attn = self._rf_attn.get_attenuation()
        context.set_info(self._pars.save_cur_attenuation_as, cur_attn)
        self.ts_verdict_msg = "VERDICT: %s stored as {0}".format(cur_attn) % self._pars.save_cur_attenuation_as
        self._logger.debug(self.ts_verdict_msg)
        
