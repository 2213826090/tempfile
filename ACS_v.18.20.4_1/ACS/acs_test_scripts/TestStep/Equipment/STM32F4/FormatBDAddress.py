"""
:copyright: (c)Copyright 2016, Intel Corporation All Rights Reserved.
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

:organization: INTEL OTC Android SSG
:summary: This test step will format the given BD Address as STM format (0xAAAAAAAAAAAA)
        or as Android format (AA:AA:AA:AA:AA:AA) according to the Test Step parameter
        STM_FORMAT and will save the output as a context variable for further use
:since: 3/29/16
:author: mvminci
"""

from Core.TestStep.EquipmentTestStepBase import EquipmentTestStepBase
import re


class FormatBDAddress(EquipmentTestStepBase):
    """
    This test step will format the given BD Address as STM format (0xAAAAAAAAAAAA)
    or as Android format (AA:AA:AA:AA:AA:AA) according to the Test Step parameter
    STM_FORMAT and will save the output as a context variable for further use
    """

    def run(self, context):
        EquipmentTestStepBase.run(self, context)
        input_bd_address = self._pars.input_bd_addr
        stm_pre_str = '0x'
        if self._pars.return_stm_format:
            if not re.match('^' + '[\:\-]'.join(['([0-9a-f]{2})'] * 6) + '$', input_bd_address.lower()):
                self._raise_config_exception("Expected Android format, but got: {0}".format(input_bd_address))
            return_address = stm_pre_str + input_bd_address.replace(':', '').upper()
        else:
            if not re.match('^' + stm_pre_str + '([0-9a-f]{2}){6}$', input_bd_address.lower()):
                self._raise_config_exception("Expected STM format, but got: %s {0}".format(input_bd_address))
            return_address = ':'.join(input_bd_address[i: i + 2] for i in range(2, 13, 2)).upper()
        self._logger.info("FORMATTED BD_ADDR: %s", return_address)
        context.set_info(self._pars.save_as, return_address)
