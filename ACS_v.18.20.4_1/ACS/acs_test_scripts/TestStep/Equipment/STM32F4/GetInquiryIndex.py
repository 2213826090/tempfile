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

:organization: INTEL OTC Android SSG
:summary: This test step will use the output of DisplayInquiryList command sent on the STM board and search
        through it for the index associated to the given MAC_ADDRESS searched. It will the save this index as a context
        variable and make it available for further use
:since: 3/18/16
:author: mmaraci
"""

from Core.TestStep.EquipmentTestStepBase import EquipmentTestStepBase


class GetInquiryIndex(EquipmentTestStepBase):
    """
    This test step will use the output of DisplayInquiryList command sent on the STM board and search
        through it for the index associated to the given MAC_ADDRESS searched. It will then save this index as a context
        variable and make it available for further use"""

    def run(self, context):
        EquipmentTestStepBase.run(self, context)
        for line in self._pars.parse_output.strip("[]").split("',"):
            if self._pars.mac_address.replace(":", "") in line:
                context.set_info(self._pars.save_as, line.split(",")[0].split(":")[1].strip())