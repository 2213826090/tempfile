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
:summary: This command is used to create a new string out of minimum 2 and max 3 pieces
:since: 3/18/16
:author: mmaraci
"""

from Core.TestStep.TestStepBase import TestStepBase


class GlueCommand(TestStepBase):
    """
    This command is used to create a new string out of minimum 2 and max 3 pieces
    """

    def run(self, context):
        TestStepBase.run(self, context)
        piece1_str = self._pars.piece1
        piece2_str = self._pars.piece2
        output = "{0} {1}".format(piece1_str, piece2_str)
        if self._pars.piece3:
            output = "{0} {1}".format(output, self._pars.piece3)
        self._logger.info("Glued Output: %s", output)
        context.set_info(self._pars.save_as, output)