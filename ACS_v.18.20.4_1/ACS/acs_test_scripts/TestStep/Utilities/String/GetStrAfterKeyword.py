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
:summary: This command is used to return the string after another string that
is found until the first whitespace
:since: 4/14/16
:author: mvminci
"""

from Core.TestStep.TestStepBase import TestStepBase
from ErrorHandling.AcsConfigException import AcsConfigException
import re


class GetStrAfterKeyword(TestStepBase):
    """
    This command is used to get the string after a string that is found until
    the first whitespace
    """

    def run(self, context):
        TestStepBase.run(self, context)

        keyword_str = self._pars.keyword
        base_str = self._pars.input_string
        regexp = re.search(keyword_str + "(\w+)", base_str)
        if not regexp:
            self._raise_config_exception(AcsConfigException.OPERATION_FAILED,
                                         "Keyword: %s was not found in -\n"
                                         "String: %s" % (keyword_str, base_str))
        result = regexp.group(1)
        self._logger.debug("%s result: %s", self.name, result)
        context.set_info(self._pars.save_as, result)
