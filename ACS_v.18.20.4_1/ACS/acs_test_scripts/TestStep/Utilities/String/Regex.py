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

:organization: INTEL NDG
:summary: This file implements Regex functions
:since: 13/01/2015
:author: gcharlex
"""
import re
from Core.TestStep.TestStepBase import TestStepBase
from ErrorHandling.AcsConfigException import AcsConfigException


class Regex(TestStepBase):
    """
    Regex command class
    """
    def __init__(self, tc_name, global_config, ts_conf, factory):
        """
        Constructor
        """
        # Call DeviceTestStepBase base Init function
        TestStepBase.__init__(self, tc_name, global_config, ts_conf, factory)


    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        TestStepBase.run(self, context)

        if self._pars.action not in ["SEARCH", "MATCH"]:
            self._raise_config_exception("ACTION argument's value (%s) is invalid" %
                                         self._pars.action)

        if self._pars.action == "SEARCH":
            self._logger.info("Regex search (Pattern: %s in string %s" % (self._pars.pattern, self._pars.string))
            re_result = re.search(self._pars.pattern, self._pars.string)
        elif self._pars.action == "MATCH":
            self._logger.info("Regex match (Pattern: %s in string %s" % (self._pars.pattern, self._pars.string))
            re_result = re.match(self._pars.pattern, self._pars.string)
        result = (re_result is not None)

        if self._pars.save_as is not None:
            context.set_info(self._pars.save_as, result)
            return

        # Invert result if needed
        if not self._pars.pass_if:
            result = not result

        if not result:
            self._raise_config_exception(AcsConfigException.OPERATION_FAILED,
                                         "Pattern search/match not satisfied\n"
                                         "Pattern: %s\n"
                                         "String: %s" % (self._pars.pattern, self._pars.string))
