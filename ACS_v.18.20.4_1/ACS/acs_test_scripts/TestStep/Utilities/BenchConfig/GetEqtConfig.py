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
:summary: Save equipment config in a context variable
:since: 15/01/2014
:author: gcharlex
"""
from Core.TestStep.TestStepBase import TestStepBase


class GetEqtConfig(TestStepBase):
    """
    Get equipment config class
    """
    def __init__(self, tc_name, global_config, ts_conf, factory):
        """
        Constructor
        """
        # Call TestStepBase base Init function
        TestStepBase.__init__(self, tc_name, global_config, ts_conf, factory)

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        TestStepBase.run(self, context)

        if self._pars.eqt in self._global_conf.benchConfig.get_dict():
            eqt_config = {}
            for key, parameters in self._global_conf.benchConfig.get_dict()[self._pars.eqt].iteritems():
                if isinstance(parameters, dict):
                    eqt_config[key] = parameters["value"]
            context.set_info(self._pars.save_as, eqt_config)
        else:
            self._raise_config_exception("Equipment : %s not found in Bench Config." % self._pars.eqt)