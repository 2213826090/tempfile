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
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

:organization: INTEL OTC Android QA
:summary: This file implements a Test Step to unlock a shared resource
:since:05/27/2014
:author: mcchilax
"""

import os

from Core.TestStep.TestStepBase import TestStepBase
from acs_test_scripts.Utilities.LockManager.LockManagerClient import LockEquipementManagerClient


class UnlockResource(TestStepBase):

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        super(UnlockResource, self).run(context)

        shared_resource = LockEquipementManagerClient(self._pars.lock_manager_address,
                                                      self._pars.lock_manager_port,
                                                      self._logger)
        pid = os.getpid()
        info = (pid, self._pars.timeout)
        self._logger.info("Unlocking resource: {0}".format(self._pars.resource))
        shared_resource.get_server_proxy()
        shared_resource.client_proxy.remove_request_resource(self._pars.resource, info)
