"""
@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

:organization: INTEL MCG PSI
:summary: This file implements a Test Step to check IMS registration before timeout
:since 27/03/2015
:author: gcharlex
"""
from acs_test_scripts.TestStep.Device.Wireless.Cellular.CellularBase import CellularBase


class CheckImsRegistration(CellularBase):
    """
    Implements the check Ims Registration test step for Cellular
    """
    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        CellularBase.run(self, context)

        self._logger.info("Check IMS registration before {0}s".format(self._pars.timeou))
        self._networking_api.check_ims_registration_before_timeout(self._pars.timeout)
