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

:organization: INTEL MCG PSI
:summary: This file implements a Test Step to check DUT RAT (network type)
:since: April 1st 2015
:author: Martin Brisbarre
"""

from acs_test_scripts.TestStep.Device.Wireless.Cellular.CellularBase import CellularBase


class CheckNetworkType(CellularBase):
    """
    Implements the test step to check DUT registration network type
    """

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """

        CellularBase.run(self, context)
        # Check that DUT is registered on the good RAT
        self._modem_api.check_network_type_before_timeout(self._pars.network_type,
                                                          self._pars.timeout)
