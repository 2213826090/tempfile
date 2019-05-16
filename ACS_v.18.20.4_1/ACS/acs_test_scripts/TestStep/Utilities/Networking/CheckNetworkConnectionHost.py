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

@summary: Check network connection of the host pc.
@since 25 Aug 2014
@author: Jongyoon Choi
@organization: INTEL PEG-SVE-DSV
"""
import time
import urllib
from Core.TestStep.TestStepBase import TestStepBase
from ErrorHandling.DeviceException import DeviceException

class CheckNetworkConnectionHost(TestStepBase):
    """
    Implements a Test Step to check network connection
    """

    def run(self, context):
        """
        Check network connection

        :type context: TestStepContext
        :param context: test case context
        """
        TestStepBase.run(self, context)

        timeout = self._pars.time_out_sec

        test_start_time = time.time()
        while (time.time() - test_start_time < timeout):
            time.sleep(10)

            # Check current state
            try:
                urllib.urlopen(self._pars.test_url).read()
                break
            except Exception:
                self._logger.info("Host network interface is not ready. Waiting for connection")

        if (time.time() - test_start_time > timeout):
                msg = "Failed to connect to {0} within {1} second CheckNetworkConnectionHost timeout".format(self._pars.test_url, timeout)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)
