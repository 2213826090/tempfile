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
:summary: This file implements a test step to stop http transfers on device
:since 26/11/2014
:author: kturban
"""

from acs_test_scripts.TestStep.Device.Comms.DataXfer.DataXferBase import DataXferBase
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import Global

class HttpDownloadStop(DataXferBase):
    """
    This file implements a test step to stop http transfers on device
    """

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DataXferBase.run(self, context)
        http_status, download_count, error_count = self._networking_api.stop_multiple_http_transfer(self._pars.url,
                                                                                                    self._pars.agent)

        if http_status != "SUCCESS":
            msg = "Error to stop HTTP transfer, result : %s" % (http_status)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        if download_count:
            self._logger.info("{0} transfers have been done".format(download_count))

        if error_count:
            self._logger.warning("{0} transfers have failed".format(error_count))
        else:
            self._logger.info("No transfer error")

        self.ts_verdict_msg = "Transfers properly stopped"
