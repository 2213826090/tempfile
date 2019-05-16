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

@summary: This file implements a Test Step to get average http download rate of the host
@since 25 Aug 2014
@author: Jongyoon Choi
@organization: INTEL PEG-SVE-DSV
"""
import time
import os
import socket
import urllib
import tempfile
from Core.TestStep.DeviceTestStepBase import TestStepBase
from ErrorHandling.DeviceException import DeviceException

class GetHostHttpDownloadRate(TestStepBase):
    """
    Implements a Test Step to get average http download rate of the host
    """

    def run(self, context):
        """
        Get average http download rate of the host

        :type context: TestStepContext
        :param context: test case context
        """
        TestStepBase.run(self, context)

        self._http_src_url = self._pars.http_source_url
        self._runTime = self._pars.test_time
        self._repetition = self._pars.repetition

        # Create the target download directory path under temporary directory
        temp_path = tempfile.gettempdir()
        self._targetPath = os.path.join(temp_path, "GetHostHttpDownloadRate_{0}".format(time.strftime('%Y%m%d_%H%M', time.localtime())))

        self._logger.debug("make {0} folder".format(self._targetPath))
        try:
            os.makedirs(self._targetPath)
        except Exception, e:
            msg = "Failed to create local download directory, \"{0}\": {1}".format(self._targetPath, e)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # Set a timeout for socket connections to allow a download to fail, rather than
        # hang, if the http connection goes down during a file download
        socket.setdefaulttimeout(180)

        # Estimate average tether downlink data rate
        accCount = self._repetition
        dlRateAccum = 0
        try:
            self._logger.info("Estimating http download rate of the host ...")
            download_file = os.path.join(self._targetPath,'testfile1.txt')
            for i in range(0,accCount):
                startTime = time.time()
                urllib.urlretrieve(self._http_src_url+'/2GBFiles/1.5GBFiles/1GBFiles/768MBFiles/512MBFiles/256MBFiles/testfile1.txt', download_file)
                elapsedTime = float(time.time() - startTime)

                if not os.path.exists(download_file):
                    msg = "Failed to download to {0}".format(download_file)
                    raise DeviceException(DeviceException.OPERATION_FAILED, msg)
                else:
                    download_file_size = os.path.getsize(download_file)

                dlRate = download_file_size/elapsedTime
                self._logger.info("Iteration {0}: downloaded {1} bytes for {2} seconds: {3} KB per second".format(i, download_file_size, elapsedTime, round(dlRate/1024)))

                dlRateAccum += dlRate

            os.remove(os.path.join(self._targetPath,'testfile1.txt'))
            dlRate_BperSec = dlRateAccum/accCount
            self._logger.info("Estimated http download rate is {0} KB per second...".format(round(dlRate_BperSec/1024)))
        except Exception, e:
            if '[Errno socket error]' in e:
                e += ': Could not access the file server at %s'%self._http_src_url
                e += '\n\tMake sure that a local HTTP file server has been setup and is available at the above address.'
                e += '\n\tIf not, documentation for setting up an HTTP file server on a local host can be found on Artifactory:'
                e += '\n\thttps://tlsstor001.tl.intel.com/artifactory/simple/acs_test_artifacts/TEST_FILES/multi_size_text_files_with_checksums/test_files.tgz '

            msg = "Failed to estimate data rate: {0}".format(e)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # Write dlRate_BperSec to the context so that it can be used by other TestSteps
        context.set_info(self._pars.avg_xfer_rate_sec, dlRate_BperSec)


