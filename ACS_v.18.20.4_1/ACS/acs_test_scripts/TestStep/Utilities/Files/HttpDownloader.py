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
:summary: This file implements a Test Step to download from http/https server
:since:12/02/2014
:author: kturban
"""


from Core.TestStep.TestStepBase import TestStepBase
from ErrorHandling.AcsToolException import AcsToolException
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.Utilities.HttpDownloaderUtil import HttpDownloaderUtil
from UtilitiesFWK.Utilities import Global


class HttpDownloader(TestStepBase):

    """
    Download from http server
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Constructor
        """
        TestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._http_downloader_obj = None

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        TestStepBase.run(self, context)

        self._http_downloader_obj = HttpDownloaderUtil(url=self._pars.url,
                                                       destination=self._pars.destination,
                                                       proxy=self._pars.http_proxy,
                                                       creds=self._pars.credential,
                                                       http_timeout=self._pars.http_timeout,
                                                       override=self._pars.override_file,
                                                       download_timeout=self._pars.transfer_timeout,
                                                       logger=self._logger)
        try:
            response = self._http_downloader_obj.init()
        except AcsConfigException:
            raise

        if not response:
            error_msg = "Cannot get response from server"
            raise AcsToolException(AcsToolException.OPERATION_FAILED, error_msg)

        verdict, output, _ = self._http_downloader_obj.download(response)
        if verdict != Global.SUCCESS:
            raise AcsToolException(AcsToolException.OPERATION_FAILED, output)
        else:
            self.ts_verdict_msg = output
