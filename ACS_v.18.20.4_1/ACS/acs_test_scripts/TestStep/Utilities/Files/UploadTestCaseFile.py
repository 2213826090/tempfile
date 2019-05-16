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
otherwise. Any license under such intellectual property rights must be expressed
and approved by Intel in writing.

:summary: Upload a file (or folder) on TCR
:since: 02/01/2016
:author: sdubrayx
"""

import os
from Core.Report.scripts.acsZipPush import ZipFolderUtilities
from Core.TestStep.TestStepBase import TestStepBase
from Core.Report.Live.LiveReporting import LiveReporting
from UtilitiesFWK.Utilities import Global

class UploadTestCaseFile(TestStepBase):

    """
    Upload a file (or folder) on TCR
    """
    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        TestStepBase.run(self, context)

        isiteration = False
        if self._tc_parameters.get_b2b_iteration() > 1:
            isiteration = True

        fileretention="SHORT"
        if self._pars.retention and "long" in self._pars.retention.lower():
            fileretention="LONG"

        # If it is a file, send it now
        if os.path.isfile(self._pars.file):
            LiveReporting.instance().send_test_case_resource(self._pars.file, retention=fileretention,
                                                             iteration=isiteration)
            self._logger.error("File %s uploaded to test case" % self._pars.file)
        # If it is a folder, zip and send
        elif os.path.isdir(self._pars.file):
            dirname = os.path.dirname(self._pars.file)
            basename = os.path.basename(self._pars.file)
            zipname = os.path.join(dirname, basename.rsplit(".", 1)[0])
            status, outfile = ZipFolderUtilities.Zip(self._pars.file, zipname)
            if status == Global.SUCCESS:
                LiveReporting.instance().send_test_case_resource(outfile, retention=fileretention,
                                                                 iteration=isiteration)
                self._logger.error("Folder %s uploaded to test case" % self._pars.file)
            else:
                self._logger.error("Could not zip %s, so upload nothing" % self._pars.file)
        else:
            self._logger.error("Could not find file %s, so upload nothing" % self._pars.file)
