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

:summary: This file implements a Test Step to run sync stress from a host using MTP.
    It continuously sync a file for a given time.
:since: 07/01/2014
:author: Jongyoon Choi
:organization: INTEL PEG-SVE-DSV
"""

import time
from time import strftime
import os
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.Utilities.MtpUtilities import MtpUtility
from acs_test_scripts.Utilities.MtpUtilities import MediaFile
from acs_test_scripts.Utilities.MtpUtilities import MtpSyncConfiguration

class RunMtpSync(DeviceTestStepBase):

    """
    Test case designed to emulate file sync operations using the Media Transfer Protocol (MTP)

    Initial steps:
    1. install windows media player (need to verify if this is really necessary)

    """

    VERSION = "TestStep MtpSync 1.0"
    LOG_FILE_NAME = "MtpSync_Log.txt"  # the standard name that will be used for this test's log file

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        DeviceTestStepBase.run(self, context)

        self._logger.info('USB_MTP_Sync: Run')

        testFilesDir = os.path.dirname(self._pars.test_files_path)
        mediaFilesDir = os.path.dirname(self._pars.resource_files_path)

        testRunnerConfig = MtpSyncConfiguration()
        testRunnerConfig.test_files_path = testFilesDir
        testRunnerConfig.test_content_path = mediaFilesDir
        testRunnerConfig.test_duration_minutes = self._pars.duration
        testRunnerConfig.test_log_dir = testFilesDir

        testRunnerConfig.target_media_files = []
        for file_obj in os.listdir(mediaFilesDir):
            fname = os.path.basename(file_obj)
            testRunnerConfig.target_media_files.append(MediaFile(fileName=fname, localPath=os.path.join(mediaFilesDir, fname), targetFilePath="/acs_media/usb_mtp_sync/" + fname))

        self._logger.info("Running MtpSync with the following configuration: " + os.linesep + str(testRunnerConfig))

        self._mtpUtility = MtpUtility(testRunnerConfig)

        self.loop_count = 1
        self.target_end_time = None

        self._mtpUtility.clear_progress_indicators()

        self._mtpUtility.acquire_target_device()

        self._mtpUtility.fetch_root_filesystem_id()

        self._mtpUtility.check_for_local_media_files(self._mtpUtility.configuration.target_media_files)

        self._mtpUtility.generate_media_file_signatures(self._mtpUtility.configuration.target_media_files)

        start_time = float(time.time())
        self.target_end_time = start_time + (float(self._mtpUtility.configuration.test_duration_minutes * 60))
        self.loop_count = 1
        overall_test_pass_count = 0
        overall_test_fail_count = 0

        while self.run_another_iteration():
            iteration_pass_count, iteration_fail_count = self._mtpUtility.sync_using_mtp_over_usb()
            self._logger.info('Iteration ' + str(self.loop_count) + ' - Pass: ' + str(iteration_pass_count) + ', Fail: ' + str(iteration_fail_count))
            overall_test_pass_count += iteration_pass_count
            overall_test_fail_count += iteration_fail_count
            self.loop_count += 1

        self._logger.info('Totals - PASS: ' + str(overall_test_pass_count) + ', FAIL: ' + str(overall_test_fail_count))

        if overall_test_fail_count > 0:
            msg = "MtpSync FAILED with %d failures"%overall_test_fail_count
            self.ts_verdict_msg = msg
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    def run_another_iteration(self):
        """
        Determines whether we should run another iteration based on Configuration.max_test_iterations or Configuration.test_duration_minutes

        :return: boolean
        """
        self._logger.info(os.linesep + os.linesep + os.linesep + os.linesep)

        if self._mtpUtility.configuration.max_test_iterations > 0 and self.loop_count <= self._mtpUtility.configuration.max_test_iterations:
            self._logger.info("************************************************")
            self._logger.info("*** Running iteration " + str(self.loop_count) + " of " + str(self.configuration.max_test_iterations))
            self._logger.info("************************************************")
            return True

        current_time = time.time()
        if self._mtpUtility.configuration.max_test_iterations <= 0 and (current_time < self.target_end_time):
            self._logger.info("************************************************")
            self._logger.info("*** Running iteration " + str(self.loop_count))
            self._logger.info("*** Current Time = " + strftime("%a, %d %b %Y %H:%M:%S", time.localtime(current_time)))
            self._logger.info("*** Target End Time = " + strftime("%a, %d %b %Y %H:%M:%S", time.localtime(self.target_end_time)))
            self._logger.info("************************************************")
            return True

        return False



