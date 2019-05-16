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

:organization: INTEL PEG SVE DSV
:summary: Analyze Youtube playback result. This optional test step can be used after running a shell script
    that controls and monitors YouTube video playback, and that logs the state transitions for later analysis. This step consumes that log
:since: 07/14/14
:author: jongyoon
"""
import os
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException
from Device.DeviceManager import DeviceManager
from acs_test_scripts.Utilities.VideoUtilities import AwesomePlayerParser

class AnalyzeYoutubeLoop(DeviceTestStepBase):

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Constructor
        """
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

        self._device_manager = DeviceManager()
        self._parser = AwesomePlayerParser()

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        self._logger.info("PlayYoutubeLoop: Run")

        # Awsomeplayer flag file in device
        flags_file_device_path = self._pars.file_path + "/" + self._pars.status_file
        flag_file_name = os.path.basename(flags_file_device_path)

        # Analyze our results. The awesome_player_flags.log file has a breakdown of the flags for every loop.
        report_path =  self._device.get_report_tree().get_report_path()
        tc_name = self._tc_parameters.get_name()
        report_root = os.path.join(report_path, tc_name, 'play_youtube_loop')
        flags_file_host_path = os.path.join(report_root, flag_file_name)

        #Set parser attributes
        self._parser._src_flags_file_path = flags_file_device_path
        self._parser._dest_flags_file_path = flags_file_host_path

        result = self._parser.pull_status_file_to_host(self._device)
        if result != True:
            msg = 'Failed to pull a flag summary file at %s. Failing test.'%report_root
            self.ts_verdict_msg = msg
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        all_loop_stats = self._parser.parse_status_file()
        if all_loop_stats is not None:
            # Print our stats
            self._logger.debug('Percentage of time spent (Min Max Avg) with the following flags set:')
            for flag in all_loop_stats:
                if all_loop_stats[flag][0] == 1.5:
                    all_loop_stats[flag][0] = 0
                self._logger.debug('%s: %.0f%% %.0f%% %.0f%%'%(flag, all_loop_stats[flag][0]*100, all_loop_stats[flag][1]*100, all_loop_stats[flag][2]*100))
        else:
            msg = 'Unable to find a flag summary file at %s. Failing test.'%flags_file_device_path
            self.ts_verdict_msg = msg
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
