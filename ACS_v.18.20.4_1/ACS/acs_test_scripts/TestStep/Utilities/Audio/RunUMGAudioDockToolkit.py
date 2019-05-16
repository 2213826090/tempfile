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
otherwise. Any license under such intellectual property rights must be expressed
and approved by Intel in writing.

@summary: This file implements a Test Step to start audio docking toolkit.  This tool
          emulates android accessory dock on a Linux desktop, giving support of Android
          Open Accessory protocol 2.0 for audio over a USB connection.  Stdout and stderr
          will be directed to log files stored in report directory. It plays back
          device audio through Linux desktop speaker/headphone. As an example, it could
          be run with DEVICE/MULTIMEDIA/AUDIO/PLAY_AUDIO_LOOP TS in order to play music
          on DUT that will play through host PC, using the dock toolkit.  Automated
          verification can be added using DEVICE/MULTIMEDIA/AUDIO/MONITOR_AUDIO_STREAM.
          To do this verification, you must also use an audio cable to loop back from
          PC audio output to PC line-in.
@since 25 August 2014
@author: Stephen A Smith
@organization: INTEL PEG-SVE-DSV
"""

import time
import os
import sys
from Core.TestStep.EquipmentTestStepBase import EquipmentTestStepBase
from Device.DeviceManager import DeviceManager
from Core.TestStep.DeviceTestStepBase import TestStepBase
from ErrorHandling.AcsConfigException import AcsConfigException

class RunUMGAudioDockToolkit(EquipmentTestStepBase):
    """
    Start audio docking toolkit on UNIX host PC
    """
    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        EquipmentTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self.local_computer = None

    def run(self, context):
        """
        Run MCG audio docking toolkit

        :type context: TestStepContext
        :param context: test case context
        """
        EquipmentTestStepBase.run(self, context)
        self.local_computer = self._equipment_manager.get_computer("COMPUTER1")

        self._logger.info(self._pars.id + ": Test step starting.")
        # Set report path
        self._device_manager = DeviceManager()
        report_path = self._device_manager.get_global_config().campaignConfig.get("campaignReportTree").create_subfolder('run_umg_audio_dock_toolkit')
        # Use these to log stdout and stderr under the report directory created for this test step in <root>\acs\src\_Reports.
        self._logger.info("{0}: stdout and stderr logs will be stored in this directory - {1}".format(self._pars.id, report_path))
        stdout_log = open(os.path.join(report_path, "dock_toolkit_stdout.log"), 'w')
        stderr_log = open(os.path.join(report_path, "dock_toolkit_stderr.log"), 'w')

        # We use os.name b/c there's a good chance the audio docking files will work on all *ix operating systems
        if os.name != 'posix':
            # We raise an exception so that the rest of this script is not executed
            raise AcsConfigException(self._pars.id + ":  The USB audio docking test is only executable in a Linux-like OS.")

        self._logger.debug(self._pars.id + ":  Report path is {0} folder".format(report_path))
        # Since this is only meant to run on ubuntu host, not worrying about OS type with regards to the cmd being run on host PC.
        try:
            # Make sure we have permission to create log files in report directory.
            os.chmod(report_path, 0o777)
            cur_dir = os.getcwd()
            os.chdir(self._pars.scripts_path)
            self.local_computer.run_cmd("./run", timeout = -1, stdout = stdout_log, stderr = stderr_log)
            os.chdir(cur_dir)
            # We want this test step to last at least as long as we are playing audio on device or longer.  Adding a couple minutes to duration.
            time.sleep((self._pars.duration + 2) * 60)
        except:
            import traceback
            self._logger.error(self._pars.id + ":  Unexpected exception -> " + str(sys.exc_info()[0]))
            self._logger.error(traceback.format_exc())
            self.ts_verdict_msg = self._pars.id + ":  Unexpected exception being raised"
            raise
        finally:
            # Using -1 timeout with run_cmd when launching the dock toolkit let us launch as async process which is stored in self.local_computer._async_process.
            # Since the toolkit will run indefinitely, we must force it to end with terminate() method.
            self.local_computer._async_process.terminate()
        self._logger.info(self._pars.id + ": Test step finished.")
