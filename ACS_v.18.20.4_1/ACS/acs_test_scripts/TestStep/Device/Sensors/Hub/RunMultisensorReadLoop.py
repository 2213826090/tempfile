"""
@summary: Repeatedly read various sensors connected to the device, such as accelerometer, gyroscope, magnetometer,
barometer, light sensor and proximity sensor.  The sensors list is maintained in the shell script
that is invoked by this test step. This uses a device-side script to maintain the activity for an extended amount of time without
intervention from the host. This requires /data/ipc_multisensor/read_sensors_loop.sh to already exist on the system.
This is done to avoid adding traffic to the host-device connection (e.g. ADB for Android) throughout the test.
WARNING: The current implementation of resp_cat makes it take almost an hour to complete one loop, so DURATION will
    be rounded up to the next hour.  Source code for resp_cat can be found in Artifactory at
    acs_test_artifacts/CONCURRENCY/TESTS/ipc_multisensor/resp_cat.rar.
    This file implements a Test Step to repeatedly read sensors for the specified amount of time.
@since 29 July 2014
@author: Jongyoon Choi
@organization: INTEL PEG-SVE-DSV

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
"""

from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException

class RunMultisensorReadLoop(DeviceTestStepBase):
    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        self._logger.info("RunMultisensorReadLoop: Run")

        runtime_minutes = self._pars.duration
        # Convert our minutes to hours and round up
        runtime_hours = runtime_minutes/60
        if runtime_minutes%60 != 0:
            runtime_hours += 1

        self.system_api = self._device.get_uecmd("System")
        self.file_api = self._device.get_uecmd("File")

        basename = "read_sensors_loop"
        #To make this OS-independent, look for different possible filenames.
        #If anyone enables this in Windows, I assume they will create a DOS batch file to do what read_sensors_loop.sh does.
        #Add to this list if required for any other OS.
        possible_extensions = [".sh", ".bat"]
        for ext in possible_extensions:
            #uecmd or underlying code should take care of changing path delimiters as needed
            script = self._pars.scripts_path + "/" + basename + ext
            found = self.file_api.exist(script)
            if found:
                break
        if not found:
            msg = "RunMultisensorReadLoop: Could not find a " + basename + ".* script."
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        args = "{0}".format(runtime_hours)

        '''Adding 60 minutes to run_cmd timeout as a buffer in case the script happens to go a little longer than expected.
        It can go as long as 60 minutes past the desired run time, if it happens to start a new iteration just before the
        time is up.'''
        '''run_shell_executable doesn't return anything, but will raise an exception if it fails'''
        self.system_api.run_shell_executable(script, args, io_redirection=2, timeout=runtime_hours*3600 + 3600)

        self._logger.info("RunMultisensorReadLoop: Done")
