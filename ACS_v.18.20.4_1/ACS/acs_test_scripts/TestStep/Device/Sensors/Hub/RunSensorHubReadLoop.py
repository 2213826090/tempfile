"""
@summary:
Repeatedly read ALS, proximity, barometer, gyroscope, accelerometer and compass sensors connected to the device. The sensors list and 
valid values are maintained in Sensor_Range.txt which can be found in Artifactory at the following location:
        acs_test_artifacts/CONCURRENCY/TESTS/sensorhub/
This file implements a Test Step (RUN_SENSORHUB_READ_LOOP) to invoke shell script (sensorhub_client.sh) that repeatedly reads sensors 
for the specified amount of time. It requires /data/sensorhub/sensorhub_client.sh to already exist on the system. It differs from 
RUN_MULTISENSOR_READ_LOOP test step which reads a different set of ipc sensors (Accelerometer, Gyroscope, 9DOF, Magnetometer and Barometer).
@since 10 Feb 2014
@author: Aamir Khowaja
@organization: INTEL TMT-TTD-AN

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

class RunSensorHubReadLoop(DeviceTestStepBase):
    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        self._logger.info("RunSensorHubReadLoop: Run")

        self._logger.info("RunSensorHubReadLoop: verify: {0}, duration: {1}, delay {2}".format(self._pars.verify, self._pars.duration, self._pars.delay))

        self.system_api = self._device.get_uecmd("System")
        self.file_api = self._device.get_uecmd("File")

        basename = "sensorhub_client"
        #To make this OS-independent, look for different possible filenames.
        #If anyone enables this in Windows, I assume they will create a DOS batch file to do what sensorhub_client.sh does.
        #Add to this list if required for any other OS.
        possible_extensions = [".sh", ".bat"]
        for ext in possible_extensions:
            #uecmd or underlying code should take care of changing path delimiters as needed
            script = self._pars.scripts_path + "/" + basename + ext
            found = self.file_api.exist(script)
            if found:
                break
        if not found:
            msg = "RunSensorHubReadLoop: Could not find a " + basename + ".* script."
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        #"sensorhub_client usage: sensorhub_client <param_file> <verify> <runtime_in_minutes> <sleep_time_between_each_sample_in_sec>"
        args = "{0} {1} {2} {3}".format("Sensor_Range.txt", self._pars.verify, self._pars.duration, self._pars.delay)

        '''Adding 1 minute to run_cmd timeout as a buffer in case the script happens to go a little longer than expected.
        It can go as long as 1 minute past the desired run time, if it happens to start a new iteration just before the
        time is up.'''
        '''run_shell_executable doesn't return anything, but will raise an exception if it fails'''
        self.system_api.run_shell_executable(script, args, io_redirection=2, timeout=self._pars.duration*60 + 60)

        self._logger.info("RunSensorHubReadLoop: Done")
