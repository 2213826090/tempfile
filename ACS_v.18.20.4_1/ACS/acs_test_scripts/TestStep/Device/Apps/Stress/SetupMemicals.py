"""
@summary: This file implements a Test Step to run a shell script that invokes an
memory stress test called "memicals".
Prerequisites: the following scripts and binaries must be located in the path
specified by SCRIPTS_PATH.
    memicals
    runMemicals.sh
These files can be found in this tarball in Artifactory:
    acs_test_artifacts/CONCURRENCY/TESTS/memicals/memicals.tgz.
@since 25 September 2014
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
import numpy as np

class SetupMemicals(DeviceTestStepBase):
    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        self._logger.info("SetupMemicals: Run")
        phonesystem_api = self._device.get_uecmd("PhoneSystem")

        #Validate the start and end addresses. It should be a hexa decimal format
        try:
            start_address = int(self._pars.start_address, 16)
        except ValueError:
            msg="SetupMemicals: Start address {0} is not hexadecimal".format(self._pars.start_address)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        if start_address < 0x30000000:
            msg="SetupMemicals: Start address {0} is too low. It should be at least 0x30000000".format(self._pars.start_address)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        try:
            end_address = int(self._pars.end_address, 16)
        except ValueError:
            msg="SetupMemicals: End address {0} is not hexa decimal".format(self._pars.end_address)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # Returned value is kB
        cmd = 'adb shell "cat /proc/meminfo | grep MemTotal"'
        result, output = self._device.run_cmd(cmd, 60)
        try:
            # Calculate exact memory size by rounding up (multiple of 512MB) and subtracting reserved 18 MB
            avaiable_memory_size = int(np.ceil(int(output.split()[1]) / (512.0 * 1024)) * (512 * 1024 * 1024)) - 0x1200001
            self._logger.info("End of available memory is {0}".format("0x%0.8X" % avaiable_memory_size))
        except ValueError, IndexError:
            msg="Unable to locate memory size on returned result"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        if end_address > avaiable_memory_size:
            msg="SetupMemicals: End address {0} is too high. It should be below {1}".format(self._pars.end_address, "0x%0.8X" % avaiable_memory_size)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        memory_top = "0x%0.8X" % (end_address + 1)

        cmd = 'adb shell insmod /lib/modules/testbox_mod.ko resource_map=1 resource_blocks="({0},{1})" mem_top={2}'.format(self._pars.start_address, self._pars.end_address, memory_top)
        self._device.run_cmd(cmd, 60)
        self._logger.info("SetupMemicals: {0}".format(cmd))

        self._logger.info("SetupMemicals: Done")
