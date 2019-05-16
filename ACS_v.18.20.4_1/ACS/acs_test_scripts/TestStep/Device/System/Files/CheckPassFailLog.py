"""
@summary: A Test Step to use for verification during the run phase, after running a Test
    Step that invokes a script or application that communicates the result by generating a 
    log file whose name indicates whether it passed or failed.
@since 21 March 2014
@author: Val Peterson
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
import time
from ErrorHandling.DeviceException import DeviceException

class CheckPassFailLog(DeviceTestStepBase):
    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self.file_api = self._device.get_uecmd("File")

    def run(self, context):
        DeviceTestStepBase.run(self, context)

        start_time = time.time()
        current_time = start_time
        src_files = self._pars.file_path
        if src_files[-1] != '/':
            src_files = src_files + '/'
        pass_file = src_files + self._pars.pass_file
        fail_file = src_files + self._pars.fail_file
        delay = float(self._pars.polling_interval)
        self._logger.info("CheckPassFailLog: waiting up to %s seconds for %s or %s to appear."%(self._pars.timeout, self._pars.pass_file, self._pars.fail_file))
        while current_time - start_time < self._pars.timeout:
            (result, output) = self.file_api.exist(pass_file)
            if result:
                self._ts_verdict_msg = self._pars.result_msg_string + ": PASSED"
                return
            (result, output) = self.file_api.exist(fail_file)
            if result:
                msg = self._pars.result_msg_string + ": FAILED!"
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

            self._logger.info(self._pars.result_msg_string + ": Waiting for log file.  Time elapsed: %d sec"%(current_time - start_time))
            time.sleep(delay)
            current_time = time.time()

        msg = self._pars.result_msg_string + ": TIMEOUT waiting for files to appear."
        raise DeviceException(DeviceException.OPERATION_FAILED, msg)
