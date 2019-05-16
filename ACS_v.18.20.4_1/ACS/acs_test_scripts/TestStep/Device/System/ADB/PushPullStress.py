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

:summary: This file implements a Test Step to run push/pull stress from a host using ADB.
    It continuously push/pull a file for a given time.
:since: 07/01/2014
:author: Jongyoon Choi
:organization: INTEL PEG-SVE-DSV
"""

import os.path
import hashlib
import time
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import Global
import acs_test_scripts.Utilities.OSBVUtilities as osbv_utils
import shutil

class PushPullStress(DeviceTestStepBase):

    """
    Run ADB Push/Pull stress
    """

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        DeviceTestStepBase.run(self, context)

        self._logger.info(self._pars.id + ': Run')

        verdict = Global.SUCCESS

        src_file = self._pars.resource_files_path
        duration = self._pars.duration
        execution_seconds = duration * 60
        # Timeout value to use that determines amount of time to wait on gui focus lock file to be removed.
        # We don't need the GUI, but we are using this mechanism to coordinate with test steps that might disconnect USB.
        # We do not want to try push/pull while USB is disconnected.
        self.usb_lock_wait_time = self._pars.usb_lock_wait_time
        appSignature = 'usb_adb_pushpull'
        try:
            # Delete any focus-lock file that may not have been released during the previous test run.
            osbv_utils.cleanup_focus_lock(appSignature)
        except:
            raise DeviceException(DeviceException.OPERATION_FAILED, self._pars.id + ": Issue trying to remove previous focus lock file.")

        self._logger.info(self._pars.id + ':  SRC is ' + src_file)

        test_files_root = os.path.dirname(self._pars.resource_files_path)

        test_start_time = time.time()
        test_end_time = test_start_time + float(execution_seconds)
        self._logger.info(self._pars.id + ':  ============================================================================')
        self._logger.info(self._pars.id + ':                           ADB Push/Pull test')
        self._logger.info(self._pars.id + ':  ============================================================================')
        md5_reference = hashlib.md5(open(src_file,'rb').read()).hexdigest()
        self._logger.info(self._pars.id + ':  [Reference] ADB transfer root path is ' + test_files_root)
        self._logger.info(self._pars.id + ':  [Reference] Test started at {0}'.format(time.strftime('%m\\%d\\%Y %H:%M:%S', time.localtime())))
        self._logger.info(self._pars.id + ':  [Reference] MD5 Check_SUM = {0}'.format(str(md5_reference)))
        self._logger.info(self._pars.id + ':  ============================================================================')

        iteration_count = 0
        pass_count = 0
        fail_count = 0
        while (time.time() < test_end_time):
            try:
                iteration_count += 1

                if not osbv_utils.set_focus_lock(appSignature, timeout_sec=self.usb_lock_wait_time):
                    # Could not set the focus-lock
                    raise DeviceException(DeviceException.OPERATION_FAILED, self._pars.id + ": Failed to set focus lock.")

                iteration_start_time = time.localtime()
                self._logger.info(self._pars.id + ':  [Iteration {0}] Started at {1}'.format(str(iteration_count), time.strftime('%H:%M:%S', iteration_start_time)))

                self._logger.info(self._pars.id + ':  [Iteration {0}] Remove files in the host folder'.format(str(iteration_count)))
                if os.path.exists(os.path.join(test_files_root,'adb_test.bin')):
                    os.remove(os.path.join(test_files_root,'adb_test.bin'))
                time.sleep(3)

                self._logger.info(self._pars.id + ':  [Iteration {0}] Remove files in the device'.format(str(iteration_count)))
                adb_command = 'adb shell "if [ -f /storage/sdcard0/adb_test.bin ]; then rm -r /storage/sdcard0/adb_test.bin; fi;"'
                self._device.run_cmd(cmd=adb_command, timeout=20)
                time.sleep(3)

                self._logger.info(self._pars.id + ':  [Iteration {0}] Push files from the source folder to the device'.format(str(iteration_count)))
                adb_command = 'adb push {0} /storage/sdcard0/adb_test.bin'.format(os.path.join(test_files_root, src_file))
                self._device.run_cmd(cmd=adb_command, timeout=20)
                time.sleep(3)

                self._logger.info(self._pars.id + ':  [Iteration {0}] Pull files from the device to the destination folder'.format(str(iteration_count)))
                adb_command = 'adb pull /storage/sdcard0/adb_test.bin {0}'.format(os.path.join(test_files_root, 'adb_test.bin'))
                self._device.run_cmd(cmd=adb_command, timeout=20)
                time.sleep(3)

                if not osbv_utils.release_focus_lock(appSignature):
                    # Could not release the focus-lock
                    raise DeviceException(DeviceException.OPERATION_FAILED, self._pars.id + ": Failed to release focus lock.")

                iteration_end_time = time.localtime()
                self._logger.info(self._pars.id + ':  [Iteration {0}] finished at {1}'.format(str(iteration_count), time.strftime('%H:%M:%S', iteration_end_time)))

                # Check and make sure our file exists
                if not os.path.exists(os.path.join(test_files_root, 'adb_test.bin')):
                    fail_count += 1
                    break

                md5_result = hashlib.md5(open(os.path.join(test_files_root, 'adb_test.bin'),'rb').read()).hexdigest()

                if md5_result == md5_reference and md5_reference != 0:
                    self._logger.info(self._pars.id + ':  [Iteration {0}] PASS : MD5 Check_SUM = {1}'.format(str(iteration_count), str(md5_result)))
                    pass_count += 1
                else:
                    self._logger.info(self._pars.id + ':  [Iteration {0}] FAIL : MD5 Check_SUM = {1} does not match'.format(str(iteration_count), str(md5_result)))
                    fail_count += 1
                self._logger.info(self._pars.id + ':  ============================================================================')

                if fail_count > 0:
                    break
            except:
                import traceback
                self._logger.error(self._pars.id + ":  Unexpected Exception being raised.")
                self.ts_verdict_msg = self._pars.id + ": Unexpected Exception being raised."
                self._logger.error(traceback.format_exc())
                raise
            finally:
                # Check if focus lock still exists with appSignature and remove if so.
                if appSignature == osbv_utils.get_lock_signature():
                    if not osbv_utils.release_focus_lock(appSignature):
                        # Unable to release the focus-lock
                        raise DeviceException(DeviceException.OPERATION_FAILED, self._pars.id + ": Failed to release focus lock.")
                #shutil.rmtree(src_file)

        self._logger.info(self._pars.id + ':  [Reference] Test finished at {0}'.format(time.strftime('%m\\%d\\%Y %H:%M:%S', time.localtime())))
        self._logger.info(self._pars.id + ':  [Reference] Total PASS count : {0}'.format(pass_count) )
        self._logger.info(self._pars.id + ':  [Reference] Total FAIL count : {0}'.format(fail_count) )
        self._logger.info(self._pars.id + ':  ============================================================================')

        if (fail_count > 0):
            msg = 'USB ADB Push/PUll FAILED'
            self.ts_verdict_msg = msg
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        self._logger.info(self._pars.id + ': PASSED')
