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

:summary: This file implements a Test Step to setup MTP sync stress from a host. In case of Windows environment, this TestStep copies a corresponding wpd.py file by referring architecture and Python version.
:since: 07/01/2014
:author: Jongyoon Choi
:organization: INTEL PEG-SVE-DSV
"""

import os
import sys
import platform
import shutil
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.AcsConfigException import AcsConfigException
from UtilitiesFWK.Utilities import Global

class SetupMtpSync(DeviceTestStepBase):

    """
    Setup MTP sync TestStep
    """
    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        DeviceTestStepBase.run(self, context)

        verdict = Global.SUCCESS
        self._logger.info('MTP Sync: Setup')

        test_files_root = os.path.dirname(self._pars.test_files_path)
        resource_files_root = os.path.dirname(self._pars.resource_files_path) #@UnusedVariable

        self.current_os = sys.platform
        self.current_python_version = sys.version_info
        self.current_architecture = platform.architecture()[0]

        if self.current_architecture != '32bit' and self.current_architecture !='64bit':
            raise Exception("ERROR! The current OS is not recognizable. Legal values are 32bit or 64bit.")

        if self.current_os == "win32":
            if self.current_python_version[0] == 2 and self.current_python_version[1] == 6 and self.current_architecture == '32bit':
                shutil.copy (os.path.join(test_files_root, "wpd_python26_32.pyd"), os.path.join(test_files_root, "wpd.pyd"))
                msg = "Installed {0} to {1} and renamed it to wpd.pyd".format("wpd_python26_32.pyd", test_files_root)
            elif self.current_python_version[0] == 2 and self.current_python_version[1] == 6 and self.current_architecture == '64bit':
                shutil.copy (os.path.join(test_files_root, "wpd_python26_64.pyd"), os.path.join(test_files_root, "wpd.pyd"))
                msg = "Installed {0} to {1} and renamed it to wpd.pyd".format("wpd_python26_32.pyd", test_files_root)
            elif self.current_python_version[0] == 2 and self.current_python_version[1] == 7 and self.current_architecture == '32bit':
                shutil.copy (os.path.join(test_files_root, "wpd_python27_32.pyd"), os.path.join(test_files_root, "wpd.pyd"))
                msg = "Installed {0} to {1} and renamed it to wpd.pyd".format("wpd_python26_32.pyd", test_files_root)
            else:
                verdict = Global.FAILURE
                msg = "ERROR! The current Python version is not supported for MTP sync"
        elif self.current_os == "linux2":
            if os.path.exists(os.path.join(test_files_root, "libmtp.so")):
                msg = "Installed {0} to {1}".format("libmtp.so", test_files_root)
            else:
                verdict = Global.FAILURE
                msg = "Failed to install {0} to {1}".format("libmtp.so", test_files_root)
        else:
            verdict = Global.FAILURE
            msg = "ERROR! The current OS is not supported. Legal values are win32 or linux2. Current OS is: " + str(self.current_os)

        if verdict != Global.SUCCESS:
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, msg)
        else:
            self.ts_verdict_msg = msg
