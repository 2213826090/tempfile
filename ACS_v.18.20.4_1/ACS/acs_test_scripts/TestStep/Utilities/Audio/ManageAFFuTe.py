"""
:copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
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

@summary: This file implements handling of AFFuTe python process
@since: 09/22/2015
@author: fbelvezx
@organization: CCG-CRD-OPM PC&WTE
"""

import subprocess, os, sys, psutil, time
from Core.TestStep.EquipmentTestStepBase import TestStepBase
from UtilitiesFWK.Utilities import str_to_bool


class ManageAFFuTe(TestStepBase):
    """
    Starts a new session of AFFuTe
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        TestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        TestStepBase.run(self, context)

        # Clean-up the existing AFFuTe process
        self._logger.info("Clean-up the existing AFFuTe process...")
        for p in psutil.get_process_list():
            for q in p.cmdline:
                if 'AudioFramework_runner' in q:
                    p.kill()

        if self._pars.start_framework:

            # Saving ACS working directory
            acs_working_dir = os.getcwd()

            # Switching to AFFuTe directory
            os.chdir(self._pars.framework_runner_path)

            self._logger.info("Starting a new session of AFFuTe")

            # By default, we prefer to rely on the official AFFuTe release, which uses .pyc files
            cmd_line = os.path.join(self._pars.framework_runner_path, "AudioFramework_runner.pyc")

            # Hide the logs of AFFuTe
            FNULL = open(os.devnull, 'w')
            subprocess.Popen([sys.executable, cmd_line], stdout=FNULL, stderr=FNULL, shell=True)

            # Switch back to ACS working directory
            os.chdir(acs_working_dir)

            # Wait for the current AFFuTe process to be fully started
            self._logger.info("Wait for the current AFFuTe process to be fully started... ")
            time.sleep(5)
