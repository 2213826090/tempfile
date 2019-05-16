"""

:copyright: (c)Copyright 2016, Intel Corporation All Rights Reserved.
The source code contained or described herein and all documents related
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

:organization: INTEL OTC ANDROID QA
:description: TestStep for running android instrumentation scripts
:since: 02/16/16
:author: apalko
"""

from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import Global, get_timestamp
import os
import re


class RunAndroidInst(DeviceTestStepBase):
    def run(self, context):
        """
        TestStep for running android instrumentation scripts
        """

        DeviceTestStepBase.run(self, context)

        # parse params
        run_instr_cmd = "adb shell {0}".format(self._pars.run_instr_cmd)
        timeout = self._pars.timeout
        # escape unicode characters
        if re.search("\\\u\d{4}", run_instr_cmd):
            run_instr_cmd = run_instr_cmd.decode('unicode-escape')

        # execute the command
        result, output = self._device.run_cmd(run_instr_cmd, timeout)

        if result == Global.SUCCESS and "OK (1 test)" in output:
            output = ""
        elif result == Global.SUCCESS and ('FAILURES!!!' not in output and "OK (" in output):
            output = ""
        elif output.strip() == "":
            result, output = Global.FAILURE, "Am command has no output"
        else:
            result = Global.FAILURE

        # raise exception if the result is failure
        if result != Global.SUCCESS:
            # retrieve artifacts on failure
            self._retrieve_artifacts()
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, output)

    def _retrieve_artifacts(self):
        """
        Retrieve artifacts if retrieve_artifact parameter is present
        :return:
        """
        retrieve_artifact = self._pars.retrieve_artifact
        if retrieve_artifact:
            for artifact in [x.strip() for x in retrieve_artifact.split(";")]:
                try:
                    # pull artifact to reporting folder
                    artifact_report_name = '{0}_{1}_{2}_{3}'.format(get_timestamp(), self._pars.device,
                                                                    self._testcase_name.split(os.sep)[-1],
                                                                    artifact.split("/")[-1])
                    artifacts_folder = "FAILURE_ARTIFACTS"
                    pull_dest_path = os.path.join(
                            self._global_conf.campaignConfig.get("campaignReportTree").get_report_path(),
                            artifacts_folder, artifact_report_name)
                    self._device.pull(artifact, pull_dest_path, 600)
                    # remove artifact from device
                    remove_cmd = "adb shell rm {0}".format(artifact)
                    self._device.run_cmd(remove_cmd, 100)
                except:
                    # log only a message to not change the result
                    self._logger.warning("Exception was encountered when retrieving artifact {0}".format(artifact))
