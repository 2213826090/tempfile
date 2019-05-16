"""
@summary: This file implements a Test Step to have ACS monitor logcat for errors
related to graphics.
@since 25 September 2014
@author: sdebuiss
@organization: INTEL

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

from UtilitiesFWK.Utilities import Global, internal_shell_exec
from Core.TestStep.TestStepBase import TestStepBase
from ErrorHandling.DeviceException import DeviceException


class CheckFps(TestStepBase):
    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        TestStepBase.run(self, context)

        self._logger.info("CheckFPS: Run")

        file_to_check = self._pars.file_to_check
        fps_value = self._pars.fps_value
        margin = self._pars.margin

        # Detect the right column in the file
        cmd = "gawk \"BEGIN{FS=\\\" \\\"}; END{print NF}\" " + file_to_check
        try:
            (return_code, first_output) = internal_shell_exec(cmd, 5)
        except Exception as e:
            message = "Error while running gawk command: {0}.\n{1}".format(cmd, e.message)
            self._logger.warning(message)

        # The command to execute is version dependent.
        cmd = "gawk"
        if int(first_output) == 7:
            cmd += " " + "\"BEGIN{frames=0;} /postFramebuffer/{if (frames == 0) first = $5; frames += 1;timestamp = $5 } END{printf(\\\"%.3f\\\", (frames - 1) / (timestamp - first))}\""
        elif int(first_output) == 8:
            cmd += " " + "\"BEGIN{frames=0;} /postFramebuffer/{if (frames == 0) first = $6; frames += 1;timestamp = $6 } END{printf(\\\"%.3f\\\", (frames - 1) / (timestamp - first))}\""
        else:
            self._logger.warning("Unknown platform, authorized")
            return Global.FAILURE

        cmd += " " + file_to_check

        # Run gawk to compute FPS
        self._logger.info("Run command : " + ' '.join(cmd))
        try:
            (return_code, output) = internal_shell_exec(cmd, 5)
        except Exception as e:
            message = "Error while running gawk command: {0}.\n{1}".format(cmd, e.message)
            output = message
            self._logger.warning(message)
        if return_code != Global.SUCCESS:
            return Global.FAILURE
        # check FPS
        min_fps = fps_value * (100 - margin) / 100
        max_fps = fps_value * (100 + margin) / 100
        if float(min_fps) <= float(output) <= float(max_fps):
            self._logger.info("CheckFPS OK with %f fps", float(output))
            return Global.SUCCESS
        else:
            # CheckFPS is non blocking now
            self._logger.error("CheckFPS Value ot reach with %f fps", float(output))
            return Global.SUCCESS
