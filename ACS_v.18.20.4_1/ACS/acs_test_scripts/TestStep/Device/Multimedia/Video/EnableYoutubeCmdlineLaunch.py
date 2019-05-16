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
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

:organization: INTEL PEG SVE DSV
:summary: Change /data/system/users/0/package-restrictions.xml to start Youtube app with playback
:since: 07/14/14
:author: jongyoon
"""
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase

class EnableYoutubeCmdlineLaunch(DeviceTestStepBase):

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Constructor
        """
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

        # Load Multimedia ue command.
        self._multimedia_api = self._device.get_uecmd("Multimedia")

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        self._logger.info("EnableYoutubeCmdlineLaunch: Run")
        android_version = self._device._android_version.upper()
        self._logger.info("EnableYoutubeCmdlineLaunch: Android version is {0}".format(android_version))

        self._multimedia_api.enable_youtube_cmdline_launch(android_version)
