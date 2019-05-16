"""
:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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

:organization: INTEL MCG PSI
:summary: This file implements a Test Step to set stream volume
:since:19/03/2013
:author: baptistx
"""

from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from types import *


class SetStreamVolume(DeviceTestStepBase):
    """
    Set stream volume
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Constructor
        """
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

        # Get system command.
        self._system = self._device.get_uecmd("System")

        self._volume_percent_level = None

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        DeviceTestStepBase.run(self, context)

        self._volume_percent_level = self._pars.volume

        assert type(self._volume_percent_level) is IntType,\
            "volume_percent_level is not an integer: %r" % self._volume_percent_level
        assert self._volume_percent_level >= 0,\
            "volume_percent_level is lower than 0: %r" % self._volume_percent_level
        assert self._volume_percent_level <= 100,\
            "volume_percent_level is higher than 100: %r" % self._volume_percent_level

        self._system.adjust_specified_stream_volume("Media", self._volume_percent_level)