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
:summary: This file implements UECmds for multimedia purposes (audio, video ...)
:since: 18/01/2013
:author: ssavrimoutou
"""

import time
import os
import urllib2
import re

from UtilitiesFWK.Utilities import Global
from ErrorHandling.DeviceException import DeviceException

from acs_test_scripts.Device.UECmd.Imp.Android.Common.BaseV2 import BaseV2
from acs_test_scripts.Device.UECmd.Imp.Android.Common.Multimedia.Multimedia import (
    Multimedia as Common_Multimedia)

class Multimedia(Common_Multimedia):
    """
    Class that handle all audio operations
    """

    def get_number_of_dropped_frames(self):
        """
        Get number of dropped video frames from dumpsys
        :rtype output : string
        :return output : dumpsys output
        :rtype no_total_frames : string
        :return no_total_frames : Total number of played frames
        :rtype  no_dropped_frames : string
        :return no_dropped_frames : number of dropped frames
        """
        cmd = 'adb shell dumpsys media.player | grep numFramesDropped'
        (status, output) = self._device.run_cmd(cmd, self._uecmd_default_timeout)

        if status != Global.SUCCESS or output is None:
            self._logger.error(output)
            raise DeviceException(DeviceException.OPERATION_FAILED, output)

        re_match = re.match(".*numFramesTotal\(([0-9]*)\).*numFramesDropped\(([0-9]*)\).*", output)

        if re_match and len(re_match.groups()) == 2:
            no_total_frames = re_match.group(1)
            no_dropped_frames = re_match.group(2)
            return (output, no_total_frames, no_dropped_frames)
        else:
            msg = "'dumpsys media.player | grep numFramesDropped' returned unexpected result as following: " + output
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
