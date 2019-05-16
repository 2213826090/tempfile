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
:summary: Host Interface with embedded input engine

:since: 2014/02/11
:author: vdechefd
"""

from UtilitiesFWK.Utilities import Global


class InputUtilities():

    def __init__(self, device=None):
        """
        Constructor
        """
        self._device = device

    def type(self, str_to_type, timeout):
        self._device.inject_device_log("d", "ACS_TEXT", "%s" % str_to_type)
        encoded_text = str_to_type.replace(" ", "%s")
        result, _ = self._device.run_cmd("adb shell input text %s" % encoded_text, timeout)
        if result == 0:
            return Global.SUCCESS
        else:
            return Global.FAILURE

    def drag(self, start_x, start_y, stop_x, stop_y, timeout):
        self._device.inject_device_log("d", "ACS_SWIPE", "%s:%s => %s:%s" % (start_x, start_y, stop_x, stop_y))
        result, _ = self._device.run_cmd("adb shell input swipe %s %s %s %s" % (start_x, start_y, stop_x, stop_y),
                                         timeout)
        if result == 0:
            return Global.SUCCESS
        else:
            return Global.FAILURE

    def press(self, key, timeout):
        self._device.inject_device_log("d", "ACS_KEYPRESS", "%s" % key)
        result, _ = self._device.run_cmd("adb shell input keyevent %s" % key, timeout)
        if result == 0:
            return Global.SUCCESS
        else:
            return Global.FAILURE

    def touch(self, loc_x, loc_y, timeout):
        self._device.inject_device_log("d", "ACS_TAP", "%s:%s" % (loc_x, loc_y))
        result, _ = self._device.run_cmd("adb shell input tap %s %s" % (loc_x, loc_y), timeout)
        if result == 0:
            return Global.SUCCESS
        else:
            return Global.FAILURE
