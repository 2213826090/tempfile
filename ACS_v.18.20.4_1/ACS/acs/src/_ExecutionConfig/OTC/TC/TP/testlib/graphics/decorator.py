# Intel Corporation All Rights Reserved.

# The source code contained or described herein and
# all documents related to the source code ("Material") are owned by
# Intel Corporation or its suppliers or licensors.

# Title to the Material remains with Intel Corporation or
# its suppliers and licensors.
# The Material contains trade secrets and proprietary and
# confidential information of Intel or its suppliers and licensors.
# The Material is protected by worldwide copyright and
# trade secret laws and treaty provisions.
# No part of the Material may be used, copied, reproduced, modified,
# published, uploaded, posted, transmitted, distributed
# or disclosed in any way without Intel's prior express written permission.
# No license under any patent, copyright, trade secret or
# other intellectual property right is granted to
# or conferred upon you by disclosure or delivery of the Materials,
# either expressly, by implication, inducement, estoppel or otherwise.

# Any license under such intellectual property rights must be express
# and approved by Intel in writing.

"""
@summary: Graphics Decorator
@since: 04/14/2017
@author: Ding, JunnanX (junnanx.ding@intel.com)
"""

from functools import wraps
from testlib.util.log import Logger
from testlib.util.process import shell_command, shell_command_ext
from testlib.util.common import g_common_obj

LOG = Logger.getlogger(__name__)


def restart_uiautomator(func):
    @wraps(func)
    def func_wrapper(*args, **kwargs):
        LOG.debug("Restart uiautomator")
        try:
            before = g_common_obj.adb_cmd_capture_msg("ps | grep 'com.github.uiautomator' |awk '{print $2}'")
            g_common_obj.adb_cmd_capture_msg('pm uninstall com.github.uiautomator')
            g_common_obj.adb_cmd_capture_msg('pm uninstall com.github.uiautomator.test')
            device = g_common_obj.get_device()
            # trigger start uiautomator
            LOG.debug(device.info)
            after = g_common_obj.adb_cmd_capture_msg("ps | grep 'com.github.uiautomator' |awk '{print $2}'")
            assert before != after, "Failed to restart uiautomator"

            msg = g_common_obj.adb_cmd_capture_msg("ps | grep 'uiautomator'")
            LOG.debug(msg)
        except Exception as e:
            LOG.warning(str(e))

        func(*args, **kwargs)

    return func_wrapper
