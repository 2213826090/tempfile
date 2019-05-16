# Copyright (C) 2015  Zhang,RongX Z <rongx.z.zhang@intel.com>
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

'''
@summary: Class for ClockImpl operation
@since: 06/24/2015
@author: Zhang,RongX Z
'''

import time
from testlib.util.common import g_common_obj


class ClockImpl:

    '''
    classdocs
    '''

    def __init__(self):
        self.device = g_common_obj.get_device()

    def launch_clock_am(self):
        """ Launch Clock via adb am command
        """
        print "Launch Clock by adb am"
        g_common_obj.launch_app_am(
            "com.google.android.deskclock", "com.android.deskclock.DeskClock")
        g_common_obj.launch_app_am(
            "com.android.deskclock", "com.android.deskclock.DeskClock")
        time.sleep(2)

    @staticmethod
    def stop_clock_am():
        """ Stop Clock via adb am command
        """
        print "Stop Clock by adb am"
        g_common_obj.stop_app_am("com.google.android.deskclock")

    def check_imge_and_graphics(self):
        assert self.device(resourceId="com.google.android.deskclock:id/fab").exists and \
        self.device(resourceId="com.google.android.deskclock:id/digital_clock").exists or \
        self.device(resourceId="com.android.deskclock:id/digital_clock") and \
        self.device(resourceId="com.android.deskclock:id/fab"), \
        "Clock display failed"