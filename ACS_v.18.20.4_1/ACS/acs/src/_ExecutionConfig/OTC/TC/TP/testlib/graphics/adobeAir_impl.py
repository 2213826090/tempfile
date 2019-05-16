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
@summary: Class for AdobeAirImpl operation
@since: 06/25/2015
@author: Zhang,RongX Z
'''

import time
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.graphics.common import osversion


class AdobeAirDefaultImpl(object):

    '''
    classdocs
    '''

    def __init__(self):
        self.device = g_common_obj.get_device()

    def install_adobeAir(self):
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        cfg = config.read(cfg_file, 'content_adobeair')
        arti = Artifactory(cfg_arti.get('location'))
        apk_name = cfg.get("name")
        file_path = arti.get(apk_name)
        result = config_handle.check_apps("com.adobe.air")
        if result == 0:
            g_common_obj.adb_cmd_common('install ' + str(file_path))

    def check_adobeAir_in_All_Apps(self):
        g_common_obj.launch_app_am("com.android.settings", "com.android.settings.Settings")
        time.sleep(1)
        self.device(text="Apps").click.wait()
        time.sleep(1)
        version_array = osversion.get_android_version()
        androidversion = version_array[0]
        if androidversion == 7:
            print "osversion is N"
        elif androidversion == 6:
            print "osversion is M"
        elif androidversion == 5:
            print "osversion is L"
            for _ in range(0, 5):
                self.device(scrollable=True).scroll.horiz.forward(steps=100)
        else:
            print "osversion is %s" % (androidversion)
            for _ in range(0, 5):
                self.device(scrollable=True).scroll.horiz.forward(steps=100)
        if not self.device(text="Adobe AIR").exists:
            self.device(scrollable=True).scroll.to(text="Adobe AIR")
        assert self.device(text="Adobe AIR").exists, "dobeAir is not in All Apps"

    def launch_adobeAir_am(self):
        """ Launch adobeAir via adb am command
        """
        print "Launch adobeAir by adb am"
        g_common_obj.launch_app_am(
            "com.adobe.air", "com.adobe.mobile_playpanel.MainActivity")
        for _ in range(0, 5):
            time.sleep(5)
            if self.device(text="Adobe AIR").exists:
                break
        output = g_common_obj.adb_cmd_capture_msg("dumpsys window | grep mCurrentFocus")
        key = output.find("com.adobe.mobile_playpanel.MainActivity")
        assert key != -1 and self.device(text="Adobe AIR").exists, "launch adobe air failed"

    def stop_adobeAir_am(self):
        """ Stop adobeAir via adb am command
        """
        print "Stop adobeAir by adb am"
        g_common_obj.stop_app_am("com.adobe.air")


class AdobeAirImpl_Gordon(AdobeAirDefaultImpl):

    def check_adobeAir_in_All_Apps(self):
        g_common_obj.launch_app_am("com.android.settings", "com.android.settings.Settings")
        time.sleep(1)
        self.device(textStartsWith="Apps").click.wait()
        time.sleep(1)
        self.device(text="App info").click.wait()
        time.sleep(1)

        assert self.device().scroll.vert.to(text="Adobe AIR"), "dobeAir is not in All Apps"


product_name = osversion.get_product_name()
if 'gordon' in product_name:
    AdobeAirImpl = AdobeAirImpl_Gordon
else:
    AdobeAirImpl = AdobeAirDefaultImpl
