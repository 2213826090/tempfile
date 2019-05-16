# -*- coding:utf-8 -*-

'''
@summary: AGPS test, this works only for broadcom chips.
@since: 06/20/2016
@author: Lijin Xiong
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.gps.common import GPS_Common

class AGPS(UIATestBase):

    def setUp(self):
        super(AGPS, self).setUp()
        self._test_name = __name__
        self.d = g_common_obj.get_device()
        print "[Setup]: %s" % self._test_name
        self.gc = GPS_Common()
        self.gc.Turn_On_Location()
        self.gc.check_if_wifi_connected()

    def test_files_downloaded(self):
        assert g_common_obj.adb_cmd_capture_msg("getprop | grep BRCM"), "DUT doesn't seem to have a broadcom gps chip, quit test"
        assert g_common_obj.adb_cmd_capture_msg('[ -e "/data/gps/lto2.dat" ] && echo "Pass" || echo "Fail"') == "Pass", \
        "The file lto2.dat not found!"
        assert g_common_obj.adb_cmd_capture_msg('[ -e "/data/gps/ltoStatus.txt" ] && echo "Pass" || echo "Fail"') == "Pass", \
        "The file ltoStatus.txt not found!"

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(AGPS, self).tearDown()
        g_common_obj.stop_app_am("com.android.settings")