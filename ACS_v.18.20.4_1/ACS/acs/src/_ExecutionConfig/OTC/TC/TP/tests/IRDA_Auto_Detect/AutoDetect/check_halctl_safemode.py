"""
@summary: check haclctl -l
@casename:/System_IRDA_Feature/Auto_detect halctl --list
@since: 7/8/2015
@author: Song, GuimeiX Z <guimeix.z.song@intel.com>
"""
import os
import time
from testlib.common.common import g_common_obj2
from testlib.autodetect.autodetect_impl import AutodetectImpl
from testlib.util.uiatestbase import UIATestBase


class Check_Halctl_list_SafeMode(UIATestBase):
    """
    check haclctl -l
    """

    def setUp(self):
        super(Check_Halctl_list_SafeMode, self).setUp()
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
            'tests.tablet.system_domains.conf')
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.hal = AutodetectImpl()
        self.cfg = self.config.read(cfg_file, 'system_domain')
    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(Check_Halctl_list_SafeMode, self).tearDown()
        self.cfg = None
        self.hal=None
    def testCheck_Halctl_list_SafeMode(self):
        """
                Test Case Step:
1    Boot  in safemode.
2    Have adb root session
3    Type halctl -l
4    Type halctl -li type=gralloc


        Expect Result:
1    Device boots successfully
2    successfully
3    Show binding list correctly
4    Gralloc entry is listed. Mount source is llvmpipe.

        """

        print "[RunTest]: %s" % self.__str__()
        self.hal.wake_up()
        self.hal.boot_safemode()
        time.sleep(10)
        res = g_common_obj2.root_on_device()
        print res
        result=self.hal.halctl_cmd("-l",para1=None).count("modalias")
        assert result>10, "[ERROR]: halctl -list is fail"
        print "[INFO]: halctl -list is success"
        g_common_obj2.system_reboot(200)

