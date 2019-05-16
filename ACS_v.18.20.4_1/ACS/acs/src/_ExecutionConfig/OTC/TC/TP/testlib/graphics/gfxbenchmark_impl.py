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
#published, uploaded, posted, transmitted, distributed
# or disclosed in any way without Intel's prior express written permission.
# No license under any patent, copyright, trade secret or
# other intellectual property right is granted to
# or conferred upon you by disclosure or delivery of the Materials,
# either expressly, by implication, inducement, estoppel or otherwise.
# Any license under such intellectual property rights must be express
# and approved by Intel in writing.
"""
@summary: GfxbenchmarkImpl class
@since: 06/11/2015
@author: Zhang,RongX Z(rongx.z.zhang@intel.com)
"""
import re
import time
import testlib.graphics.common as common
from testlib.graphics.common import pkgmgr
from testlib.graphics.glbenckmark_impl import GLBenchmarkImpl
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle


class GfxbenchmarkImpl(GLBenchmarkImpl):

    """ GfxbenchmarkImpl """
    CONFIG_FILE = 'tests.common.gfxbench31.conf'
    PKG_NAME = "com.glbenchmark.glbenchmark27.corporate"
    MAIN_ACTIVITY = "net.kishonti.gfxbench.GfxMainActivity"
    TEST_ACTIVITY = "net.kishonti.gfxbench.TfwActivityEx"
    LOADING_ACTIVITY ="net.kishonti.benchui.corporate.TestActivityCorporate"
    TEST_NAME =["Manhattan","T-Rex","ALU","Alpha Blending","Driver Overhead","Fill"]
    TEST_Manhattan = 'Manhattan'
    TEST_Manhattan_Offscreen="1080p Manhattan Offscreen"
    TEST_TRex = 'T-Rex'
    TEST_TRex_Offscreen="1080p T-Rex Offscreen"
    TEST_RenderQuality_HighPrecision="Render Quality (high precision)"
    TEST_RenderQuality="Render Quality"
    TEST_DriverOverhead="Driver Overhead"
    TEST_DriverOverhead_Offscreen="1080p Driver Overhead Offscreen"
    TEST_ALU="ALU"
    TEST_ALU_Offscreen="1080p ALU Offscreen"
    TEST_Alpha_Blending="Alpha Blending"
    TEST_Alpha_Blending_offscreen="1080p Alpha Blending Offscreen"
    TEST_Fill="Fill"
    TEST_Fill_Offscreen="1080p Fill Offscreen"
    RESULT_DIR = "/storage/emulated/legacy/Android/data/com.glbenchmark.glbenchmark27.corporate/files/results/"
    RESULT_DIR2 = "/storage/emulated/0/Android/data/com.glbenchmark.glbenchmark27.corporate/files/results/"

    class HomeUI(object):

        """Home UI"""

        def __init__(self, device):
            self.device = device

        @property
        def main_circle_control(self):
            return self.device(resourceId='net.kishonti.gfxbench.gl:id/main_circleControl')

        def test_select(self):
            """skip selection guide"""
            self.device(text="Test select").click.wait()
            time.sleep(2)

    class TestList(object):

        """Tests List UI"""

        def __init__(self, device):
            self.device = device

        @property
        def btn_start(self):
            return self.device(text='Start')

        def search_checkbox(self, name):
            """scroll to find item checkbox"""
            y = self.device.info["displayHeight"]
            x = self.device.info["displayWidth"]
            self.device(scrollable=True).scroll.to(text=name)
            if self.device(text=name).right(text="Not supported") != None:
                assert not self.device(text=name).right(text="Not supported").exists,\
                "[FAILURE] This item %s is not supported" % (name)
            self.device(text=name).drag.to(x/2,y/2)
            return self.device(text=name)\
                .right(className="android.widget.RelativeLayout", index=1)\
                .child(className="android.widget.ImageView")

        def search_item_onscreen_checkbox(self, name):
            """scroll to find item onscreen checkbox"""
            y = self.device.info["displayHeight"]
            x = self.device.info["displayWidth"]
            for _ in range(10):
                self.device.swipe(x / 2, y / 2, x / 2, y, steps=5)
            assert self.device(scrollable=True).scroll.to(text=name),\
                "[FAILURE] Not found %s item" % (name)
            return self.device(text=name)\
                .right(className="android.widget.RelativeLayout", index=1)\
                .child(className="android.widget.ImageView")

        def search_item_offscreen_checkbox(self, name):
            """scroll to find item offscreen checkbox"""
            y = self.device.info["displayHeight"]
            x = self.device.info["displayWidth"]
            for _ in range(10):
                self.device.swipe(x / 2, y / 2, x / 2, y, steps=5)
            assert self.device(scrollable=True).scroll.to(text=name),\
                "[FAILURE] Not found %s item" % (name)
            return self.device(text=name)\
                .right(className="android.widget.RelativeLayout", index=2)\
                .child(className="android.widget.ImageView")

        def search_item_special_checkbox(self, name):
            """scroll to find item offscreen checkbox"""
            y = self.device.info["displayHeight"]
            x = self.device.info["displayWidth"]
            for _ in range(10):
                self.device.swipe(x / 2, y / 2, x / 2, y, steps=5)
            assert self.device(scrollable=True).scroll.to(text=name),\
                "[FAILURE] Not found %s item" % (name)
            return self.device(text=name)\
                .right(className="android.widget.RelativeLayout")\
                .child(className="android.widget.ImageView")

    def __init__(self):
        GLBenchmarkImpl.__init__(self)
        self.device = g_common_obj.get_device()
        self.home = GfxbenchmarkImpl.HomeUI(self.device)
        self.testlist = GfxbenchmarkImpl.TestList(self.device)

        self.configer = TestConfig()
        self.config = self.configer.read(self.CONFIG_FILE, "GfxbenchmarkImpl")
        config_handle = ConfigHandle()
        self.config["artifactory_location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat', 'sys.conf')
        self.arti = Artifactory(self.config.get('artifactory_location'))

    def setup(self):
        """install apk"""
        if not pkgmgr._package_installed(pkgName=self.PKG_NAME):
            file_path = self.config.get("apk")
            apk_path = self.arti.get(file_path)
            pkgmgr.apk_install(apk_path)

    def set_workaround(self):
        pid = g_common_obj.adb_cmd_capture_msg("ps | grep 'com.glbenchmark.glbenchmark27.corporate' |awk '{print $2}' ")
        print "PID is %s"%pid
        if pid is not None:
            g_common_obj.adb_cmd_capture_msg("'echo -12>/proc/%s/oom_adj'"%pid)
            oom_adj= g_common_obj.adb_cmd_capture_msg("cat proc/%s/oom_adj"%pid)
            print oom_adj

    def clean(self):
        """uninstall apk"""
        cmd = "rm -rf  %s*.json" % (self.RESULT_DIR)
        cmd2 = "rm -rf %s*.json" % (self.RESULT_DIR2)
        g_common_obj.adb_cmd_capture_msg(cmd)
        g_common_obj.adb_cmd_capture_msg(cmd2)
        cmd = "uninstall %s" % (self.PKG_NAME)
        g_common_obj.adb_cmd_common(cmd)

    def launch_app_from_home_sc(self, appname, appgallery="Apps", inspection=None):
        """
        Launch App from app gallery
        Parameter appname is the app's widget name in app gallery
        Parameter appgallery is the widget name of app in home screen
        Parameter inspection is the text to validate if given app launched

        Please be aware that this fucntion assumes:
        1) There is a app gallery icon in home screen;
        2) Your app widget could be found in home screen.

        If your app meets above 2 assumtions, then it could be a convenient function
        call for you. Otherwise you may consider write your own app launch steps or
        using launch_app_am function instead.

        """
        iffind = False
        d = g_common_obj.get_device()
        g_common_obj.back_home()
        d(description="Apps").click.wait()
        if d(text="Widgets").exists:
            d(text="Widgets").click.wait()
            d(text="Apps").click.wait()
            if d(scrollable=True):
                d(scrollable=True).scroll.horiz.toBeginning(steps=20)
        for _ in range(5):
            if d(text=appname).exists:
                d(text=appname).click.wait()
                iffind = True
                break
            d(scrollable=True).scroll.horiz()
        if not iffind:
            for _ in range(5):
                if d(text=appname).exists:
                    time.sleep(2)
                    d(text=appname).click.wait()
                    iffind = True
                    break
                d(scrollable=True).scroll.horiz.backward()
        assert iffind == True
        if (inspection is not None):
            assert d(textMatches=inspection)

    def launch(self):
        """launch app"""
        self.launch_app_from_home_sc("GFXBench Corporate")
        time.sleep(3)
        self.home.test_select()

    def run_test(self, name, timeout=900, onscreen=False, offscreen=False,special=False,others=False):
        """run test"""
        print "[Debug] test: %s onscreen:%s offscreen:%s special:%s others:%s" % (name, onscreen, offscreen,special,others)
        if onscreen:
            onscreen_checkbox = self.testlist.search_item_onscreen_checkbox(name)
            onscreen_checkbox.click.wait()
        if offscreen:
            offscreen_checkbox = self.testlist.search_item_offscreen_checkbox(name)
            offscreen_checkbox.click.wait()
        if special:
            special_checkbox = self.testlist.search_item_special_checkbox(name)
            special_checkbox.click.wait()
        if others:
            others_checkbox = self.testlist.search_checkbox(name)
            others_checkbox.click.wait()

        cmd = "sync; echo 3 > /proc/sys/vm/drop_caches"
        g_common_obj.adb_cmd_capture_msg(repr(cmd))

        self.testlist.btn_start.click()
        time.sleep(5)
        _, current_activity = common.get_current_focus_window()
        assert current_activity == self.TEST_ACTIVITY or current_activity ==self.LOADING_ACTIVITY,\
            "[FAILURE] Tests failed to start"

        print "[Debug] start test: %s" % (name)
        start_time = time.time()
        is_completed = False
        while time.time() - start_time < timeout:
            cmd = "ls %s*.json" % (self.RESULT_DIR)
            cmd2 = "ls %s*.json" % (self.RESULT_DIR2)
            msg = g_common_obj.adb_cmd_capture_msg(repr(cmd))
            if msg.find("No such file or directory") != -1:
                msg = g_common_obj.adb_cmd_capture_msg(repr(cmd2))
            if msg.find("No such file or directory") == -1:
                break
            time.sleep(10)
        assert msg.find("No such file or directory") == -1,"Test time out"

    def get_result(self):
        cmd = "cat %s*.json" % (self.RESULT_DIR)
        cmd2 = "cat %s*.json" % (self.RESULT_DIR2)
        msg = g_common_obj.adb_cmd_capture_msg(repr(cmd))
        if msg.find("No such file or directory") != -1:
            msg = g_common_obj.adb_cmd_capture_msg(repr(cmd2))
        return msg

    def run_test_trex_onscreen(self):
        """run_test_trex_onscreen"""
        print "run_test_trex_onscreen"
        y = self.device.info["displayHeight"]
        x = self.device.info["displayWidth"]
        if x>y:
            self.run_test(self.TEST_TRex, onscreen=True,offscreen=False)
        else:
            self.run_test(self.TEST_TRex,others=True)
        result = self.get_result()
        match= re.search(r'("score":\d+.\d+)',result).group()
        score=match.split(":")[1].strip()
        print "score is %s" % score
        assert float(score)>0,"get score failed,and score is %d"% score
        assert result.find('status":"OK"')!=-1,"run %s failed,and result is %s"%(self.TEST_TRex,result)

    def run_test_trex_offscreen(self):
        """run_test_trex_offscreen"""
        print "run_test_trex_offscreen"
        y = self.device.info["displayHeight"]
        x = self.device.info["displayWidth"]
        if x>y:
            self.run_test(self.TEST_TRex, onscreen=False,offscreen=True)
        else:
            self.run_test(self.TEST_TRex_Offscreen,others=True)
        result = self.get_result()
        match= re.search(r'("score":\d+.\d+)',result).group()
        score=match.split(":")[1].strip()
        print "score is %s" % score
        assert float(score)>0,"get score failed,and score is %d"% score
        assert result.find('status":"OK"')!=-1,"run %s failed,and result is %s"%(self.TEST_TRex_Offscreen,result)

    def run_test_manhattan_onscreen(self):
        """run_test_manhattan_onscreen"""
        print "run_test_manhattan_onscreen"
        y = self.device.info["displayHeight"]
        x = self.device.info["displayWidth"]
        if x>y:
            self.run_test(self.TEST_Manhattan, onscreen=True,offscreen=False)
        else:
            self.run_test(self.TEST_Manhattan,others=True)
        result = self.get_result()
        match= re.search(r'("score":\d+.\d+)',result).group()
        score=match.split(":")[1].strip()
        print "score is %s" % score
        assert float(score)>0,"get score failed,and score is %d"% score
        assert result.find('status":"OK"')!=-1,"run %s failed,and result is %s"%(self.TEST_Manhattan,result)

    def run_test_manhattan_offscreen(self):
        print "run_test_manhattan_offscreen"
        y = self.device.info["displayHeight"]
        x = self.device.info["displayWidth"]
        if x>y:
            self.run_test(self.TEST_Manhattan, onscreen=False, offscreen=True)
        else:
            self.run_test(self.TEST_Manhattan_Offscreen,others=True)
        result = self.get_result()
        match= re.search(r'("score":\d+.\d+)',result).group()
        score=match.split(":")[1].strip()
        print "score is %s" % score
        assert float(score)>0,"get score failed,and score is %d"% score
        assert result.find('status":"OK"')!=-1,"run %s failed,and result is %s"%(self.TEST_Manhattan_Offscreen,result)


    def run_test_renderquality_highprecision(self):
        print "run_test_renderquality_highprecision"
        y = self.device.info["displayHeight"]
        x = self.device.info["displayWidth"]
        if x>y:
            self.run_test(self.TEST_RenderQuality_HighPrecision, special=True)
        else:
            self.run_test(self.TEST_RenderQuality_HighPrecision,others=True)
        result = self.get_result()
        match= re.search(r'("score":\d+.\d+)',result).group()
        score=match.split(":")[1].strip()
        print "score is %s" % score
        assert float(score)>0,"get score failed,and score is %d"% score
        assert result.find('status":"OK"')!=-1,"run %s failed,and result is %s"%(self.TEST_RenderQuality_HighPrecision,result)

    def run_test_renderquality(self):
        print "run_test_renderquality"
        y = self.device.info["displayHeight"]
        x = self.device.info["displayWidth"]
        if x>y:
            self.run_test(self.TEST_RenderQuality, special=True)
        else:
            self.run_test(self.TEST_RenderQuality,others=True)
        result = self.get_result()
        match= re.search(r'("score":\d+.\d+)',result).group()
        score=match.split(":")[1].strip()
        print "score is %s" % score
        assert float(score)>0,"get score failed,and score is %d"% score
        assert result.find('status":"OK"')!=-1,"run %s failed,and result is %s"%(self.TEST_RenderQuality,result)

    def run_test_driveroverhead_onscreen(self):
        """run_test_driveroverhead_onscreen"""
        print "run_test_driveroverhead_onscreen"
        y = self.device.info["displayHeight"]
        x = self.device.info["displayWidth"]
        if x>y:
            self.run_test(self.TEST_DriverOverhead, onscreen=True,offscreen=False)
        else:
            self.run_test(self.TEST_DriverOverhead,others=True)
        result = self.get_result()
        match= re.search(r'("score":\d+.\d+)',result).group()
        score=match.split(":")[1].strip()
        print "score is %s" % score
        assert float(score)>0,"get score failed,and score is %d"% score
        assert result.find('status":"OK"')!=-1,"run %s failed,and result is %s"%(self.TEST_DriverOverhead,result)

    def run_test_driveroverhead_offscreen(self):
        """run_test_driveroverhead_offscreen"""
        print "run_test_driveroverhead_offscreen"
        y = self.device.info["displayHeight"]
        x = self.device.info["displayWidth"]
        if x>y:
            self.run_test(self.TEST_DriverOverhead, onscreen=False, offscreen=True)
        else:
            self.run_test(self.TEST_DriverOverhead_Offscreen,others=True)
        result = self.get_result()
        match= re.search(r'("score":\d+.\d+)',result).group()
        score=match.split(":")[1].strip()
        print "score is %s" % score
        assert float(score)>0,"get score failed,and score is %d"% score
        assert result.find('status":"OK"')!=-1,"run %s failed,and result is %s"%(self.TEST_DriverOverhead_Offscreen,result)

    def run_test_ALU_onscreen(self):
        """run_test_ALU_onscreen"""
        print "run_test_ALU_onscreen"
        y = self.device.info["displayHeight"]
        x = self.device.info["displayWidth"]
        if x>y:
            self.run_test(self.TEST_ALU, onscreen=True, offscreen=False)
        else:
            self.run_test(self.TEST_ALU,others=True)
        result = self.get_result()
        match= re.search(r'("score":\d+.\d+)',result).group()
        score=match.split(":")[1].strip()
        print "score is %s" % score
        assert float(score)>0,"get score failed,and score is %d"% score
        assert result.find('status":"OK"')!=-1,"run %s failed,and result is %s"%(self.TEST_ALU,result)

    def run_test_ALU_offscreen(self):
        """run_test_ALU_offscreen"""
        print "run_test_ALU_offscreen"
        y = self.device.info["displayHeight"]
        x = self.device.info["displayWidth"]
        if x>y:
            self.run_test(self.TEST_ALU, onscreen=False, offscreen=True)
        else:
            self.run_test(self.TEST_ALU_Offscreen,others=True)
        result = self.get_result()
        match= re.search(r'("score":\d+.\d+)',result).group()
        score=match.split(":")[1].strip()
        print "score is %s" % score
        assert float(score)>0,"get score failed,and score is %d"% score
        assert result.find('status":"OK"')!=-1,"run %s failed,and result is %s"%(self.TEST_ALU_Offscreen,result)

    def run_test_alphablending_onscreen(self):
        """run_test_alphablending_onscreen"""
        print "run_test_alphablending_onscreen"
        y = self.device.info["displayHeight"]
        x = self.device.info["displayWidth"]
        if x>y:
            self.run_test(self.TEST_Alpha_Blending, onscreen=True, offscreen=False)
        else:
            self.run_test(self.TEST_Alpha_Blending,others=True)
        result = self.get_result()
        match= re.search(r'("score":\d+.\d+)',result).group()
        score=match.split(":")[1].strip()
        print "score is %s" % score
        assert float(score)>0,"get score failed,and score is %d"% score
        assert result.find('status":"OK"')!=-1,"run %s failed,and result is %s"%(self.TEST_Alpha_Blending,result)

    def run_test_alphablending_offscreen(self):
        """run_test_alphablending_offscreen"""
        print "run_test_alphablending_offscreen"
        y = self.device.info["displayHeight"]
        x = self.device.info["displayWidth"]
        if x>y:
            self.run_test(self.TEST_Alpha_Blending, onscreen=False, offscreen=True)
        else:
            self.run_test(self.TEST_Alpha_Blending_offscreen,others=True)
        result = self.get_result()
        match= re.search(r'("score":\d+.\d+)',result).group()
        score=match.split(":")[1].strip()
        print "score is %s" % score
        assert float(score)>0,"get score failed,and score is %d"% score
        assert result.find('status":"OK"')!=-1,"run %s failed,and result is %s"%(self.TEST_Alpha_Blending_offscreen,result)

    def run_test_fill_onscreen(self):
        """run_test_fill_onscreen"""
        print "run_test_fill_onscreen"
        y = self.device.info["displayHeight"]
        x = self.device.info["displayWidth"]
        if x>y:
            self.run_test(self.TEST_Fill, onscreen=True, offscreen=False)
        else:
            self.run_test(self.TEST_Fill,others=True)
        result = self.get_result()
        match= re.search(r'("score":\d+.\d+)',result).group()
        score=match.split(":")[1].strip()
        print "score is %s" % score
        assert float(score)>0,"get score failed,and score is %d"% score
        assert result.find('status":"OK"')!=-1,"run %s failed,and result is %s"%(self.TEST_Fill,result)

    def run_test_fill_offscreen(self):
        """run_test_fill_offscreen"""
        print "run_test_fill_offscreen"
        y = self.device.info["displayHeight"]
        x = self.device.info["displayWidth"]
        if x>y:
            self.run_test(self.TEST_Fill, onscreen=False, offscreen=True)
        else:
            self.run_test(self.TEST_Fill_Offscreen,others=True)
        result = self.get_result()
        match= re.search(r'("score":\d+.\d+)',result).group()
        score=match.split(":")[1].strip()
        print "score is %s" % score
        assert float(score)>0,"get score failed,and score is %d"% score
        assert result.find('status":"OK"')!=-1,"run %s failed,and result is %s"%(self.TEST_Fill_Offscreen,result)