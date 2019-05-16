# -*- coding: utf-8 -*-
"""
@author: yusux
"""

import re
import subprocess
import tempfile
import time
from collections import Iterable
from testlib.graphics.sample_apidemo import SampleApiDemoImpl
from testlib.util.common import g_common_obj


class LaunchFromDrawer(object):

    def __init__(self):
        self._device = g_common_obj.get_device()

    def back_to_home(self):
        self._device.press.home()

    def open_drawer(self):
        if self._device(description="Apps"):
            return self._device(description="Apps").click()
        else:
            self._device.press.home()
            return self._device(description="Apps").click()

    def swipe_screen(self):
        self._device(resourceId="android:id/tabcontent").swipe.left()

    def swipe_screen_reverse(self):
        self._device(resourceId="android:id/tabcontent").swipe.right()

    def scan_all_apps(self):
        self._device.press.home()
        self._device(
            description="Apps", className="android.widget.TextView").click()
        scaned_screen_index = 0

        """
        return numbers of indicator of Apps drawer
        """
        while True:
            print "[INFO] Scan %d screen apps" % (scaned_screen_index + 1)
            if self._device(description="Apps", className="android.widget.TextView").exists is False:
                print "[INFO] Scan complete"
                word = self._device(descriptionStartsWith="Apps", ).info[  # @ReservedAssignment
                    "contentDescription"]  # @
                fillter = re.findall('\d+', word)
                print '[INFO]current launcher totally have ' + str(unicode(len(fillter))) + ' indicators'
            return len(fillter)
            break
            # for _ in range(0, scaned_screen_cnt):

    def swichbetweenhomemenus(self, homemenucycle=10):
        """
            action as method names
        """
        SampleApiDemoImpl().unlock()
        self._device.screen.on()
        self._device.press.menu()

        for x in xrange(1, homemenucycle):
            self._device.press.home()
            if self._device(description="Apps").wait.exists(timeout=3000):
                self._device(description="Apps").click()
            else:
                self._device(description="Apps list").click.wait(timeout=3000)
            self._device.press.menu()
            print "[Info:complete %x times]" % x

    def backlight_on_off(self, intcycle=15):
        """
            action as method names
        """
        for x in xrange(0, intcycle):
            self._device.sleep()
            time.sleep(4)
            self._device.wakeup()
            print "[Info:]suspend & wakeup %d times" % (x + 1)

    @staticmethod
    def getlistoflauncheableapps(
            cmd='adb shell monkey -p --ignore-native-crashes -v -v -v 99'):
        """
            return a two dimension list concludes all launchable apps \
            package Name and activity name which DUT was installed
        """
        try:
            out_temp = tempfile.SpooledTemporaryFile(bufsize=10 * 1000)
            fileerr = out_temp.fileno()
            p = subprocess.Popen(
                cmd.split(), stdout=subprocess.PIPE, stderr=fileerr)
            p.wait()
            list_a = p.stdout.readlines()
            # print type(p.stdout.readlines())
            # print len(list_a)
            str_list = []
            for line in list_a:
                if line.__contains__('MONKEY'):  # or :
                    pass
                if not line.__contains__('$') and not line.__contains__(
                        'com.google.android.launcher.StubApp') and line.startswith(
                        "//   - NOT USING main activity"):
                    line = line.strip('\n\t\r\f')[31:-1]
                    line = line.split(' (from package ', 1)
                    if isinstance(str_list, Iterable):
                        str_list.append(line)
                    print line
            # print str_list
            print len(str_list)
            return str_list
        except Exception as e:
            print e
        finally:
            if out_temp:
                out_temp.close()

    @staticmethod
    def get_current_focus():
        cmd = "dumpsys window|grep mCurrentFocus"
        pattern = r"(?P<PKG_NAME>[\w.]+)/(?P<ACT_NAME>[\w.]+)}"
        for _ in range(6):
            msg = g_common_obj.adb_cmd_capture_msg(cmd)
            match = re.search(pattern, msg)
            if not match:
                time.sleep(4)
                continue
            break
        return str(re.search(pattern, msg).group('PKG_NAME', 'ACT_NAME'))

    def check_app_start_succsessfully(self, pkg_name, act_name):
        """
            @summary: check the app is launched successfully
        """
        f = lambda: self.get_current_focus()
        pn = str(f()[0])
        print "pn is:" + pn + ",pkg_name received: " + pkg_name
        an = str(f()[1])
        # print an
        appcombine = str(f())
        # print appcombine
        errormessage = "[Error]: the app %s does not launch successfully" % appcombine
        if appcombine.__contains__(str(pkg_name)) or appcombine.__contains__(str(act_name)):
            print "[INFO]: launch app %s successfully  " % appcombine
            return True
        # handle network connection to google failure:
        elif str(pn).startswith("com.google.android") and appcombine.__contains__("auth.login"):
            print "[Error Because google network connection is down]: the \
            app %s does not launch successfully" % appcombine
            return False
        elif str(pn) == "com.google.android.talk":
            print "[Error Because google Hangout show a popupwindow not initialized!"
            return False
        else:
            print "[Error because: '" + pn + "' is empty not " + pkg_name + " ; '" + an + "' is empty not " + act_name
            return False
            assert appcombine.__contains__(str(pkg_name)) or appcombine.__contains__(str(act_name)), errormessage

    def launch_specific_and_check(self, pn, an):
        """
        @Summary:launch specific acitvity and check if its successful
        :param pn:pkgname
        :param an:activityname
        """
        g_common_obj.launch_app_am(pn, an)
        return self.check_app_start_succsessfully(pn, an)

    def launch_app_from_drawer(self):
        """
            launch all installed apps in DUT
            will require gmail account loggable and logged in
            else will throw error message
        """
        instanc = self.getlistoflauncheableapps()
        for items in instanc:
            g_common_obj.launch_app_am(items[1], items[0])
            time.sleep(3)
            if items[1] == "com.google.android.talk":
                continue
            if items[1] == "com.android.providers.downloads.ui":
                continue
            if str(items[1]).__contains__("com.google.android.apps.docs"):
                continue
            if str(items[1]).__contains__("com.google.android.street"):
                continue
            if str(items[0]).__contains__("com.google.android.launcher.GEL") or str(items[1]).__contains__(
                    "com.google.android.googlequicksearchbox"):
                continue
            if str(items[1]).__contains__("com.google.android.gms"):
                continue
            self.check_app_start_succsessfully(items[1], items[0])
            time.sleep(3)
