# coding: utf-8
import time
import os
from testlib.util.common import g_common_obj
from testlib.common.common import g_common_obj2
from testlib.util.config import TestConfig

class Function():
# this class declare the functions that need before case running

    def __init__(self):
        self.d = g_common_obj.get_device()
        self.serial = self.d.server.adb.device_serial()
        self.x = self.d.info["displayWidth"]
        self.y = self.d.info["displayHeight"]

    def settings_ON_OFF(self, _text, enable = True):
    #uasge such as:
    #assert not self.settings_ON_OFF("Verify apps over USB", enable = False) or
    #assert self.settings_ON_OFF("Verify apps over USB", enable = True)
        widget = self.d(text = _text).right(className = "android.widget.Switch")
        if widget:
            if enable:
                if not widget.checked:
                    widget.click()
            else:
                if widget.checked:
                    widget.click()
            status = widget.checked
        return status

    def if_adb_exists(self):
    # Check if adb command link exists
        result = os.popen("which adb").read()
        if "" != result:
            print "adb command exists"
        else:
            assert False, "adb command isn't found, please install Android SDK firstly!"

    def push_uiautomator_jar(self):
    # put bundle.jar and uiautomator-stub.jar to the right place
        print "Check RPC server ..."
        if os.system("adb -s %s shell ps | grep uiautomator > /dev/null 2>&1" % self.serial) == 0:
            print "RPC server started"
            return
        else:
            from uiautomator import device as d
            if d.info:
                print "RPC server started"
                return
        print os.system("adb -s %s shell getprop | grep ro.build.version.release" % self.serial)
        if os.path.exists("/usr/lib/python2.7/dist-packages/uiautomator/libs"):
            jar_path = "/usr/lib/python2.7/dist-packages/uiautomator/libs"
        else:
            jar_path = "/usr/local/lib/python2.7/dist-packages/uiautomator/libs"
        bundle_path = jar_path + "/bundle.jar"
        uiautomator_path = jar_path + "/uiautomator-stub.jar"
        push_bundle_jar = "adb -s " + self.serial + " push " + bundle_path + " /data/local/tmp"
        push_uiautomator_jar = "adb -s " + self.serial + " push " + uiautomator_path + " /data/local/tmp"
        os.system("adb -s %s root > /dev/null 2>&1" % self.serial)
        for i in range(20):
            self.d.info
            time.sleep(5)
            if os.system("adb -s %s shell ps |grep uiautomator" % self.serial) == 0:
                print "RPC server started"
                break
            else:
                print "push jar start ========"
                os.system(push_bundle_jar)
                os.system(push_uiautomator_jar)
                print "push jar end ========="
                time.sleep(5)
        else:
            raise Exception('RPC server start failed')

    def wake_up(self):
    # wake up system
        print "start to wakeup and unlock screen"
        self.d.wakeup()
        if self.d(resourceId="com.android.keyguard:id/keyguard_selector_view_frame").exists:
            self.d(resourceId="com.android.keyguard:id/keyguard_selector_view_frame").swipe.down()
        if self.d(description="Unlock").exists:
            self.d(description="Unlock").drag.to(resourceId="com.android.systemui:id/clock_view")

    def keep_vertical(self):
    # keep vertical screen
        self.d.orientation = "n"
        self.d.freeze_rotation()

    def continuous_click_input(self, selector_value_list):
        #selector_value_list is like this:
        #[{'description': 'Start'}, [{'text': 'Next'}, 'passwd'], {'text': "CONNECT"}]
        i = 0
        while i+1 <= len(selector_value_list):
            if type(selector_value_list[i]) == dict and self.d(**selector_value_list[i]).exists:
                print 'Clicking: ' + str(selector_value_list[i])
                self.d(**selector_value_list[i]).click.wait()
                self.d(**selector_value_list[i]).wait.gone(timeout=3000)
            elif type(selector_value_list[i]) == list and self.d(**selector_value_list[i][0]).exists:
                if type(selector_value_list[i][1]) != dict:
                    print 'Inputing: ' + str(selector_value_list[i][0])+' '+str(selector_value_list[i][1])
                    time.sleep(1)
                    self.d(**selector_value_list[i][0]).set_text(selector_value_list[i][1])
                    time.sleep(0.5)
                    self.d.press('enter')
                elif type(selector_value_list[i][1]) == dict and \
                        self.d(**selector_value_list[i][0]).right(**selector_value_list[i][1]).exists:
                    print 'Clicking: ' + str(selector_value_list[i][0]) + str(selector_value_list[i][1])
                    self.d(**selector_value_list[i][0]).right(**selector_value_list[i][1]).click.wait()
            if i + 1 < len(selector_value_list):
                timeout = 500
                o_appeared = False
                while timeout != 0:
                    for o in selector_value_list[i+1:]:
                        if type(o) == dict and self.d(**o).exists:
                            o_appeared = True
                            break
                        elif type(o) == list and self.d(**o[0]).exists:
                            o_appeared = True
                            break
                    if o_appeared:
                        break
                    time.sleep(0.001)
                    timeout -= 1
                    print "waiting UI: " + str(timeout)
                if not timeout:
                    break
            i += 1

    def setup_guideline(self):
        cmd = " -s %s shell getprop ro.build.version.release" % self.serial
        version = g_common_obj2.adb_message(cmd).strip().split('.')
        if version[0] == '7' and version[1] == '1':
            wifi_conf = TestConfig().read('tests.tablet.dut_init.conf', 'wifisetting')
            google_account_conf = TestConfig().read('tests.tablet.dut_init.conf', 'google_account')

            mr1_ui_order = [{'text': "GET STARTED"}, {'text': "SKIP"}, {'text': "Set up as new"},
                        {'text': 'Keep your apps & data'}, {'text': 'Your Google Account'},
                        {'textContains': 'Wi‑Fi networks'}, {'text': wifi_conf["name"]},
                        {'text': wifi_conf["ssid"]}, [{'resourceId': 'com.android.settings:id/password'},
                        wifi_conf['passwd']], {'text': 'CONNECT'}, {'text': 'NEXT'},
                        [{'textContains': 'Email or phone'}, google_account_conf['user_name']],
                        [{'className': 'android.widget.EditText', 'resourceId': 'password'},
                        google_account_conf['password']], {'description': 'ACCEPT'}, {'text': 'NEXT'},
                        [{'textContains': 'back up'},{'text': 'ON'}], [{'textContains': 'Android experience'},
                        {'text': 'ON'}], {'text': 'NEXT'}, {'text': "Don't restore"}, {'text': 'Not now'},
                        {'text': 'SKIP ANYWAY'}, {'text': 'Set up later'}, {'text': 'OK'}]

            Function().continuous_click_input(mr1_ui_order)
            return
    # setup guideline
        self.d.press.home()
        Imagetype = g_common_obj2.getAndroidVersion()
        time.sleep(2)
        if self.d(description="Apps").exists and not self.d(text="GOT IT").exists:
            return
        for i in range(10):
            self.d.press.back()
            if self.d(description="Comenzar").exists:
                self.d(scrollable=True).scroll.vert.to(textContains="English (United States)")
            if self.d(description="Start").exists or self.d(description="Iniciar").exists:
                break
        if self.d(description="Start").exists or \
           self.d(description="Iniciar").exists:
            if self.d(description="Start").exists:
                self.d(description="Start").click.wait()
            elif self.d(description="Iniciar").exists:
                self.d(description="Iniciar").click.wait()
            if self.d(text="Skip").exists:
                self.d(text="Skip").click.wait()
            if self.d(text="Skip").exists:
                self.d(text="Skip").click.wait()
            if self.d(text="Skip anyway"):
                self.d(text="Skip anyway").click.wait()
            if self.d(text="Date & time").exists:
                if self.d(text="Next").exists:
                    self.d(text="Next").click.wait()
            time.sleep(1)
            self.d(text="First").click.wait()
            time.sleep(1)
            os.system("adb -s %s shell input text 'First'" % self.serial)
            time.sleep(1)
            self.d.press.back()
            self.d(text="Next").click.wait()
            if (Imagetype == "M"):
                if self.d(text="Protect this device and require a PIN, pattern, or password to unlock the screen").exists:
                    if self.d(text="Protect this device and require a PIN, pattern, or password to unlock the screen").checked:
                        self.d(text="Protect this device and require a PIN, pattern, or password to unlock the screen").click.wait()
            if self.d(text="Skip").exists:
                self.d(text="Skip").click.wait()
                self.d(text="Skip anyway").click.wait()
            if self.d(text="More").exists:
                self.d(text="More").click.wait()
            if (Imagetype == "M"):
                if self.d(description="More").exists:
                    self.d(description="More").click.wait()
            self.d(text="Next").click.wait()
            if self.d(text="Finish").exists:
                self.d(text="Finish").click.wait()
        if self.d(text="Allow").exists:
            self.d(text="Allow").click.wait()
        while self.d(text="OK").exists:
            self.d(text="OK").click.wait()
        if self.d(text="GOT IT").exists:
            self.d(text="GOT IT").click.wait()
        assert self.d(description="Apps").exists and not self.d(text="GOT IT").exists
        if (Imagetype == "M"):
            if self.d(description="Apps").exists:
                self.d(description="Apps").click.wait()
                if self.d(text="OK").exists:
                    self.d(text="OK").click.wait()
                self.d.press("home")

    def enable_developer_option(self):
    # enable developer option
        android_version_release = g_common_obj.adb_cmd_capture_msg("getprop |grep 'version.release'").split(':')[-1].strip()
        if (android_version_release == '[8.0.0]') or (android_version_release == '[8.1.0]'):
            os.system("adb -s %s shell am start -S com.android.settings/.Settings" % self.serial)
        else:
            os.system("adb -s %s shell am start -S com.android.settings/.Settings" % self.serial)
        time.sleep(2)
        ui = self.d.dump()
        if 'scrollable="true"' in ui:
            self.d(scrollable=True).scroll.toEnd()
            if self.d(textContains="Developer options").exists:
                return
            time.sleep(2)
            ui = self.d.dump()
        if 'System' in ui:
            system_or_about = 'System'
        elif 'About ' in ui:
            system_or_about = 'About'
        print 'click ' + system_or_about + ' ...'
        self.d(textContains=system_or_about).click.wait()
        if self.d(textContains="Developer options").exists:
            return
        if self.d(textContains="About ").exists:
            self.d(textContains="About ").click()
        # Fix ui not detected on bxt-p
        # Change structure to try/except
        try:
            self.d(scrollable=True).scroll.vert.to(textContains="Build number")
            time.sleep(1)
        except:
            print "No need scroll."
        for _ in range(8):
            self.d(textContains="Build number").click()
            time.sleep(.5)

    def keep_awake(self):
    # keep screen awake
        android_version_release = g_common_obj.adb_cmd_capture_msg("getprop |grep 'version.release'").split(':')[-1].strip()
        if (android_version_release == '[8.0.0]') or (android_version_release == '[8.1.0]'):
            os.system(" adb -s %s root" % self.serial)
            os.system("adb -s %s shell am start -S com.android.settings/.Settings" % self.serial)
            ui = self.d.dump()
            if not self.d(text="Display").exists:
                self.d(scrollable=True).scroll.vert.to(text="Display")
            self.d(text="Display").click.wait()
            if not self.d(text="After 30 minutes of inactivity").exists:
                self.d(text="Sleep").click.wait()
                self.d(text="30 minutes").click.wait() 
                
            if os.system("adb -s %s shell dumpsys power | grep 'mStayOn=true'" % self.serial) == 0:
                return
            elif os.system("adb -s %s shell dumpsys power | grep 'mStayOn=false'" % self.serial) == 0:
                os.system(" adb -s %s root" % self.serial)
                os.system("adb -s %s shell am start -S com.android.settings/.Settings" % self.serial)
                ui = self.d.dump()
                if 'scrollable="true"' in ui:
                    self.d(scrollable=True).scroll.toEnd()
                    time.sleep(0.5)
                self.d(textContains='System').click.wait()
                time.sleep(0.5)
                self.d(textContains="Developer options").click.wait()
                time.sleep(0.5)
                self.settings_ON_OFF("Stay awake", enable=True)
                assert os.system("adb -s %s shell dumpsys power | grep 'mStayOn=true'" % self.serial) == 0
                return
        if os.system("adb -s %s shell dumpsys power | grep 'mStayOn=true'" % self.serial) == 0:
            if os.system("adb -s %s shell dumpsys power | grep 'mScreenOffTimeoutSetting=1800000'" % self.serial) == 0:
                return
        os.system(" adb -s %s shell am start -S com.android.settings/.Settings" % self.serial)
        if not self.d(textContains="Developer options").exists:
            self.d(scrollable=True).scroll.vert.to(textContains="Developer options")
        self.d(textContains="Developer options").click.wait()
#        if self.d(text="Stay awake").right(className="android.widget.Switch"):
#            if not self.d(text="Stay awake").right(className="android.widget.Switch").checked:
#                self.d(text="Stay awake").right(className="android.widget.Switch").click()
        self.settings_ON_OFF("Stay awake", enable = True)
        self.d.press.back()
        if not self.d(text="Display").exists:
            self.d(scrollable=True).scroll.vert.to(text="Display")
        self.d(text="Display").click.wait()
        if not self.d(text="After 30 minutes of inactivity").exists:
            self.d(text="Sleep").click.wait()
            self.d(text="30 minutes").click.wait()
        assert self.d(text="After 30 minutes of inactivity").exists

    def connect_AP(self, ssid, password, security="WPA/WPA2 PSK"):
    # connect to WIFI
        if os.system("adb -s %s shell dumpsys connectivity | grep 'CONNECTED/CONNECTED.*%s' > /dev/null 2>&1" % (self.serial, ssid)) == 0:
            print "Already connect wifi"
            return
        android_version_release = g_common_obj.adb_cmd_capture_msg("getprop |grep 'version.release'").split(':')[-1].strip()
        if (android_version_release == '[8.0.0]') or (android_version_release == '[8.1.0]'):
            print '1'
            os.system("adb -s %s shell am start -S com.android.settings/.Settings" % self.serial)
        else:
            os.system(" adb -s %s shell am start -S com.android.settings/.Settings" % self.serial)
        time.sleep(2)
        if self.d(textMatches="Wi.*Fi").exists:
            self.d(textMatches="Wi.*Fi").click.wait()
        if self.d(textContains='Wi-Fi').exists:
            self.d(textContains='Wi-Fi').click.wait()
            if self.d(textContains='Wi-Fi').exists:
                self.d(textContains='Wi-Fi').click.wait()
        if self.d(textContains='Network').exists:
            self.d(textContains='Network').click.wait()
            if self.d(textContains='Wi‑Fi').exists:
                self.d(textContains='Wi‑Fi').click.wait()
        if not self.d(className="android.widget.Switch").checked:
            self.d(className="android.widget.Switch").click.wait()
            time.sleep(3)
        if self.d(text=ssid).exists and self.d(text=ssid).down(text="Connected") != None:
            print "connect wifi pass..."
            return
        if self.d(text="Connected").exists:
            self.d(text="Connected").click.wait()
            #self.d(text="Forget").click.wait()
            # made case insensitive as text case varies in different Android releases
            self.d(textMatches="(F|f)(O|o)(R|r)(G|g)(E|e)(T|t)").click.wait()
        if not self.d(textContains="Add network"):
            #self.d.press.menu()
            #self.d(text="Add network").click.wait()
            if not self.d(text="Enter the SSID"):
                try:
                    ui = self.d.dump()
                    if 'scrollable="true"' in ui:
                        self.d(scrollable=True).scroll.vert.to(textContains="Add network")
                    if self.d(text="Add network"):
                        self.d(text="Add network").click.wait()
                    else:
                        # since 'Add network' option available under menu in Android-M below lines are added
                        self.d.press.menu()
                        if self.d(text="Add network").exists:
                            self.d(text="Add network").click.wait()
                except:
                    print "can not scroll or can not find 'Add network' option"
        elif self.d(text="Add network"):
            self.d(text="Add network").click.wait()
        else:
            if not self.d(text="Add network").exists:
                self.d.press.menu()
            self.d(text="Add network").click.wait()
        self.d(text="Enter the SSID").set_text(ssid)
        self.d.press.back()
        #self.d(resourceId="com.android.settings:id/security", index=1).click.wait()
        self.d(text="None").click.wait()
        self.d(text="WPA/WPA2 PSK").click.wait()
        self.d(resourceId="com.android.settings:id/password").set_text(password)
        android_version_release = g_common_obj.adb_cmd_capture_msg("getprop |grep 'version.release'").split(':')[-1].strip()
        if (android_version_release == '[8.0.0]') or (android_version_release == '[8.1.0]'):
            self.d(textMatches="(S|s)(A|a)(V|v)(E|e)").click.wait()
        else:
            self.d.press.back()
            # made case insensitive as text case varies in different Android releases
            self.d(textMatches="(S|s)(A|a)(V|v)(E|e)").click.wait()
        #time.sleep(3)
        #KB_appear=g_common_obj2.adb_message('shell dumpsys SurfaceFlinger |grep "| InputMethod"')
        #print KB_appear
        #if KB_appear:
        #    self.d.press.back()
        #self.d(resourceId="android:id/button1").click.wait()
        for j in range(10):
            if not self.d(text="Connected").exists:
                try:
                    self.d(scrollable=True).scroll.vert.to(text=ssid)
                except:
                    print "can not scroll"
                    break
                if self.d(text="Connected").exists:
                    print "connect wifi pass..."
                    break
                else:
                    self.d(text=ssid).click.wait()
                    # using 'display text' to make it consist when UI changes. When going with
                    # 'buttons' as it will not be consistent when more buttons added.
                    if self.d(text="CONNECT"):
                        self.d(text="CONNECT").click.wait()
                        time.sleep(3)
                    else:
                        self.d.press.back()
                    time.sleep(10)
            else:
                print "connect wifi pass..."
                break
        assert self.d(text=ssid).down(text="Connected").exists, "Connect AP failed"

    def add_google_account(self, username, password):
    # add google account
        os.system("adb -s %s shell am start -S com.android.settings/.Settings" % self.serial)
        self.d(text="Accounts").click.wait()
        if self.d(text="Google").exists:
            self.d.press.menu()
            if self.d(resourceId="android:id/checkbox").checked:
                self.d(resourceId="android:id/checkbox").click.wait()
                self.d(text="OK").click.wait()
            else:
                self.d.press.back()
            return
        self.d(text="Add account").click.wait()
        self.d(text="Google").click.wait()
        for i in range(20):
            if self.d(description="Add your account").exists:
                break
            assert not self.d(text="Couldn't sign in").exists
            time.sleep(3)
        assert self.d(description="Add your account").exists
        time.sleep(3)
        y = self.d.displayHeight
        x = self.d.displayWidth
        self.d.click(200, y / 1.77454545)
        # self.d.click(200, 710)
        os.system("adb -s %s shell input text '%s'" % (self.serial, username))
        time.sleep(1)
        self.d.click(x / 1.08108108, y / 1.28)
        time.sleep(10)
        self.d.click(x / 4.44444444, y / 2.16888889)
        os.system("adb -s %s shell input text '%s'" % (self.serial, password))
        time.sleep(2)
        self.d.click(x / 1.08108108, y / 1.28)
        time.sleep(5)
        os.system("adb -s %s shell input tap '%d' '%d'" % (self.serial, x / 1.15384615, y / 1.02521008))
        for i in range(50):
            if self.d(textStartsWith="Back up your phone").exists:
                break
            assert not self.d(text="Couldn't sign in").exists
            time.sleep(3)
        assert self.d(textStartsWith="Back up your phone").exists
        self.d(text="Next").click.wait()
        if self.d(text="SKIP").exists:
            self.d(text="SKIP").click.wait()
        time.sleep(3)
        if self.d(text="Remind me later").exists:
            self.d(text="Remind me later").click.wait()
            self.d(text="Next").click.wait()
        time.sleep(2)
        self.d.press.menu()
        if self.d(resourceId="android:id/checkbox").checked:
            self.d(resourceId="android:id/checkbox").click.wait()
            self.d(text="OK").click.wait()
        os.system("adb -s %s shell am force-stop com.android.settings" % self.serial)

    def add_google_account_mr1(self, username, password):
    # add google account
        g_common_obj.launch_app_am("com.android.settings", ".Settings")
        android_version_release = g_common_obj.adb_cmd_capture_msg("getprop |grep 'version.release'").split(':')[-1].strip()
        if  android_version_release == '[8.1.0]':
            if self.d(textContains="accounts").exists:
                self.d(textContains="accounts").click.wait()
        else:
            if not self.d(text="Accounts").exists:
                self.d(scrollable=True).scroll.vert.to(text="Accounts")
            if self.d(text="Accounts").exists:
                self.d(text="Accounts").click.wait()
        if self.d(text="Google").exists:
            if  android_version_release == '[8.1.0]':
                if self.d(textContains="@gmail.com").exists:
                    return
            else:
                self.d.press.menu()
                if self.d(resourceId="android:id/checkbox").checked:
                    self.d(resourceId="android:id/checkbox").click.wait()
                    self.d(text="OK").click.wait()
                else:
                    self.d.press.back()
                return
        self.d(text="Add account").click.wait()
        if self.d(text="Google"):
            self.d(text="Google").click.wait()
        self.d(className="android.widget.EditText").wait.exists(timeout=60000)
        assert not self.d(textContains="problem communicating with Google servers")
        time.sleep(2)
        self.d(className="android.widget.EditText").set_text(username)
        time.sleep(1)
        self.d.press.enter()
        self.d(className="android.widget.EditText").wait.exists(timeout=10000)
        time.sleep(10)
        self.d(className="android.widget.EditText").set_text(password)
        self.d.press.enter()
        #For Joule I Agree button
        time.sleep(2)
        self.d(description="I AGREE").click.wait().exists(timeout=3000)
        time.sleep(5)
        self.d(description="NEXT").click.wait().exists(timeout=3000)
        #for 3gr: click OK
        if self.d(descriptionContains="Recover your account"):
            g_common_obj.adb_cmd("input tap 882 1855")
        # for 3gr: click ACCEPT
        if self.d(descriptionContains="Sign in"):
            g_common_obj.adb_cmd("input tap 882 1855")
        # for bxtp abl: may not accept
        for i in range(2):
            if self.d(text="Next").wait.exists(timeout=8000):
                self.d(text="Next").click.wait()
            if self.d(description="ACCEPT").wait.exists(timeout=8000):
                bounds = self.d(description="ACCEPT").bounds
                x = (bounds["left"] + bounds["right"]) / 2
                y = (bounds["top"] + bounds["bottom"]) / 2
                g_common_obj.adb_cmd("input tap %d %d" % (x, y))
                g_common_obj.adb_cmd("input tap 1055 1800")
                g_common_obj.adb_cmd("input tap 675 1180")
        #self.d(resourceId="com.google.android.gms:id/agree_backup").wait.exists(timeout=60000)
        if self.d(resourceId="com.google.android.gms:id/agree_backup").exists:
            self.d(resourceId="com.google.android.gms:id/agree_backup").click.wait()
        assert self.d(textContains="back up").exists
        self.d(text="NEXT").click.wait()
        assert not self.d(descriptionContains="Sign in") or self.d(descriptionContains="password")
        # Set up payment info
        for i in range(10):
            if self.d(text="SKIP").exists:
                self.d(text="SKIP").click.wait()
            if self.d(text="Remind me later").exists:
                self.d(text="Remind me later").click.wait()
                self.d(text="Next").click.wait()
                break
            if self.d(text="No thanks").exists:
                self.d(text="No thanks").click.wait()
                self.d(text="Continue").click.wait()
                break

        self.d.press.menu()
        if self.d(resourceId="android:id/checkbox").checked:
            self.d(resourceId="android:id/checkbox").click.wait()
            self.d(text="OK").click.wait()
        g_common_obj.adb_cmd("am force-stop com.android.settings")

    def remove_google_accout(self):
        os.system("adb -s %s shell am start -S com.android.settings/.Settings" % self.serial)
        self.d(text="Accounts").click.wait()
        if self.d(text="Google").exists:
            self.d(text= "Google").click.wait()
            self.d.press.menu()
            if self.d(text="Remove account").exists:
                self.d(text="Remove account").click.wait()
                if self.d(text="Remove account").exists:
                    self.d(text="Remove account").click.wait()

    def close_lock_screen(self):
    # delete clock screen
        self.d.wakeup()
        if self.d(resourceId="com.android.systemui:id/lock_icon"):
            self.d.swipe(self.x / 2, self.y, self.x / 2, 0)
            self.d.press("menu")
            self.d.press("home")
        Imagetype = g_common_obj2.getAndroidVersion()

        android_version_release = g_common_obj.adb_cmd_capture_msg("getprop |grep 'version.release'").split(':')[-1].strip()
        if (android_version_release == '[8.0.0]') or (android_version_release == '[8.1.0]'):
            os.system("adb -s %s shell am start -S com.android.settings/.Settings" % self.serial)
        else:
            os.system(" adb -s %s shell am start -S com.android.settings/.Settings" % self.serial)
        time.sleep(2)
        if not self.d(textContains="Security").exists:
            try:
                self.d(scrollable=True).scroll.vert.to(textContains="Security")
            except:
                print "Can't scroll"
        self.d(textContains="Security").click()
        if not (Imagetype == "M"):
            if self.d(text="Screen lock").down(text="None") != None:
                return
        self.d(text="Screen lock").click()
        self.d(text="None").click()
        assert self.d(text="Screen lock").down(text="None") != None
        self.d.press("home")

    def set_language(self):
        android_version_release = g_common_obj.adb_cmd_capture_msg("getprop |grep 'version.release'").split(':')[-1].strip()
        if (android_version_release == '[8.0.0]') or (android_version_release == '[8.1.0]'):
            os.system("adb -s %s shell am start -S com.android.settings/.Settings" % self.serial)
        else:
            os.system("adb -s %s shell am start -S com.android.settings/.Settings" % self.serial)
        if android_version_release == '[8.1.0]':
            self.d(text="System").click()
            self.d(textContains="Languages").click()
            if self.d(textContains="United States").exists:
                return
            self.d(text="Languages").click()
            self.d(textContains="United States").click()
            assert self.d(text="English (United States)").exists
        else:
            if not self.d(textContains="Language").exists:
                self.d(scrollable=True).scroll.vert.to(textContains="Language")
            self.d(textContains="Language").click()
            if self.d(text="English (United States)").exists:
                return
            self.d(text="Language").click()
            self.d(scrollable=True).scroll.vert.to(textContains="United States")
            self.d(textContains="United States").click()
            assert self.d(text="English (United States)").exists

    def root_DUT(self):
    # root DUT
        os.system("adb -s %s root" % self.serial)
        time.sleep(3)
        result = os.popen("adb -s %s root" % self.serial).read()
        if " is already running as root" in result:
            print "root succeed"
        else:
            assert False, "the DUT can't root"

    def init_PlayStore(self):
        os.system("adb -s %s shell am start -S com.android.vending/.AssetBrowserActivity" % self.serial)
        time.sleep(3)
        if self.d(textStartsWith="Just a sec").exists or self.d(text="Couldn't sign in").exists:
            self.d.press.back()
            assert False
        device = None
        for i in range(30):
            time.sleep(1)
            if self.d(text="Accept").exists:
                self.d(text="Accept").click.wait()
            if self.d(text="ACCEPT").exists:
                self.d(text="ACCEPT").click.wait()
            if self.d(description="Menu").exists:
                device = self.d(description="Menu")
                break
            if self.d(description="Show navigation drawer").exists:
                device = self.d(description="Show navigation drawer")
                break
            time.sleep(2)
        assert device != None
        device.click.wait()
        self.d(text="Settings").click.wait()
        self.d(text="Auto-update apps").click.wait()
        self.d(text="Do not auto-update apps").click.wait()
        os.system("adb -s %s shell am force-stop com.android.vending" % self.serial)

    def init_youtube(self):
    # click ok in youtube
        self.d.press.home()
        os.system("adb -s %s shell am start -S com.google.android.youtube/com.google.android.apps.youtube.app.WatchWhileActivity" % self.serial)
        time.sleep(5)
        for i in range(60):
            if self.d(text="What to Watch").exists and not self.d(text="OK").exists:
                break
            if self.d(text="OK").exists:
                self.d(text="OK").click.wait()
            if self.d(text="Skip").exists:
                self.d(text="Skip").click.wait()
            if self.d(text="Retry").exists:
                self.d(text="Retry").click.wait()
            time.sleep(2)
        assert self.d(text="What to Watch").exists
        self.d.press.back()
        self.d(description="YouTube").click.wait()
        time.sleep(2)
        if self.d(text="Skip").exists:
            self.d(text="Skip").click.wait()
        assert not self.d(text="Skip").exists
        os.system("adb -s %s shell am force-stop com.google.android.youtube" % self.serial)

    def init_chrome(self):
    # init chrome's two options
        os.system("adb -s %s shell am start -S com.android.chrome/com.google.android.apps.chrome.Main" % self.serial)
        time.sleep(5)
        if self.d(resourceId='com.android.chrome:id/negative_button').exists:
            self.d(resourceId='com.android.chrome:id/negative_button').click.wait()
        while self.d(text="ACCEPT & CONTINUE").exists:
            self.d(text="ACCEPT & CONTINUE").click.wait()
        if self.d(text="Accept & continue"):
            self.d(text="Accept & continue").click.wait()
        if self.d(resourceId="com.android.chrome:id/positive_button"):
            self.d(resourceId="com.android.chrome:id/positive_button").click.wait()
        if self.d(text="OK, got it"):
            self.d(text="OK, got it").click.wait()
        if self.d(text="Done").exists:
            self.d(text="Done").click.wait()
        if self.d(text="No thanks").exists:
            self.d(text="No thanks").click.wait()
        if self.d(text="No Thanks").exists:
            self.d(text="No Thanks").click.wait()
        if self.d(description="More options"):
            self.d(description="More options").click.wait()
        if self.d(text="Settings"):
            self.d(text="Settings").click.wait()
        if self.d(text="Sign in to Chrome").exists:
            self.d(text="Sign in to Chrome").click.wait()
            if self.d(text="Add a Google Account").exists:
                self.d(text="Cancel").click.wait()
            elif self.d(resourceId='com.android.chrome:id/negative_button').exists:
                    self.d(resourceId='com.android.chrome:id/negative_button').click.wait()
            elif self.d(text="Sign in").exists:
                self.d(text="Sign in").click.wait()

        if self.d(text="Basics").down(resourceId="com.android.chrome:id/header_title", textContains="@gmail.com") != None:
            self.d(text="Basics").down(resourceId="com.android.chrome:id/header_title", textContains="@gmail.com").click.wait()
            if self.d(textContains="Auto sign").right(resourceId="android:id/checkbox").checked:
                self.d(textContains="Auto sign").right(resourceId="android:id/checkbox").click()
            self.d.press.back()
        self.d(text="Privacy").click.wait()
        if self.d(textContains="Navigation").right(resourceId="android:id/checkbox").checked:
            self.d(textContains="Navigation").right(resourceId="android:id/checkbox").click.wait()
        if self.d(textContains="Search and").right(resourceId="android:id/checkbox").checked:
            self.d(textContains="Search and").right(resourceId="android:id/checkbox").click.wait()
#        if self.d(textContains="Usage and crash").right(resourceId="android:id/checkbox").checked:
#            self.d(textContains="Usage and crash").right(resourceId="android:id/checkbox").click.wait()
        self.d(text="Clear browsing data").click.wait()
        if not self.d(text="Browsing history").checked:
            self.d(text="Browsing history").click.wait()
        if self.d(text="Cache"):
            if not self.d(text="Cache").checked:
                self.d(text="Cache").click.wait()
        if not self.d(textContains="Cookies").checked:
            self.d(textContains="Cookies").click.wait()
        if self.d(text="Clear"):
            self.d(text="Clear").click.wait()
        if self.d(text="Clear data"):
            self.d(text="Clear data").click.wait()
        if self.d(text="Accept").exists:
            self.d(text="Accept").click.wait()
        os.system("adb -s %s shell am start -S com.android.chrome/com.google.android.apps.chrome.Main" % self.serial)

        # url = "https://www.google.com/url?sa=t&source=web&rct=j&ei=tC9HVOD3KsmbyASH04DwCg&url=http://www.youtube.com/&ved=0CB8QFjAA&usg=AFQjCNGSle0TFsQ2TAB0ZyJ8XkUguVQKpA"
        url = "https://m.youtube.com"
        self.d(resourceId="com.android.chrome:id/url_bar").clear_text()
        self.d(resourceId="com.android.chrome:id/url_bar").set_text(url)
        self.d.press.enter()
        time.sleep(2)
        for i in range(10):
            if self.d(resourceId="com.android.chrome:id/url_bar", textContains="m.youtube.com").exists:
                os.system("adb -s %s shell am force-stop com.android.chrome" % self.serial)
                return
            if self.d(textStartsWith="Open with", resourceId="android:id/title").exists:
                self.d(textContains="Chrome").click.wait()
                if self.d(text="Always").exists:
                    self.d(text="Always").click.wait()
                    if self.d(resourceId="com.google.android.gms:id/webview").exists:
                        self.d.press.back()
                time.sleep(5)
                if self.d(text="OK").exists:
                    self.d(text="OK").click.wait()
                os.system("adb -s %s shell am force-stop com.android.chrome" % self.serial)
                return
            time.sleep(3)
        os.system("adb -s %s shell am force-stop com.android.chrome" % self.serial)
        assert False

    def init_camera(self):
        os.system("adb -s %s shell am start -S com.android.camera2/com.android.camera.CameraLauncher" % self.serial)
        time.sleep(4)
        if self.d(description="Shutter").exists:
            os.system("adb -s %s shell am force-stop com.android.camera2" % self.serial)
            return
        if self.d(text="DENY"):
            self.d(text="DENY").click.wait()
        #for 3gr
        for i in range(4):
            if self.d(text="Allow"):
                self.d(text="Allow").click.wait()
        if self.d(text="NEXT").exists:
            self.d(text="NEXT").click.wait()
        time.sleep(2)
        assert self.d(description="Shutter").exists
        os.system("adb -s %s shell am force-stop com.android.camera2" % self.serial)

    def disable_verify_apps(self):
        android_version_release = g_common_obj.adb_cmd_capture_msg("getprop |grep 'version.release'").split(':')[-1].strip()
        if (android_version_release == '[8.0.0]') or (android_version_release == '[8.1.0]'):
            os.system("adb -s %s shell am start -S com.android.settings/.Settings" % self.serial)
        else:
            os.system(" adb -s %s shell am start -S com.android.settings/.Settings" % self.serial)
        if not self.d(textContains="Developer options").exists:
            try:
                self.d(scrollable=True).scroll.vert.to(textContains="Developer options")
                if not self.d(textContains="Developer options").exists:
                    raise Exception
            except:
                self.d(textContains="System").click.wait()
        self.d(textContains="Developer options").click.wait()
        self.d(scrollable=True).scroll.vert.to(textContains="Verify apps over USB")
        assert not self.settings_ON_OFF("Verify apps over USB", enable = False)

    def fill_internal_storage(self, percent):
        dest = 100 - percent
        cmd = "adb -s %s shell dumpsys diskstats | grep Data-Free | awk '{print $7}'" % self.serial
        for i in range(500):
            free_percent = os.popen(cmd).read().splitlines()
            free_percent = int(free_percent[0][:-1])
            print 100 - free_percent
            if free_percent <= dest:
                return
            os.system("adb -s %s shell dd if=/dev/zero of=/sdcard/test_%03d bs=50m count=1 >/dev/null 2>&1" % (self.serial, i))
        assert free_percent <= dest

    def deploy_content(self, location, content, script_name):
    # execute deploy_content_file
        from testlib.util.repo import Artifactory
        arti_obj = Artifactory(location)
        ret_file = arti_obj.get(content)
        print ret_file

        base_path = os.path.dirname(os.path.abspath(__file__))
        script_path = os.path.join(base_path, script_name)
        print script_path
        assert os.path.isfile(script_path), "%s does not exist" % script_path
        os.system("bash %s -s %s -f %s" % (script_path, self.serial, ret_file))

    def init_photo(self):
        self.d.press.home()
        os.system("adb -s %s shell am start -S com.google.android.apps.plus/com.google.android.apps.photos.phone.PhotosLauncherActivity" % self.serial)
        time.sleep(2)
        if self.d(text="Later").exists:
            self.d(text="Later").click()
            time.sleep(2)
        if self.d(text="ALL").exists and not self.d(text="No thanks").exists:
            os.system("adb -s %s shell am force-stop com.google.android.apps.photos" % self.serial)
            return
        for i in range(10):
            if self.d(text="ALL").exists and not self.d(text="No thanks").exists:
            # if not self.d(resourceId="android:id/progress").exists:
                break
            if self.d(text="Cancel").exists:
                self.d(text="Cancel").click.wait()
            if self.d(text="No thanks").exists:
                self.d(text="No thanks").click.wait()
            if self.d(text="All").exists:
                self.d(text="ALL").click.wait()
            time.sleep(2)
        assert self.d(text="Photos").exists and not self.d(text="No thanks").exists
        self.d.press.back()
        self.d(description="Photos").click.wait()
        time.sleep(2)
        if self.d(text="No thanks").exists:
            self.d(text="No thanks").click.wait()
        assert self.d(text="Photos").exists and not self.d(text="No thanks").exists
        os.system("adb -s %s shell am force-stop com.google.android.apps.photos" % self.serial)

    def init_PlayMusic(self):
        os.system("adb -s %s shell am start -S com.google.android.music/com.android.music.activitymanagement.TopLevelActivity" % self.serial)
        for i in range(30):
            time.sleep(2)
            if not self.d(resourceId="com.google.android.music:id/tutorial_logo").exists:
                break
        assert not self.d(resourceId="com.google.android.music:id/tutorial_logo").exists
        if self.d(text="Skip").exists:
            self.d(text="Skip").click.wait()
        if self.d(text="Use Standard").exists:
            self.d(text="Use Standard").click.wait()
        if self.d(text="Done").exists:
            self.d(text="Done").click.wait()
        if self.d(resourceId="com.google.android.music:id/play_drawer_list").exists:
            self.d.press.back()
        if self.d(text="Got it").exists:
            self.d(text="Got it").click.wait()
        assert self.d(text="Listen Now").exists
        os.system("adb -s %s shell am start -S com.google.android.music/com.android.music.activitymanagement.TopLevelActivity" % self.serial)
        time.sleep(5)
        assert self.d(text="Listen Now").exists
        os.system("adb -s %s shell am force-stop com.google.android.music" % self.serial)

    def set_timedate(self):
        android_version_release = g_common_obj.adb_cmd_capture_msg("getprop |grep 'version.release'").split(':')[-1].strip()
        if (android_version_release == '[8.0.0]') or (android_version_release == '[8.1.0]'):
            os.system("adb -s %s shell am start -S com.android.settings/.Settings" % self.serial)
        else:
            os.system(" adb -s %s shell am start -S com.android.settings/.Settings" % self.serial)
        if android_version_release == '[8.1.0]':
            self.d(text="System").click()
            self.d(textContains="Date").click.wait()
        else:
            if not self.d(text="Date & time").exists:
                self.d(scrollable=True).scroll.vert.to(text="Date & time")
            self.d(text="Date & time").click()
        # set 24 hour format
        if android_version_release != '[8.1.0]':
            if self.d(text="Use 24-hour format").right(resourceId="android:id/checkbox") != None:
                if not self.d(text="Use 24-hour format").right(resourceId="android:id/checkbox").checked:
                    self.d(text="Use 24-hour format").right(resourceId="android:id/checkbox").click.wait()
                time.sleep(1)
                assert self.d(text="Use 24-hour format").right(resourceId="android:id/checkbox").checked
            else:
                if not self.d(text="Use 24-hour format").right(resourceId="android:id/switchWidget").checked:
                    self.d(text="Use 24-hour format").right(resourceId="android:id/switchWidget").click.wait()
                time.sleep(1)
                assert self.d(text="Use 24-hour format").right(resourceId="android:id/switchWidget").checked
        # set timezone to beijing
        if not self.d(text="GMT+08:00 China Standard Time").exists:
            self.d(text="Select time zone").click()
            self.d(scrollable=True).scroll.vert.to(text="Shanghai")
            self.d(text="Shanghai").click()
        # set date and time
        date_time = time.strftime('%Y%m%d.%H%M%S')
        os.system('adb -s %s root' % self.serial)
        os.system('adb -s %s shell date -s "%s"' % (self.serial, date_time))
    
    def delete_camera_folder_file(self):
        # need root permission
        folder_list = ["/storage/sdcard0/DCIM/Camera/", "/storage/sdcard0/Download/", \
                     "/storage/sdcard0/Movies/", "/storage/sdcard0/Music/", \
                     "/storage/sdcard1/"]
        for i in folder_list:
            os.system("adb -s %s shell rm -rf %s*" % (self.serial, i))
            time.sleep(3)
            result = os.popen("adb -s %s shell ls %s" % (self.serial, i)).read()
            if "" != result:
                print "wipe data failed. result=" + str(result)
            else:
                print "wipe data success!"

    def upgrade_hangouts_app(self):
        for _ in range(2):  # make sure both hangouts and play service are latest.
            os.system("adb -s %s shell am start -S \
                com.google.android.talk/.SigningInActivity" % self.serial)
            try:
                timeout = time.time() + 60
                update_required = False
                hangouts_update_required = False
                googleservices_update_required = False
                while not update_required:
                    if self.d(text='ACCEPT').exists:
                        self.d(text='ACCEPT').click.wait()
                    if self.d(text='Skip').exists & self.d(text='Next').exists:  # for M device
                        self.d(text='Skip').click.wait()
                    if self.d(text='SKIP').exists & self.d(text='NEXT').exists:  # for N device
                        self.d(text='SKIP').click.wait()
                    if self.d(text='Allow').exists:
                        self.d(text='Allow').click.wait()
                    # check hangouts update required or not.
                    if self.d(text='Update required').exists:
                        self.d(text='OK').click.wait()
                        update_required = True
                        hangouts_update_required = True
                    # check play service update required or not.
                    if self.d(text='Update Google Play services').exists:
                        self.d(text='Update').click.wait()
                        update_required = True
                        googleservices_update_required = True
                    if self.d(description='New conversation').exists:
                        print "No need update hangouts."
                        return  # final status, no need to update.
                    assert time.time() < timeout, Exception
                if hangouts_update_required:
                    for _ in range(2):
                        if self.d(text='CONTINUE').exists:  # New in Android 6.0 and higher pop up
                            self.d(text='CONTINUE').click.wait()
                            time.sleep(2)
                        if self.d(text='UPDATE').exists:
                            self.d(text='UPDATE').click.wait()
                            time.sleep(2)
                    timeout = time.time() + 300
                    finished = False
                    while not finished:
                        hangouts_version = g_common_obj.adb_cmd_capture_msg("dumpsys package \
                        com.google.android.talk | grep versionName")
                        if len(hangouts_version.split()) == 2:
                            finished = True
                        if self.d(textContains='Error code').exists:
                            print "Update error."
                            return
                        assert time.time() < timeout, Exception
                if googleservices_update_required:
                    for _ in range(2):
                        if self.d(text='CONTINUE').exists:  # New in Android 6.0 and higher pop up
                            self.d(text='CONTINUE').click.wait()
                            time.sleep(2)
                        if self.d(text='UPDATE').exists:
                            self.d(text='UPDATE').click.wait()
                            time.sleep(2)
                    timeout = time.time() + 300
                    finished = False
                    while not finished:
                        services_version = g_common_obj.adb_cmd_capture_msg("dumpsys package \
                        com.google.android.gms | grep versionName")
                        if len(services_version.split()) == 2:
                            finished = True
                        if self.d(textContains='Error code').exists:
                            print "Update error."
                            return
                        assert time.time() < timeout, Exception
            except:
                raise Exception("Update time out!")
            finally:
                os.system("adb -s %s shell am force-stop com.google.android.talk" % self.serial)
                os.system("adb -s %s shell am force-stop com.android.vending" % self.serial)

    def check_gms_launcher(self):
        '''
        Check if gms launcher is pre-installed;
        If not exist, then get source apk and install it directly.
        After installed, it will be set as default launcher.
        '''
        from testlib.audio import resource
        _adb = g_common_obj
        pkgname = "com.google.android.launcher"
        apk = "google_android_launcher.apk"
        cmd = "pm list package %s" % pkgname
        ret = _adb.adb_cmd_capture_msg(cmd)
        n_ret = ret.split("\n")
        for s in n_ret:
            if s == "package:%s" % pkgname:
                print "GMS launcher is already installed."
                return
        app = resource.get_app(apk)
        cmd = "install -r %s" % app
        _adb.adb_cmd_common(cmd)
        home_cmd = "am start -a android.settings.HOME_SETTINGS"
        _adb.adb_cmd_capture_msg(home_cmd)
        if self.d(text="Home app").exists:
            self.d(text="Home app").click.wait()
            time.sleep(1)
        if self.d(text="Google Now Launcher").exists:
            self.d(text="Google Now Launcher").click.wait()
            time.sleep(1)
        self.d.press.home()
        time.sleep(2)
        try:
            # Make sure Google Launcher is actually set to default.
            if self.d(resourceId="android:id/text1", text="Google Now Launcher").exists:
                self.d(resourceId="android:id/text1", text="Google Now Launcher").click.wait()
                time.sleep(1)
                self.d(text="ALWAYS").click()
                time.sleep(3)
            timeout = time.time() + 60
            finish = 0
            while finish < 3:
                if self.d(text="CONTINUE").exists:
                    self.d(text="CONTINUE").click.wait()
                    time.sleep(1)
                    finish += 1
                if self.d(text="SKIP").exists:
                    self.d(text="SKIP").click.wait()
                    finish += 1
                    time.sleep(1)
                if self.d(text="NO THANKS").exists:
                    self.d(text="NO THANKS").click.wait()
                    finish += 1
                    time.sleep(1)
                if self.d(text="START FRESH").exists:
                    self.d(text="START FRESH").click.wait()
                    finish += 1
                    time.sleep(1)
                if self.d(text="GOT IT").exists:
                    self.d(text="GOT IT").click.wait()
                    finish += 1
                    time.sleep(1)
                    return
                if finish >= 1:
                    self.d.press.home()
                    assert self.d(resourceId="android:id/text1",
                                  text="Google Now Launcher").exists, Exception
                    print "Setup finished."
                    return
                assert time.time() < timeout, Exception
        except:
            os.system("adb -s %s uninstall %s" % (self.serial, pkgname))
            self.d.press.home()
            raise Exception("Setup GMS home failed.")
