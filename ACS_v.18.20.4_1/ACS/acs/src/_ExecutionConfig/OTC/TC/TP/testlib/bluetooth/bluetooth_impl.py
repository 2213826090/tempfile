from testlib.util.common import g_common_obj
import time
import os

class BluetoothImpl:
    """
    Bluetooth Test Impl Class
    """
    bt_package = "com.android.settings"
    bt_activity = ".Settings"
    es_package = "com.estrongs.android.pop"
    es_activity = ".view.FileExplorerActivity"

#--------- begin locator -------------
    class Locator(object):
        """
            Helper for locator UI Object
        """

        def __init__(self, device):
            self.d = device

        @property
        def btn_bluetooth(self):
            """ UI button bluetooth """
            return self.d(text = "Bluetooth")

        @property
        def btn_switch(self):
            """ UI button switch """
            return self.d(resourceId="com.android.settings:id/switch_widget")

        @property
        def enabled_btn_switch(self):
            """ UI button switch """
            return self.d(resourceId="com.android.settings:id/switch_widget", enabled=True)

        @property
        def btn_off(self):
            """ UI button off """
            return self.d(text = "Off", enabled=True)

        @property
        def btn_on(self):
            """ UI button on """
            return self.d(text = "On", enabled=True)

        @property
        def status(self):
            """ UI button status """
            return self.d(
                resourceId="com.android.settings:id/switch_text").\
            info.get("text").decode('gbk')

#--------- end locator -------------

    def __init__ (self, cfg):
        self.d = g_common_obj.get_device()
        self.cfg = cfg
        self._locator = BluetoothImpl.Locator(self.d)
        base_path = os.path.split(os.path.realpath(__file__))[0].split(os.sep)
        g_common_obj.adb_cmd_capture_msg("rm -rf /storage/sdcard0/bluetooth/*");
        self.index = 0
        self.targetDir = (os.sep).join(base_path[:])
        self.file_dir = "Download"
        self.pair_name = self.cfg.get("host_bt_name")

    def set_orientation_n(self):
        """
        @summary: set orientation as n
        """
        self.d.orientation = "n"

    def launch_from_am(self,packagename,activityname):
        """launch item in the settings pannel"""
        print "start to launch ->"
        g_common_obj.launch_app_am(packagename, activityname)
        time.sleep(2)
        print "launch success <-"

    def launchESFileExplorer(self):
        self.d.press.home()
        g_common_obj.launch_app_am(self.es_package, self.es_activity)

    def check_item(self, item):
        """check if item exists in the settings panel"""
        iffind = False
        for i in range(5):
            if self.d(text=item).exists:
                iffind = True
                break
            self.d(scrollable=True).scroll.vert()
        for i in range(8):
            if self.d(text=item).exists:
                iffind = True
                break
            self.d(scrollable=True).scroll.vert.backward()
        return iffind

    def on(self):
        """turn on the bluetooth"""
        self.launch_from_am(BluetoothImpl.bt_package,BluetoothImpl.bt_activity)
        if self.d(text = "Bluetooth").exists:
            self.d(text = "Bluetooth").click.wait()
        time.sleep(1)
        if self.d(text = "OFF", className = "android.widget.Switch").exists:
            self.d(text = "OFF", className = "android.widget.Switch").click.wait()
        time.sleep(2)
        assert self.d(text = "ON").exists,"not found text"
        time.sleep(1)

    def off(self):
        """turn off the bluetooth"""
        self.launch_from_am(BluetoothImpl.bt_package,BluetoothImpl.bt_activity)
        if self.d(text = "Bluetooth").exists:
            self.d(text = "Bluetooth").click.wait()
        time.sleep(2)
        if self.d(text = "ON", className = "android.widget.Switch").exists:
            self.d(text = "ON", className = "android.widget.Switch").click.wait()
        time.sleep(3)
        assert self.d(text = "OFF").exists,"not found text"
        time.sleep(1)

    def switch_on_off(self, repeat, time_sleep):
        """switch the bluetooth"""
        self.launch_from_am(BluetoothImpl.bt_package, BluetoothImpl.bt_activity)
        if self._locator.btn_bluetooth.exists:
            self._locator.btn_bluetooth.click.wait()
        time.sleep(3)

        for i in range (0, int(repeat)):
            print "click %d" % i
            assert self._locator.btn_switch.exists
            if self._locator.btn_on.exists:
                self._locator.enabled_btn_switch.click()
#                if not self._locator.btn_off.wait.exists(timeout=60000):
#                    self._locator.btn_switch.click()
                assert self.d(text="OFF", resourceId="com.android.settings:id/switch_widget", enabled=True).wait.exists(timeout=60000)

            elif self._locator.btn_off.exists:
                self._locator.enabled_btn_switch.click()
#                if not self._locator.btn_on.wait.exists(timeout=60000):
#                    self._locator.btn_switch.click()
                assert self.d(text="ON", resourceId="com.android.settings:id/switch_widget", enabled=True).wait.exists(timeout=60000)
            else:
                print "[WARNING] Bluetooth status is [%s]" % self._locator.status
            time.sleep(time_sleep)

    def toggle(self):
        """turn on/off the bluetooth"""
        bt_btn = self.d(className="android.widget.Switch")
        bt_btn.click.wait()
        time.sleep(2)

    def launch_file_explore(self):
        """launch the ex file explore"""
        print "[INFO]: Start to launch file explore ->"
        self.d.press.home()
        g_common_obj.launch_app_am(BluetoothImpl.es_package,BluetoothImpl.es_activity)
        print "[INFO]: Launch ex file explore success <-"
        time.sleep(5)
        if self.d(resourceId="com.estrongs.android.pop:id/upgrade_disable_auto_check").exists:
            self.d(resourceId="com.estrongs.android.pop:id/upgrade_disable_auto_check").click()
            self.d(text="Cancel").click()
        for i in range(10):
            if self.d(resourceId="com.estrongs.android.pop:id/long_press").exists:
                self.d(resourceId="com.estrongs.android.pop:id/long_press").click()
                time.sleep(1)
            else:
                break

    def _get_point(self, text=None):
        """return the specified point"""
        if text:
            if self.check_item(text):
                rect = self.d(text=text).info["bounds"]
                x = (rect["left"] + rect["right"])/2
                y = (rect["top"] + rect["bottom"])/2
                return (x, y)
            return (-1, -1)
        else:
            return (-1, -1)

    def cd_file_dir(self):
        """cd to the file directory"""
        if self.check_item(self.file_dir) and \
           self.d(text=self.file_dir).exists:
            self.d(text=self.file_dir).click.wait()
            self.d(text = "bttest").click.wait()

    def select_files(self, files=[]):
        """select files"""
        self.cd_file_dir()
        if files:
            x, y = self._get_point(files[0])
            self.d.swipe(x, y, x, y, steps=50)
            for f in files[1:]:
                x, y = self._get_point(f)
                self.d.click(x, y)

    def shareFileByBluetooth(self):
        if self.d(className = "android.widget.LinearLayout",index = 4).exists:
            self.d(className = "android.widget.LinearLayout",index = 4).click()
        elif self.d(resourceId = "com.estrongs.android.pop:id/menuButton").exists:
            self.d(resourceId = "com.estrongs.android.pop:id/menuButton").click.wait()
        iffind = False
        item = "Share"
        for i in range(10):
            if self.d(text=item).exists:
                self.d(text=item).click()
                iffind = True
                break
            self.d(scrollable=True).scroll.vert()
        assert iffind == True
        self.d(text = "Share Via").swipe.up()
        self.d(text="Bluetooth").click.wait()
        time.sleep(5)
        m = 1
        while True:
            if self.d(text = self.pair_name).exists:
                self.d(text = self.pair_name).click.wait()
                break
            else :
                #self.d(text="".info["bounds"]
                if self.d(resourceId = "android:id/list").info["scrollable"]:
                    self.d(scrollable = True).scroll.to(text = self.pair_name)
                if self.d(text = self.pair_name).exists:
                    self.d(text = self.pair_name).click.wait()
                else:
                    self.d(scrollable = True).scroll.toBeginning()
                    self.d(text = "Scan for devices").click.wait()
                    time.sleep(10)
            if m > 10:
                assert self.d(text = self.pair_name).exists, "[ERROR]: Not found text"
                break
            m = m + 1
        time.sleep(2)

    def send_files(self):
        """send files"""
        print "[INFO]: Send file start"
        time.sleep(2)
        self.select_files(["1M.txt", "2M.txt", "10M.txt"])
        self.shareFileByBluetooth()

    def check_finish(self, number, timeout):
        """check files transfer finish"""
        y = self.d.info["displayHeight"]
        time.sleep(2)
        iffind = False
        while True:
            self.d.swipe(0,0,0,y/2,steps=10)
            if self.d(text="%d successful, 0 unsuccessful." % number).exists:
                iffind = True
                break
            time.sleep(5)
            timeout = timeout - 5
            if timeout <= 0:
                break
        assert iffind == True, "[ERROR]: Notification not found!"

    def multiple_file_transfer(self):
        """transfer multiple files"""
        print "[INFO]: Start to transfer multiple files ->"
        self.cleanNotification()
        time.sleep(2)
        if self.d(text="Refresh").exists:
            self.d(text="Refresh").click.wait()
        time.sleep(2)
        self.send_files()
        self.check_finish(3, 300)
        print "[INFO]: Transfer multiple files success <-"

    def back_home(self):
        for i in range(3):
            self.d.press.back()

    def search_bluetooth(self):
        """ press 'search for devices' button"""
        target_name = self.cfg.get("host_bt_name")
        print "[INFO]: Target_name=" , target_name
        self.on()
        print "[INFO]: Waitting for 18s until search for devices completed"
        time.sleep(30)
        if not self.d(text = target_name).exists:
            self.d(scrollable=True).scroll.vert.to(text=target_name)
        assert self.d(text = target_name).exists, "[ERROR]: Target not found!"
        print "[INFO]: Search bluetooth success!"

    def showNotificationAndFileReceive(self,height,timeout):
        """start receive file"""
        self.d.swipe(0, 0, 0, height,steps=10)
        notify = self.d(text="Do you want to receive this file?")
        iffind = False
        while True:
            if notify.exists:
                iffind = True
                break
            time.sleep(5)
            timeout = timeout - 5
            if timeout <= 0:
                break
        assert iffind == True
        notify.click.wait()
        accept = self.d(text="Accept",resourceId="android:id/button1")
        if accept.exists:
            accept.click.wait()

    def cleanNotification(self):
        y = self.d.info["displayHeight"]
        self.d.swipe(0,0,0,y/2,steps=10)
        time.sleep(1)
        btn_clear = self.d(resourceId="com.android.systemui:id/clear_all_button")
        btn_clear2 = self.d(resourceId="com.android.systemui:id/dismiss_text")
        if btn_clear.exists:
            btn_clear.click.wait()
        elif btn_clear2.exists:
            btn_clear2.click.wait()
        else:
            self.d.press.back()

    def checkFileHasReceived(self, number, timeout):
        """check files transfer finish"""
        print "[INFO]: Check start"
        y = self.d.info["displayHeight"]
        self.showNotificationAndFileReceive(y/2,timeout)
        #self.d.swipe(0,0,0,y/2,steps=10)
        iffind = False
        while True:
            self.d.swipe(0,0,0,y/2,steps=10)
            if self.d(text="%d successful, 0 unsuccessful." % number).exists:
                iffind = True
                print "[INFO]: File %d received!" % number
                break
            time.sleep(5)
            timeout = timeout - 5
            if timeout <= 0:
                break
        assert iffind == True
        print "[INFO]: Check finish"

    def checkPaired(self,device):
        """chekc whether the headset pairing"""
        print "[INFO]: Device name = %s" %(device)
        if self.d(text = device).exists:
            if self.d(text = "Connected (no phone)").exists:
                if self.d(text = device).down(text = "Connected (no phone)").exists:
                    print "[ERROR]: The bluetooth headset not paired"
            elif self.d(text = "Connected").exists:
                if self.d(text = device).down(text = "Connected").exists:
                    print "[INFO]: The bluetooth headset paired"
                else :
                    print "[ERROR]: The bluetooth headset not paired"
        assert self.d(text = device).exists, "[ERROR]: Pair device not found!"

    def playMoviesFromPhotosApp(self):
        """play movies from the Photos application and select a video file to play"""
        self.d.press.home()
        g_common_obj.launch_app_am(self.es_package, self.es_activity)
        self.d(resourceId = "com.estrongs.android.pop:id/tool_fast_access").click.wait()
        time.sleep(2)
        if self.d(text = "Find a new version, update now?").exists:
            self.d(text = "Cancel").click.wait()
        if not self.d(text = "Device").exists:
            self.d(text = "Local").click.wait()
        self.d(text = "Device").click.wait()
        time.sleep(1)
        self.d(text = "storage").click.wait()
        time.sleep(1)
        self.d(text = "sdcard0").click.wait()
        time.sleep(1)
        self.d(text="Movies").click.wait()
        if self.d(text = "mpeg4part2").exists:
            self.d(text = "mpeg4part2").click.wait()
        else :
            self.d(scrollable = True).scroll.to(text = "mpeg4part2")
            self.d(text = "mpeg4part2").click.wait()
        for i in range(24):
            if self.d(text = "mpeg4part2.mp4").exists:
                self.d(text = "mpeg4part2.mp4").click()
            time.sleep(2)
            if self.d(text = "ES Media Player").exists:
                self.d(text = "ES Media Player").click.wait()
            print "wait 10 minutes until video play completed"
            time.sleep(2)
            for j in range(10):
                assert not self.d(resourceId = "").exists, "[ERROR]: Video is not playing"
                time.sleep(60)

    def imageFileTransfer(self):
        """image transfer"""
        self.launchESFileExplorer()
        print "[INFO]: Image file transfer"
        time.sleep(2)
        self.d(text ="Pictures").click.wait()
        if self.d(text = "multi_media_test").exists:
            self.d(text = "multi_media_test").click.wait()
        else :
            self.d(scrollable = True).scroll.to(text = "multi_media_test")
            self.d(text = "multi_media_test").click.wait()
        if self.d(text = "jpg.jpg").exists:
            self.d(text = "jpg.jpg").click.wait()
        if self.d(text = "ES Image Browser").exists:
            self.d(text = "ES Image Browser").click.wait()
        time.sleep(1)

        self.shareFileByBluetooth()
        self.d.press.home()
        self.check_finish(1, 300)

    def bigFileTransfer(self):
        """big file transfer"""
        self.launchESFileExplorer()
        time.sleep(2)
        self.d(text ="Movies").click.wait()
        print "[INFO]: Big file transfer"
        if self.d(text = "bluetoothtest").exists:
            self.d(text = "bluetoothtest").click.wait()
        else :
            self.d(scrollable = True).scroll.to(text = "bluetoothtest")
            self.d(text = "bluetoothtest").click.wait()
            print "[INFO]: Click mpeg4"
        if self.d(text = "bigfile.mp4").exists:
            self.d(text = "bigfile.mp4").click.wait()
        if self.d(text = "ES Media Player").exists:
            self.d(text = "ES Media Player").click.wait()
        time.sleep(1)
 
        self.shareFileByBluetooth()
        self.d.press.home()
        print "[INFO]: Transfer start"
        self.check_finish(1, 2400)
