import time
from testlib.util.common import g_common_obj
from testlib.util.log import Logger


class BluetoothSetting(object):
    def __init__(self):
        self.d = g_common_obj.get_device()
        self.adb = g_common_obj
        self.log = Logger.getlogger(self.__class__.__name__)

    def _click_if_exists(self, **kwargs):
        '''
        click elements if exists
        return True if exists, otherwise False
        '''
        if self.d(**kwargs).exists:
            self.d(**kwargs).click()
            return True
        else:
            return False

    def launch(self):
        '''
        launch Bluetooth Settings page
        '''
        self.adb.adb_cmd("am start -W com.android.settings")
        time.sleep(1)
        if self.d(scrollable=True).exists:
            self.d(scrollable=True).scroll.to(text="Bluetooth")
        self.d(text="Bluetooth").click.wait()

    def enable(self, enable):
        '''
        enable/disable Bluetooth
        '''
        self.log.info("set BT enable: %s" % enable)
        switch_id = "com.android.settings:id/switch_widget"

        def is_enabled():
            return self.d(text="ON",
                          enabled=True,
                          resourceId=switch_id).exists #noqa

        enabled = is_enabled()
        if enable != enabled:
            self.log.info("click switch button")
            self.d(resourceId=switch_id).click()
            # waits for status changed
            for _ in range(30):
                if is_enabled() == enable:
                    break
                time.sleep(1)
            else:
                raise Exception("BT %s fails after 30s timeout" %
                                ("enable" if enable else "disable"))

    def refresh(self):
        '''
        refresh BT scan
        '''
        self.log.info("refresh BT scan")
        self.d.press.menu()
        clicked = self._click_if_exists(text="Refresh")
        if not clicked:
            self.d.press.back()

    def rename(self, name):
        '''
        assign bluetooth adapter a different name
        '''
        self.log.info("rename device to %s" % name)
        self.d.press.menu()
        self.d(text="Rename this device").click.wait()
        self.d.press(123)  # move to end
        for _ in range(50):  # del text
            self.d.press(67)
        self.adb.adb_cmd("input text " + name)  # input new name
        time.sleep(0.5)
        self.d(text="Rename").click.wait()

    def wait_pair(self, devname, timeout=10):
        '''
        wait for pair request, and click pair button

        if pair request found, return True, otherwise False
        '''
        watcher = "BT_PAIR_WATCHER"
        self.d.watcher(watcher).when(text="Bluetooth pairing request")\
            .click(text="Pair", className="android.widget.Button")
        self.d.watchers.run()
        elipse = 0
        triggered = False
        dev = BTDevice(devname)
        while elipse < timeout and not triggered:
            state = dev.get_state()
            if state == BTDevice.STATE_PAIRED or \
               state == BTDevice.STATE_CONNECTING or \
               state == BTDevice.STATE_CONNECTED:  # auto connected without pair
                break
            time.sleep(0.5)
            elipse += 0.5
            btn = self.d(text="Pair", className="android.widget.Button")
            if btn.exists:
                self.log.info("click Pair Button")
                btn.click.wait()

            triggered = self.d.watcher(watcher).triggered
        self.d.watchers.remove(watcher)
        return triggered

    def connect(self, devname, retry=5):  # NOQA
        '''
        connect to a bluetooth device
        if not paired, pair first
        '''
        self.log.info("trying connect to BT dev: " + devname)
        dev = BTDevice(devname)
        for count in range(retry):
            self.log.info("Loop: %d" % count)
            if dev.get_state() == BTDevice.STATE_NOT_FOUND:
                self.log.info("BT not found currently, wait scan ...")
                for i in range(30):  # wait 30s for scan device
                    if dev.get_state() != BTDevice.STATE_NOT_FOUND:
                        break
                    if i % 5 == 0:  # don't refresh all the time
                        self.refresh()  # refresh when fist glance not found
                    time.sleep(2)
                else:
                    raise Exception("Can't find Bluetooth: " + str(dev))
            if dev.get_state() == BTDevice.STATE_AVAIL:
                self.log.info("BT found, try to pair ...")
                dev.click()  # after click, it will state pairing
                self.wait_pair(dev.name)
                time.sleep(3)
            if dev.get_state() == BTDevice.STATE_PAIRED:
                self.log.info("device paired, click to connect")
                dev.click()  # click to connect
                time.sleep(3)
            if dev.get_state() == BTDevice.STATE_CONNECTING:
                self.log.info("device connecting, wait for done")
                start = time.time()
                while time.time() - start < 90:
                    if dev.get_state() != BTDevice.STATE_CONNECTING:
                        break
                    else:
                        time.sleep(3)
                else:
                    raise Exception("device still connecting after 90s timeout")
            if dev.get_state() == BTDevice.STATE_CONNECTED:
                self.log.info("device connected")
                break
        assert dev.get_state() == BTDevice.STATE_CONNECTED, \
            "Can't connect to " + str(dev)

    def disconnect(self, devname):
        '''
        disconnect bluetooth device
        '''
        self.log.info("disconnect BT dev: " + devname)
        dev = BTDevice(devname)
        if dev.get_state() == BTDevice.STATE_CONNECTED:
            dev.click()  # click, pop a dialog to confirm
            time.sleep(2)
            if self.d(text="OK").exists:
                self.d(text="OK").click.wait()

    def remove(self, devname):
        '''
        remove paired device
        '''
        self.log.info("remove paired device: " + devname)
        dev = BTDevice(devname)
        if dev.get_state() == BTDevice.STATE_PAIRED or \
           dev.get_state() == BTDevice.STATE_CONNECTED:
            ret = dev.forget()
            if ret != 0:
                self.log.info("Can't forget device")


class BTDevice(object):
    STATE_CONNECTED = "Connected"
    STATE_CONNECTING = "Connecting"
    STATE_PAIRED = "Paired"
    STATE_AVAIL = "Avail"  # available, but not paired
    STATE_NOT_FOUND = "NoFound"  # not in scanned list

    def __init__(self, name):
        self.name = name
        self.d = g_common_obj.get_device()

    def _get_ui(self):
        if self.d(scrollable=True).exists:
            self.d(scrollable=True).scroll.toBeginning()
        return self.d(className="android.widget.FrameLayout")\
            .child_by_text(self.name,
                           className="android.widget.RelativeLayout",
                           allow_scroll_search=True)

    def get_state(self):
        '''
        get device current state
        '''
        try:
            ui = self._get_ui()
            if ui.child(textContains="Connected").exists:
                return self.STATE_CONNECTED
            if ui.child(textContains="Connecting").exists:
                return self.STATE_CONNECTING
            elif ui.sibling(className="android.widget.LinearLayout").exists:
                return self.STATE_PAIRED
            else:
                return self.STATE_AVAIL
        except:
            return self.STATE_NOT_FOUND

    def open_details(self):
        '''
        open detail dialog, only avaliable in paired or connected device
        '''
        ui = self._get_ui()
        details = ui.sibling(className="android.widget.LinearLayout")
        details.click.wait()

    def forget(self):
        '''
        forget this paired device

        if success, return 0, else return -1
        '''
        self.open_details()
        time.sleep(5)
        texts = ["Forget", "FORGET"]
        ret = -1
        for text in texts:
            if self.d.exists(text=text):
                self.d(text=text).click.wait()
                ret = 0
                break
        return ret

    def click(self):
        '''
        click device item for further operation
        '''
        self._get_ui().click()

    def __str__(self):
        return self.name
