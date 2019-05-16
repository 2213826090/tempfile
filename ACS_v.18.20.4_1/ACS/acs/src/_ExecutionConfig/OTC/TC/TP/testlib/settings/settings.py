#coding=utf8=
from testlib.common.common import g_common_obj2, g_logger
from testlib.common.impl import ImplCommon
import time


class Settings(ImplCommon):

    def launchSettings(self):
        g_common_obj2.launchAppByName("Settings")

    def launchSetting(self):
        self.launchSettings()

    @property
    def uiDeveloperOptionsKwargs(self):
        return {"text":"Developer options"}

    @property
    def uiDeveloperOptions(self):
        return self.d(**self.uiDeveloperOptionsKwargs)

    @property
    def uiStayAwake(self):
        return self.d(text="Stay awake")

    @property
    def uiStayAwakeCheckBox(self):
        return self.uiStayAwake.right(className="android.widget.CheckBox")

    @property
    def uiDisplayKwargs(self):
        return {"text":"Display"}

    @property
    def uiCastScreenKwargs(self):
        return {"text":"Cast screen"}

    def openDisplay(self):
        self.d(text="Display").click()

    def openDisplaySleepTime(self):
        self.d(text="Sleep").click()

    def setDisplaySleepTime(self, timeText="2 minutes"):
        self.d(text=timeText).click()

    def openSecurity(self):
        self.d(text="Security").click()

    def openScreenLock(self):
        self.d(text="Screen lock").click()

    def ChoseScreenLock(self, t="None"):
        self.d(text=t).click()

    def setOrientation(self, orientation='n'):
        self.d.orientation = orientation

    def pressBack(self):
        self.d.press.back()

    def openSoundNotification(self):
        self.d(text="Sound & notification").click()

    def openDefaultNotificationRingtone(self):
        self.d(text="Default notification ringtone").click()

    def setDefaultNotificationRing(self, text=None, index=None, **kwargs):
        if text is not None:
            kwargs["text"] = text
        if index is not None:
            kwargs["index"] = index
        self.ringListview().child(**kwargs).click()

    def ringListview(self):
        return self.d(resourceId="android:id/select_dialog_listview")

    def saveNotificationRing(self):
        self.openDefaultNotificationRingtone()
        self.oldRing = self.ringListview().child(checked=True).text
        self.pressBack()

    def restoreNotificationRing(self):
        self.openDefaultNotificationRingtone()
        self.ringListview().scroll.vert.to(text=self.oldRing)
        self.d(text=self.oldRing).click()

    def clickOk(self):
        self.d(text="OK").click()

    def openDeveloperOptions(self):
        self.d().scroll.vert.to(**self.uiDeveloperOptionsKwargs)
        self.uiDeveloperOptions.click()

    def getStayAwakeStatus(self):
        return self.uiStayAwakeCheckBox.checked

    def setStayAwakeStatus(self, stat=None):
        if stat is None or (self.getStayAwakeStatus() != stat):
            self.uiStayAwakeCheckBox.click()

    def touchList(self,*kwargsList):
        time.sleep(1)
        s=-1
        for i in range(len(kwargsList)-1,-1,-1):
            each = kwargsList[i]
            if self.d(**each).exists:
                s=i
                break
        if s<0:
            return
        if s==0:
            self.launchSettings()
        for each in kwargsList:
            self.d(**each).click.wait()
            time.sleep(1)

    def openCastScreen(self):
        self.launchSettings()
        self.d(text = "Display").click.wait()
        self.d(text = "Cast screen").click.wait()

    def setEnableWirelessDisplay(self, on=True):
        self.d(description="More options", \
               className="android.widget.ImageButton").click.wait()
        checked = self.d(resourceId="android:id/checkbox", \
        className="android.widget.CheckBox").info["checked"]
        if on and not checked or not on and checked:
            self.d(resourceId="android:id/checkbox", \
                   className="android.widget.CheckBox").click.wait()
        else :
            self.d(resourceId="android:id/checkbox", \
                   className="android.widget.CheckBox").click.wait()
            self.d(description="More options", \
                   className="android.widget.ImageButton").click.wait()
            self.d(resourceId="android:id/checkbox", \
                   className="android.widget.CheckBox").click.wait()


    def isWidiConnect(self, adapterName=None):
        time.sleep(0.5)
        if adapterName is None:
            return self.d(text="Connected").exists
        else:
            return (self.d(text = adapterName).down().info["text"] == "Connected")

    def connectWidi(self, adapterName=None):
        """connect Miracast adapter"""
        self.openCastScreen()
        time.sleep(5)
        if adapterName and not self.d(text=adapterName, enabled=True).exists:
            adapterName=None
            print "adapter %s not found, try to connect the other one"%adapterName
        else:
            print "adapter name:",adapterName
        if self.isWidiConnect(adapterName):
            return self.d(text="Connected").up(resourceId="android:id/title").text
        self.setEnableWirelessDisplay(True)
        time.sleep(10)
        def _iterWiDi():
            if adapterName and self.d(text=adapterName, enabled=True).exists:
                yield self.d(text=adapterName, enabled=True)
            else:
                i=0
                while True:
                    if self.d(resourceId="android:id/list").child(index=i,className="android.widget.LinearLayout").child(resourceId="android:id/title", enabled=True).exists:
                        yield self.d(resourceId="android:id/list").child(index=i,className="android.widget.LinearLayout").child(resourceId="android:id/title", enabled=True)
                    else:
                        break
                    i+=1
        s=[_iterWiDi()]
        def _getOne():
            try:
                return s[0].next()
            except StopIteration:
                s[0]=_iterWiDi()
                try:
                    return s[0].next()
                except StopIteration:
                    assert False, "Couldn't find widi adapter!"
        for repeat_time in range(10):
            tmp = _getOne()
            if not self.d(text="Connecting").exists:
                tmp.click()
                time.sleep(2)
            self.d(text="Connecting").wait.gone(timeout=60000)
            time.sleep(3)
            if self.isWidiConnect(adapterName):
                return self.d(text="Connected").up(resourceId="android:id/title").text
        else:
            assert False, "couldn't connect to widi"

    def disconnectWidi(self, widiName=None, raiseError=True):
        self.openCastScreen()
        if not self.isWidiConnect(widiName):
            if raiseError:
                assert False,  "widi disconnect!"
            else:
                g_logger.error("widi disconnect!")
        else:
            self.d(text="Connected").click.wait(timeout=10000)
            time.sleep(1)
            self.d(text="Disconnect").click()

    def openBluetooth(self):
        self.d(text="Bluetooth").click()
        time.sleep(1)

    def isBluetoothConnect(self):
        return self.d(text="Connected").exists
