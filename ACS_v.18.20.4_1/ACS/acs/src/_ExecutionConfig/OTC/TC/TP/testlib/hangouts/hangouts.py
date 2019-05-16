'''
Created on Dec 15, 2014

@author: zjh
'''

from testlib.common.impl import ImplCommon
from testlib.hangouts.hangouts_impl import HangoutsImpl
import time
from testlib.common.common import g_logger

class Hangouts(ImplCommon):

    def __init__(self, *args, **kwargs):
        super(Hangouts, self).__init__(*args, **kwargs)
        self._impl = HangoutsImpl()

    def __getattr__(self, name):
        return getattr(self._impl, name)

    @property
    def uiOK(self):
        return self.d(text="OK")

    @property
    def uiSettings(self):
        return self.d(text="Settings")

    @property
    def uiVideoCall(self):
        return self.d(resourceId="com.google.android.apps.hangouts:id/start_hangout_menu_item")

    @property
    def uiEndCall(self):
        return self.d(resourceId="com.google.android.apps.hangouts:id/action_icon")

    @property
    def uiCallView(self):
        return self.d(resourceId="com.google.android.apps.hangouts:id/hangout_focused_textureview")

    @property
    def uiMyCallView(self):
        return self.d(resourceId="com.google.android.apps.hangouts:id/hangout_participant_textureview")

    @property
    def uiAnswer(self):
        return self.d(text="Answer")

    @property
    def uiMainMenu(self):
        return self.d(description="Open navigation drawer")

    @property
    def uiPackageTalk(self):
        return self.d(packageName="com.google.android.talk")

    @property
    def uiTimeoutDialog(self):
        return self.d(textContains="Timed out")

    @property
    def uiExceptionMsg(self):
        return self.d(resourceId="android:id/message")

    def launchHangouts(self):
        self._impl.launch_from_am()

    def openContact(self, name):
        if not self.d(text=name).exists:
            self.d(scrollable=True).scroll.vert.to(text=name)
        self.d(text=name).click()
        time.sleep(1)
        assert self.uiVideoCall.wait.exists(timeout=60000), "open contact: %s failed!"%name

    def startVideoCall(self, timeout=40):
        self.uiVideoCall.click()
        time.sleep(3)
        assert self.d(textStartsWith="Calling").exists
        s = time.time()
        while time.time() - s < timeout:
            if self.isInCalling():
                break
        else:
            assert False, "Calling no answer!"
        g_logger.info("video calling...")

    def isInCalling(self, raiseError=True):
#         if self.uiOK.exists:
#             self.uiOK.click()
        for i in range(5):
            if self.uiExceptionMsg.exists:
                msg=self.uiExceptionMsg.text
                self.uiOK.click()
                if raiseError:
                    assert False, msg
            if self.uiMyCallView.exists:
                return True
            time.sleep(2)
        return False

    def endVideoCall(self):
        self.uiCallView.click()
        self.uiEndCall.click()

    def AnswerVideoCall(self):
        if self.uiPackageTalk.exists:
            l=self.d.displayWidth
            h=self.d.displayHeight
            x = 0.5 * l
            y = 0.05 * h
            self.d.click(x,y)
            time.sleep(1)
            if self.uiAnswer.exists:
                self.uiAnswer.click()
                return True
            return False
        else:
            raise Exception("Not in Hanouts")
