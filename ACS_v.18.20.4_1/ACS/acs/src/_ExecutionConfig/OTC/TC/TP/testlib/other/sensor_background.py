'''
Created on Dec 10, 2015

@author: zjh
'''
import os
from testlib.common.impl import ImplCommon
from testlib.common.common import g_common_obj2
from testlib.common.base import getTmpDir
import time

class SensorBackground(ImplCommon):
    '''
    SleepWakup app
    '''
    APP_NAME = "SensorBackground"
    PACKAGE_ACTIVITY=g_common_obj2.getPackageActiveByName(APP_NAME)
    PACKAGE_NAME, ACTIVITY_NAME = PACKAGE_ACTIVITY.split("/")

    def __getattr__(self, name):
        if name.startswith("ui"):
            return self.d(text=name[2:])
    @property
    def uiSettingButton(self):
        return self.d(text="Setting",enabled=True)

    @property
    def uiResumeButton(self):
        return self.d(text="Resume",enabled=True)

    @property
    def uiPauseButton(self):
        return self.d(text="Pause",enabled=True)

    @property
    def uiStartButton(self):
        return self.d(text="Start",enabled=True)

    @property
    def uiStopButton(self):
        return self.d(text="Stop",enabled=True)

    def launch(self):
        self.commonObj.launchAppByName(self.APP_NAME)
        time.sleep(3)


    def setting(self):
        if not self.uiSettingButton.exists and self.uiPauseButton.exists:
            self.uiPauseButton.click()
            self.uiSettingButton.wait.exist(timeout=10000)
        self.uiSettingButton.click()
        time.sleep(2)
        self.d.press.back()
        if self.uiSettingButton.exists:
            self.uiSettingButton.click()
            time.sleep(2)
        self.uierror.click()
        self.__checkedAll()
        self.setChecked(self.uiNoChange, False)
        self.uiOK.click()
        time.sleep(2)

    def __checkedAll(self):
        for i in range(1000):
            if self.d(resourceId="com.gyz.sensorbackground:id/settinglayout").child(index=i,className="android.widget.CheckBox").exists:
                self.setChecked(self.d(resourceId="com.gyz.sensorbackground:id/settinglayout").child(index=i,className="android.widget.CheckBox"))
            else:
                if not self.d(resourceId="com.gyz.sensorbackground:id/settinglayout").child(index=i).exists:
                    break
    def resume(self):
        self.uiResumeButton.click()

    def start(self):
        if self.uiStartButton.exists:
            self.uiStartButton.click()
            time.sleep(2)
        if self.uiResumeButton.exists:
            self.uiResumeButton.click()
            time.sleep(2)
        assert self.uiPauseButton.exists, "start failed"

    def stop(self):
        if self.uiStopButton.exists:
            self.uiStopButton.click()
        assert self.uiStartButton.exists, "sotp failed"




