'''
Created on Dec 10, 2014

@author: zjh
'''
import os
from testlib.common.impl import ImplCommon
from testlib.common.common import g_common_obj2
from testlib.common.base import getTmpDir
import time

class SleepWakup(ImplCommon):
    '''
    SleepWakup app
    '''
    APP_NAME = "SleepWakup"
    PACKAGE_ACTIVITY=g_common_obj2.getPackageActiveByName(APP_NAME)
    PACKAGE_NAME, ACTIVITY_NAME = PACKAGE_ACTIVITY.split("/")

    @property
    def uiDelayEdit(self):
        return self.d(resourceId="com.intel.sleepwakup:id/edtDelay")

    @property
    def uiLoopEdit(self):
        return self.d(resourceId="com.intel.sleepwakup:id/edtLoop")

    @property
    def uiRunButton(self):
        return self.d(resourceId="com.intel.sleepwakup:id/btnRun")

    def launch(self):
        self.commonObj.launchAppByName(self.APP_NAME)
        time.sleep(1)

    def launchSleepWakup(self):
        return self.launch()

    def setDelay(self, delay):
        return self.uiDelayEdit.set_text(delay)

    def setLoop(self, loop):
        return self.uiLoopEdit.set_text(loop)

    def run(self):
        return self.uiRunButton.click()

    def isPass(self, loops, logdir=getTmpDir(), passRate=0.99):
        logFile = os.path.join(logdir, "sleep_wakup.log")
        self.commonObj.adb_cmd_common("pull /data/data/com.intel.sleepwakup/files/sleep_wakup.log \"%s\""%logFile)
        if not os.path.isfile(logFile):
            return False
        print logFile
        retMsg = os.popen("cat %s | grep PASS | wc -l"%logFile).read().strip()
        return  int(retMsg) >= float(loops) * passRate




