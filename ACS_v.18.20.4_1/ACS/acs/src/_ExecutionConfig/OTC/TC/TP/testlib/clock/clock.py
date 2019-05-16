# coding: UTF-8
import os
import sys
import time
from random import randrange
import ConfigParser
from testlib.common.common import g_common_obj2
from testlib.common.base import UiWindows
import math
from testlib.common.impl import ImplCommon
from testlib.common.base import UiWindows
from testlib.util.common import g_common_obj

BASE_PATH = os.path.dirname(__file__)

class Clock(ImplCommon):
    pkgname = 'com.android.deskclock'
    ref_pkgname = 'com.google.android.deskclock'

    def launch(self):
        self.commonObj.launchAppByName("Clock")
        time.sleep(1)
        if self.d.info['currentPackageName'] not in (self.pkgname, self.ref_pkgname):
            # previous launch method fails
            g_common_obj.launch_app_am(self.pkgname, '.DeskClock')
            time.sleep(1)
            if self.d.info['currentPackageName'] != self.pkgname:
                g_common_obj.launch_app_am(self.ref_pkgname, '.DeskClock')
                time.sleep(1)

    def switchToAlarm(self):
        self.d(description="Alarm", index=0).click()

    def launchAlarm(self):
        self.launch()
        self.switchToAlarm()

    def lunchAlarm(self):
        self.d.press.home()
        if self.d(resourceId="com.android.deskclock:id/analog_appwidget").exists:
            self.d(resourceId="com.android.deskclock:id/analog_appwidget").click()
        else:
            sys.stderr.write("Warning: Couldn't find clock widget on home screen!"+os.linesep)
            self.launch()
        time.sleep(1)
        self.switchToAlarm()

    def clickAdd(self):
        self.d(resourceId="com.android.deskclock:id/fab").click()

    def setAlarm(self):
        self.lunchAlarm()
        self.clickAdd()
        self.d(text="OK").click()
        #self.d.press.home()

    def delAlarm(self):
        self.lunchAlarm()
        try:  # first try old way
            uiOn = UiWindows(self.d(resourceId="com.android.deskclock:id/onoff", text="ON").info)
            uiTime = UiWindows(self.d(resourceId="com.android.deskclock:id/onoff", text="ON").\
                               sibling(resourceId="com.android.deskclock:id/digital_clock").info)
            x,y = reduce(lambda x,y:((x[0]+y[0])/2, (x[1]+y[1])/2), [uiOn.getMidPoint(),uiTime.getMidPoint()])
            self.d.click(x,y)
        except:  # new way
            alarm_on = self.d(resourceId="%s:id/onoff" % self.pkgname,
                              checked=True)
            if alarm_on.exists:
                alarm_on.sibling(className="android.widget.TextView").click()
                if self.d(text="Cancel").exists:
                    self.d(text="Cancel").click.wait()
        time.sleep(1)
        self.d(resourceId="com.android.deskclock:id/delete").click()


    def getAlarmTime(self):
        dateStr = self.commonObj.adb_cmd_capture_msg("date +%I:%M:%S:%p").strip()
        dateList = dateStr.split(":")
        for i in range(len(dateList)-1):
            dateList[i] = int(dateList[i])
        if dateList[2]< 30:
            dateList[1] += 1
        else:
            dateList[1] += 2
        if dateList[1]>=60:
            dateList[1]=dateList[1]-60
            dateList[0]=dateList[0]+1
        if dateList[0]>=12:
            dateList[0] = dateList[0]-12
            dateList[3] = "AM" if dateList[3]=="PM" else "PM"
        self.dateList = dateList
        return dateList

    def setAlarm2(self):
        self.clickAdd()
        time.sleep(1)
        self.setAlarmDate()
        assert self.d(text="%d"%self.dateList[0]).exists
        assert self.d(text="%02d"%self.dateList[1]).exists
        assert self.d(text="%s"%self.dateList[3]).exists
        self.d(text="OK").click()

    def setAlarmDate(self):
        dateList = self.getAlarmTime()
        print dateList
        if not self.d(text=dateList[3]).exists:
            self.d(resourceId="android:id/ampm_label").click()
        timePickerInfo = self.d(resourceId="android:id/radial_picker").info
        timePickerBounds = timePickerInfo[u"bounds"]
        print timePickerBounds
        centerX = (timePickerBounds[u'left'] + timePickerBounds[u'right'])/2
        centerY = (timePickerBounds[u'top'] + timePickerBounds[u'bottom'])/2 +\
                  (timePickerBounds[u'top'] - timePickerBounds[u'bottom'])/25.382
        radius =  ( timePickerBounds[u'right'] - timePickerBounds[u'left'])*0.332
        print "radius",radius
        print math.cos((dateList[0]/12.0)*math.pi*2)
        x = int(round(centerX + radius * math.sin((dateList[0]/12.0)*math.pi*2)))
        y = int(round(centerY - radius * math.cos((dateList[0]/12.0)*math.pi*2)))
        print "center:",centerX, centerY
        print x,y
        self.d.click(x, y)
        time.sleep(1)
        def setMin(minute):
            m=float(minute)
            if m%5==1:
                m=m+0.6
            elif m%5==2:
                m=m+0.3
            elif m%5==3:
                m=m-0.1
            elif m%5==4:
                m=m-0.4
            x1 = int(round(centerX + radius * math.sin(((m-m%5)/60.0)*math.pi*2)))
            y1 = int(round(centerY - radius * math.cos(((m-m%5)/60.0)*math.pi*2)))
            x2 = int(round(centerX + radius * math.sin((m/60.0)*math.pi*2)))
            y2 = int(round(centerY - radius * math.cos((m/60.0)*math.pi*2)))
            #print i, x1,y1,x2,y2
            self.d.swipe(x1,y1, x2,y2)
        _debug=False
        if _debug:
            for i in range(60):
                setMin(i)
                print i, self.d(text="%02d"%i).exists
        else:
            setMin(dateList[1])

    def waitForAlarm(self):
        start = time.time()
        print "Wait for alarm..."
        while time.time() - start<130:
            if self.d(resourceId="com.android.deskclock:id/alarm").exists:
                x,y = UiWindows(self.d(resourceId="com.android.deskclock:id/alarm").info).getMidPoint()
                endX = self.d.info[u'displayWidth']
                self.d.swipe(x,y,endX,y)
                return True
        return False

    def delAlarm2(self):
        self.launchAlarm()
        timeText = u"%d:%02d %s"%(self.dateList[0], self.dateList[1],self.dateList[3])
        print timeText
        if not self.d(text = timeText).exists:
            self.d(scrollable = True).scroll.to(text = timeText)
        if self.d(text = timeText).exists and self.d(resourceId="com.android.deskclock:id/alarms_list").exists:
            print "exists"
            ui1 = UiWindows(self.d(text = timeText).info)
            ui2 = UiWindows(self.d(text = timeText).\
                               sibling(resourceId="com.android.deskclock:id/onoff").info)
            x,y = reduce(lambda x,y:((x[0]+y[0])/2, (x[1]+y[1])/2), [ui1.getMidPoint(),ui2.getMidPoint()])
            self.d.click(x,y)
            time.sleep(1)
            self.d(resourceId="com.android.deskclock:id/delete").click()

    def unlock(self):
        self.commonObj.unlock()

    def returnToHomeScreenAndSleep(self):
        self.d.press.home()
        self.d.sleep()
