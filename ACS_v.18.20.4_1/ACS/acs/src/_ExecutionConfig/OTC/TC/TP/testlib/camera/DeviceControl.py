'''
Created on Oct 26, 2015

@author: shankang
'''
from uiautomator import device as d
import time
import os
import subprocess
from subprocess import Popen, PIPE
import numpy as np
from scipy.linalg import solve
import threading
from threading import Thread
import sys
from testlib.util.common import g_common_obj
from testlib.common.common import g_common_obj2

class DeviceControl:

    def findSpan(self, zoomin, x, y, device):
        cmd = "adb shell /data/eventHunter -i " + device + " -g TOUCH -p " + str(x) + " -q " + str(y) + " -t B"
        os.system(cmd)
        text = d(resourceId="com.example.showxy:id/TextView01").text
        option = "SPREAD"
        if zoomin == False:
            option = "PINCH"
        count = 50
        while True:
            cmd = "adb shell /data/eventHunter -i " + device + " -g " + option + " -p " + str(x) + " -q " + str(y) + " -c " + str(count) + " -s 2 -t B -o B -d 1"
            os.system(cmd)
            text = d(resourceId="com.example.showxy:id/TextView01").text
            if text.find("zoom takes effect now") != -1:
                break
            count += 10
        print "===find span=" + str(count)
        return count

    def getRobotDeviceName(self):
#         import subprocess
        cmd = "ls -l /dev/ttyACM*"
        output = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE).communicate()
        s = output[0][:-1]
        deviceName = ""
        for s1 in s.split('\n'):
            s1 = " ".join(s1.split())
            device = s1.split(" ")[-1].split('/')[-1]
            print device
            cmd = "ls -l /sys/class/tty/" + device
            output = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE).communicate()
            s2 = output[0][:-1].split('->')[-1].split("..")[-1]
            print s2
            cmd = "cat /sys" + s2 + "/../../../idVendor"
            # print cmd
            output = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE).communicate()
            if output[0][:-1] == "2341":
                deviceName = "/dev/" + device
                break
        return deviceName

    def getInputDevice(self, x, y):
        cmd = "adb shell getevent -p"
        output = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE).communicate()
        device = ""
        for s in output[0][:-1].split("\n"):
            if s.find("add device") != -1:
                device = s.split(":")[1]
    #            print device
            elif s.find("name:") != -1:
                name_str = s.split(":")[1]
                if name_str.lower().find("ts") != -1 or name_str.lower().find("touch") != -1:
                    break
        return device.strip("\r")

    #Need to know the effective region for zoom operatios, so the FOV region need to be passed into it
    def getZoomCommands(self, region="center"):
        """
        Need to download showxp apk and eventHunter at first; 
        adb root
        adb push eventHunter /data/
        """
        from testlib.camera.CameraCommon import CameraCommon
        from testlib.multimedia.multimedia_setting import MultiMediaSetting
        fileDir = logDir = g_common_obj.get_user_log_dir()
        print "===="+logDir
        if ".mtbf" in logDir:
            i = logDir.find("report")+len("report/")
            subString = logDir[i:]
            dateStrLen = len(subString.split("/")[0])
            fileDir = fileDir[:i+dateStrLen]
        print "----"+fileDir
        fileName = fileDir+os.sep+CameraCommon().getPlatform()+"_zoom_"+region+".txt"
        if os.path.exists(fileName)==False:
            mmSetting = MultiMediaSetting(CameraCommon().DEFAULT_CONFIG_FILE)
            mmSetting.install_apk("showxy_apk")
            self.pushEventHunterToDevices(mmSetting)
            os.system("adb shell am start -n %s/%s" %("com.example.showxy", "com.example.showxy.MainActivity"))
            bounds = d(resourceId="com.example.showxy:id/TextView01").bounds
            print bounds
            x_direct = bounds["right"]
            y_direct = bounds["bottom"]
            cmd = "adb shell wm size"
            output = subprocess.Popen(cmd,shell=True,stdout=subprocess.PIPE).communicate()
            result = output[0][:-1]
            l=(''.join(result.split())).split(":")[1].split("x")
            v1 = int(l[0])
            v2 = int(l[1])
            x_base = 0
            y_base = 0
            if x_direct < y_direct:
                x_base = min(v1,v2)
                y_base = max(v1,v2)
            else:
                x_base = max(v1,v2)
                y_base = min(v1,v2)
            print "x_base=%d, y_base=%d"%(x_base,y_base)
            device=self.getInputDevice(x_base/2,y_base/2)
            print device
        #    print (bounds["left"], y_base/2) 
        #    print (bounds["right"], y_base/2)
            cmd="adb shell dumpsys display| grep DisplayDeviceInfo"
            output=subprocess.Popen(cmd,shell=True,stdout=subprocess.PIPE).communicate()
            result = output[0][:-1]
            print result
            dpi = ""
            for s in result.split(","):
                if s.find("dpi")!=-1:
                    dpi=s
                    break
            l=(''.join(dpi[:-3].split())).split("x")
            dpi1 = float(l[0])
            dpi2 = float(l[1])
            print "dpi1=%f,dpi2=%f"%(dpi1,dpi2)
        #    device = "/dev/input/event4"
            last_str = d(resourceId="com.example.showxy:id/TextView01").text
            x_number = 2*x_base
            y_number = 2*y_base
            print "x_number=%d,y_number=%d"%(x_number,y_number)
            data_count = 0
            a=None
            b=[]
            
            delta_x = x_base/8
            delta_y = y_base/8
            while data_count<3:
                print "==="
                print "x_number=%d, y_number=%d"%(x_number,y_number)
                cmd = "adb shell /data/eventHunter -i "+device+" -g TOUCH -p "+str(x_number)+" -q "+str(y_number)+" -t B"
                os.system(cmd)
                text = d(resourceId="com.example.showxy:id/TextView01").text
                if text!=last_str:
                    data_count+=1
                    x1=int(text.split(",")[0].split("=")[1])
                    y1=int(text.split(",")[1].split("=")[1])
                    print x1,y1
                    b.append(x_number)
                    b.append(y_number)
                    if a==None:
                        a = np.array([[x1,y1,1,0,0,0], [0,0,0,x1,y1,1]])
                    else:
                        a = np.vstack([a, [x1,y1,1,0,0,0]])
                        a = np.vstack([a, [0,0,0,x1,y1,1]])
                    print "---"+str(data_count)+"-----"
                if (x_number-delta_x)<0 or (y_number-delta_y) < 0:
                    print "Error!"
                    exit(-1)
                x_number-=delta_x
                y_number-=delta_y
                if data_count!=0:
                    tmp=x_number
                    x_number = y_number
                    y_number = tmp
                    tmp=delta_x
                    delta_x=delta_y
                    delta_y=tmp
                last_str=text
            print a
            print b
            x = solve(a, b)
            print(x)
            factorX = []
            factorY = []
            for i in range(3):
        #        factorX.append(int(round(x[i])))
                if x[i]>-0.1 and x[i]<0.1:
                    factorX.append(0)
                else:
                    factorX.append(x[i])
            for i in range(3,6):
        #        factorY.append(int(round(x[i])))
                if x[i]>-0.1 and x[i]<0.1:
                    factorY.append(0)
                else:
                    factorY.append(x[i])
            print factorX
            print factorY
            if region=="center":
                startPoint = np.array((x_base/2, y_base/2))
            elif region=="topLeft":
                startPoint = np.array((x_base/4, y_base/4))
            elif region=="topRight":
                startPoint = np.array((x_base*2/3, y_base/4))
            elif region=="bottomLeft":
                startPoint = np.array((x_base/4, y_base*2/3))
            elif region=="bottomRight":
                startPoint = np.array((x_base*2/3, y_base*2/3))
            p = factorX[0]*startPoint[0]+factorX[1]*startPoint[1]+factorX[2]
            q = factorY[0]*startPoint[0]+factorY[1]*startPoint[1]+factorY[2]
            print p,q
            spreadspan = self.findSpan(True, int(round(p)), int(round(q)), device)+20
            pinchspan = self.findSpan(False, int(round(p)), int(round(q)), device)+20
            cmds=[]
            cmd="adb shell /data/eventHunter -i "+device+" -g SPREAD -p "+str(int(round(p)))+" -q "+str(int(round(q)))+" -c "+str(spreadspan)+" -s 2 -t B -o B -d 1"
            print cmd
            cmds.append(cmd)
            cmd="adb shell /data/eventHunter -i "+device+" -g PINCH -p "+str(int(round(p)))+" -q "+str(int(round(q)))+" -c "+str(pinchspan)+" -s 2 -t B -o B -d 1"
            print cmd
            cmds.append(cmd)
            os.system("adb shell am force-stop com.example.showxy")
            #g_common_obj.stop_app_am("com.example.showxy")
            #Need to return to the camera UI
            fileObj = open(fileName, "w")
            fileObj.write(",".join(cmds))
            fileObj.close()
            return cmds
        else:
            fileObj = open(fileName, "r")
            mlist = fileObj.read().split(",")  
            return mlist

    def pushEventHunterToDevices(self, multimedia_setting):
        g_common_obj2.root_on_device()
        cmd="adb shell ls /data/eventHunter"
        output=subprocess.Popen(cmd,shell=True,stdout=subprocess.PIPE).communicate()
        if output[0][:-1].find("No such file or directory")!=-1:
            multimedia_setting.push_file_new("Multimedia_Camera/apk/eventHunter", "/data/eventHunter")
            g_common_obj.adb_cmd("chmod 777 /data/eventHunter")

    def pressLightBoxButton(self, index):
        from testaid.usbrly import UsbRelay
#         deviceName = self.getRobotDeviceName()
#         print deviceName
        lightBox = UsbRelay("/dev/ttyUSB0")
        lightBox.turn_on(index)
        lightBox.turn_off(index)

    """
    Moving the device to adjust the FOV content
    
    """
    def adjustFOV(hostDir, region, robot, times=10, far=False):
        from testlib.camera.checkIQ import CheckIQ
        from testaid.usbrly import UsbRelay
        checkIQ = CheckIQ()
        targetDir = "/mnt/sdcard/"
        expectedStr = "yes"
        if far==True:
            expectedStr = "small"
        #hostDir = "/data/"
        import time
        for i in range(times):
            cmd = "adb shell screencap -p " + targetDir + str(i) + ".png"
            os.system(cmd)
            cmd = "adb pull " + targetDir + str(i) + ".png " + hostDir
            os.system(cmd)
            ret = checkIQ.findBorder(hostDir+"/" + str(i) + ".png", region)  # CheckIQ.Region(0,144,720,998,False)
            if ret == expectedStr:
                break
            else:
                print ret
                robot.move(1)
                #time.sleep(5)
                
    """
    Scroll the system to switch to specified picture
    """            
    def selectTestObject(self, picNo=0):
        pass
