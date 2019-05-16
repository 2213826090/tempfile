'''
@author: rqu1x
'''
import os
import time
import csv

resultFlag = False

No = 1
curTime = time.strftime("%Y_%m_%d_%H_%M_%S")
filename = "Script_Output_" + curTime + ".csv"
f = open(filename, 'wb')
writer = csv.writer(f)
writer.writerow(['Time','TCPU','Power_CAP'])
f.close()

def Check_log_file():
    if os.path.exists(r'Thermal_TEM_Check.log'):
        os.remove(r'Thermal_TEM_Check.log')
        return True

def log_String(msg):
    logfile = os.getcwd() + "/Thermal_TEM_Check.log"
    curTime = time.strftime("%y-%m-%d %H:%M:%S ")
    print(curTime + msg)
    os.system("echo " + curTime + msg + " >> " + logfile)

def check_ADB():
    output = os.popen("adb shell echo hello")
    connectResponce = output.read()
    result = connectResponce.find("hello")
    if (result != -1):
        log_String("adb connected")
        return True
    else:
        log_String("adb disconnected")
        return False

def TCPU_Check():
    output = os.popen("adb shell cat /sys/class/thermal/thermal_zone0/temp")
    TCPU_Str = output.read()
    TCPU_int = int(TCPU_Str)
    ccc = TCPU_int / 1000
    return ccc

def POWERCAP_Check():
	output = os.popen("adb shell cat /sys/class/powercap/intel-rapl:0/constraint_0_power_limit_uw")
	Powercap_Str = output.read()
	Powercap_int = int(Powercap_Str)
	return Powercap_int

def CSV_File_Write(TCPU,POWERCAP):
    with open(filename, 'ab') as csvfile:
        writer = csv.writer(csvfile)
        #time_Str = time.strftime("%H%M%S")
        #time_int = int(time_Str)
        writer.writerow([No,TCPU,POWERCAP])
    f.close()

if __name__ == "__main__":
    Check_log_file()
    os.system("adb root")
    print "adb root device, please wait 2s!"
    time.sleep(2)
    while (check_ADB() == True):
		TCPU = TCPU_Check()
		POWERCAP = POWERCAP_Check()
		log_String("The TCPU is " + str(TCPU) + "C" + " AND The CPU POWER is " + str(POWERCAP) + " mw")
		CSV_File_Write(TCPU,POWERCAP)
		No = No + 1
		time.sleep(1)
		log_String("------------------------------")
    else:
    	print "Please check the connection, ensure the device connect to PC!"
