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
writer.writerow(['Time','TAMB','TSKN','TCPU','Power_CAP'])
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

def TAMB_Check():
    output = os.popen("adb shell cat /sys/class/thermal/thermal_zone2/temp")
    TAMB_Str = output.read()
    TAMB_int = int(TAMB_Str)
    aaa = TAMB_int / 1000
    return aaa

def TSKN_Check():
	output = os.popen("adb shell cat /sys/class/thermal/thermal_zone1/temp")
	TSKN_Str = output.read()
	TSKN_int = int(TSKN_Str)
	bbb = TSKN_int / 1000
	return bbb

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

def DFPT_POLICY_Check(TAMB,TSKN,TCPU):
    if ((TAMB >= 89) or (TSKN >= 105) or (TCPU >= 109)):
        log_String("Device will SHUTDOWN!")
    elif (((TAMB < 60) & (TCPU < 90)) or ((TAMB >= 60) & (TCPU < 85))):
		log_String("Device is running on Active Policy")
		CPU_POWERCAP = POWERCAP_Check()
		log_String("The CPU POWER is " + str(CPU_POWERCAP) + "mw")
    elif ((TAMB < 60) & (TCPU >= 90)):
        log_String("Device is running on Adaptive Performance Policy" + "----PSVT_NORM")
        if (TCPU < 105):
            CPU_POWERCAP = POWERCAP_Check()
            if ((CPU_POWERCAP < 13000000) & (CPU_POWERCAP > 6000000)):
                log_String("The CPU POWER is " + str(CPU_POWERCAP) + "mw" + "---Normal Range")
            else:
                log_String("The CPU POWER is " + str(CPU_POWERCAP) + "mw" + "---Wrong Range")
        elif ((TCPU >= 105) & (TCPU < 110)):
            CPU_POWERCAP = POWERCAP_Check()
            if((CPU_POWERCAP < 6000000) & (CPU_POWERCAP > 3000000)):
                log_String("The CPU POWER is " + str(CPU_POWERCAP) + "mw" + "---Normal Range")
            else:
                log_String("The CPU POWER is " + str(CPU_POWERCAP) + "mw" + "---Wrong Range")
        elif(TCPU >= 110):
            CPU_POWERCAP = POWERCAP_Check()
            if (CPU_POWERCAP < 3000000):
                log_String("The CPU POWER is " + str(CPU_POWERCAP) + "mw" + "---Normal Range")
            else:
                log_String("The CPU POWER is " + str(CPU_POWERCAP) + "mw" + "---Wrong Range")
    elif((TAMB >= 60) & (TCPU >= 85)):
        log_String("Device is running on Adaptive Performance Policy" + "----PSVT_HIGH")
        if (TCPU < 100):
            CPU_POWERCAP = POWERCAP_Check()
            if ((CPU_POWERCAP < 13000000) & (CPU_POWERCAP > 6000000)):
                log_String("The CPU POWER is " + str(CPU_POWERCAP) + "mw" + "---Normal Range")
            else:
                log_String("The CPU POWER is " + str(CPU_POWERCAP) + "mw" + "---Wrong Range")
        elif ((TCPU >= 100) & (TCPU < 110)):
            CPU_POWERCAP = POWERCAP_Check()
            if((CPU_POWERCAP < 6000000) & (CPU_POWERCAP > 3000000)):
                log_String("The CPU POWER is " + str(CPU_POWERCAP) + "mw" + "---Normal Range")
            else:
                log_String("The CPU POWER is " + str(CPU_POWERCAP) + "mw" + "---Wrong Range")
        elif(TCPU >= 110):
            CPU_POWERCAP = POWERCAP_Check()
            if (CPU_POWERCAP < 3000000):
                log_String("The CPU POWER is " + str(CPU_POWERCAP) + "mw" + "---Normal Range")
            else:
                log_String("The CPU POWER is " + str(CPU_POWERCAP) + "mw" + "---Wrong Range")

def CSV_File_Write(TAMB,TSKN,TCPU,POWERCAP):
    with open(filename, 'ab') as csvfile:
        writer = csv.writer(csvfile)
        #time_Str = time.strftime("%H%M%S")
        #time_int = int(time_Str)
        writer.writerow([No,TAMB,TSKN,TCPU,POWERCAP])
    f.close()

if __name__ == "__main__":
    Check_log_file()
    os.system("adb root")
    print "adb root device, please wait 2s!"
    time.sleep(2)
    while (check_ADB() == True):
    	TAMB = TAMB_Check()
        log_String("The TAMB is " + str(TAMB) + "C")
        TSKN = TSKN_Check()
        log_String("The TSKN is " + str(TSKN) + "C")
        TCPU = TCPU_Check()
        log_String("The TCPU is " + str(TCPU) + "C")
        POWERCAP = POWERCAP_Check()
        DFPT_POLICY_Check(TAMB,TSKN,TCPU)
        CSV_File_Write(TAMB,TSKN,TCPU,POWERCAP)
        No = No + 1
        time.sleep(1)
        log_String("------------------------------")
    else:
    	print "Please check the connection, ensure the device connect to PC!"
