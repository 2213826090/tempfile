# @PydevCodeAnalysisIgnore
# pylint: disable=E0602,W0212

#########################################################################
#
# @filename:    wifi_conn.py
# @description: verify wifi connection
#
#
# @run example:
#
#              run under ACS framework
#
# @author:      jerry.yu@intel.com
#
##########################################################################

"""
DEVICE   => ACS phone instance, check IPhone interface
DEVICE.run_cmd => if you do adb shell cmd, please do not add the \"
                 e.g: "adb shell echo hello" instead of
                      "adb shell \"echo hello\""

IO_CARD => IOCard instance (usb relay or ariane board), check IIOCard interface

PRINT_INFO  => log std message in ACS log
PRINT_DEBUG => log debug message in ACS log
PRINT_ERROR => log error message in ACS log
LOCAL_EXEC => run local cmd on the bench
TC_PARAMETERS => get a paramater value from xml test case file. Name of this parameter is defined by the user.

REPORT_PATH => path of the ACS report folder
ACS_REPORT_FILE_PATH => path of the ACS report file .xml used by AWR
MY_PATH => folder path of the python script
PATH_MANAGER => Give access to all ACS paths (ex: PATH_MANAGER.FLASH_FILE)
 - CATALOGS
 - CONFIGS
 - EMBEDDED
 - EXECUTION_CONFIG
 - REPORTS
 - TEMPLATES
 - TOOLS
 - IMAGE
 - GUI_TEXT
 - TEST_SCRIPTS
 - CACHE_PUSH_REPORTS
 - FWK_USECASE_CATALOG
 - TEST_SCRIPTS_USECASE_CATALOG
 - FWK_TESTSTEP_CATALOG
 - TEST_SCRIPTS_TESTSTEP_CATALOG
 - PARAMETER_CATALOG
 - FLASH_FILES

CREATE SECOND REPORT:
ADD_RESULT("TEST_1", PASS, "test 1 comment")
ADD_RESULT("TEST_2", FAIL, ["test 2 sub_comment", "test 2 sub_comment"])
ADD_RESULT("TEST_3", BLOCKED, "test 3 comment")

VERDICT => verdict of the test, SUCCESS or FAILURE
OUTPUT  => message that will be displayed in ACS report
           (mostly used in case of error)
"""

import time
import os
import subprocess
import sys

serialNum = DEVICE.get_serial_number()
if not serialNum:
    PRINT_ERROR("SERIAL number is not set")
    serialNum = 'SERIAL_NUM_NOT_SET'

ap_ssid = TC_PARAMETERS("AP_SSID")
ap_pass = TC_PARAMETERS("AP_PASS")
PRINT_INFO("AP_SSID extracted from TC XML file : %s" % ap_ssid)
PRINT_INFO("AP_PASS extracted from TC XML file : %s" % ap_pass)

# ADB cmd
VERDICT = SUCCESS
#------- Ensure boot is completed -----------------
exec_status, output = DEVICE.run_cmd("adb wait-for-device", 60)
if exec_status == SUCCESS :
    PRINT_INFO("Device alive")
    i = 0
    while i < 20 :
        exec_status, output = DEVICE.run_cmd("adb shell getprop sys.boot_completed", 2)
        if exec_status == SUCCESS and '1' in output:
            PRINT_INFO("Boot completed")
            break
        time.sleep(6)
        i = i + 1

    if i == 20 : 
        PRINT_ERROR("Boot NOT completed")
        VERDICT = FAILURE
else :
    VERDICT = FAILURE
    PRINT_ERROR("Device not alive: %s" % output)
#----------------------------------------------------

if VERDICT == SUCCESS:
    exec_status, output = DEVICE.run_cmd(MY_PATH+"/fb-adb shell date +'%m-%d %H:%M:%S.000'", 2)
    t_since = output
    PRINT_INFO("timestamp on device: %s" % output)
############ step 1: Connect to AP
    cmdstr = "adb shell am startservice \
        -n com.google.wifisetup/.WifiSetupService \
        -a WifiSetupService.Connect \
        -e ssid " + ap_ssid + " -e passphrase " + ap_pass

    exec_status, output = DEVICE.run_cmd(cmdstr, 5)

    success_str = 'Successfully connected to ' + ap_ssid
    i = 0
    while i < 20:
        cmdstr = 'adb logcat -t "' + t_since + '" >tmp.log'
        ret = subprocess.call(cmdstr, shell=True)
        if ret != 0:
            PRINT_INFO("Failed to run: %s" % cmdstr)
            VERDICT = FAILURE
            break

        cmdstr = 'grep "' + success_str + '" tmp.log'
        ret = subprocess.call(cmdstr, shell=True)
        if ret == 0:
            PRINT_INFO("Connection established: %s" % success_str)
            break

        time.sleep(6)
        i = i + 1

    if i == 20:
        PRINT_ERROR("Failed to establish connection in 2 mins")
        VERDICT = FAILURE

    if VERDICT == SUCCESS :
        time.sleep(2)
############ step 2: Ping Google
#       cmdstr = MY_PATH+"/fb-adb shell ping -c5 8.8.8.8"
        cmdstr = "adb shell ping -c5 10.250.2.1"
        exec_status, output = DEVICE.run_cmd(cmdstr, 30)
#    	exec_status, output = DEVICE.run_cmd(MY_PATH+"/fb-adb shell ping -c5 8.8.8.8", 30)
#       exec_status, output = LOCAL_EXEC(cmdstr, 20)
#       if exec_status == SUCCESS:
        if '10.250.2.1: icmp_seq=1' in output:
            PRINT_INFO("ping gw: 10.250.2.1 succeeds")
        else :
            VERDICT = FAILURE
            PRINT_ERROR("ping gw: 10.250.2.1 failed")
            
############ step 3: Reset and forget SSID 
        cmdstr = "adb shell am startservice \
            -n com.google.wifisetup/.WifiSetupService \
            -a WifiSetupService.Reset"
        exec_status, output = DEVICE.run_cmd(cmdstr, 5)
        time.sleep(10)

OUTPUT = output
