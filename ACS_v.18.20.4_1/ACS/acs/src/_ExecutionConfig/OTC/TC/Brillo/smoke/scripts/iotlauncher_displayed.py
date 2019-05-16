# @PydevCodeAnalysisIgnore
# pylint: disable=E0602,W0212

#########################################################################
#
# @filename:    iotlauncher_displayed.py
# @description: verify the iotlauncher (example home screen) is displayed
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
    success_str = 'ActivityManager: Displayed com.android.iotlauncher/.IoTLauncher'
    i = 0
    while i < 20:
        cmdstr = 'adb logcat -d >tmp.log'
        ret = subprocess.call(cmdstr, shell=True)
        if ret != 0:
            PRINT_INFO("Failed to run: %s" % cmdstr)
            VERDICT = FAILURE
            break

        cmdstr = 'grep "' + success_str + '" tmp.log'
        ret = subprocess.call(cmdstr, shell=True)
        if ret == 0:
            PRINT_INFO("iotlauncher is displayed: %s" % success_str)
            break

        time.sleep(6)
        i = i + 1

    if i == 20:
        PRINT_ERROR("Failed to start iotlauncher in 2 mins")
        VERDICT = FAILURE

    if VERDICT == SUCCESS :
        time.sleep(2)

OUTPUT = output
