# @PydevCodeAnalysisIgnore
# pylint: disable=E0602,W0212

#########################################################################
#
# @filename:    check_build_num.py
# @description: verify the build# is exactly the build it's flashed
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
MY_PATH => folder path of the python scritp
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

buildNum = os.getenv('RELEASE_NUMBER', '')
if not buildNum:
    PRINT_ERROR("RELEASE_NUMBER is not set")
    buildNum = 'RELEASE_NUMBER_IS_NOT_SET_ERR'

i = 0
while i < 10:
    exec_status, output = DEVICE.run_cmd("adb shell getprop ro.build.fingerprint", 2)
    PRINT_INFO("%s" % output)
    if exec_status == SUCCESS and buildNum in output:
        VERDICT = SUCCESS
        PRINT_INFO("check_build_num: %s" % output)
	break
    time.sleep(4)
    i = i + 1

if i == 10:
    VERDICT = FAILURE

OUTPUT = output
