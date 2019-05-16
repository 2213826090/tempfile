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

VERDICT => verdict of the test, SUCCESS or FAILURE
OUTPUT  => message that will be displayed in ACS report
           (mostly used in case of error)
"""
import os
import shutil
VERDICT = SUCCESS

old_ltp_dir = os.path.join(os.getcwd(),"android-ltp")
if os.path.isdir(old_ltp_dir):
    #remove old android-ltp folder
    PRINT_INFO("removing old android-ltp folder")
    shutil.rmtree(old_ltp_dir)

PRINT_INFO("Cloning LTP repository")
(error_code, error_msg) = LOCAL_EXEC("git clone ssh://git@jfumg-git3.jf.intel.com/android-ltp", 900)
if not error_code == Global.SUCCESS :
    VERDICT = FAILURE
    OUTPUT = "Can't get LTP repository: FAILURE"
else:
    ltp_dir = os.path.join(os.getcwd(),"android-ltp")
    if os.path.isdir(ltp_dir):
        #Change directory
        os.chdir(ltp_dir)
        LOCAL_EXEC("adb root",10)
    else:
        PRINT_ERROR("Can't find android-ltp folder")
        OUTPUT = "LTP setup KO"
        VERDICT = FAILURE
