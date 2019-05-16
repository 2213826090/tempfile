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
VERDICT = FAILURE

ltp_dir = os.path.join(os.getcwd(),"android-ltp")
if os.path.isdir(ltp_dir):
    #Change directory
    os.chdir(ltp_dir)
    #LTP RUN Tests
    PRINT_INFO("run LTP")
    OUTPUT = []
    (error_code, error_msg) = LOCAL_EXEC("./runandroidltp.sh", 7200)
    if not error_code == Global.SUCCESS :
        VERDICT = FAILURE
        PRINT_ERROR("LTP execution has not finished properly")
        OUTPUT.append("LTP execution error")

    #Retrieve output file
    file_name = ""
    for root_dir,sub_directories,files in os.walk("results"):
        for file_result in files:
            if file_result[-3:] == "csv":
                file_name = os.path.join(root_dir,file_result)
                break

    #Parse file to retrieve the verdict
    pass_counter = 0
    fail_counter = 0
    if file_name != "":
        f = open(file_name,"r")
        for line in f:
            if line.find(",FAIL") != -1:
                OUTPUT.append(line)
                fail_counter += 1
            elif line.find(",PASS") != -1:
                pass_counter += 1
        f.close()
        result_dir = os.path.join(os.getcwd(),"results")
        if os.path.isdir(result_dir):
            shutil.copytree("results",os.path.join(REPORT_PATH,"results"))
        else:
            VERDICT = FAILURE
            OUTPUT = "Can't retrieve results folder"

        if ((pass_counter > 0) & (fail_counter == 0)):
            VERDICT = SUCCESS
        OUTPUT.append("Global result : %s tests pass and %s tests fail" %(pass_counter,fail_counter))
    else:
        VERDICT = FAILURE
        OUTPUT = "Can't retrieve results file"

else:
    PRINT_ERROR("Can't find android-ltp folder")
    OUTPUT = "Can't run LTP tests"
    VERDICT = FAILURE
