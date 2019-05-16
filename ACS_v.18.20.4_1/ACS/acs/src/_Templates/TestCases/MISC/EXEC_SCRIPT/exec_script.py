# @PydevCodeAnalysisIgnore
# pylint: disable=E0602,W0212
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

# Switch on
# DEVICE.switch_on()

# Plug
# IO_CARD.usb_host_pc_connector(True)
# DEVICE.connect_board()
# time.sleep(2)

# Press power button
# IO_CARD.press_power_button(1)
# time.sleep(2)

# Set trigger log
DEVICE_LOGGER.add_trigger_message("hello world!")  # @UndefinedVariable

# Local cmd
exec_status, output = LOCAL_EXEC("ls", 2)  # @UndefinedVariable

# Optional parameter
param_1 = TC_PARAMETERS("OPTIONAL_PARAMETER_1")  # @UndefinedVariable
PRINT_INFO("Param 1 extracted from TC XML file : %s" % param_1)  # @UndefinedVariable

# ADB cmd
exec_status, output = DEVICE.run_cmd("adb shell echo hello world!", 3)  # @UndefinedVariable
if exec_status == SUCCESS:  # @UndefinedVariable
    VERDICT = SUCCESS  # @UndefinedVariable
    PRINT_INFO("ECHO SUCCESS: %s" % output)  # @UndefinedVariable
    DEVICE.inject_device_log("i", "HELLO", "hello world!")  # @UndefinedVariable
elif exec_status == FAILURE:  # @UndefinedVariable
    VERDICT = FAILURE  # @UndefinedVariable
    PRINT_ERROR("ECHO FAILURE: %s" % output)  # @UndefinedVariable
else:
    PRINT_DEBUG("UNKNOWN STATUS")  # @UndefinedVariable
    VERDICT = FAILURE  # @UndefinedVariable
OUTPUT = output

# Get trigger log
time.sleep(5)
messages = DEVICE_LOGGER.get_message_triggered_status("hello world!")  # @UndefinedVariable
if len(messages) > 0:
    for message in messages:
        PRINT_DEBUG(message)  # @UndefinedVariable
    VERDICT = SUCCESS  # @UndefinedVariable
else:
    VERDICT = FAILURE  # @UndefinedVariable

# Clean
DEVICE_LOGGER.remove_trigger_message("hello world!")  # @UndefinedVariable

# Unplug
# DEVICE.disconnect_board()
# IO_CARD.usb_host_pc_connector(False)
# time.sleep(2)

# Switch off
# DEVICE.switch_off()
