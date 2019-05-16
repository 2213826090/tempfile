# @PydevCodeAnalysisIgnore
# pylint: disable=E0602,W0212
"""
Available globals:

DEVICE          => ACS main device instance
DEVICE_MANAGER  => ACS device manager instance
DEVICE_LOGGER   => ACS device logger instance
DEVICE.run_cmd  => if you do adb shell cmd, please do not add the \"
                    e.g: "adb shell echo hello" instead of "adb shell \"echo hello\""

EXECUTION_CONFIG_PATH   => absolute path to execution config folder
BENCH_CONFIG            => bench config dict
EQUIPMENT_MANAGER       => ACS equipment manager instance
IO_CARD                 => IOCard instance (usb relay or ariane board), check IIOCard interface
LOCAL_EXEC              => run local cmd on the bench (not on the DUT !)

CTX     => test step's context (same as "FROM_CTX" in xml test step file).
MY_PATH => absolute path to this exec script
EXEC_TS => this test step instance

ERROR_DEVICE    => raise a DeviceException. Usage: ERROR_DEVICE("My error message")
ERROR_ACSTOOL   => raise an AcsToolException. Usage: ERROR_ACSTOOL("My error message")
ERROR_ACSCONFIG => raise an AcsConfigException. Usage: ERROR_ACSCONFIG("My error message")
ERROR_EQUIPMENT => raise an EquipmentException. Usage: ERROR_EQUIPMENT("My error message")

PRINT_DEBUG     => log debug message in ACS log
PRINT_INFO      => log info message in ACS log
PRINT_WARNING   => log warning message in ACS log
PRINT_ERROR     => log error message in ACS log

REPORT_PATH             => path of the ACS report folder
ACS_REPORT_FILE_PATH    => path of the ACS report file .xml used by AWR
PATH_MANAGER            => Give access to all ACS paths (ex: PATH_MANAGER.FLASH_FILE)
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

FAILURE => Global.Failure
SUCCESS => Global.Success

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
exec_status, output = LOCAL_EXEC("pwd", 2)  # @UndefinedVariable

# context
my_value = CTX.get_info("KEY_FROM_XML")  # @UndefinedVariable
PRINT_INFO("Value from TestStep XML file : %s" % my_value)  # @UndefinedVariable

# ADB cmd
exec_status, output = DEVICE.run_cmd("adb shell echo hello world!", 3)  # @UndefinedVariable
if exec_status == SUCCESS:  # @UndefinedVariable
    PRINT_INFO("ECHO SUCCESS: %s" % output)  # @UndefinedVariable
    DEVICE.inject_device_log("i", "HELLO", "hello world!")  # @UndefinedVariable
elif exec_status == FAILURE:  # @UndefinedVariable
    PRINT_ERROR("ECHO FAILURE: %s" % output)  # @UndefinedVariable
    ERROR_DEVICE("Device command failed %s" % output)
else:
    PRINT_DEBUG("UNKNOWN STATUS")  # @UndefinedVariable
    ERROR_ACSTOOL("Device command failed with unknown status %s" % output)

# Get trigger log
time.sleep(5)
messages = DEVICE_LOGGER.get_message_triggered_status("hello world!")  # @UndefinedVariable
if messages:
    for message in messages:
        PRINT_DEBUG(message)  # @UndefinedVariable
else:
    ERROR_DEVICE("No message trigged by device")

# Clean
DEVICE_LOGGER.remove_trigger_message("hello world!")  # @UndefinedVariable

# Unplug
# DEVICE.disconnect_board()
# IO_CARD.usb_host_pc_connector(False)
# time.sleep(2)

# Switch off
# DEVICE.switch_off()
