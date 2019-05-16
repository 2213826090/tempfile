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

import time

iteration = 10
cpt = 0

if IO_CARD is None:
    VERDICT = FAILURE
    OUTPUT = "Cannot do that test without relay card."

VERDICT = SUCCESS
OUTPUT = "SUCCESS"

while cpt < iteration:
    # Unplug
    DEVICE.disconnect_board()
    IO_CARD.usb_host_pc_connector(False)
    time.sleep(2)

    # Plug
    IO_CARD.usb_host_pc_connector(True)
    DEVICE.connect_board()
    time.sleep(2)

    device_state = DEVICE.get_state()
    if device_state != "alive":
        VERDICT = FAILURE
        OUTPUT = "Device state not alive: %s" % device_state
        break

    cpt += 1
