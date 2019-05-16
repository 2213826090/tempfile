import time

high_accuracy = "gps,network"
gps_chipset = TC_PARAMETERS("GPS_CHIPSET")
exec_status, output = DEVICE.run_cmd("adb shell ls -l /sys/class/rfkill | grep {}".format(gps_chipset), 10)
if gps_chipset in output:
    VERDICT = SUCCESS
    test_output = "No errors"
else:
    VERDICT = FAILURE
    test_output = "GPS chipset doesn't match the one provided in the test case"
rfkill = output.split("/")[-1].rstrip("\r\n")
cmd = "adb shell cat /sys/class/rfkill/{}/state".format(rfkill)
exec_status, output = DEVICE.run_cmd(cmd, 10)
if output[0] is not '0':
    VERDICT = FAILURE
    test_output = "GPS is in high accuracy mode"
exec_status, output = DEVICE.run_cmd("adb shell sqlite3 "
                                         "data/data/com.android.providers.settings/databases/settings.db "
                                         "\"update \'secure\' set value = \'gps,network\' "
                                         "where name = \'location_providers_allowed\'\"", 10)
start_settings = "adb shell am start com.android.settings/com.android.settings.Settings"
exec_status, output = DEVICE.run_cmd(start_settings, 10)
time.sleep(5)
exec_status, output = DEVICE.run_cmd("adb shell am force-stop com.android.settings", 10)
start_maps = "adb shell am start com.google.android.apps.maps/com.google.android.maps.MapsActivity"
exec_status, output = DEVICE.run_cmd(start_maps, 10)
time.sleep(2)
exec_status, output = DEVICE.run_cmd(cmd, 10)
if output[0] is "1":
    VERDICT = SUCCESS
    test_output = "No errors"
else:
    VERDICT = FAILURE
    test_output = "GPS is not in High Accuracy mode"
exec_status, output = DEVICE.run_cmd("adb shell am force-stop com.google.android.apps.maps", 10)
time.sleep(2)
exec_status, output = DEVICE.run_cmd(cmd, 10)
if output[0] is "0":
    VERDICT = SUCCESS
    test_output = "No errors"
else:
    VERDICT = FAILURE
    test_output = "GPS is in High Accuracy mode"
OUTPUT = test_output