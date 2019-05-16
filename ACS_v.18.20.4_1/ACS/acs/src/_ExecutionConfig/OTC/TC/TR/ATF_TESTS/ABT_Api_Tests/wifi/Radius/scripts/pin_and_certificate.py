from uiautomator import device as dut
import subprocess
import time

orientation = dut.orientation
dut.orientation = "natural"
dut.freeze_rotation()

intent = "com.android.settings/.Settings"
_cmd = "adb shell am start -n {0}".format(intent)
process = subprocess.Popen(_cmd, shell=True)
process.wait()
package_name = intent.split("/")[0]
assert dut(packageName=package_name).exists is True
dut(text="Security").click()
dut(text="Screen lock").click()
dut(text="PIN").click()
dut(text="Continue").click()
dut(resourceId="com.android.settings:id/password_entry").set_text("1234")
dut(resourceId="com.android.settings:id/next_button").click()
dut(resourceId="com.android.settings:id/password_entry").set_text("1234")
dut(resourceId="com.android.settings:id/next_button").click()
dut(resourceId="com.android.settings:id/next_button").click()


dut(scrollable=True).scroll.to(text="Install from SD card")
dut(text="Install from SD card").click()
dut(text="Internal storage").click()
dut(scrollable=True).scroll.to(text="client.p12")
dut(text="client.p12").click()
dut(resourceId="com.android.certinstaller:id/credential_password").set_text("whatever")
dut(text="OK").click()
time.sleep(2)
dut(text="VPN and apps").click()
dut(text="Wi-Fi").click()
dut(text="OK").click()

dut.orientation = orientation
dut.freeze_rotation(False)
global_values = globals()
global_values["VERDICT"] = 0
