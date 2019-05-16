from uiautomator import device as dut
import subprocess

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
dut(scrollable=True).scroll.to(text="Clear credentials")
dut(text="Clear credentials").click()
dut(text="OK").click()
dut(scrollable=True).scroll.to(text="Screen lock")
dut(text="Screen lock").click()
dut(resourceId="com.android.settings:id/password_entry").set_text("1234")
dut(resourceId="com.android.settings:id/next_button").click()
dut(text="Swipe").click()

dut.orientation = orientation
dut.freeze_rotation(False)
global_values = globals()
global_values["VERDICT"] = 0
