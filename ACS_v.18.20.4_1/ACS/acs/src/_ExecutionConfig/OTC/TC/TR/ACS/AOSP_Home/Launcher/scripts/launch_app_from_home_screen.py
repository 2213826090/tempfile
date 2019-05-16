from uiautomator import device as d
import acs_test_suites.OTC.TC.ACS.PyUIAutomator.applications as applications
import time

# Drag application to home screen
applications.open_all_apps()
d(scrollable=True).scroll.to(text="Settings")
sx = d.info[u'displaySizeDpX'] / 2
sy = d.info[u'displaySizeDpY'] / 2

# Launch application from home screen
d(text="Settings").drag.to(sx, sy, steps=50)
applications.go_to_home()
d(text="CSettings").click()
time.sleep(2)
if d(packageName="com.android.settings").exists:
    VERDICT = SUCCESS
    OUTPUT = ""
else:
    VERDICT = FAILURE
    OUTPUT = "Could not verify that the application is on the home screen"