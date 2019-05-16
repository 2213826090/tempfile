from uiautomator import device as d
import time
import acs_test_suites.OTC.TC.ACS.PyUIAutomator.applications as applications
import acs_test_suites.OTC.TC.ACS.PyUIAutomator.utilities as utilities

applications.open_all_apps()
d(scrollable=True).scroll.to(text="Settings")
sx = d.info[u'displaySizeDpX'] / 3
sy = d.info[u'displaySizeDpY'] - 30
d(text="Settings").drag.to(sx, sy, steps=50)
d(resourceId="com.android.launcher:id/hotseat").child(index=0).child(index=0).child(text="Settings").click()

if d(packageName="com.android.settings").exists:
    VERDICT = SUCCESS
    OUTPUT = ""
else:
    VERDICT = FAILURE
    OUTPUT = "Could not launch app from dock"