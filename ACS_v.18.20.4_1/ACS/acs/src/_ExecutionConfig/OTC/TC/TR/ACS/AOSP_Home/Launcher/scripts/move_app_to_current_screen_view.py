from uiautomator import device as d
import acs_test_suites.OTC.TC.ACS.PyUIAutomator.applications as applications
import time


applications.open_all_apps()
d(scrollable=True).scroll.to(text="Settings")
sx = d.info[u'displaySizeDpX'] / 2
sy = d.info[u'displaySizeDpY'] / 2
d(text="Settings").drag.to(sx, sy, steps=50)
time.sleep(2)
if d(text="Settings").exists:
    VERDICT = SUCCESS
    OUTPUT = ""
else:
    VERDICT = FAILURE
    OUTPUT = "Could not verify that the application is on the home screen"