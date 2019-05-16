from uiautomator import device as d
import acs_test_suites.OTC.TC.ACS.PyUIAutomator.applications as applications
import time


applications.open_all_apps()
d(scrollable=True).scroll.to(text="Chess Chess")
sx = d.info[u'displaySizeDpX'] / 3
sy = d.info[u'displaySizeDpY'] - 30
d(text="Chess Chess").drag.to(sx, sy, steps=50)
applications.uninstall_app("Chess Chess")
time.sleep(2)
applications.go_to_home()
if not d(resourceId="com.android.launcher:id/hotseat").child(index=0).child(index=0).child(text="Chess Chess").exists:
    VERDICT = SUCCESS
    OUTPUT = ""
else:
    VERDICT = FAILURE
    OUTPUT = "Application shortcut was not removed from dock"