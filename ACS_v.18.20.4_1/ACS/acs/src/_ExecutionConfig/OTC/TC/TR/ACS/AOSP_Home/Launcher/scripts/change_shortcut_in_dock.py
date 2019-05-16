from uiautomator import device as d
import time
import acs_test_suites.OTC.TC.ACS.PyUIAutomator.applications as applications
import acs_test_suites.OTC.TC.ACS.PyUIAutomator.utilities as utilities

applications.go_to_home()
applications.go_to_home()
applications.open_all_apps()
d(scrollable=True).scroll.to(text="Settings")
sx = d.info[u'displaySizeDpX'] / 3
sy = d.info[u'displaySizeDpY'] - 30
d(text="Settings").drag.to(sx, sy, steps=50)
assert d(resourceId="com.android.launcher:id/hotseat").\
       child(index=0).child(index=0).\
       child(text="Settings").exists is True
sx = d.info[u'displaySizeDpX'] / 2
sy = d.info[u'displaySizeDpY'] / 2
d(resourceId="com.android.launcher:id/hotseat").child(index=0).child(index=0).child(text="Settings").drag.to(sx,
                                                                                                             sy,
                                                                                                             steps=50)
if d(resourceId="com.android.launcher:id/cell3").child(index=0).child(text="Settings").exists:
    VERDICT = SUCCESS
    OUTPUT = ""
else:
    VERDICT = FAILURE
    OUTPUT = "Could not move shortcut from dock to home screen"