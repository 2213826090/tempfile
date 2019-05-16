from uiautomator import device as d
import time
import acs_test_suites.OTC.TC.ACS.PyUIAutomator.applications as applications
import acs_test_suites.OTC.TC.ACS.PyUIAutomator.utilities as utilities

applications.open_all_apps()
d(scrollable=True).scroll.to(text="Settings")
sx = d.info[u'displaySizeDpX'] / 3
sy = d.info[u'displaySizeDpY'] - 30
d(text="Settings").drag.to(sx, sy, steps=50)
applications.open_all_apps()
d(scrollable=True).scroll.to(text="Gallery")
_sx = d.info[u'displaySizeDpX'] / 2
_sy = d.info[u'displaySizeDpY'] / 2
d(text="Gallery").drag.to(_sx, _sy, steps=50)
time.sleep(2)
d(text="Gallery").drag.to(sx, sy, steps=50)

if d(resourceId="com.android.launcher:id/hotseat").child(index=0).child(index=0).child(text="Settings").exists\
        and d(resourceId="com.android.launcher:id/hotseat").child(index=0).child(index=0).child(text="Gallery").exists:
    VERDICT = SUCCESS
    OUTPUT = ""
else:
    VERDICT = FAILURE
    OUTPUT = "Could not add shortcut from all apps"