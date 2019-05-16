from uiautomator import device as d
import time
import acs_test_suites.OTC.TC.ACS.PyUIAutomator.applications as applications
import acs_test_suites.OTC.TC.ACS.PyUIAutomator.utilities as utilities

condition1 = False
condition2 = False
condition3 = False
applications.move_to_dockbar("Settings")
applications.move_to_dockbar("Gallery")
applications.move_to_dockbar("Camera")
sx = d.info[u'displaySizeDpX'] / 2
d(resourceId="com.android.launcher:id/hotseat").child(index=0).child(index=0).child(text="Settings").drag.to(sx,
                                                                                                             50,
                                                                                                             steps=50)
time.sleep(1)
if not d(resourceId="com.android.launcher:id/hotseat").child(index=0).child(index=0).child(text="Settings").exists:
    condition1 = True
sx = d.info[u'displaySizeDpX'] - d.info[u'displaySizeDpX'] / 4
sy = d.info[u'displaySizeDpY'] - 30
d(resourceId="com.android.launcher:id/hotseat").child(index=0).child(index=0).child(text="Gallery").drag.to(sx,
                                                                                                            sy,
                                                                                                            steps=50)
if d(resourceId="com.android.launcher:id/layout").child(index=0).child(text="Gallery").exists:
    condition2 = True
sx = d.info[u'displaySizeDpX'] / 2
sy = d.info[u'displaySizeDpY'] / 2
d(resourceId="com.android.launcher:id/hotseat").child(index=0).child(index=0).child(text="Camera").drag.to(sx,
                                                                                                           sy,
                                                                                                           steps=50)
if d(resourceId="com.android.launcher:id/cell3").child(index=0).child(text="Camera").exists:
    condition3 = True
if condition1 and condition2 and condition3:
    VERDICT = SUCCESS
    OUTPUT = ""
else:
    VERDICT = FAILURE
    OUTPUT = "Could not change shortcuts from dock bar"