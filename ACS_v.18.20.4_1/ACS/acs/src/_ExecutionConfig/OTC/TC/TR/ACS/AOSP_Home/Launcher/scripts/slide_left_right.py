from uiautomator import device as d
import time
import acs_test_suites.OTC.TC.ACS.PyUIAutomator.applications as applications
import acs_test_suites.OTC.TC.ACS.PyUIAutomator.utilities as utilities


applications.go_to_home()
applications.go_to_home()
utilities.swipe_left()
time.sleep(2)
if d(resourceId="com.android.launcher:id/cell1").exists and d(resourceId="com.android.launcher:id/cell3").exists:
    VERDICT = SUCCESS
else:
    VERDICT = FAILURE
    OUTPUT = "Not on the left Home Screen"
applications.go_to_home()
utilities.swipe_right()
time.sleep(2)
if d(resourceId="com.android.launcher:id/cell3").exists and d(resourceId="com.android.launcher:id/cell5").exists:
    VERDICT = SUCCESS
else:
    VERDICT = FAILURE
    OUTPUT = "Not on the right Home Screen"