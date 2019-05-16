from uiautomator import device as d
import acs_test_suites.OTC.TC.ACS.PyUIAutomator.applications as applications


d.press.home()
if not d(resourceId="com.android.launcher:id/qsb_search_bar", packageName="com.android.launcher").exists:
    VERDICT = FAILURE
else:
    VERDICT = SUCCESS
applications.open_all_apps()
d.press.home()
if not d(resourceId="com.android.launcher:id/qsb_search_bar", packageName="com.android.launcher").exists:
    VERDICT = FAILURE
else:
    VERDICT = SUCCESS