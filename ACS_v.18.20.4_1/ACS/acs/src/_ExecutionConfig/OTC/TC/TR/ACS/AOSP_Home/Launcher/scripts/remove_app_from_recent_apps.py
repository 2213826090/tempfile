from uiautomator import device as d
import time
import acs_test_suites.OTC.TC.ACS.PyUIAutomator.applications as applications


applications.go_to_home()
applications.open_all_apps()
d(scrollable=True).text("Settings")
d(text="Settings").click()
applications.go_to_home()
d.press.recent()
time.sleep(2)
if d.info[u'displayWidth'] < d.info[u'displayHeight']:
    d(resourceId="com.android.systemui:id/app_thumbnail_image").swipe.left()
else:
    d(resourceId="com.android.systemui:id/app_thumbnail_image").swipe.down()
if not d(resourceId="com.android.systemui:id/app_thumbnail_image").exists:
    VERDICT = SUCCESS
else:
    VERDICT = FAILURE
    OUTPUT = "Application was not closed"