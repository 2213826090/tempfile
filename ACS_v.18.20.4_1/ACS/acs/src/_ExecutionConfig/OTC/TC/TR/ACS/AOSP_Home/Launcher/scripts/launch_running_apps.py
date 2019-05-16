from uiautomator import device as d
import time
import acs_test_suites.OTC.TC.ACS.PyUIAutomator.applications as applications

applications.start_application("com.google.android.gallery3d/com.android.gallery3d.app.GalleryActivity")
time.sleep(2)
applications.start_application("com.google.android.apps.genie.geniewidget/com.google.android.apps.genie."
                               "geniewidget.activities.NewsActivity")
time.sleep(2)
applications.start_application("com.android.music/com.android.music.ArtistAlbumBrowserActivity")
time.sleep(2)
applications.go_to_home()
d.press.recent()
time.sleep(2)
gallery_exists = d(resourceId="com.android.systemui:id/app_label", text="Gallery").exists
music_exists = d(resourceId="com.android.systemui:id/app_label", text="Music").exists
news_exists = d(resourceId="com.android.systemui:id/app_label", text="News & Weather").exists

if gallery_exists and music_exists and news_exists:
    VERDICT = SUCCESS
else:
    VERDICT = FAILURE
    OUTPUT = "Started apps are missing"
applications.go_to_home()
