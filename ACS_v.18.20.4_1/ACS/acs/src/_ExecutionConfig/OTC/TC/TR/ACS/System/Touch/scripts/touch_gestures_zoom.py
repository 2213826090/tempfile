from uiautomator_utils import *

test_url = "http://upload.wikimedia.org/wikipedia/commons/2/2c/Rotating_earth_%28large%29.gif"

VERDICT = SUCCESS


def zoom_in_out():
    global VERDICT
    Chrome.launch()
    Chrome.verify_open_tab_or_make_new_tab()
    Chrome.go_to_url(test_url)
    d(packageName=CHROME_PACKAGE_NAME, className=ANDROID_WIDGET_IMAGE).wait.exists(timeout=5000)
    d(packageName=CHROME_PACKAGE_NAME, className=ANDROID_WIDGET_IMAGE).pinch.Out(percent=500)
    ScreenSwiper.swipe_up()
    ScreenSwiper.swipe_up()
    if d(resourceId=CHROME_ADDRESS_BAR_RESID).exists:
        VERDICT = FAILURE  # zoom did not work
        print "FAILURE"
        return
    ScreenSwiper.swipe_down()
    ScreenSwiper.swipe_down()
    d(packageName=CHROME_PACKAGE_NAME, className=ANDROID_WIDGET_IMAGE).pinch.In(percent=500)
    ScreenSwiper.swipe_up()
    ScreenSwiper.swipe_up()
    if not d(resourceId=CHROME_ADDRESS_BAR_RESID).exists:
        VERDICT = FAILURE  # swiping did not work
        print "FAILURE"
        return
    print "SUCCESS"
    VERDICT = SUCCESS

try:
    zoom_in_out()
    Chrome.close_all_tabs_and_count()
except Exception as e:
    VERDICT = FAILURE
    print "Exception caught while executing test"
    print_exception()