from uiautomator_utils import *

test_url = "http://ro.wikipedia.org/wiki/B%C4%83icoi"

VERDICT = SUCCESS


def swipe_left_right():
    global VERDICT
    UiAutomatorUtils.go_to_apps_beginning()
    shortcuts = d(className=ANDROID_WIDGET_TEXT_VIEW)
    fifth_shortcut = shortcuts[6]  # the first shortcuts are the name of the tabs (Apps, Widgets, Shop)
    reference_text = fifth_shortcut.info["text"]
    ScreenSwiper.swipe_left()
    if d(text=reference_text).exists:
        VERDICT = FAILURE  # the swiping did not succeed
        print "FAILURE1"
        return False
    ScreenSwiper.swipe_right()
    if not d(text=reference_text).exists:
        VERDICT = FAILURE  # the swiping back did not succeed
        print "FAILURE2"
        return False
    VERDICT = SUCCESS
    return True


def swipe_up_down():
    global VERDICT
    Chrome.launch()
    Chrome.verify_open_tab_or_make_new_tab()
    Chrome.go_to_url(test_url)
    ScreenSwiper.swipe_up(horizontal_offset=0.9)
    ScreenSwiper.swipe_up(horizontal_offset=0.9)
    if d(resourceId=CHROME_ADDRESS_BAR_RESID).exists:
        VERDICT = FAILURE  # swiping did not work
        print "FAILURE3"
        return
    ScreenSwiper.swipe_down(horizontal_offset=0.9)
    ScreenSwiper.swipe_down(horizontal_offset=0.9)
    if not d(resourceId=CHROME_ADDRESS_BAR_RESID).exists:
        VERDICT = FAILURE  # swiping did not work
        print "FAILURE4"
        return
    print "SUCCESS"
    VERDICT = SUCCESS


def touch_gestures_swipe():
    if swipe_left_right():
        swipe_up_down()


try:
    touch_gestures_swipe()
    Chrome.close_all_tabs_and_count()
except Exception as e:
    VERDICT = FAILURE
    print "Exception caught while executing test"
    print_exception()