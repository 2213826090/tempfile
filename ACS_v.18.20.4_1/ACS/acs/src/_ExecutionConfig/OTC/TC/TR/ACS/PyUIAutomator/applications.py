import subprocess
from uiautomator import device as android_device
import time


def start_application(intent):
    _cmd = "adb shell am start -n {0}".format(intent)
    process = subprocess.Popen(_cmd, shell=True)
    process.wait()
    package_name = intent.split("/")[0]
    assert android_device(packageName=package_name).exists is True


def close_focus_application(package_name):
    _cmd = "adb shell am force-stop {0}".format(package_name)
    if android_device(packageName=package_name).exists:
        process = subprocess.Popen(_cmd, shell=True)
        process.wait()
    else:
        raise Exception("Application is not in focus")


def close_application(package_name):
    _cmd = "adb shell am force-stop {0}".format(package_name)
    process = subprocess.Popen(_cmd, shell=True)
    process.wait()


def go_to_home():
    android_device.press.home()
    assert android_device(resourceId="com.android.launcher:id/qsb_search_bar",
                          packageName="com.android.launcher").exists is True


def open_all_apps():
    android_device(description="Apps").click()
    if android_device(resourceId="com.android.launcher:id/apps_customize_content").exists:
        pass
    else:
        android_device(text="Apps").click()
    assert android_device(resourceId="com.android.launcher:id/apps_customize_pane_content").exists is True


def open_widgets():
    open_all_apps()
    android_device(text="Widgets").click()
    assert android_device(className="android.widget.TabWidget").exists is True


def uninstall_app(app_name):
    start_application("com.android.settings/com.android.settings.Settings")
    android_device(scrollable=True).scroll.to(text=app_name)
    android_device(text="Apps").click()
    time.sleep(1)
    android_device(text="Chess Chess").click()
    time.sleep(1)
    android_device(text="Uninstall").click()
    time.sleep(1)
    android_device(resourceId="com.android.packageinstaller:id/ok_button").click()


def move_to_dockbar(app_name):
    go_to_home()
    go_to_home()
    open_all_apps()
    android_device(scrollable=True).scroll.to(text=app_name)
    sx = android_device.info[u'displaySizeDpX'] / 3
    sy = android_device.info[u'displaySizeDpY'] - 30
    android_device(text=app_name).drag.to(sx, sy, steps=50)