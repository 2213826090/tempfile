adb push bundle.jar /data/local/tmp
adb push uiautomator-stub.jar /data/local/tmp
adb shell uiautomator runtest bundle.jar uiautomator-stub.jar -c com.github.uiautomatorstub.Stub
