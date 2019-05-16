from PyUiApi.adb_helper.adb_utils import AdbUtils


class AndroidVersion(object):
    def __init__(self):
        android_version_prop = AdbUtils.run_adb_cmd("getprop ro.build.version.release")
        self.major_version_nr = ""
        self.version_name_initial = ""
        if "." in android_version_prop:
            self.major_version_nr = android_version_prop.split(".")[0]
        elif "M" in android_version_prop:
            self.version_name_initial = "M"
        elif "N" in android_version_prop:
            self.version_name_initial = "N"
