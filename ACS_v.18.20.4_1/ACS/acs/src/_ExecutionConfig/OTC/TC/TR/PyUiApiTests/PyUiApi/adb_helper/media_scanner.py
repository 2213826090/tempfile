from PyUiApi.adb_helper.adb_utils import *
import time


class MediaScanner(object):
    single_file_scan_cmd = "am broadcast -a android.intent.action.MEDIA_SCANNER_SCAN_FILE -d file://$FILE_PATH$"

    @staticmethod
    def scan_single_file(file_path):
        cmd = MediaScanner.single_file_scan_cmd.replace("$FILE_PATH$", file_path)
        AdbUtils.run_adb_cmd(cmd)
        time.sleep(2)