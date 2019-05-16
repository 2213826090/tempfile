import subprocess
from log.logging_utils import Log
from adb_helper import *


class AdbUtils:
    def __init__(self):
        Log.debug("AdbUtils is online ...")

    @staticmethod
    def run_adb_cmd(cmd_string, adb_shell=True):
        cmd = AdbUtils.prepare_cmd(cmd_string, adb_shell)
        print cmd
        output = subprocess.check_output(cmd, shell=True)
        return output

    @staticmethod
    def prepare_cmd(cmd_string, adb_shell=True):
        if adb_shell:
            return ADB_SHELL_CMD_PREFIX + TICK + cmd_string + TICK
        else:
            return ADB_CMD_PREFIX + cmd_string

    @staticmethod
    def pull(file_path_on_dut, local_dir):
        cmd = ADB_PULL_CMD + file_path_on_dut + SPACE + local_dir
        Log.debug(cmd)
        output = subprocess.check_call(cmd, shell=True)
        Log.debug(output)

    @staticmethod
    def delete_file(dut_file_path):
        AdbUtils.run_adb_cmd(DEL_CMD + dut_file_path)

    @staticmethod
    def send_key_event(KEY_STRING):
        AdbUtils.run_adb_cmd(ADB_INPUT_KEYEVENT_CMD + KEY_STRING)