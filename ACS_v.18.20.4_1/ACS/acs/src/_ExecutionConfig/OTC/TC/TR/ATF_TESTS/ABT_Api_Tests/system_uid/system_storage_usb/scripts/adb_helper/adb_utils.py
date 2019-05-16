import subprocess
from log.logging_utils import Log
from adb_helper import *
import os.path
import sys

if os.getenv('ACS_EXECUTION_CONFIG_PATH') is not None:
    sys.path.insert(0, (os.getenv('ACS_EXECUTION_CONFIG_PATH') + '/TC/TP'))
else:
    sys.path.insert(0,(os.getcwd() + '/_ExecutionConfig/OTC/TC/TP'))
from testlib.androidframework.dut_manager import dut_manager

class AdbUtils:
    verbose = True

    def __init__(self):
        Log.debug("AdbUtils is online ...")

    @staticmethod
    def run_adb_cmd(cmd_string, adb_shell=True, add_ticks=True, dut_serial=None,
                    verbose=True):
        cmd = AdbUtils.prepare_cmd(cmd_string, adb_shell, add_ticks, dut_serial)
        if verbose and AdbUtils.verbose:
            print cmd
        try:
            output = subprocess.check_output(cmd, shell=True)
            return output
        except:
            return ""

    @staticmethod
    def prepare_cmd(cmd_string, adb_shell=True, add_ticks=True, dut_serial=None):
        adb_cmd_serial = dut_manager.active_uiautomator_device_serial
        if dut_serial is not None:
            adb_cmd_serial = dut_serial
        dut_cmd = cmd_string
        if add_ticks:
            dut_cmd = TICK + cmd_string + TICK
        if adb_shell:
            return ADB_SHELL_CMD_PREFIX % adb_cmd_serial + dut_cmd
        else:
            return ADB_CMD_PREFIX % adb_cmd_serial + cmd_string

    @staticmethod
    def pull(file_path_on_dut, local_dir):
        cmd = ADB_PULL_CMD + file_path_on_dut + SPACE + local_dir
        Log.debug(cmd)
        output = subprocess.check_call(cmd, shell=True)
        Log.debug(output)

    @staticmethod
    def push(file_path_on_dut, local_dir):
        cmd = ADB_PUSH_CMD + file_path_on_dut + SPACE + local_dir
        Log.debug(cmd)
        output = subprocess.check_call(cmd, shell=True)
        Log.debug(output)

    @staticmethod
    def delete_file(dut_file_path):
        AdbUtils.run_adb_cmd(DEL_CMD + dut_file_path)

    @staticmethod
    def send_key_event(KEY_STRING):
        AdbUtils.run_adb_cmd(ADB_INPUT_KEYEVENT_CMD + KEY_STRING)