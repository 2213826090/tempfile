import os
import subprocess
from PyUiApi.log.logging_utils import *
from PyUiApi.adb_helper import *
from string import Template
from PyUiApi.multi_dut_support.dut_parameters import *
from PyUiApi.common.process import shell_command_ext
import time


class AdbUtils:
    verbose = True

    def __init__(self):
        print "AdbUtils is online ..."

    @staticmethod
    def disable_enable_adb_debugging(disable_enable_timeout=5000):
        cmd = Template("am start -a android.intent.action.MAIN -c android.intent.category.LAUNCHER "
                       "-e timeout $timeout -n "
                       "com.intel.test.systemapitests/com.intel.test.systemapitests.DisableEnableAdb")\
            .substitute(timeout=disable_enable_timeout)
        AdbUtils.run_adb_cmd(cmd)

    @staticmethod
    def start_activity_from_shell(activity_string):
        cmd = "am start -n $ACTIVITY$"
        AdbUtils.run_adb_cmd(cmd.replace("$ACTIVITY$", activity_string))

    @staticmethod
    def start_activity_with_package_name(package_name):
        cmd = "monkey -p $PACKAGE_NAME$ -c android.intent.category.LAUNCHER 1"
        AdbUtils.run_adb_cmd(cmd.replace("$PACKAGE_NAME$", package_name))

    @staticmethod
    def force_stop_app(app_string):
        cmd = "am force-stop $APP$"
        AdbUtils.run_adb_cmd(cmd.replace("$APP$", app_string))

    @staticmethod
    def run_adb_cmd(cmd_string, adb_shell=True, add_ticks=True, dut_serial=None, verbose=True):
        cmd = AdbUtils.prepare_cmd(cmd_string, adb_shell, add_ticks, dut_serial)
        if verbose and AdbUtils.verbose:
            print cmd
        try:
            output = subprocess.check_output(cmd, shell=True)
            return output
        except:
            return ""

    @staticmethod
    def run_adb_cmd_ext(cmd_string, adb_shell=True, add_ticks=True, dut_serial=None, verbose=True, timeout=None):
        cmd = AdbUtils.prepare_cmd(cmd_string, adb_shell, add_ticks, dut_serial)
        if verbose and AdbUtils.verbose:
            print cmd
        exit_code, stdout, stderr = shell_command_ext(cmd, timeout=timeout)
        output = stdout.strip('\r\n') + stderr.strip('\r\n')

        return exit_code, output

    @staticmethod
    def wait_for_adb(wait_poll_steps=15):
        for i in range(wait_poll_steps):
            adb_cmd_output = AdbUtils.run_adb_cmd('ls')
            if "error:" not in adb_cmd_output:  # if there is no error, adb is connected
                return True
            time.sleep(1)
        return False

    @staticmethod
    def get_adb_cmd_process(cmd_string, adb_shell=True, add_ticks=True, dut_serial=None):
        cmd = AdbUtils.prepare_cmd(cmd_string, adb_shell, add_ticks, dut_serial)
        print cmd
        try:
            proc = subprocess.Popen(cmd, shell=True)
            return proc
        except:
            return None

    @staticmethod
    def reboot_device():
        cmd = AdbUtils.prepare_cmd("reboot")
        subprocess.check_call(cmd, shell=True)
        # wait for dut to reboot
        time.sleep(5)
        poll_wait_times = 50
        adb_devices_cmd = "adb devices"
        reboot_time = 5
        for i in xrange(poll_wait_times):
            time.sleep(5)
            reboot_time += 5
            devices_connected = subprocess.check_output(adb_devices_cmd, shell=True)
            if dut_manager.active_uiautomator_device_serial in devices_connected:
                # the dut has rebooted
                LOG.info("dut rebooted successfully")
                break
        return reboot_time

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
        cmd = ADB_PULL_CMD % dut_manager.active_uiautomator_device_serial + file_path_on_dut + SPACE + local_dir
        print "cmd: ", cmd
        output = subprocess.check_call(cmd, shell=True)
        print "output: ", output

    @staticmethod
    def push(file_path_on_host, file_path_on_dut):
        cmd = ADB_PUSH_CMD % dut_manager.active_uiautomator_device_serial + file_path_on_host + SPACE + file_path_on_dut
        print "cmd: ", cmd
        output = subprocess.check_call(cmd, shell=True)
        print "output: ", output

    @staticmethod
    def delete_files(*dut_file_paths):
        for dut_file_path in dut_file_paths:
            AdbUtils.run_adb_cmd(DEL_CMD + dut_file_path)

    @staticmethod
    def copy_file(source_path, destination_path):
        AdbUtils.run_adb_cmd(CP_CMD + source_path + SPACE + destination_path)

    @staticmethod
    def send_key_event(KEY_STRING):
        AdbUtils.run_adb_cmd(ADB_INPUT_KEYEVENT_CMD + KEY_STRING)

    @staticmethod
    def get_adb_devices_serials():
        output = AdbUtils.run_adb_cmd("devices", adb_shell=False)
        serials = []
        for line in output.split('\n'):
            if 'device' in line and 'attached' not in line:
                serials.append(line.split()[0])
        return serials

    @staticmethod
    # this creates a file on the device and the parent folders if they don't exist
    def create_file(file_name, create_dir_on_dut, file_content):
        AdbUtils.run_adb_cmd(MKDIRS_CMD + create_dir_on_dut)
        AdbUtils.run_adb_cmd(CREATE_FILE_CMD + create_dir_on_dut + file_name)
        AdbUtils.run_adb_cmd(PRINT_INTO_FILE_CMD + QUOTE + file_content + QUOTE +
                             SPACE + REDIRECT_SYMBOL + SPACE + create_dir_on_dut + file_name)

    @staticmethod
    def create_dir(dir_path_on_dut):
        AdbUtils.run_adb_cmd(MKDIRS_CMD + dir_path_on_dut)

    @staticmethod
    def ls_dir(dir_path_on_dut):
        return AdbUtils.run_adb_cmd(LS_CMD + dir_path_on_dut)

    @staticmethod
    def cat_file(file_path_on_dut):
        return AdbUtils.run_adb_cmd(CAT_CMD + file_path_on_dut)

    @staticmethod
    def rename(old_absolute_path, new_absolute_path):
        AdbUtils.run_adb_cmd(RENAME_CMD + old_absolute_path + SPACE + new_absolute_path)

    @staticmethod
    def copy(source_absolute_path, destination_absolute_path):
        destination_dir = os.path.dirname(destination_absolute_path)
        AdbUtils.create_dir(destination_dir)
        AdbUtils.run_adb_cmd(COPY_R_CMD + source_absolute_path + SPACE + destination_absolute_path)

    @staticmethod
    def tap(x_coordinate, y_coordinate):
        print "taping: " + str(x_coordinate) + " " + str(y_coordinate)
        cmd = ADB_INPUT_TAP_CMD + str(x_coordinate) + SPACE + str(y_coordinate)
        AdbUtils.run_adb_cmd(cmd)

    @staticmethod
    def long_tap(x_coordinate, y_coordinate, press_time=2500):
        print "long taping: " + str(x_coordinate) + " " + str(y_coordinate)
        cmd = ADB_INPUT_LONG_TAP_CMD + str(x_coordinate) + SPACE + str(y_coordinate) +\
            SPACE + str(x_coordinate) + SPACE + str(y_coordinate) + SPACE + str(press_time)
        AdbUtils.run_adb_cmd(cmd)

    @staticmethod
    def input_text(text):
        AdbUtils.run_adb_cmd(ADB_INPUT_TEXT_CMD + text)

    @staticmethod
    def grant_permission(package_name, permission):
        cmd = "pm grant %s %s" % (str(package_name), str(permission))
        AdbUtils.run_adb_cmd(cmd)

    @staticmethod
    def kill_python_uiautomator_rpc_server_on_dut():
        for i in range(20):
            time.sleep(1)
            AdbUtils.run_adb_cmd("pkill uiautomator")
            if "uiautomator" not in AdbUtils.run_adb_cmd("ps | grep uiautomator",add_ticks=False):
                return True
        return False

    @staticmethod
    def wait_for_main_ui_services():
        ui_services_strings = ["android.ui.ISurfaceComposer", "android.view.IWindowManager",
                               "android.app.IWallpaperManager", "com.android.internal.appwidget.IAppWidgetService",
                               "android.service.dreams.IDreamManager", "android.content.pm.ILauncherApps"]
        service_list_cmd = "service list"
        for i in range(10):
            time.sleep(8)
            started_services = AdbUtils.run_adb_cmd(service_list_cmd)
            ui_services_started = True
            for service_string in ui_services_strings:
                if service_string not in started_services:
                    LOG.info("service %s not started yet" % service_string)
                    ui_services_started = False
            if ui_services_started:
                return
