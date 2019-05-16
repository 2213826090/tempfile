from testlib.util.common import g_common_obj
from testlib.util.log import Logger
import subprocess,time

LOG = Logger.getlogger(__name__)

#adb strings
ADB_CMD_PREFIX = "adb "
ADB_SHELL_CMD_PREFIX = "adb -s {0} shell "
ADB_PULL_CMD = "adb pull "
ADB_INPUT_KEYEVENT_CMD = " input keyevent "

DUT_SDCARD_DIR = "/sdcard/"
DEL_CMD = "rm -rf "
TICK = "'"
SPACE = " "
ADB_INPUT_TEXT_CMD = " input text "
from testlib.androidframework.dut_manager import dut_manager
d = g_common_obj.get_device(dut_manager.active_uiautomator_device_serial)

class AdbUtils(object):

    verbose = True
    def __init__(self):
        LOG.debug("AdbUtils is online ...")

    @staticmethod
    def run_adb_cmd(cmd_string, adb_shell=True):
        cmd = AdbUtils.prepare_cmd(cmd_string, adb_shell)
        # Log.debug(cmd)
        output = subprocess.check_output(cmd, shell=True)
        # Log.debug(output)
        return output

    @staticmethod
    def _run_adb_cmd(cmd_string, adb_shell=True, add_ticks=True, dut_serial=None, verbose=True):
        cmd = AdbUtils._prepare_cmd(cmd_string, adb_shell, add_ticks, dut_serial)
        if verbose and AdbUtils.verbose:
            print cmd
        try:
            output = subprocess.check_output(cmd, shell=True)
            return output
        except:
            return ""

    @staticmethod
    def prepare_cmd(cmd_string, adb_shell=True):
        if adb_shell:
            return ADB_SHELL_CMD_PREFIX.format(dut_manager.active_uiautomator_device_serial) + TICK + cmd_string + TICK
        else:
            return ADB_CMD_PREFIX + cmd_string

    @staticmethod
    def _prepare_cmd(cmd_string, adb_shell=True, add_ticks=True, dut_serial=None):
#         adb_cmd_serial = DutManager().active_uiautomator_device_serial
#         if dut_serial is not None:
#             adb_cmd_serial = dut_serial
        dut_cmd = cmd_string
        if add_ticks:
            dut_cmd = TICK + cmd_string + TICK
        if adb_shell:
            return ADB_SHELL_CMD_PREFIX + dut_cmd
        else:
            return ADB_CMD_PREFIX + cmd_string

    @staticmethod
    def tap(x_coordinate, y_coordinate):
        print "taping: " + str(x_coordinate) + " " + str(y_coordinate)
        cmd = "input tap " + str(x_coordinate) + SPACE + str(y_coordinate)
        AdbUtils.run_adb_cmd(cmd)

    @staticmethod
    def input_text(text):
        AdbUtils.run_adb_cmd(ADB_INPUT_TEXT_CMD + text)

    @staticmethod
    def force_stop_app(app_string):
        cmd = "am force-stop $APP$"
        AdbUtils.run_adb_cmd(cmd.replace("$APP$", app_string))

    @staticmethod
    def start_activity_from_shell(activity_string):
        cmd = "am start -n $ACTIVITY$"
        AdbUtils.run_adb_cmd(cmd.replace("$ACTIVITY$", activity_string))

    @staticmethod
    def pull(file_path_on_dut, local_dir):
        cmd = ADB_PULL_CMD + file_path_on_dut + SPACE + local_dir
        LOG.debug(cmd)
        output = subprocess.check_call(cmd, shell=True)
        LOG.debug(output)

    @staticmethod
    def get_adb_cmd_process(cmd_string, adb_shell=True, add_ticks=True, dut_serial=None):
        cmd = AdbUtils._prepare_cmd(cmd_string, adb_shell, add_ticks, dut_serial)
        print cmd
        try:
            proc = subprocess.Popen(cmd, shell=True)
            return proc
        except:
            return None

    @staticmethod
    def start_activity_with_package_name(package_name, para=''):
        cmd = "monkey -p $PACKAGE_NAME$ %s -c android.intent.category.LAUNCHER 1" % para
        AdbUtils.run_adb_cmd(cmd.replace("$PACKAGE_NAME$", package_name))
        time.sleep(5)

    @staticmethod
    def reboot_device(timeout=300):
        cmd = AdbUtils.prepare_cmd("reboot")
        subprocess.check_call(cmd, shell=True)
#         wait for dut to reboot
        time.sleep(5)
#         poll_wait_times = 50
#         adb_devices_cmd = "adb devices"
#         reboot_time = 5
#         for i in xrange(poll_wait_times):
#             time.sleep(5)
#             reboot_time += 5
#             devices_connected = subprocess.check_output(adb_devices_cmd, shell=True)
#             if DutManager().active_uiautomator_device_serial in devices_connected:
#                  the dut has rebooted
#                 LOG.info("dut rebooted successfully")
#                 break
        count = 5
        sleep_time = 5
        while count < timeout:
            prop_val = g_common_obj.adb_cmd_capture_msg('getprop sys.boot_completed')
            if '1' in prop_val:
                print "dut rebooted successfully"
                return
            count += sleep_time
            time.sleep(sleep_time)

    @staticmethod
    def kill_python_uiautomator_rpc_server_on_dut():
        for i in range(20):
            time.sleep(1)
            AdbUtils.run_adb_cmd("pkill uiautomator")
            if "uiautomator" not in AdbUtils._run_adb_cmd("ps | grep uiautomator",add_ticks=False):
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
