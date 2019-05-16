# -*- coding: utf-8 -*-
import os
import time
import subprocess
import re
from testlib.util.config import TestConfig
from testlib.util.device import TestDevice
from testlib.util.common import g_common_obj
from testlib.util.log import Logger
from testlib.util.process import shell_command
from constants_def import *

LOG = Logger.getlogger(__name__)

def shell_cmd(cmd, timeout = 90):
    LOG.debug("Exec cmd: " + cmd)
    shell_command(cmd)

def get_config_by_tag(tag):
    cfg = TestConfig().read(DEFAULT_CONFIG, tag)
    return cfg

def get_config_value(tag, name):
    cfg = TestConfig().read(DEFAULT_CONFIG, tag)
    return cfg.get(name)

def get_server_ip():
    if os.environ.get(NOSERUNNER):
        config_tag = "server_2"
    else:
        config_tag = "server_1"
    cfg = get_config_by_tag(config_tag)
    base_path = os.path.dirname(os.path.realpath(__file__))
    shell_file = os.path.join(base_path, "get_server_ip.sh")
    host = cfg.get("host")
    user = cfg.get("user")
    passwd = cfg.get("passwd")
    #print ["expect", shell_file, host, user, passwd]
    cmd = ' '.join(["expect", shell_file, host, user, passwd])
    return os.popen("%s| grep '192.168*'| awk -F ' ' '{print $2}'| awk -F ':' '{print $2}'" % cmd).read().strip()

def download_artifactory_content(option_file):
    from testlib.util.repo import Artifactory
    cfg = TestConfig().read(DEFAULT_CONFIG, "artifactory")
    arti_obj = Artifactory(cfg.get("location"))
    ret_file = arti_obj.get(cfg.get(option_file))
    return ret_file

def read_file_by_line(file_name):
    with open(file_name) as file_obj:
        while True:
            line = file_obj.readline()
            if not line:
                break
            yield line.strip()

def write_to_file(str_to_write, file_name, mode='w'):
    with open(file_name, mode) as file_obj:
        file_obj.write(str_to_write)

def get_tmp_dir():
    cur_dir = os.path.dirname(__file__)
    tmp_dir = os.path.join(cur_dir, "tmp")
    if not os.path.exists(tmp_dir):
        os.mkdir(tmp_dir)
    return tmp_dir

def gen_tmp_file_path_by_time(name):
    tmp_dir = get_tmp_dir()
    file_name = time.strftime("%Y%m%d_%H%M%S") + "_" + name
    return os.path.join(tmp_dir, file_name)

def remove_tmp_dir(tmp_dir):
    os.system("rm -rf %s" % tmp_dir)

def mount_mtp():
    mount_dir = os.path.join(get_tmp_dir(), "mtp")
    if not os.path.exists(mount_dir):
        os.mkdir(mount_dir)
    cmd = "mtpfs %s" % mount_dir
    LOG.debug("Exec cmd: " + cmd)
    subprocess.call(cmd, shell = True)
    cmd = "ls %s" % mount_dir
    child = subprocess.Popen(cmd, shell = True, stdout = subprocess.PIPE)
    child.wait()
    out = child.stdout.read()
    return "storage" in out

def umount_mtp():
    mount_dir = os.path.join(get_tmp_dir(), "mtp")
    cmd = "fusermount -u %s" % mount_dir
    subprocess.call(cmd, shell = True)

def get_ptp_mount_dir(serialno):
    cmd = "mount | grep gvfsd-fuse"
    child = subprocess.Popen(cmd, shell = True, stdout = subprocess.PIPE)
    child.wait()
    out = child.stdout.read()
    gvfsd_fuse_dir = out.split()[2]

    cmd = "ls %s" % gvfsd_fuse_dir
    child = subprocess.Popen(cmd, shell = True, stdout = subprocess.PIPE)
    child.wait()
    out = child.stdout.read()
    pattern = re.compile("gphoto2:host=%5Busb%3A(\d+)%2C(\d+)%5D")
    for line in out.splitlines():
        result = pattern.match(line)
        dev_path = os.path.join("/dev/bus/usb/", result.group(1), result.group(2))
        cmd = "lsusb -D %s | grep %s" % (dev_path, serialno)
        ret_code = subprocess.call(cmd, shell = True, stdout = subprocess.PIPE)
        if ret_code == 0:
            return os.path.join(gvfsd_fuse_dir, line)
    else:
        return None

def get_geo_location():
    cfg = get_config_by_tag("geo_location")
    latitude = cfg.get("latitude")
    longitude = cfg.get("longitude")
    return float(latitude), float(longitude)


class ADBTools(object):

    def __init__(self, serial = None):
        if not serial:
            self.testDevice = g_common_obj.get_test_device()
        else:
            self.testDevice = TestDevice(serial)

    def check_adb(self):
        if "hello" == self.testDevice.adb_cmd_capture_msg("echo hello"):
            return True
        else:
            return False

    def get_state(self):
        state = self.testDevice.adb_cmd_common("get-state")
        LOG.debug("Device state: " + state)
        if 'device' in state:
            return True
        else:
            return False

    def get_serialno(self):
        msg = self.testDevice.adb_cmd_capture_msg("getprop ro.serialno")
        return msg

    def get_time(self, time_format = r"%H:%M:%S"):
        time_str = self.testDevice.adb_cmd_capture_msg('''"date +'%s'"''' % time_format)
        return time_str

    def check_boot_completed(self):
        msg = self.testDevice.adb_cmd_capture_msg("getprop sys.boot_completed")
        if '1' == msg:
            return True
        return False

    def get_product(self):
        if not hasattr(self, "_product_name"):
            cmd = "getprop ro.product.name"
            self._product_name = self.testDevice.adb_cmd_capture_msg(cmd)
        return self._product_name

    def get_build_version(self):
        if not hasattr(self, "_build_version"):
            cmd = "getprop ro.build.version.release"
            self._build_version = self.testDevice.adb_cmd_capture_msg(cmd)
        return self._build_version

    def adb_root(self):
        self.testDevice.root_on_device()

    def adb_usb(self):
        self.testDevice.adb_cmd_common("usb")

    def input_text(self, text):
        cmd = "input text %s" % text
        self.testDevice.adb_cmd(cmd)

    def input_keyevent(self, key_code = "KEYCODE_HOME"):
        cmd = "input keyevent %s" % key_code
        self.testDevice.adb_cmd(cmd)

    def input_tap(self, x, y):
        cmd = "input tap %s %s" % (x, y)
        self.testDevice.adb_cmd(cmd)

    def get_process_pid(self, process_name):
        build = self.get_build_version()
        if build == "O" or build.startswith("8"):
            cmd = "ps -A | grep %s" % process_name
        else:
            cmd = "ps | grep %s" % process_name
        msg = self.testDevice.adb_cmd_capture_msg(cmd)
        pid = msg.split()[1]
        return pid

    def grant_permission(self, package, permission):
        self.testDevice.adb_cmd("pm grant %s %s" % (package, permission))

    def clear_app_data(self, app_package):
        self.testDevice.adb_cmd("pm clear %s"% app_package)

    def launch_app(self, app_package, app_activity):
        self.testDevice.launch_app_am(app_package, app_activity)
        time.sleep(5)

    def stop_app(self, app_package):
        self.testDevice.stop_app_am(app_package)

    def stop_focused_activity(self):
        app = self.get_focused_app()
        if app:
            package = app.split("/")[0]
            self.testDevice.stop_app_am(package)

    def get_focused_app(self):
        cmd = "dumpsys window windows | grep 'mCurrentFocus'"
        #pattern = re.compile("{f3d7382 u0 com.android.settings/com.android.settings.Settings}")
        pattern = re.compile("{.* (\S+/.*)}")
        msg = self.testDevice.adb_cmd_capture_msg(cmd)
        result = pattern.search(msg)
        if result:
            return result.group(1)
        return None

    def install_artifactory_app(self, option_file, package):
        cmd = "pm list package %s" % package
        check_str = "package:" + package
        msg_list = self.testDevice.adb_cmd_capture_msg(cmd).splitlines()
        if check_str in msg_list:
            LOG.debug("App %s already installed." % package)
            return
        LOG.debug("Install %s." % package)
        ret_file = download_artifactory_content(option_file)
        assert os.path.isfile(ret_file)
        self.testDevice.adb_cmd_common("install -r %s >/dev/null" % ret_file, 300)
        time.sleep(2)
        msg_list = self.testDevice.adb_cmd_capture_msg(cmd).splitlines()
        assert check_str in msg_list, "Install app failed!"

    def push_artifactory_resource(self, option_file, remote_dir = "/mnt/sdcard/Movies/"):
        cfg = TestConfig().read(DEFAULT_CONFIG, "artifactory")
        res_file = cfg.get(option_file).split("/")[-1]
        cmd = "'ls %s 2>/dev/null'| grep %s" % (remote_dir, res_file)
        msg = self.testDevice.adb_cmd_capture_msg(cmd)
        if msg != "":
            LOG.debug("res file %s already existed." % res_file)
            return os.path.join(remote_dir, res_file)
        ret_file = download_artifactory_content(option_file)
        assert os.path.isfile(ret_file)
        self.testDevice.adb_cmd("mkdir -p %s" % remote_dir)
        self.testDevice.adb_cmd_common("push %s %s >/dev/null" % (ret_file, remote_dir), 600)
        return os.path.join(remote_dir, res_file)

    def broadcast_media_mounted(self):
        cmd = "am broadcast -a android.intent.action.MEDIA_MOUNTED -d file:///sdcard/"
        self.testDevice.adb_cmd(cmd)

    def get_logcat_format_time(self):
        time_str = self.testDevice.adb_cmd_capture_msg('''"date +'%m-%d %H:%M:%S'"''')
        time_str += ".000"
        LOG.debug("Get logcat format time: %s" % time_str)
        return time_str

    def remove_file(self, file_path):
        self.testDevice.adb_cmd("rm -rf %s" % file_path)

    def get_screen_status(self):
        cmd = "dumpsys power | grep mWakefulness= | awk -F = '{print $2}'"
        msg = self.testDevice.adb_cmd_capture_msg(cmd)
        return msg == "Awake"

    def verify_screen_status(self, status): # to be removed
        print "[info]--- Verify screen is %s" % status
        if self.get_screen_status():
            assert status == "on"
        else:
            assert status != "on"

    def capture_screen(self, screenshot):
        screenshot_remote = "/mnt/sdcard/screenshot.png"
        self.testDevice.adb_cmd("screencap -p %s" % screenshot_remote)
        self.testDevice.adb_cmd_common("pull %s %s" % (screenshot_remote, screenshot))

    def get_external_sdcard_dirname(self):
        msg = self.testDevice.adb_cmd_capture_msg("ls /mnt/media_rw/")
        import re
        p=re.compile("([0-9A-Z]+-[0-9A-Z]+)")
        for item in msg.split():
            m = p.match(item)
            if m:
                sdcard_dir = m.group(1)
                return sdcard_dir
        return ""

    def get_cpus(self):
        cmd = "ls /sys/devices/system/cpu/ | grep -e cpu[0-9]"
        msg = self.testDevice.adb_cmd_capture_msg(cmd)
        cpu_arr = msg.splitlines()
        cpuid_arr = [int(cpu[-1]) for cpu in cpu_arr]
        return cpuid_arr

    def ivi_get_cpu_temp(self):
        cmd = "cat /sys/class/thermal/thermal_zone0/temp"
        temp = self.testDevice.adb_cmd_capture_msg(cmd)
        return int(temp)

    def ivi_get_board_temp(self):
        cmd = "cat /sys/class/thermal/thermal_zone1/temp"
        temp = self.testDevice.adb_cmd_capture_msg(cmd)
        return int(temp)

    def ivi_get_ambient_temp(self):
        cmd = "cat /sys/class/thermal/thermal_zone2/temp"
        temp = self.testDevice.adb_cmd_capture_msg(cmd)
        return int(temp)

    def ivi_get_cpu_freq(self, cpuid = 0, freq_name = "cpuinfo_cur_freq"):
        cmd = "cat /sys/devices/system/cpu/cpu%s/cpufreq/%s" % (cpuid, freq_name)
        freq = self.testDevice.adb_cmd_capture_msg(cmd)
        return int(freq)

    def ivi_get_cpus_freq(self, cpu_arr, freq_name = "cpuinfo_cur_freq"):
        freqs = []
        for i in cpu_arr:
            cmd = "cat /sys/devices/system/cpu/cpu%s/cpufreq/%s" % (i, freq_name)
            freq = self.testDevice.adb_cmd_capture_msg(cmd)
            freqs.append(int(freq))
        return freqs

    def ivi_get_power(self):
        cmd = "cat /sys/class/powercap/intel-rapl:0/constraint_0_power_limit_uw"
        power = self.testDevice.adb_cmd_capture_msg(cmd)
        return int(power)

    def get_gpu_freq(self, freq_name = "Current freq"):
        cmd = "cat /d/dri/0/i915_frequency_info | grep -i '%s' | grep -o '[0-9]\+'" % freq_name
        freq = self.testDevice.adb_cmd_capture_msg(cmd)
        return int(freq)

    def get_external_disk_mount_path(self, flags):
        """
        flags value:
        USB -> USB drive
        SD  -> SD Card
        """
        msg = self.testDevice.adb_cmd_capture_msg("dumpsys mount")
        lines = msg.splitlines()
        i = 0
        while not lines[i].startswith("Disks:"):
            i += 1
        i += 1
        diskid = None
        while lines[i].startswith(" "):
            diskid = lines[i].split("{")[1].split("}")[0]
            i += 1
            f = lines[i].split()[0].split("=")[1]
            if flags in f:
                break
            i += 2
        while not lines[i].startswith("Volumes:"):
            i += 1
        i += 2
        path = None
        if diskid:
            while lines[i].startswith(" "):
                if diskid in lines[i]:
                    i += 2
                    path = lines[i].split()[0].split("=")[1]
                    break
                else:
                    i += 4
        return path

    def check_boot_completed(self):
        msg = self.testDevice.adb_cmd_capture_msg("getprop sys.boot_completed")
        if '1' == msg:
            return True
        return False

    def wait_boot_completed(self):
        for _ in range(20):
            time.sleep(5)
            if self.check_boot_completed():
                print "[info]--- Boot up completed"
                break
        else:
            return False
        return True

    def adb_remount(self):
        msg = self.testDevice.adb_cmd_common("remount")
        LOG.debug(msg)
        if "remount succeeded" == msg:
            return True
        version = self.get_build_version()
        if version.startswith("8"):
            self.disable_verity_bxt_omr1()
        elif version.startswith("6"):
            self.disable_verity_bxt_m()
        msg = self.testDevice.adb_cmd_common("remount")
        if "remount succeeded" == msg:
            return True
        return False

    def disable_verity_bxt_omr1(self):
        vbmeta_file = download_artifactory_content("vbmeta")
        assert vbmeta_file, "Download failed"
        self.testDevice.adb_cmd_common("reboot fastboot")
        time.sleep(10)
        shell_cmd("fastboot flashing unlock")
        shell_cmd("fastboot flash vbmeta %s" % vbmeta_file)
        shell_cmd("fastboot flashing lock")
        shell_cmd("fastboot reboot")
        assert self.wait_boot_completed(), "Boot up failed"
        self.adb_root()

    def disable_verity_bxt_m(self):
        self.testDevice.adb_cmd_common("disable-verity")
        self.testDevice.adb_cmd_common("reboot")
        assert self.wait_boot_completed(), "Boot up failed"
        self.adb_root()

    def get_md5sum(self, path):
        msg = self.testDevice.adb_cmd_capture_msg("md5sum " + path)
        return msg.split()[0]


class Logcat(ADBTools):

    def __init__(self, filter_tag = None, grep_opt = None, serial = None):
        ADBTools.__init__(self, serial)
        self.filter_tag = filter_tag
        self.grep_opt = grep_opt
        self.start_time = None

    def get_format_time(self):
        time_str = self.get_time(r'%m-%d %H:%M:%S')
        time_str += ".000"
        LOG.debug("Get logcat format time: %s" % time_str)
        return time_str

    def set_start_time(self, start_time = None):
        if start_time:
            self.start_time = start_time
        else:
            self.start_time = self.get_format_time()

    def get_log(self):
        if self.start_time:
            cmd = "logcat -t '%s'" % self.start_time
        else:
            cmd = "logcat -d"
        if self.filter_tag:
            cmd += " -s " + self.filter_tag
        if self.grep_opt:
            cmd += " | " + self.grep_opt
        log = self.testDevice.adb_cmd_common(cmd)
        return log


class UIBase(ADBTools):

    def __init__(self, serial = None):
        ADBTools.__init__(self, serial)
        self.d = self.testDevice.get_device()

    def set_screen_status(self, status):
        self.d.screen(status)

    def press_home_button(self):
        self.d.press("home")

    def unlock_screen(self):
        if self.d(description="Unlock").exists:
            self.d.press("menu")

    def set_screen_orientation(self, orientation):
        self.d.orientation = orientation

    def open_notification(self, open_all = False):
        #self.d.open.notification()
        if self.get_build_version() == O_MR1:
            self.d.open.notification()
        else:
            info = self.d.info
            x = info["displayWidth"] / 2
            y = info["displayHeight"] / 2
            self.d.swipe(x, 0, x, y, steps = 10)
            if open_all:
                self.d.swipe(x, 0, x, y, steps = 10)

    def select_usb_option(self, option):
        """
        M and O MR0
        """
        self.open_notification()
        if self.get_product() == BXT_O:
            option_dict = {
                "charge": "Charge this device",
                "MTP": "Transfer files",
                "PTP": "Transfer photos (PTP)"
                }
            self.d(resourceId = "android:id/header_text_divider").click()
            self.d(text = "Tap for more options.").click()
        else:
            option_dict = {
                "charge": "Charging",
                "MTP": "File transfers",
                "PTP": "Photo transfer (PTP)"
                }
            self.d(text = "Touch for more options.").click()
        bounds = self.d(text = option_dict[option]).bounds
        x = (bounds["left"] + bounds["right"]) / 2
        y = (bounds["top"] + bounds["bottom"]) / 2
        cmd = "input tap %s %s" % (x, y)
        self.testDevice.adb_cmd(cmd)

    def check_notification_status(self):
        if self.get_build_version() == O_MR1:
            if self.d(resourceId = "com.android.systemui:id/multi_user_switch").exists:
                return True
        elif self.d(resourceId = "com.android.systemui:id/notification_stack_scroller").exists:
            return True
        return False

    def rotate(self, orientation):
        self.d.orientaton = orientation
        time.sleep(2)
        assert self.d.orientaton == orientation

    def resume_app_from_task_manager(self, app):
        print "[info]--- Resume %s from task manager" % app
        self.d.press.recent()
        time.sleep(2)
        if self.d(resourceId="com.android.systemui:id/activity_description", text=app).exists:
            self.d(resourceId="com.android.systemui:id/activity_description", text=app).click.wait()
        else:
            self.d(resourceId="com.android.systemui:id/title", text=app).click.wait()
        time.sleep(2)

    def play_media_file(self, data_type, data_uri):
        print "[info]--- Play media file"
        cmd = "am start -a %s -t %s -d %s" % (INTENT_VIEW, data_type, data_uri)
        self.testDevice.adb_cmd(cmd)
        if data_type.startswith("audio/"):
            if self.d(text="Open with").exists:
                self.d(text="Google Play Music").click()
                self.d(resourceId="android:id/button_always").click()

    def skip_drive_safly_interface(self):
        if self.d(text="Owner").exists:
            self.d(text="Owner").click()

