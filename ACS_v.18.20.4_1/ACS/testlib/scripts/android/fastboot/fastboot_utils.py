#!/usr/bin/env python

import ConfigParser
import os
import re
import requests
import subprocess
import threading
import time
import warnings
import zipfile
from testlib.scripts.connections.local import local_steps
from testlib.scripts.connections.local import local_utils
from testlib.utils.connections.adb import Adb as connection_adb
from testlib.utils.connections.local import Local as connection_local
from uiautomator import Device


def get_var(var, serial=None):
    if serial:
        fastboot_cmd = "fastboot -s {0} getvar {1} 2>&1".format(serial, var)
    else:
        fastboot_cmd = "fastboot getvar {0} 2>&1".format(var)
    local_connection = connection_local()
    # Get the output of the command
    value = local_connection.run_cmd(command=fastboot_cmd)
    # Get the var value (if the fastboot var exists)
    var_value = [i for i in value if var in i and "UNKNOWN COMMAND" not in i]
    if len(var_value):
        return var_value[0].split('\n')[0].split(':')[-1].strip().lower()
    return None


def var_exists(var, serial=None):
    return bool(get_var(var, serial))


def is_verified_boot_state(serial = None, state = None):
    if serial:
        adb_connection = connection_adb(serial = serial)
    else:
        adb_connection = connection_adb()
    cmd = "getprop ro.boot.verifiedbootstate"
    return state in adb_connection.parse_cmd_output(cmd = cmd, grep_for = state)


def unpack_the_zip(file_name=None, temp_path=None):
    file_zip = zipfile.ZipFile(file_name, "r")
    for f in file_zip.namelist():
        file_zip.extract(f, temp_path)
    file_zip.close()


def make_the_zip(dir_name=None, file_name=None):
    file_zip = zipfile.ZipFile(file_name, "w", allowZip64=True)
    pre_len = len(os.path.dirname(dir_name))
    for parent, dirnames, filenames in os.walk(dir_name):
        for filename in filenames:
            pathfile = os.path.join(parent, filename)
            arcname = pathfile[pre_len:].strip(os.path.sep)
            file_zip.write(pathfile, arcname)
    file_zip.close()


def get_platform_name(serial=None):
    if serial in local_utils.get_connected_android_devices()["android"]:
        return_result = os.popen("adb shell getprop ro.product.device").readlines()
        return return_result[0].strip("\r\n")
    if serial in local_utils.get_connected_android_devices()["fastboot"]:
        if not os.path.exists("./temp/files"):
            os.makedirs("./temp/files")
        os.system("fastboot getvar product > ./temp/files/temp.txt 2>&1")
        return_result = open("./temp/files/temp.txt").readlines()
        for line in return_result:
            if "product" in line:
                return line.split(":")[1].strip()
    return None


def get_zip_name(zip_path=None):
    zip_name = zip_path.split("/")[-1]
    platform_name = zip_name.split("-")[0]
    return platform_name


def get_bxt_ram(serial=None):
    cmd = "adb -s {0} shell dumpsys meminfo".format(serial)
    return_result = os.popen(cmd).readlines()
    for line in return_result:
        if "Total RAM" in line:
            ram = float(re.sub("[,K]", "", line.split(" ")[2]))
            if round(ram/(1024*1024)) == 2.0:
                return "2g"
            if round(ram/(1024*1024)) == 4.0:
                return "4g"
            if round(ram/(1024*1024)) == 8.0:
                return "8g"
    return None


def fastboot_command_result(file_name=None):
    result = True
    f = open(file_name)
    return_result = f.readlines()
    for line in return_result:
        line = line.strip("\r\n").split()
        if len(line) > 0 and "FAILED" == line[0]: result = False
    f.close()
    return result


def adb_command_file_exists(command=None):
    result = False
    return_result = os.popen(command).readlines()
    for line in return_result:
        line = line.strip("\r\n").split(": ")
        if len(line) == 1: result = True
    return result


def adb_command_file_or_directory_exists(name=None, command=None):
    result = False
    return_result = os.popen(command).readlines()
    for line in return_result:
        line = line.strip("\r\n")
        if name in line: result = True
    return result


def adb_command_process_exists(serial=None, strpro="com.google.android.setupwizard"):
    local_steps.command("adb -s {} reboot".format(serial))()
    local_steps.wait_for_adb(timeout = 300, serial = serial)()
    time.sleep(10)
    adb_command = "adb shell ps | grep \"" + strpro + "\""
    return_result = subprocess.Popen(adb_command, stdout=subprocess.PIPE, shell=True).stdout.readline()
    if return_result != "": return True
    return False


def creat_big_file(file_name, size):
    big_file = open(file_name, "w")
    big_file.seek(1024*1024*1024*size)
    big_file.write("\x00")
    big_file.close()


def adb_install_apk(platform_name=None, apk_path=None, file_path=None, serial=None):
    if platform_name == "bxtp_abl":
        install_command = "adb -s {0} install {1}".format(serial, apk_path)
        return_result = os.popen(install_command).readlines()
    if platform_name == "gordon_peak" or platform_name == "androidia_64":
        return_result = open(file_path).readlines()
    for line in return_result:
        if "Success" in line.strip(): return True
    return False


def make_simg2img(file_path=None, return_path=None):
    os.chdir(file_path)
    os.system("make")
    os.chdir(return_path)


def get_image_sha1(command=None):
    return_result = os.popen(command).readlines()
    for line in return_result:
        if line == "": continue
        return line.split("  ")[0]
    return None


def fastboot_command_get_hashes(file_path=None, partition_name=None):
    tag_value = False
    return_result = open(file_path).readlines()
    for line in return_result:
        line = line.strip("\r\n").split(" ")
        if line[0] != "(bootloader)": continue
        if line[1] == "hash:" and tag_value == True: return line[2]
        if partition_name in line[2]: tag_value = True
    return None


def get_device_state(file_path=None):
    return_result = open(file_path).readlines()
    for line in return_result:
        if "device-state" in line:
            return line.split(":")[1].strip()
    return None


def tipc_negativetipc_execution_result(case_type=None, command=None, case_name=None):
    return_result = os.popen(command).readlines()
    for line in return_result:
        if case_type == "tipc_test":
            if line.strip("\r\n") != "" and "result" in line.strip("\r\n") and "PASS" not in line.strip("\r\n"): return False
        if case_type == "negative_tipc_test":
            if line.strip("\r\n") != "" and case_name + " test" in line.strip("\r\n") and "Pass" not in line.strip("\r\n"): return False
    return True


def push_uiautomator_jar(serial=None):
    # local_steps.command("adb -s {} reboot".format(serial))()
    # local_steps.wait_for_adb(timeout = 300, serial = serial)()
    # time.sleep(15)
    print "Check RPC server ..."
    if os.system("adb -s %s shell ps | grep uiautomator > /dev/null 2>&1" % serial) == 0:
        print "RPC server started"
        return
    else:
        from uiautomator import device as d
        if d.info:
            print "RPC server started"
            return
    print os.system("adb -s %s shell getprop | grep ro.build.version.release" % serial)
    if os.path.exists("/usr/lib/python2.7/dist-packages/uiautomator/libs"):
        jar_path = "/usr/lib/python2.7/dist-packages/uiautomator/libs"
    else:
        jar_path = "/usr/local/lib/python2.7/dist-packages/uiautomator/libs"
    bundle_path = jar_path + "/bundle.jar"
    uiautomator_path = jar_path + "/uiautomator-stub.jar"
    push_bundle_jar = "adb -s " + serial + " push " + bundle_path + " /data/local/tmp"
    push_uiautomator_jar = "adb -s " + serial + " push " + uiautomator_path + " /data/local/tmp"
    os.system("adb -s %s root > /dev/null 2>&1" % serial)
    for i in range(20):
        d.info
        time.sleep(5)
        if os.system("adb -s %s shell ps |grep uiautomator" % serial) == 0:
            print "RPC server started"
            break
        else:
            print "push jar start ========"
            os.system(push_bundle_jar)
            os.system(push_uiautomator_jar)
            print "push jar end ========="
            time.sleep(5)
    else:
        raise Exception("RPC server start failed")


def download_file(url=None, local_filename=None):
    warnings.filterwarnings("ignore")
    try:
        r = requests.get(url, stream = True, verify = False)
        with open(local_filename, "wb") as f:
            for chunk in r.iter_content(chunk_size = 1024):
                if chunk:
                    f.write(chunk)
                    f.flush()
    except warning as e:
        logger.error(e)
    return local_filename


def execute_minicom_command(port=None):
    minicom_cmd = "sudo minicom -D {0} -C ./temp/files/minicom_result.txt > ./temp/files/minicom_result_backup.txt 2>&1".format(port)
    os.system(minicom_cmd)


def start_minicom(serial=None):
    debugcard_port_list = get_debugcard_port(serial=serial)
    port = debugcard_port_list[-1]
    threads = []
    t1 = threading.Thread(target=execute_minicom_command, args=(port,))
    threads.append(t1)
    for t in threads:
        t.setDaemon(True)
        t.start()


def kill_minicom():
    cmd = "ps -ef | grep minicom"
    return_result = os.popen(cmd).readlines()
    for line in return_result:
        line = line.split()
        kill_process = "sudo kill -9 {0}".format(line[1])
        os.system(kill_process)


def get_debugcard_port(serial=None):
    debugcard_port_list = []
    return_result = os.popen("ls /dev | grep {0}".format(serial)).readlines()
    for line in return_result:
        if "debugCard" in line.strip():
            debugcard_port_list.append("/dev/"+line.strip())
    if not debugcard_port_list:
        return_result = os.popen("ls /dev/ttyUSB*").readlines()
        for line in return_result:
            if "ttyUSB" in line.strip():
                debugcard_port_list.append(line.strip())
    debugcard_port_list.sort()
    return debugcard_port_list


def to_fastboot_by_script(serial=None):
    debugcard_port_list = get_debugcard_port(serial=serial)
    os.system("sudo python ./temp/files/flash/bootDeviceTo.py --port {0} --mode fastboot".format(debugcard_port_list[-2]))


def flash_bxt(flash_ioc="True", flash_ifwi="True", flash_android="True", zip_file=None, serial=None, sleep_time=120, wait_for_adb=True):
    debugcard_port_list = get_debugcard_port(serial=serial)
    os.system("sudo python ./temp/files/flash/flash_image.py --flash_ioc=\"{0}\" --flash_ifwi=\"{1}\" --flash_android=\"{2}\" --zip_file=\"{3}\" --port=\"{4}\""\
                .format(flash_ioc, flash_ifwi, flash_android, zip_file, debugcard_port_list[-2]))
    time.sleep(sleep_time)
    if wait_for_adb:
        local_steps.wait_for_adb(timeout=300, serial=serial)()


def flash_bxt_m_fused(eb_user_patch_flashfiles_zip_name=None, eb_userdebug_patch_flashfiles_zip_name=None, support_flash=False, serial=None):
    unpack_the_zip(file_name="./temp/image/eb/userdebug/"+eb_userdebug_patch_flashfiles_zip_name, temp_path=r"./temp/image/eb/userdebug/flashfiles")
    unpack_the_zip(file_name="./temp/image/eb/user/"+eb_user_patch_flashfiles_zip_name, temp_path=r"./temp/image/eb/user/flashfiles")
    os.system("cp -f ./temp/image/eb/userdebug/flashfiles/boot.img ./temp/image/eb/user/flashfiles")
    if support_flash:
        os.system("cp -f ./temp/image/eb/userdebug/flashfiles/ifwi_gr_mrb_b1.bin ./temp/image/eb/user/flashfiles")
    make_the_zip(dir_name="./temp/image/eb/user/flashfiles/", file_name="./temp/image/eb/flashfiles.zip")

    flash_bxt(flash_ioc="True", flash_ifwi="False", flash_android="True", zip_file="./temp/image/eb/flashfiles.zip", serial=serial, sleep_time=30, wait_for_adb=False)
    flash_bxt(flash_ioc="False", flash_ifwi="True", flash_android="False", zip_file="./temp/image/eb/flashfiles.zip", serial=serial, sleep_time=30, wait_for_adb=False)
    to_fastboot_by_script(serial=serial)
    time.sleep(30)
    local_steps.wait_for_fastboot(timeout=300, serial=serial)()
    os.system("fastboot reboot > /dev/null 2>&1")
    time.sleep(120)

    push_uiautomator_jar(serial=serial)
    d = Device(serial)
    time.sleep(5)
    if d(text="To start Android, enter your password").exists:
        d(resourceId="com.android.settings:id/passwordEntry").click.wait()
        time.sleep(5)
        os.system("adb -s {0} shell input text 1234".format(serial))
        time.sleep(5)
        os.system("adb -s {0} shell input keyevent 66".format(serial))
        time.sleep(5)
    if d(text="Decryption unsuccessful").exists:
        d(text="Reset phone").click.wait()
        time.sleep(120)
    local_steps.wait_for_adb(timeout=300, serial=serial)()


def enable_oem_unlock(serial=None):
    d = Device(serial)
    os.system("adb -s {} shell am start -S com.android.settings/.Settings".format(serial))
    time.sleep(2)
    if not d(text="About phone").exists and d(scrollable=True).exists:
        time.sleep(1)
        d(scrollable=True).scroll.to(text="About phone")
    time.sleep(1)
    d(text="About phone").click.wait()
    time.sleep(1)
    for _ in range(10):
        time.sleep(1)
        d(text="Build number").click()
    d.press.back()
    time.sleep(1)
    d(text="Developer options").click.wait()
    time.sleep(1)
    if not d(text="OEM unlocking").right(className="android.widget.Switch").checked:
        time.sleep(1)
        d(text="OEM unlocking").right(className="android.widget.Switch").click.wait()
    time.sleep(1)
    if d(text="ENABLE").exists:
        time.sleep(1)
        d(text="ENABLE").click.wait()
    time.sleep(1)

def download_flash_scripts():
    conf_url = "https://shstor001.sh.intel.com/artifactory/acs_test_artifacts/OTC_Android_Auto_Test_Suite/resources/EBImage/System_FastBoot/fastboot.conf"
    download_file(url=conf_url, local_filename="./temp/fastboot.conf")
    config = ConfigParser.ConfigParser()
    config.read("./temp/fastboot.conf")
    fastboot_path = config.get("fastboot", "path")
    boot_device_script_path = config.get("files", "boot_device_script")
    boot_device_script_name = boot_device_script_path.split("/")[-1]
    download_file(url=fastboot_path+boot_device_script_path, local_filename="./temp/files/flash/"+boot_device_script_name)
    flash_image_script_path = config.get("files", "flash_image_script")
    flash_image_script_name = flash_image_script_path.split("/")[-1]
    download_file(url=fastboot_path+flash_image_script_path, local_filename="./temp/files/flash/"+flash_image_script_name)