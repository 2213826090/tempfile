#!/usr/bin/env python

##### imports #####
import os
import sys
import time
import ConfigParser
from testlib.base.base_utils import get_args
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.connections.local import local_steps
from testlib.scripts.android.fastboot import fastboot_steps
from testlib.scripts.android.fastboot import fastboot_utils

##### initialization #####
globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val
flash_files = args["flash_files"]

##### test start #####
try:
    os.system("mkdir -p ./temp/files/flash ./temp/image/n")
    fastboot_utils.download_flash_scripts()

    conf_url = "https://shstor001.sh.intel.com/artifactory/acs_test_artifacts/OTC_Android_Auto_Test_Suite/resources/EBImage/System_FastBoot/fastboot.conf"
    fastboot_utils.download_file(url=conf_url, local_filename="./temp/fastboot.conf")
    config = ConfigParser.ConfigParser()
    config.read("./temp/fastboot.conf")
    fastboot_path = config.get("fastboot", "path")

    platform_name = fastboot_utils.get_platform_name(serial=serial)
    if platform_name == "bxtp_abl": image_platform = "image_m_bxt"
    if platform_name == "gordon_peak":
        ram_value = fastboot_utils.get_bxt_ram(serial=serial)
        if ram_value == "2g": image_platform = "image_o_bxt_2g"
        if ram_value == "4g": image_platform = "image_o_bxt_4g"
        if ram_value == "8g": image_platform = "image_o_bxt_4g"
    n_img_zip_path = config.get(image_platform, "n_img_zip")
    n_img_zip_name = n_img_zip_path.split("/")[-1]
    fastboot_utils.download_file(url = fastboot_path + n_img_zip_path, local_filename = "./temp/image/n/" + n_img_zip_name)

    adb_steps.reboot(command = "fastboot", reboot_timeout = 300, serial = serial)()
    fastboot_steps.unlock_device(serial = serial)()
    fastboot_steps.command(command = "update ./temp/image/n/" + n_img_zip_name + " > ./temp/files/temp.txt 2>&1", serial=serial)()
    time.sleep(120)
    local_steps.wait_for_adb(timeout = 300, serial=serial)()

    return_result = fastboot_utils.fastboot_command_result(file_name = "./temp/files/temp.txt")
    if not return_result:
        raise Exception("The test result did not achieve the desired results")

    fastboot_utils.flash_bxt(zip_file=flash_files, serial=serial)
    os.system("sudo rm -rf ./temp")

except:
    fastboot_utils.flash_bxt(zip_file=flash_files, serial=serial)
    os.system("sudo rm -rf ./temp")
    raise
##### test end #####