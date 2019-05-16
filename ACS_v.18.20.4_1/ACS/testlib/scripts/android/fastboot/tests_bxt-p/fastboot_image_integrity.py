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
    os.system("mkdir -p ./temp/files/flash ./temp/files/resources")

    conf_url = "https://shstor001.sh.intel.com/artifactory/acs_test_artifacts/OTC_Android_Auto_Test_Suite/resources/EBImage/System_FastBoot/fastboot.conf"
    fastboot_utils.download_file(url=conf_url, local_filename="./temp/fastboot.conf")
    config = ConfigParser.ConfigParser()
    config.read("./temp/fastboot.conf")
    fastboot_path = config.get("fastboot", "path")

    simg2img_zip_path = config.get("files", "simg2img_zip")
    simg2img_zip_name = simg2img_zip_path.split("/")[-1]
    fastboot_utils.download_file(url = fastboot_path + simg2img_zip_path, local_filename = "./temp/files/resources/" + simg2img_zip_name)

    fastboot_utils.unpack_the_zip(file_name = flash_files, temp_path = r"./temp/image/n/flashfiles")
    fastboot_utils.unpack_the_zip(file_name = "./temp/files/resources/" + simg2img_zip_name, temp_path = r"./temp/files/resources")
    fastboot_utils.make_simg2img(file_path = "./temp/files/resources/" + simg2img_zip_name[:-4], return_path = "../../../..")
    os.system("simg2img ./temp/image/n/flashfiles/system.img ./temp/files/unsparse.img")

    adb_steps.reboot(command = "fastboot", reboot_timeout = 300, serial = serial)()
    fastboot_steps.command(command = "oem get-hashes > ./temp/files/oem_gethashes_result.txt 2>&1", serial=serial)()
    fastboot_steps.continue_to_adb(serial=serial)()
    time.sleep(60)
    local_steps.wait_for_adb(timeout = 300, serial=serial)()

    system_image_sha1 = fastboot_utils.get_image_sha1("sha1sum ./temp/files/unsparse.img")
    boot_image_sha1 = fastboot_utils.get_image_sha1("sha1sum ./temp/image/n/flashfiles/boot.img")
    system_partition_sha1 = fastboot_utils.fastboot_command_get_hashes(file_path = "./temp/files/oem_gethashes_result.txt", partition_name = "/system")
    boot_partition_sha1 = fastboot_utils.fastboot_command_get_hashes(file_path = "./temp/files/oem_gethashes_result.txt", partition_name = "/boot")
    if system_image_sha1 != system_partition_sha1 or boot_image_sha1 != boot_partition_sha1:
        raise Exception("The test result did not achieve the desired results")

    os.system("sudo rm -rf ./temp")

except:
    fastboot_utils.download_flash_scripts()
    fastboot_utils.flash_bxt(zip_file=flash_files, serial=serial)
    os.system("sudo rm -rf ./temp")
    raise
##### test end #####