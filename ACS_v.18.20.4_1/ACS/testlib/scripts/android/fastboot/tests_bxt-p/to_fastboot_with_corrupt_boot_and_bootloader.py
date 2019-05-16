#!/usr/bin/env python

##### imports #####
import os
import sys
import time
from testlib.base.base_utils import get_args
from testlib.scripts.android.fastboot import fastboot_steps
from testlib.scripts.android.fastboot import fastboot_utils

##### initialization #####
globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val
flash_files = args["flash_files"]
partition_name = args["partition_name"]
partition_name = partition_name.split(",")
file_name = args["file_name"]
file_name = file_name.split(",")

##### test start #####
try:
    os.system("mkdir -p ./temp/files/flash ./temp/image/n")
    fastboot_utils.download_flash_scripts()

    fastboot_utils.unpack_the_zip(file_name = flash_files, temp_path = r"./temp/image/n/flashfiles")
    fastboot_steps.flash_wrong_file(partition_name = partition_name[0], serial = serial)()
    fastboot_steps.flash_wrong_file(partition_name = partition_name[1], file_name = "./temp/image/n/flashfiles/boot.img", serial = serial)()

    check_point1 = False
    check_point2 = False

    fastboot_utils.start_minicom(serial=serial)

    os.system("fastboot reboot > /dev/null 2>&1")
    time.sleep(60)
    fastboot_utils.to_fastboot_by_script(serial=serial)

    fastboot_utils.kill_minicom()

    file_path = "./temp/files/minicom_result.txt"
    return_result = open(file_path).readlines()
    for line in return_result:
        if "read_osloader_img fail" in line: check_point1 = True
        if "ELK: copy ELK from SPI in" in line: check_point2 = True

    if not check_point1 or not check_point2:
    	raise Exception("The test result did not achieve the desired results")

    fastboot_utils.flash_bxt(zip_file=flash_files, serial=serial)
    os.system("sudo rm -rf ./temp")

except:
    fastboot_utils.kill_minicom()
    fastboot_utils.flash_bxt(zip_file=flash_files, serial=serial)
    os.system("sudo rm -rf ./temp")
    raise
##### test end #####