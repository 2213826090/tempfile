#!/usr/bin/env python

##### imports #####
import os
import sys
import time
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
partition_name = args["partition_name"]
file_name = args["file_name"]

##### test start #####
try:
    os.system("mkdir -p ./temp/files/flash ./temp/image/n")
    fastboot_utils.download_flash_scripts()

    fastboot_utils.unpack_the_zip(file_name = flash_files, temp_path = r"./temp/image/n/flashfiles")
    adb_steps.reboot(command = "fastboot", reboot_timeout = 300, serial = serial)()
    fastboot_steps.flash_image(partition_name = partition_name, file_name = "./temp/image/n/flashfiles/"+file_name, serial = serial)()
    fastboot_steps.continue_to_adb(serial=serial)()
    time.sleep(60)
    local_steps.wait_for_adb(timeout = 300, serial=serial)()

    os.system("sudo rm -rf ./temp")

except:
    fastboot_utils.flash_bxt(zip_file=flash_files, serial=serial)
    os.system("sudo rm -rf ./temp")
    raise
##### test end #####