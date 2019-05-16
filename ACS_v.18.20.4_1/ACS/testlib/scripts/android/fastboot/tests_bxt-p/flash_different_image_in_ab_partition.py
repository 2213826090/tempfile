#!/usr/bin/env python

##### imports #####
import os
import re
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
	os.system("mkdir -p ./temp/files/flash ./temp/image/n ./temp/image/n-1")
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
	n_1_flashfiles_zip_path = config.get(image_platform, "n_1_flashfiles_zip")
	n_1_flashfiles_zip_name = n_1_flashfiles_zip_path.split("/")[-1]
	fastboot_utils.download_file(url = fastboot_path + n_1_flashfiles_zip_path, local_filename = "./temp/image/n-1/" + n_1_flashfiles_zip_name)

	fastboot_utils.unpack_the_zip(file_name = flash_files, temp_path = r"./temp/image/n/flashfiles")
	fastboot_utils.unpack_the_zip(file_name = "./temp/image/n-1/" + n_1_flashfiles_zip_name, temp_path = r"./temp/image/n-1/flashfiles")

	fastboot_utils.flash_bxt(flash_ioc="True", flash_ifwi="True", flash_android="False", zip_file=flash_files, serial=serial, sleep_time=60, wait_for_adb=False)
	fastboot_utils.to_fastboot_by_script(serial=serial)
	local_steps.wait_for_fastboot(timeout=300, serial=serial)()

	n_img_path = "./temp/image/n/flashfiles/"
	fastboot_steps.flash_image(partition_name="gpt", file_name=n_img_path+"gpt_gr_mrb_b1.bin", unlock_dut=True, lock_dut=False, serial=serial)()
	fastboot_steps.fastboot_erase_partition(partition_name="teedata", unlock_dut=False, lock_dut=False, reboot=False, serial=serial)()
	fastboot_steps.fastboot_erase_partition(partition_name="misc", unlock_dut=False, lock_dut=False, reboot=False, serial=serial)()
	fastboot_steps.fastboot_erase_partition(partition_name="persistent", unlock_dut=False, lock_dut=False, reboot=False, serial=serial)()
	fastboot_steps.fastboot_erase_partition(partition_name="metadata", unlock_dut=False, lock_dut=False, reboot=False, serial=serial)()
	fastboot_steps.format_partition(partition_name="data", unlock_dut=False, lock_dut=False, serial=serial)()
	fastboot_steps.format_partition(partition_name="config", unlock_dut=False, lock_dut=False, serial=serial)()
	fastboot_steps.flash_image(partition_name="system_a", file_name=n_img_path+"system.img", unlock_dut=False, lock_dut=False, serial=serial)()
	fastboot_steps.flash_image(partition_name="vendor_a", file_name=n_img_path+"vendor.img", unlock_dut=False, lock_dut=False, serial=serial)()
	fastboot_steps.flash_image(partition_name="vbmeta_a", file_name=n_img_path+"vbmeta.img", unlock_dut=False, lock_dut=False, serial=serial)()
	fastboot_steps.flash_image(partition_name="multiboot_a", file_name=n_img_path+"multiboot.img", unlock_dut=False, lock_dut=False, serial=serial)()
	fastboot_steps.flash_image(partition_name="tos_a", file_name=n_img_path+"tos.img", unlock_dut=False, lock_dut=False, serial=serial)()
	fastboot_steps.flash_image(partition_name="bootloader_a", file_name=n_img_path+"bootloader_gr_mrb_b1", unlock_dut=False, lock_dut=False, serial=serial)()
	fastboot_steps.flash_image(partition_name="boot_a", file_name=n_img_path+"boot.img", unlock_dut=False, lock_dut=False, serial=serial)()
	fastboot_steps.command(command="--set-active=_a > ./temp/files/temp.txt 2>&1", serial=serial)()
	return_result = fastboot_utils.fastboot_command_result(file_name="./temp/files/temp.txt")
	if not return_result: raise Exception("The test result did not achieve the desired results")
	fastboot_steps.lock_device(serial=serial)()
	fastboot_steps.continue_to_adb(serial=serial)()
	time.sleep(60)
	local_steps.wait_for_adb(timeout = 300, serial=serial)()

	is_n_version_number = False
	n_version_number = re.split("-|\.", flash_files.split("/")[-1])[-2]
	return_result = os.popen("adb -s {0} shell getprop | grep ro.build.version.incremental".format(serial)).readlines()
	for line in return_result:
		if n_version_number in line: is_n_version_number = True
	if not is_n_version_number: raise Exception("The test result did not achieve the desired results")

	adb_steps.reboot(command = "fastboot", reboot_timeout = 300, serial = serial)()

	n_1_img_path = "./temp/image/n/flashfiles/"
	fastboot_steps.flash_image(partition_name="gpt", file_name=n_1_img_path+"gpt_gr_mrb_b1.bin", unlock_dut=True, lock_dut=False, serial=serial)()
	fastboot_steps.fastboot_erase_partition(partition_name="teedata", unlock_dut=False, lock_dut=False, reboot=False, serial=serial)()
	fastboot_steps.fastboot_erase_partition(partition_name="misc", unlock_dut=False, lock_dut=False, reboot=False, serial=serial)()
	fastboot_steps.fastboot_erase_partition(partition_name="persistent", unlock_dut=False, lock_dut=False, reboot=False, serial=serial)()
	fastboot_steps.fastboot_erase_partition(partition_name="metadata", unlock_dut=False, lock_dut=False, reboot=False, serial=serial)()
	fastboot_steps.format_partition(partition_name="data", unlock_dut=False, lock_dut=False, serial=serial)()
	fastboot_steps.format_partition(partition_name="config", unlock_dut=False, lock_dut=False, serial=serial)()
	fastboot_steps.flash_image(partition_name="system_b", file_name=n_1_img_path+"system.img", unlock_dut=False, lock_dut=False, serial=serial)()
	fastboot_steps.flash_image(partition_name="vendor_b", file_name=n_1_img_path+"vendor.img", unlock_dut=False, lock_dut=False, serial=serial)()
	fastboot_steps.flash_image(partition_name="vbmeta_b", file_name=n_1_img_path+"vbmeta.img", unlock_dut=False, lock_dut=False, serial=serial)()
	fastboot_steps.flash_image(partition_name="multiboot_b", file_name=n_1_img_path+"multiboot.img", unlock_dut=False, lock_dut=False, serial=serial)()
	fastboot_steps.flash_image(partition_name="tos_b", file_name=n_1_img_path+"tos.img", unlock_dut=False, lock_dut=False, serial=serial)()
	fastboot_steps.flash_image(partition_name="bootloader_b", file_name=n_1_img_path+"bootloader_gr_mrb_b1", unlock_dut=False, lock_dut=False, serial=serial)()
	fastboot_steps.flash_image(partition_name="boot_b", file_name=n_1_img_path+"boot.img", unlock_dut=False, lock_dut=False, serial=serial)()
	fastboot_steps.command(command="--set-active=_b > ./temp/files/temp.txt 2>&1", serial=serial)()
	return_result = fastboot_utils.fastboot_command_result(file_name="./temp/files/temp.txt")
	if not return_result: raise Exception("The test result did not achieve the desired results")
	fastboot_steps.lock_device(serial=serial)()
	fastboot_steps.continue_to_adb(serial=serial)()
	time.sleep(60)
	local_steps.wait_for_adb(timeout = 300, serial=serial)()

	is_n_1_version_number = False
	n_1_version_number = re.split("-|\.", n_1_flashfiles_zip_name)[-2]
	return_result = os.popen("adb -s {0} shell getprop | grep ro.build.version.incremental".format(serial)).readlines()
	for line in return_result:
		if n_1_version_number in line: is_n_1_version_number = True
	if not is_n_1_version_number: raise Exception("The test result did not achieve the desired results")

	adb_steps.reboot(command = "fastboot", reboot_timeout = 300, serial = serial)()

	fastboot_steps.unlock_device(serial=serial)()
	fastboot_steps.command(command="--set-active=_a > ./temp/files/temp.txt 2>&1", serial=serial)()
	return_result = fastboot_utils.fastboot_command_result(file_name="./temp/files/temp.txt")
	if not return_result: raise Exception("The test result did not achieve the desired results")
	fastboot_steps.lock_device(serial=serial)()
	fastboot_steps.continue_to_adb(serial=serial)()
	time.sleep(60)
	local_steps.wait_for_adb(timeout = 300, serial=serial)()

	is_n_version_number = False
	return_result = os.popen("adb -s {0} shell getprop | grep ro.build.version.incremental".format(serial)).readlines()
	for line in return_result:
		if n_version_number in line: is_n_version_number = True
	if not is_n_version_number: raise Exception("The test result did not achieve the desired results")

	fastboot_utils.to_fastboot_by_script(serial=serial)
	fastboot_steps.corrupt_esp_partition(reboot=False, serial=serial)()
	fastboot_utils.flash_bxt(zip_file=flash_files, serial=serial)
	os.system("sudo rm -rf ./temp")

except:
	fastboot_utils.to_fastboot_by_script(serial=serial)
	fastboot_steps.corrupt_esp_partition(reboot=False, serial=serial)()
	fastboot_utils.flash_bxt(zip_file=flash_files, serial=serial)
	os.system("sudo rm -rf ./temp")
	raise
##### test end #####