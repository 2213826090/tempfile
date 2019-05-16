#!/usr/bin/env python

##### imports #####
import os
import sys
import ConfigParser
from testlib.base.base_utils import get_args
from testlib.scripts.android.adb import adb_steps
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
	os.system("mkdir -p ./temp/files/flash ./temp/image/n ./temp/image/eb/user ./temp/image/eb/userdebug")
	fastboot_utils.download_flash_scripts()

	conf_url = "https://shstor001.sh.intel.com/artifactory/acs_test_artifacts/OTC_Android_Auto_Test_Suite/resources/EBImage/System_FastBoot/fastboot.conf"
	fastboot_utils.download_file(url=conf_url, local_filename="./temp/fastboot.conf")
	config = ConfigParser.ConfigParser()
	config.read("./temp/fastboot.conf")
	fastboot_path = config.get("fastboot", "path")

	platform_name = fastboot_utils.get_platform_name(serial=serial)
	if platform_name == "bxtp_abl":
		image_platform = "image_m_bxt"
		eb_user_patch = "eb_579061_user_flashfiles_zip"
		eb_userdebug_patch = "eb_579061_userdebug_flashfiles_zip"
	if platform_name == "gordon_peak":
		zip_name = fastboot_utils.get_zip_name(zip_path=flash_files)
		if zip_name == "r0_bxtp_abl":
			image_platform = "image_m_bxt"
			eb_user_patch = "eb_579061_user_flashfiles_zip"
			eb_userdebug_patch = "eb_579061_userdebug_flashfiles_zip"
		if zip_name == "gordon_peak":
			ram_value = fastboot_utils.get_bxt_ram(serial=serial)
			if ram_value == "2g": image_platform = "image_o_bxt_2g"
			if ram_value == "4g": image_platform = "image_o_bxt_4g"
			if ram_value == "8g": image_platform = "image_o_bxt_4g"
			eb_user_patch = None
			eb_userdebug_patch = "eb_615671_flashfiles_zip"

	if eb_user_patch != None:
		eb_user_patch_flashfiles_zip_path = config.get(image_platform, eb_user_patch)
		eb_user_patch_flashfiles_zip_name = eb_user_patch_flashfiles_zip_path.split("/")[-1]
		fastboot_utils.download_file(url=fastboot_path+eb_user_patch_flashfiles_zip_path, local_filename="./temp/image/eb/user/"+eb_user_patch_flashfiles_zip_name)

	eb_userdebug_patch_flashfiles_zip_path = config.get(image_platform, eb_userdebug_patch)
	eb_userdebug_patch_flashfiles_zip_name = eb_userdebug_patch_flashfiles_zip_path.split("/")[-1]
	fastboot_utils.download_file(url=fastboot_path+eb_userdebug_patch_flashfiles_zip_path, local_filename="./temp/image/eb/userdebug/"+eb_userdebug_patch_flashfiles_zip_name)

	if platform_name == "bxtp_abl":
		fastboot_utils.flash_bxt_m_fused(eb_user_patch_flashfiles_zip_name=eb_user_patch_flashfiles_zip_name, eb_userdebug_patch_flashfiles_zip_name=eb_userdebug_patch_flashfiles_zip_name, serial=serial)
		flashfiles_type = "user"
	if platform_name == "gordon_peak":
		zip_name = fastboot_utils.get_zip_name(zip_path=flash_files)
		if zip_name == "r0_bxtp_abl":
			fastboot_utils.flash_bxt_m_fused(eb_user_patch_flashfiles_zip_name=eb_user_patch_flashfiles_zip_name, eb_userdebug_patch_flashfiles_zip_name=eb_userdebug_patch_flashfiles_zip_name, serial=serial)
			flashfiles_type = "user"
		if zip_name == "gordon_peak":
			fastboot_utils.unpack_the_zip(file_name="./temp/image/eb/userdebug/"+eb_userdebug_patch_flashfiles_zip_name, temp_path=r"./temp/image/eb/userdebug/flashfiles")
			fastboot_utils.flash_bxt(zip_file="./temp/image/eb/userdebug/"+eb_userdebug_patch_flashfiles_zip_name, serial=serial)
			flashfiles_type = "userdebug"

	adb_steps.reboot(command="fastboot", reboot_timeout=300, serial=serial)()

	zip_name = fastboot_utils.get_zip_name(zip_path=flash_files)
	if zip_name == "r0_bxtp_abl":
		fastboot_steps.command(command="flashing unlock > ./temp/files/unlock_result.txt 2>&1", serial=serial)()
	fastboot_steps.command(command="flash {0} ./temp/image/eb/{3}/flashfiles/{1} > ./temp/files/{2}.txt 2>&1"\
		.format("gpt", "gpt_gr_mrb_b1.bin", "flash_gpt_result", flashfiles_type), serial=serial)()
	fastboot_steps.command(command="erase {0} > ./temp/files/{1}.txt 2>&1".format("teedata", "erase_teedata_result"), serial=serial)()
	fastboot_steps.command(command="erase {0} > ./temp/files/{1}.txt 2>&1".format("misc", "erase_misc_result"), serial=serial)()
	fastboot_steps.command(command="erase {0} > ./temp/files/{1}.txt 2>&1".format("persistent", "erase_persistent_result"), serial=serial)()
	fastboot_steps.command(command="erase {0} > ./temp/files/{1}.txt 2>&1".format("metadata", "erase_metadata_result"), serial=serial)()
	fastboot_steps.command(command="format {0} > ./temp/files/{1}.txt 2>&1".format("data", "format_data_result"), serial=serial)()
	fastboot_steps.command(command="format {0} > ./temp/files/{1}.txt 2>&1".format("config", "format_config_result"), serial=serial)()
	fastboot_steps.command(command="flash {0} ./temp/image/eb/{3}/flashfiles/{1} > ./temp/files/{2}.txt 2>&1"\
		.format("bootloader", "bootloader_gr_mrb_b1", "flash_bootloader_result", flashfiles_type), serial=serial)()
	fastboot_steps.command(command="flash {0} ./temp/image/eb/{3}/flashfiles/{1} > ./temp/files/{2}.txt 2>&1"\
		.format("multiboot", "multiboot.img", "flash_multiboot_result", flashfiles_type), serial=serial)()
	fastboot_steps.command(command="flash {0} ./temp/image/eb/{3}/flashfiles/{1} > ./temp/files/{2}.txt 2>&1"\
		.format("tos", "tos.img", "flash_tos_result", flashfiles_type), serial=serial)()
	fastboot_steps.command(command="flash {0} ./temp/image/eb/{3}/flashfiles/{1} > ./temp/files/{2}.txt 2>&1"\
		.format("vendor", "vendor.img", "flash_vendor_result", flashfiles_type), serial=serial)()
	fastboot_steps.command(command="flash {0} ./temp/image/eb/{3}/flashfiles/{1} > ./temp/files/{2}.txt 2>&1"\
		.format("boot", "boot.img", "flash_boot_result", flashfiles_type), serial=serial)()
	fastboot_steps.command(command="flash {0} ./temp/image/eb/{3}/flashfiles/{1} > ./temp/files/{2}.txt 2>&1"\
		.format("system", "system.img", "flash_system_result", flashfiles_type), serial=serial)()

	if zip_name == "r0_bxtp_abl":
		unlock_result = fastboot_utils.fastboot_command_result(file_name="./temp/files/unlock_result.txt")
	flash_gpt_result = fastboot_utils.fastboot_command_result(file_name="./temp/files/flash_gpt_result.txt")
	erase_teedata_result = fastboot_utils.fastboot_command_result(file_name="./temp/files/erase_teedata_result.txt")
	erase_misc_result = fastboot_utils.fastboot_command_result(file_name="./temp/files/erase_misc_result.txt")
	erase_persistent_result = fastboot_utils.fastboot_command_result(file_name="./temp/files/erase_persistent_result.txt")
	erase_metadata_result = fastboot_utils.fastboot_command_result(file_name="./temp/files/erase_metadata_result.txt")
	format_config_result = fastboot_utils.fastboot_command_result(file_name="./temp/files/format_config_result.txt")
	format_data_result = fastboot_utils.fastboot_command_result(file_name="./temp/files/format_data_result.txt")
	flash_bootloader_result = fastboot_utils.fastboot_command_result(file_name="./temp/files/flash_bootloader_result.txt")
	flash_multiboot_result = fastboot_utils.fastboot_command_result(file_name="./temp/files/flash_multiboot_result.txt")
	flash_tos_result = fastboot_utils.fastboot_command_result(file_name="./temp/files/flash_tos_result.txt")
	flash_vendor_result = fastboot_utils.fastboot_command_result(file_name="./temp/files/flash_vendor_result.txt")
	flash_boot_result = fastboot_utils.fastboot_command_result(file_name="./temp/files/flash_boot_result.txt")
	flash_system_result = fastboot_utils.fastboot_command_result(file_name="./temp/files/flash_system_result.txt")

	if zip_name == "r0_bxtp_abl":
		if unlock_result:
			raise Exception("The test result did not achieve the desired results")

	if flash_gpt_result or erase_teedata_result or erase_misc_result or erase_persistent_result or erase_metadata_result or\
			format_config_result or format_data_result or flash_bootloader_result or flash_multiboot_result or\
			flash_tos_result or flash_vendor_result or flash_boot_result or flash_system_result:
		raise Exception("The test result did not achieve the desired results")

	fastboot_utils.flash_bxt(zip_file=flash_files, serial=serial)
	os.system("sudo rm -rf ./temp")

except:
	fastboot_utils.flash_bxt(zip_file=flash_files, serial=serial)
	os.system("sudo rm -rf ./temp")
	raise
##### test end #####