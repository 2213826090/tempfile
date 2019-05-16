#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related
to the source code ("Material") are owned by Intel Corporation or its
suppliers or licensors. Title to the Material remains with Intel Corporation
or its suppliers and licensors. The Material contains trade secrets and
proprietary and confidential information of Intel or its suppliers and
licensors.

The Material is protected by worldwide copyright and trade secret laws and
treaty provisions. No part of the Material may be used, copied, reproduced,
modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual
property right is granted to or conferred upon you by disclosure or delivery
of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

@organization: INTEL MCG PSI
@summary: Pupdr Library - FlashModule
@since: 11/17/2014
@author: travenex
"""

import os
import re
import time
import json
import hashlib
import urllib2
import shutil
import xml.etree.ElementTree as ET
import tempfile
import LoggerModule
import HostModule
import RelayCardModule
import DeviceModule
import WorkaroundModule
import MiscModule
import OsManagerModule
import FlashFileModule
import LogsModule
import EfiVarModule
import DediprogModule
import OutputModule
import CampaignModule
import ConfigurationModule
import DownloadModule

class FlashModule(object):

    __instance = None
    __globalConf = None
    __logger = None
    __host = None
    __relayCard = None
    __device = None
    __workaround = None
    __getEndTime = None
    __misc = None
    __osManager = None
    __flashFile = None
    __logs = None
    __efiVar = None
    __dediprog = None
    __output = None
    __configuration = None
    __campaign = None
    __info = None
    __build_target = None
    __block_osip = None
    __flash_key = None
    __block_gpt = None
    __already_flashed_bios = None
    __user_flashing = None
    __download = None
    pupdrTempDirectory = None
    fileMatchingTable = None

    def __new__(cls):
        """
        Constructor that always returns the same instance
        """
        if not cls.__instance:
            cls.__instance = object.__new__(cls)
        return cls.__instance

    def init(self, globalConf=None):
        self.__globalConf = globalConf
        self.__logger = LoggerModule.LoggerModule()
        self.__host = HostModule.HostModule()
        self.__relayCard = RelayCardModule.RelayCardModule()
        self.__device = DeviceModule.DeviceModule()
        self.__workaround = WorkaroundModule.WorkaroundModule()
        self.__getEndTime = None
        self.__misc = MiscModule.MiscModule()
        self.__osManager = OsManagerModule.OsManagerModule()
        self.__flashFile = FlashFileModule.FlashFileModule()
        self.__logs = LogsModule.LogsModule()
        self.__efiVar = EfiVarModule.EfiVarModule()
        self.__dediprog = DediprogModule.DediprogModule()
        self.__output = OutputModule.OutputModule()
        self.__configuration = ConfigurationModule.ConfigurationModule()
        self.__campaign = CampaignModule.CampaignModule()
        self.__download = DownloadModule.DownloadModule()
        self.__info = ""
        self.__build_target = None
        self.__block_osip = ""
        self.__block_gpt = ""
        self.__flash_key = ""
        self.__already_flashed_bios = False
        self.__user_flashing = False
        # empty pupdr output zip directory if created by previous tests:
        output_directory_ok = False
        index = -1
        while index < 5 and not output_directory_ok:
            try:
                self.pupdrTempDirectory = os.path.join(tempfile.gettempdir(), "bootota_{1}{0}".format("_" + str(index) if index >= 0 else "", self.__misc.getUser()))
                if os.path.isdir(self.pupdrTempDirectory):
                    shutil.rmtree(self.pupdrTempDirectory)
            except Exception as e:
                index += 1
                self.__logger.printLog("WARNING", "failure to erase directory because written by another user: {0} "
                                                  "(error={1})".format(self.pupdrTempDirectory, e))
                continue
            else:
                output_directory_ok = True
                os.mkdir(self.pupdrTempDirectory)
                self.__misc.local_files_handler.addEntry(self.pupdrTempDirectory)
        if not output_directory_ok:
            self.pupdrTempDirectory = os.path.join(tempfile.gettempdir(), "pupdr_out")
            self.__logger.printLog("INFO", "failure to clean directory, this may lead to disk storage issue. "
                                           "Please manually erase directories {0}*".format(self.pupdrTempDirectory))
            self.__misc.local_files_handler.addEntry(self.pupdrTempDirectory)
        self.fileMatchingTable = dict()

    def flashBios(self, flash_file):
        log = "flashBios(): "
        verdict = True
        output = ""
        if not self.__configuration.flash.DEDIPROG:
            output = "cannot dediprog board, configuration parameter is False"
            verdict = False
        else:
            ifwi_size = 0
            start_os = "offline"
            ifwi_regex = ".*IFWI.*bin|ifwi\.bin"
            if isinstance(self.__configuration.flash.DEDIPROG, dict):
                ifwi_size = self.__configuration.flash.DEDIPROG.get("ifwi_size", 0)
                start_os = self.__configuration.flash.DEDIPROG.get("start_os", "offline")
                ifwi_regex = self.__configuration.flash.DEDIPROG.get("ifwi_regex", ".*IFWI.*bin|ifwi\.bin")

            self.__logger.printLog("INFO", log + "start (ifwi_size={0}, start_os={1}, ifwi_regex={2})".format(ifwi_size, start_os, ifwi_regex))
            local_flash_file = self.__flashFile.createFile(flash_file)
            file_list = self.getZipContentList(local_flash_file.get_localFlashFile(), ifwi_regex)
            stage_2 = None
            # If no bios found
            if file_list is None or not file_list:
                # try to check directly in BUT:
                pft_location = local_flash_file.get_pft_json()
                isBUT = "/BUT/" in pft_location
                if not isBUT:
                    output = "cannot find BUT directory and no BIOS files in {0}".format(local_flash_file.get_localFlashFile())
                    verdict = False
                else:
                    BUT_location = os.path.join(pft_location.split("BUT")[0], "BUT")
                    self.__logger.printLog("INFO", log + "searching for ifwi files in {0}".format(BUT_location))
                    stage_1_path = os.path.join(BUT_location, "stage1_spi.bin")
                    stage_2_path = os.path.join(BUT_location, "stage2_emmc.bin")
                    if os.path.isfile(stage_1_path) and os.path.isfile(stage_2_path):
                        file_list = [stage_1_path]
                        stage_2 = stage_2_path
                        self.__logger.printLog("DEBUG", log + "found stage 1 {0}".format(stage_1_path))
                        self.__logger.printLog("DEBUG", log + "found stage 2 {0}".format(stage_2_path))
                    else:
                        self.__logger.printLog("WARNING", log + "one or all of {0} and {1} not found "
                                                                "(BUT content = {2})".format(stage_1_path, stage_2_path,
                                                                                             ", ".join(os.listdir(BUT_location))))
                        output = "issue while looking for stage1 and stage2 in BUT directory"
                        verdict = False
            if file_list:
                self.__logger.printLog("INFO", log + "Flashing Stage 1")
                # In case we are switching from edk2 to fdk, PKPON = 10s and it won't shutdown the board at it is in edk2.
                if start_os == "offline" and not self.__osManager.forceShutdown(delay=12):
                    output = "failure to force shutdown before BIOS flashing"
                    verdict = False
                elif start_os == "dnx":
                    self.__osManager.forceShutdown()
                    self.__osManager.comboBoot(self.__relayCard.relayConfiguration["VolumeUp"], self.__relayCard.relayConfiguration["VolumeDown"], delay=5, wait_dnx=True)
                if verdict:
                    # plug and replug dediprog if rework is done
                    self.__relayCard.replugDediprog()
                    if not self.dediprog(file_list[0], ifwi_size):
                        self.__logger.printLog("WARNING", "Failure to flash BIOS {0} - Retry once".format(file_list[0]))
                        self.__relayCard.replugDediprog()
                        if not self.dediprog(file_list[0], ifwi_size):
                            output = "Failed to flash BIOS {0} - Retry also failed".format(file_list[0])
                            verdict = False

            # eventually flash stage2
            if stage_2:
                self.__logger.printLog("INFO", log + "Flashing Stage 2")
                if not self.__osManager.gotoDnx():
                    output = "failure to go to DNX or BIOS-recovery after stage1 flashing"
                    verdict = False
                else:
                    self.__relayCard.enableLine(self.__relayCard.relayConfiguration["VolumeUp"])
                    self.__relayCard.enableLine(self.__relayCard.relayConfiguration["VolumeDown"])
                    self.__host.commandExec("fastboot flash fw_stage2 {0}".format(stage_2))
                    self.__misc.waitDelay("waiting for stage2 flashing then DNX boot", 60)
                    self.__osManager.waitOs("dnx", 120)
                    self.__relayCard.disableLine(self.__relayCard.relayConfiguration["VolumeUp"])
                    self.__relayCard.disableLine(self.__relayCard.relayConfiguration["VolumeDown"])

        self.__output.appendOutput(output, verdict)
        return verdict

    def flashIfwiBxtp(self, flash_file):
        log = "flashIfwiBxtp(): "
        verdict = True

        self.__logger.printLog("INFO", log + "Create the path to the ABL file")
        local_flash_file = self.__flashFile.createFile(flash_file)
        ifwi_filepath = local_flash_file.get_localFlashFile()
        ifwi_filepath = ifwi_filepath.replace(".zip", "")

        self.__logger.printLog("INFO", log + "Launch ABL flashing")
        if self.__configuration.flash.BXTP_ABL_BUILD == "gr_mrb_b1":
            ifwi_file = os.path.join(ifwi_filepath, "ifwi_gr_mrb_b1.bin")
        else:
            ifwi_file = os.path.join(ifwi_filepath, "ifwi_gr_mrb.bin")
        exec_cmd = "/opt/intel/platformflashtool/bin/ias-spi-programmer --write " + ifwi_file
        exec_status = None
        counter = 0
        while exec_status != 0 and counter < 3:
            if not self.__osManager.gotoFastbootBxtpMrb():
                output = "fail to entry fastboot for MRB board - Android build - " \
                         "before ABL flashing - Try number: " + str(counter + 1)
            else:
                exec_status, output_cmd = self.__host.commandExec(exec_cmd, timeout=600)
            counter += 1

        if exec_status == 0:
            output = "BXTP IFWI flashing succesfull"
        else:
            output = "BXTP IFWI flashing error"
            verdict = False

        self.__output.appendOutput(output, verdict)
        return verdict

    def flashIocBxtp(self, flash_file):
        log = "flashIocBxtp(): "
        verdict = True

        self.__logger.printLog("INFO", log + "Create the path to the ABL file")
        local_flash_file = self.__flashFile.createFile(flash_file)
        ioc_filepath = local_flash_file.get_localFlashFile()
        ioc_filepath = ioc_filepath.replace(".zip", "")

        self.__logger.printLog("INFO", log + "Launch ABL flashing")
        if self.__configuration.flash.BXTP_ABL_BUILD == "gr_mrb_b1":
            ioc_file = os.path.join(ioc_filepath, "ioc_firmware_gp_mrb_fab_d_slcan.ias_ioc")
        else:
            ioc_file = os.path.join(ioc_filepath, "ioc_firmware_gp_mrb_fab_c_slcan.ias_ioc")
        exec_cmd = "/opt/intel/platformflashtool/bin/ioc_flash_server_app -s /dev/ttySerial2 -grfabc -t " + ioc_file
        exec_status = None
        counter = 0
        while exec_status != 0 and counter < 3:
            if not self.__osManager.gotoDebugBxtpMrb():
                output = "fail to entry debug mode for MRB board - Android build - " \
                         "before ABL flashing - Try number: " + str(counter + 1)
            else:
                exec_status, output_cmd = self.__host.commandExec(exec_cmd, timeout=600)
            counter += 1

        if exec_status == 0:
            output = "BXTP IOC flashing succesfull"
        else:
            output = "BXTP IOC flashing error"
            verdict = False

        self.__output.appendOutput(output, verdict)
        return verdict

    # Inner flash method
    def __runFlash(self, flash_file, force, forced_configuration=""):
        self.__logger.printLog("INFO", self.__info + "start flashing:\n"
                                                     "{0}".format(json.dumps(flash_file,
                                                                             sort_keys=True,
                                                                             indent=4)))
        no_reboot = ""
        failure = False
        output = ""
        local_flash_file = self.__flashFile.createFile(flash_file)

        # get timeout for cflasher command
        pft_flash_file = local_flash_file.get_pft_flash_file()
        pft_timeout = self.getCflasherTimeout(local_flash_file.get_localFlashFile(), flash_xml=pft_flash_file, config=local_flash_file.get_flashConfiguration())

        # force is True, board will put in right state before sending PFT command
        if force and local_flash_file.get_flashConfigurationStartState() in ("dnx_fw", "dnx_os", "dnx", "dnx_pos") \
                and "bxtp" not in self.__configuration.board:
            # force PFT not to launch reboot commands
            no_reboot = " --reboot 0"

             # dediprog devices if possible and 'flashBIOS' set to true in BenchConf
            if self.__configuration.flash.DEDIPROG:
                if not self.__already_flashed_bios:
                    if self.__globalConf.get("FLASH_BIOS") and not self.flashBios(flash_file):
                        failure = True
                        output = "failure to dediprog BIOS"
                    else:
                        self.__already_flashed_bios = True
                if not failure:
                    if self.__configuration.boot.CSE_DNX_BOOT_WITH_COMBO and local_flash_file.get_flashConfigurationStartState() == "dnx_fw":
                        if not self.__osManager.gotoCseDnx():
                            failure = True
                            output = "failure to force CSE DnX"
                    else:
                        if not self.__osManager.gotoDnx():
                            failure = True
                            output = "failure to force DnX"
            # Else DnX boots automatically, run thread to boot DnX during cflasher
            else:
                # get OS state for info, then always force shutdown the DUT to ensure OFF state
                # and avoid corner case where board is rebooting
                verdict = True
                current_Os = self.__osManager.getOs(check_charger=False)

                if self.__configuration.boot.DNX_BOOT_ON_POWER_SUPPLY_INSERT:
                    # full fugu behavior
                    if current_Os not in ("offline", "fastboot"):
                        if self.__osManager.gracefulShutdown(plug=True, intent=True):
                            settle_down = 20
                            self.__misc.waitDelay("wait settle down duration", settle_down)
                        else:
                            self.__logger.printLog("WARNING", "failure to shutdown, trying hard shutdown before powering off")
                            self.__osManager.forceShutdown(plug=True)
                    else:
                        self.__osManager.forceShutdown(plug=True)
                    self.__relayCard.powerSupply(False)
                    capacity_out = 30
                    self.__misc.waitDelay("wait until board capacitors are empty", capacity_out)
                else:
                    if self.__configuration.boot.BOOT_ON_FORCED_SHUTDOWN:
                        # if board boots on forced shutdown, force shutdown anyway in case board is frozen
                        self.__osManager.forceShutdown(plug=True)
                        local_timeout = self.__configuration.timeout.MOS_BOOT
                        self.__misc.waitDelay("check boot of any OS", local_timeout)
                        if self.__osManager.getOs() != "offline":
                            # if board booted, force shutdown again to power_off the board
                            verdict = self.__osManager.forceShutdown(plug=False)
                        else:
                            self.__relayCard.usbConnection(False)
                    else:
                        verdict = self.__osManager.forceShutdown(plug=False)

                if not verdict:
                    output = "fail to force shutdown"
                    failure = True
                else:
                    # run parallel process to boot the board in COS during PFT process attempting to catch DNX
                    if self.__configuration.boot.DNX_BOOT_ON_POWER_SUPPLY_INSERT:
                        # for full_fugu, replug power supply
                        self.__relayCard.runThread(["time.sleep(7)",
                                                    "self.powerSupply(True)"])
                    elif "force_shutdown_during_PFT_execution" in self.__configuration.flash.FORCE_FLASH_ALGO:
                        self.__relayCard.usbConnection(False)
                        self.__relayCard.runThread(["time.sleep(15)",
                                                    "self.powerButtonPress(self.configuration.timeout.PKPOFF)",
                                                    "self.usbConnection(True)"])
                    elif "pf450cl" in self.__configuration.board:
                        # press down button to force dnx boot for flying fish device
                        self.__relayCard.usbConnection(False)
                        self.__relayCard.runThread(["time.sleep(10)",
                                                    "self.enableLine(self.relayConfiguration['VolumeDown'])",
                                                    "self.powerButtonPress(self.configuration.timeout.PKPON)",
                                                    "self.usbConnection(True)",
                                                    "self.disableLine(self.relayConfiguration['VolumeDown'])"])
                    else:
                        # for other boards, booting the board with charger insertion
                        self.__relayCard.runThread(["self.usbConnection(False)",
                                                    "time.sleep(7)",
                                                    "self.usbConnection(True)"])

        # Else if bxtp special fastboot entry
        elif "bxtp" in self.__configuration.board and not local_flash_file.get_flashConfigurationStartState() == "mos":
            if "lh" in self.__configuration.board:
                if not self.__osManager.gotoFastbootBxtpLh():
                    failure = True
                    output = "fail to entry fastboot for LH board"
            elif "yocto" in self.__configuration.branch:
                if not self.__osManager.gotoFastbootBxtpMrbYocto():
                    failure = True
                    output = "fail to entry fastboot for MRB Board - Yocto build"
            elif "mrb" or "car" in self.__configuration.board:
                if self.__configuration.flash.BXTP_FLASH_IOC:
                    if not self.flashIocBxtp(flash_file):
                        failure = True
                        output = "failure to flash IOC"
                if not self.flashIfwiBxtp(flash_file):
                    failure = True
                    output = "failure to flash ABL"
                else:
                    if not self.__osManager.gotoFastbootBxtpMrb():
                        failure = True
                        output = "fail to entry fastboot for MRB board - Android build - " \
                                 "after ABL flashing and before flash start"
            else:
                failure = True
                output = "wrong BXTP board configuration"

        # Else fastboot
        elif force and local_flash_file.get_flashConfigurationStartState() == "pos":
            if not self.__osManager.gotoPos():
                failure = True
                output = "fail to reboot in POS"
            else:
                no_reboot = " --reboot 0"

        # Else mos
        elif force and local_flash_file.get_flashConfigurationStartState() == "mos":
            if not self.__osManager.gotoMos():
                failure = True
                output = "failed to boot in MOS"
            else:
                no_reboot = " --reboot 0"

        # Else let cflasher manage the board
        else:
            self.__logger.printLog("INFO", self.__info + "not forcing board initial state and letting PFT handling it")

        if failure:
            self.__logger.printLog("WARNING", self.__info + "failure to set device in proper start state")
            verdict = False
        else:
            if forced_configuration:
                self.__logger.printLog("INFO", self.__info + "forcing '{0}' configuration instead of '{1}'".format(forced_configuration, local_flash_file.get_flashConfiguration()))
                config = " -c " + forced_configuration
            elif local_flash_file.get_flashConfiguration() != "":
                config = " -c " + local_flash_file.get_flashConfiguration()
            else:
                config = ""

            if local_flash_file.get_pft_flash_file() != "":
                pft_file = " -x " + local_flash_file.get_pft_flash_file()
            else:
                pft_file = ""

            if self.__host.serial_number:
                device_ssn = " --os-sn " + self.__host.serial_number
            else:
                device_ssn = ""
            if self.__osManager.dnx_enumeration:
                device_ssn += " --soc-sn " + self.__osManager.dnx_enumeration
            elif self.__host.dnx_ssn:
                device_ssn += " --soc-sn " + self.__host.dnx_ssn

            if local_flash_file.get_flash_group_state():
                enable_group = ""
                disable_group = ""
                for element in local_flash_file.get_flash_group_state():
                    if local_flash_file.get_flash_group_state()[element]:
                        enable_group += " --enable-group " + element
                    else:
                        disable_group += " --disable-group " + element
            else:
                enable_group = ""
                disable_group = ""

            all_pft_options = "-f " + local_flash_file.get_localFlashFile() + no_reboot + device_ssn + config + pft_file + enable_group + disable_group

            # Execute cflasher
            exec_status, output = self.__host.commandExec("cflasher -l 5 " + all_pft_options + " -t", pft_timeout)

            # get PFT duration data and store in json
            duration = 0
            search = re.search("\(duration=([0-9]+:[0-9]+:[0-9]+\.[0-9]+)\)", output)
            if search:
                out = search.group(1)
                out = out.split(":")
                if len(out) == 3:
                    try:
                        duration = float(out[2]) + 60 * int(out[1]) + 3600 * int(out[0])
                    except Exception as e:
                        self.__logger.printLog("WARNING", self.__info + "impossible to get PFT duration from: {0} ({1})".format(":".join(out), e))
            if duration:
                self.__campaign.Campaign_information.updateJsonWithPftTimeData(duration,
                                                                               os.path.basename(local_flash_file.get_localFlashFile()),
                                                                               forced_configuration if forced_configuration else local_flash_file.get_flashConfiguration(),
                                                                               local_flash_file.get_pft_flash_file(),
                                                                               self.__build_target.get("tag"),
                                                                               self.__flash_key)
                self.__campaign.Campaign_information.TCR_data_handler.addMetric(
                    "BOOTOTA_PFT_FLASH_{0}".format(local_flash_file.get_file_type().upper()),
                    duration,
                    "second")

            # If cflasher fails
            if exec_status != 0:
                # get cflasher error code
                try:
                    code = re.search("Flash Tool exited with code ([0-9]*)", output.splitlines()[-1]).group(1)
                    error_code_content = self.__host.commandExec("cflasher --get-error-code {0}".format(code), 5)[1]
                except Exception as e:
                    code = ""
                    error_code_content = "unknown error"
                    self.__logger.printLog("WARNING", "failure to get PFT error code ({})".format(e))
                # try to get fail command:
                failures = re.findall("command (.*) failed", output, re.I)
                if failures:
                    failed_cmd = failures[-1].replace("\"", "").replace("`", "")
                    # reduce log length
                    if len(failed_cmd) > 50:
                        failed_cmd = failed_cmd[:50] + "..."
                else:
                    failed_cmd = ""
                output = "failure to flash{2} (error-code {0}='{1}')".format(code, error_code_content, " because '{0}' command failed ".format(failed_cmd) if failed_cmd else "")
                verdict = False
            else:
                output = "flashing successful"
                verdict =  True

            if self.__user_flashing and \
                    self.__configuration.boot.ADB_WITH_USER_BUILD == "activate_with_fastboot_commands_after_blankphone" and \
                    local_flash_file.get_file_type() == "blankphone":
                self.__logger.printLog("INFO", "launching set of fastboot commands to enable adb once flashing is done")
                self.__host.commandExecFastboot("oem unlock", timeout=60)
                time.sleep(3)
                adb_key_file = os.path.join(os.path.dirname(__file__), "Extras", "adb_keys")
                self.__host.commandExecFastboot("flash adb_keys {0}".format(adb_key_file), timeout=60)
                time.sleep(3)
                self.__host.commandExecFastboot("oem verified", timeout=60)

        argument = "zip={0}".format(os.path.basename(local_flash_file.get_localFlashFile()))
        if local_flash_file.get_pft_flash_file():
            argument += ", file={0}".format(local_flash_file.get_pft_flash_file())
        if local_flash_file.get_flashConfiguration() or forced_configuration:
            argument += ", config={0}".format(forced_configuration if forced_configuration else local_flash_file.get_flashConfiguration())
        self.__output.appendOutput(output, verdict, argument=argument)
        return verdict

    def flash(self, build, key,
              check_tag=True,
              check_ifwi=True,
              check_modem=True,
              check_img=True,
              check_ssn=True,
              adb_after_flashing=True):
        """ Method to flash a zip flash file.
            If check is True, build must contain blankphone, fastboot and tag.
        """
        self.__info = "flash({0}): ".format(key)
        self.__logger.printLog("INFO", self.__info + "start")
        self.__logger.printLog("DEBUG", self.__info + "build={0}".format(str(build["flash_list"])))
        self.__build_target = build
        self.__flash_key = key
        self.__already_flashed_bios = False
        self.__user_flashing = self.__globalConf.get("PROVISIONING_DATA", {}).get("build_variant", "") == "user"
        output = ""
        verdict = True
        allowed_keys = ("blank", "fastboot", "ota", "fota", "force", "recover", "update")

        # Check the keys
        if any (k not in allowed_keys for k in key.split(";")):
            output = self.__info + "key '{0}' not in list of allowed keys " \
                                   "({1})".format(str(key), ", ".join(allowed_keys))
            verdict = False

        # Check if force flashing requested
        force = False
        if "force" in key.split(";"):
            force = True

        if "update" in key:
            forced_configuration = self.__configuration.flash.UPDATE_CONFIGURATION
            if forced_configuration:
                forced_configuration = forced_configuration.replace(
                    "board_id",
                    re.sub(self.__configuration.download.MULTI_VARIANT_PREFIX, "", self.__configuration.board))
        else:
            forced_configuration = ""

        # Read SSN
        if "ssn" not in build or ("ssn" in build and not build["ssn"]):
            self.__logger.printLog("INFO", self.__info + "try to read ro.serialno")
            build["ssn"] = self.__device.getProperty("ro.serialno")

        if "pre_flash_configuration" in build:
            self.__configuration.switchConfig(build["pre_flash_configuration"])

        # external command handling
        if verdict and self.__globalConf.get("CUSTOM_CONFIG", {}).get("extra_flash_cmd_list"):
            cmd_list = self.__globalConf["CUSTOM_CONFIG"]["extra_flash_cmd_list"]
            if not isinstance(cmd_list, list) or any(not isinstance(single_cmd, dict) for single_cmd in cmd_list):
                self.__logger.printLog("WARNING", self.__info + "invalid command list, list of dictionaries expected")
            else:
                self.__logger.printLog("INFO", self.__info + "extra command list detected:\n" +
                                       json.dumps(cmd_list))
                for index, single_cmd in enumerate(cmd_list):
                    cmd = single_cmd.get("cmd", "")
                    mandatory = single_cmd.get("mandatory", True)
                    position = single_cmd.get("position", "post-flash")
                    if position != "pre-flash":
                        self.__logger.printLog("DEBUG", self.__info + "skipping command "
                                                                     "{}/{}".format(index + 1, len(cmd_list)))
                        continue
                    exec_status, cmd_output = self.__host.commandExec(cmd, 200)
                    if not mandatory:
                        self.__logger.printLog("DEBUG", self.__info + "skipping command output check as not mandatory")
                        continue
                    if exec_status != 0:
                        verdict = False
                        output = "failure to execute command"
                        break
                    elif "/system" in cmd_output or "No such file or directory" in cmd_output:
                        verdict = False
                        output = "failure with adb command"
                        break

        # Execute flashing
        for flash_file in build["flash_list"]:
            if not verdict:
                pass
            elif re.search("blank|recover", key) and flash_file["file_type"] in ("ota", "fota"):
                # skip ota flash if blank requested
                pass
            elif re.search("update", key) and flash_file["file_type"] in ("blankphone", "ota", "fota", "firmware"):
                # only flash fastboot file if requested
                pass
            elif "ota" in key.split(";") and flash_file["file_type"] != "ota":
                # only flash ota if requested
                pass
            elif "fota" in key.split(";") and flash_file["file_type"] != "fota":
                # only flash fota if requested
                pass
            else:
                verdict = self.__runFlash(flash_file, force, forced_configuration=forced_configuration)
                if not verdict:
                    output = "failure to flash build"
                    break
                time.sleep(5)

        if not verdict:
            self.__output.appendOutput(output, False, argument="key={0}".format(key))
            return False
        #
        # WAIT boot completes
        #
        output = ""
        start = time.time()
        expected_os = "main"
        # if capsule exist, wait longer
        if self.__configuration.flash.DEDIPROG:
            capsule = 300
        else:
            capsule = 0

        if key in ("ota", "fota"):
            # if cflasher is successful: confirm popup low battery
            self.__logger.printLog("INFO", "in doubt confirm in popup if there")
            if not self.__osManager.waitOs("offline", timeout=60, waitWdExp=False):
                self.confirmOtaUpdate()
            # wait for ROS for update
            if not self.__osManager.waitOs("offline|recovery",
                                         timeout=150,
                                         blind=True):
                verdict = False
                output = "failure to boot recovery after ota update"
            # then wait main
            timeout = 1200 + capsule
        else:
            timeout = self.__configuration.timeout.BOOT_TIMEOUT + capsule

        if self.__user_flashing:
            timeout += 300

        # if configuration is changed after flashing
        if "change_configuration" in build:
            self.__configuration.switchConfig(build["change_configuration"])

        # If the build is a user variant there is no adb. Skip all the check but usb enumeration
        if self.__user_flashing and verdict and not (self.__configuration.boot.ADB_WITH_USER_BUILD and adb_after_flashing):
            verdict = self.__osManager.waitLsusbEnumeration(["8087:0a20", "8087:0a5e", "8087:0a15", "8087:0a5f", "8087:092a"], timeout)
            if verdict:
                output = "board detected in list of USB devices - no adb"
            else:
                output = "board was not detected on USB after flashing"
            self.__output.appendOutput(output, verdict, argument="key={0}".format(key))
            return verdict

        if "bxtp" in self.__configuration.board and not key in ("ota", "fota"):
            if "lh" in self.__configuration.board:
                if not self.__osManager.rebootAfterFlashBxtpLh():
                    verdict = False
                    output = "fail to reboot after flash for LH board"
            elif "yocto" in self.__configuration.branch:
                if not self.__osManager.rebootAfterFlashBxtpMrbYocto():
                    verdict = False
                    output = "Fail to reboot MRB board Yocto build after flash"
            elif "mrb" or "car" in self.__configuration.board:
                if not self.__osManager.rebootBxtpMrb():
                    verdict = False
                    output = "fail to reboot after flash for MRB board Android build"
            else:
                verdict = False
                output = "wrong BXTP board configuration"

        # Blind wait for boot of os (timeout value from start)
        elif verdict and not self.__osManager.waitOs(expected_os,
                                                   timeout=timeout,
                                                   start=start,
                                                   blind=True,
                                                   waitWdExp=False):
            if key in ("ota", "fota"):
                current_Os = self.__osManager.getOs()
                if current_Os == "recovery":
                    self.__logs.pull("/tmp/recovery.log", target_path="")
                elif "main" in current_Os:
                    self.__logs.pull("/cache/recovery/last_log", target_path="")
            verdict = False
            output = "failure to fully boot MOS after flashing"

        # accept EULA
        if verdict and self.__configuration.boot.ACCEPT_EULA:
            verdict = self.__osManager.acceptEula()
            if not verdict:
                output = "failure to accept EULA after MOS boot"

        # Imei flashing
        if verdict and self.__globalConf.get("CUSTOM_CONFIG", {}).get("imei_config", {}).get("imei") \
                and key not in ("ota", "fota"):
            if "sofia_imei_flashing" in self.__configuration.flash.FORCE_FLASH_ALGO:
                if len(self.__globalConf["CUSTOM_CONFIG"]["imei_config"]["imei"].split(";")) > 2:
                    verdict = False
                    output = "invalid IMEI input {} (expected: '<IMEI>' or '<IMEI1>;<IMEI2>')"
                elif self.__user_flashing:
                    self.__logger.printLog("DEBUG", self.__info + "IMEI not to be flashed on USER build")
                elif not self.imeiFlashing():
                    verdict = False
                    output = "failure provisioning imei"
            else:
                self.__logger.printLog("DEBUG", self.__info + "IMEI provided but flashing not supported on {} "
                                                              "board".format(self.__configuration.board))

        default_ssn = self.__device.getProperty("ro.serialno")

        # SSN flashing
        if verdict and self.__globalConf.get("CUSTOM_CONFIG", {}).get("ssn_provisioning", ""):
            if "sofia_ssn_flashing" in self.__configuration.flash.FORCE_FLASH_ALGO:
                if self.__user_flashing:
                    self.__logger.printLog("DEBUG", self.__info + "SSN not to be provisioned on USER build")
                elif not self.ssnFlashing():
                    verdict = False
                    output = "failure provisioning ssn"
                if self.__download.getBranchFromTag(
                    str(self.__download.build_but)) == "m_mr1" and default_ssn == self.__device.getProperty("ro.serialno"):
                    if self.__user_flashing:
                        self.__logger.printLog("DEBUG", self.__info + "SSN not to be provisioned on USER build")
                    elif not self.ssnViaImeiFlashing():
                        verdict = False
                        output = "failure provisioning ssn via IMEI"
            else:
                self.__logger.printLog("DEBUG", self.__info + "SSN provided but flashing not supported on {} "
                                                              "board".format(self.__configuration.board))

        # external command handling
        if verdict and self.__globalConf.get("CUSTOM_CONFIG", {}).get("extra_flash_cmd_list"):
            cmd_list = self.__globalConf["CUSTOM_CONFIG"]["extra_flash_cmd_list"]
            if not isinstance(cmd_list, list) or any(not isinstance(single_cmd, dict) for single_cmd in cmd_list):
                self.__logger.printLog("WARNING", self.__info + "invalid command list, list of dictionaries expected")
            else:
                self.__logger.printLog("INFO", self.__info + "extra command list detected:\n" +
                                       json.dumps(cmd_list))
                for index, single_cmd in enumerate(cmd_list):
                    cmd = single_cmd.get("cmd", "")
                    mandatory = single_cmd.get("mandatory", True)
                    position = single_cmd.get("position", "post-flash")
                    if position != "post-flash":
                        self.__logger.printLog("DEBUG", self.__info + "skipping command "
                                                                     "{}/{}".format(index + 1, len(cmd_list)))
                        continue
                    exec_status, cmd_output = self.__host.commandExec(cmd, 200)
                    if not mandatory:
                        self.__logger.printLog("DEBUG", self.__info + "skipping command output check as not mandatory")
                        continue
                    if exec_status != 0:
                        verdict = False
                        output = "failure to execute command"
                        break
                    elif "/system" in cmd_output or "No such file or directory" in cmd_output:
                        verdict = False
                        output = "failure with adb command"
                        break

        # store data (system size and time to first boot)
        if verdict:
            system_data = self.__device.getPartitionInformation("system")
            self.__campaign.Campaign_information.updateJsonWithSystemData(self.__build_target.get("tag"),
                                                                          self.__flash_key,
                                                                          system_data)
            for element in system_data:
                unit = "o"
                value = system_data[element]
                if value.endswith("M"):
                    value = value.rstrip("M")
                    unit = "M"
                if value.endswith("G"):
                    value = value.rstrip("G")
                    unit = "G"
                if value.endswith("K"):
                    value = value.rstrip("K")
                    unit = "K"
                try:
                    value = float(value)
                except Exception as e:
                    self.__logger.printLog("WARNING", "failure to convert '{0}' into float (error={1})".format(value, e))
                    value = 0
                if value:
                    self.__campaign.Campaign_information.TCR_data_handler.addMetric(
                        "BOOTOTA_SYSTEM_{0}".format(element.upper()),
                        value,
                        unit)

            self.__campaign.Campaign_information.updateJsonWithFirstBootTimeData(self.__osManager.waitOsDuration,
                                                                                 self.__flash_key,
                                                                                 self.__build_target.get("tag"))
            self.__campaign.Campaign_information.TCR_data_handler.addMetric(
                    "BOOTOTA_FIRST_BOOT_TIME_{0}".format(self.__flash_key.replace("force;", "").upper()),
                    self.__osManager.waitOsDuration,
                    "second")
            # if self.__globalConf.get("TCNAME_FLAG") == "FLASH_UC":
            #     self.__campaign.Campaign_information.sendFlashDataEmail()

        if verdict and key in ("ota", "fota"):
            self.__logs.pull("/cache/recovery/last_log", target_path="")

        #
        # Check flashed build (main)
        #
        if verdict and (check_img or check_ifwi or check_modem or check_ssn or check_tag):
            if self.__user_flashing:
                self.__logger.printLog("INFO", "user build detected: not checking flashed images sha1sum")
                forced_check_img = False
            else:
                forced_check_img = check_img

            if "bxtp" in self.__configuration.board:
                verdict = self.checkFlashedBuildBxtpMrb(build,
                                            check_tag=False,
                                            check_ifwi=False,
                                            check_modem=False,
                                            check_img=True,
                                            check_ssn=False)
            else:
                verdict = self.checkFlashedBuild(build,
                                            check_tag=check_tag,
                                            check_ifwi=check_ifwi,
                                            check_modem=check_modem,
                                            check_img=forced_check_img,
                                            check_ssn=check_ssn)

            if verdict:
                output = ""
            else:
                output = "post flash checks have failed"
        self.__output.appendOutput(output, verdict, argument="key={0}".format(key))
        return verdict

    def imeiFlashing(self):
        imei = self.__globalConf["CUSTOM_CONFIG"]["imei_config"]["imei"]
        log = "imeiFlashing({}): ".format(imei)
        imei_list = imei.split(";")
        verdict = True
        output = ""
        self.__logger.printLog("INFO", log + "start")

        if self.__download.getBranchFromTag(str(self.__download.build_but)) == "m_mr1":
            first_reboot_status = self.__osManager.adbRebootPtest()
        else:
            first_reboot_status = self.__osManager.adbReboot()

        if self.__download.getBranchFromTag(str(self.__download.build_but)) != "m_mr1" and not first_reboot_status and not self.__osManager.adbReboot():
            output = "failure to reboot the board in MOS before IMEI flashing"
            verdict = False
        elif self.__download.getBranchFromTag(str(self.__download.build_but)) == "m_mr1" and not first_reboot_status and not self.__osManager.adbRebootPtest():
            output = "failure to reboot the board in PTEST before IMEI flashing"
            verdict = False
        else:
            if not self.__device.adbRoot():
                output = "failure to root device before imei flashing"
                verdict = False
            else:
                setprop_verdict = self.__device.setProp("persist.tool_enable", "1")
                if not setprop_verdict:
                    setprop_verdict = self.__device.setProp("persist.tool_enable", "1")
                if len(imei_list) == 2:
                    setprop_dsds_verdict = self.__device.setProp("persist.radio.multisim.config", "dsds")
                else:
                    setprop_dsds_verdict = True
                local_module = os.path.join(self.__misc.extra_path, "sec_provision")
                exec_status_push_module = self.__host.commandExecAdb(
                        "push {} /data/".format(local_module),
                        timeout=20)[0]
                exec_status_chmod = self.__host.commandExecAdb(
                        "shell chmod +x /data/sec_provision".format(local_module),
                        timeout=10)[0]
                if not setprop_verdict:
                    output = "failure with setprop command for persist.tool_enable=1"
                    verdict = False
                elif not setprop_dsds_verdict:
                    output = "failure with setprop command for persist.radio.multisim.config=dsds"
                    verdict = False
                elif exec_status_push_module != 0 or not self.__device.checkPresence("/data/sec_provision"):
                    output = "failure to push file to /data/sec_provision"
                    verdict = False
                elif exec_status_chmod != 0:
                    output = "failure to change mode on /data/sec_provision"
                    verdict = False
                else:
                    command_list = list()
                    if len(imei_list) == 1:
                        command_list.append(
                                "/data/sec_provision -k DEFAULT_KEY_2048 -c UDSOCKgateway -a3 -i1 {}"
                                    .format(imei_list[0]))
                    elif len(imei_list) == 2:
                        command_list.append(
                                "/data/sec_provision -k DEFAULT_KEY_2048 -c UDSOCKgateway -a3 -i2 {} {}"
                                    .format(imei_list[0], imei_list[1]))

                    if not self.__device.scriptPush(command_list, run_script=True, timeout=60):
                        output = "failure with imei flash command"
                        verdict = False
                    else:
                        self.__host.commandExecAdb("shell rm /data/sec_provision", 10)
                        if self.__download.getBranchFromTag(str(self.__download.build_but)) == "m_mr1":
                            first_reboot_status = self.__osManager.adbRebootFromPtest()
                            if not first_reboot_status and not self.__osManager.adbRebootFromPtest():
                                output = "failure to reboot the board after imei setup (PTest Mode)"
                                verdict = False
                        else:
                            first_reboot_status = self.__osManager.adbReboot()
                            if not first_reboot_status and not self.__osManager.adbReboot():
                                output = "failure to reboot the board after imei setup (MOS)"
                                verdict = False

        self.__osManager.gotoMos()

        if not verdict and self.__globalConf["ORGANISATION_TEAM"] == "BOOT_OTA":
            self.__logger.printLog("WARNING", "IMEI incorrectly flashed on BOOTOTA bench - no major impacts")
            verdict = True

        self.__output.appendOutput(output, verdict, "imei={}".format(imei))
        return verdict

    def ssnFlashing(self):
        ssn = self.__globalConf["CUSTOM_CONFIG"]["ssn_provisioning"]
        log = "ssnFlashing({}): ".format(ssn)
        verdict = True
        output = ""
        self.__logger.printLog("INFO", log + "start")

        if not self.__device.adbRoot():
            output = "failure to root device before ssn flashing"
            verdict = False
        else:
            if not self.__device.getProperty("persist.tool_enable") == "1":
                setprop_verdict = self.__device.setProp("persist.tool_enable", "1")
                if not setprop_verdict:
                    setprop_verdict = self.__device.setProp("persist.tool_enable", "1")
                if not setprop_verdict:
                    output = "failure with setprop command for persist.tool_enable=1"
                    verdict = False
                else:
                    first_reboot_status = self.__osManager.adbReboot()
                    if not first_reboot_status and not self.__osManager.adbReboot():
                        output = "failure to reboot device after having pushed files"
                        verdict = False
            if verdict:
                command_list = list()
                command_list.append("setenforce 0")
                command_list.append("ipcs_client -c 'sec:serial_number_set(\"device\",\"{}\")' -p SECVM".format(ssn))
                if not self.__device.scriptPush(command_list, run_script=True, timeout=60):
                    output = "failure with ssn provisioning command"
                    verdict = False
                else:
                    first_reboot_status = self.__osManager.adbReboot()
                    if not first_reboot_status and not self.__osManager.adbReboot():
                        output = "failure to reboot the board after ssn setup"
                        verdict = False
        self.__output.appendOutput(output, verdict, "ssn={}".format(ssn))
        return verdict


    def ssnViaImeiFlashing(self):
        ssn = self.__globalConf["CUSTOM_CONFIG"]["imei_config"]["imei"]
        log = "ssnFlashing via IMEI({}): ".format(ssn)
        verdict = True
        output = ""
        self.__logger.printLog("INFO", log + "start")

        if not self.__device.adbRoot():
            output = "failure to root device before ssn flashing via IMEI"
            verdict = False
        else:
            if not self.__device.getProperty("persist.tool_enable") == "1":
                setprop_verdict = self.__device.setProp("persist.tool_enable", "1")
                if not setprop_verdict:
                    setprop_verdict = self.__device.setProp("persist.tool_enable", "1")
                if not setprop_verdict:
                    output = "failure with setprop command for persist.tool_enable=1"
                    verdict = False
                else:
                    first_reboot_status = self.__osManager.adbReboot()
                    if not first_reboot_status and not self.__osManager.adbReboot():
                        output = "failure to reboot device after having pushed files"
                        verdict = False
            if verdict:
                command_list = list()
                command_list.append("setenforce 0")
                command_list.append("ipcs_client -c 'sec:serial_number_set(\"device\",\"{}\")' -p SECVM".format(ssn))
                if not self.__device.scriptPush(command_list, run_script=True, timeout=60):
                    output = "failure with ssn provisioning command"
                    verdict = False
                else:
                    first_reboot_status = self.__osManager.adbReboot()
                    if not first_reboot_status and not self.__osManager.adbReboot():
                        output = "failure to reboot the board after ssn setup"
                        verdict = False
        self.__output.appendOutput(output, verdict, "ssn={}".format(ssn))
        return verdict

    ###
    # Check ro.build.version.incremental
    #
    def __tag_check(self):
        log = self.__info + " TAG CHECK - "
        self.__logger.printLog("INFO", log + "start")
        verdict = True
        output = ""
        flashed_build = self.__device.getProperty("ro.build.version.incremental")
        if not flashed_build:
            output = "ro.build.version.incremental is empty !"
            verdict = False
        elif "tag" in self.__build_target and self.__build_target["tag"]:
            if self.__build_target["tag"] == flashed_build:
                self.__logger.printLog("INFO", log + "build as expected: {0}".format(flashed_build))
            else:
                output = "expected_build='{0}', actual_build='{1}'".format(self.__build_target["tag"], flashed_build)
                verdict = False
        else:
            output = "no tag, cannot check build"
            verdict = False

        # check if WA before leaving
        if not verdict and self.__workaround.isWorkaround(data_type=self.__workaround.Function, output=output)[0]:
            verdict = True
        self.__output.appendOutput(output, verdict)
        return verdict

    ###
    # Check ifwi
    #
    def __ifwi_check(self, tag_check):
        log = self.__info + " IFWI CHECK - "
        self.__logger.printLog("INFO", log + "start")
        verdict = True
        output = ""

        ifwi_flashed = self.__device.getBiosVersion()
        if ifwi_flashed:
            self.__campaign.Campaign_information.updateJsonWithIfwiVersion(ifwi_flashed)
        ifwi_flashed = ifwi_flashed.replace(".", "").lower()

        if not ifwi_flashed:
            output = "sys.ifwi.version property is empty"
            verdict = False

        # store found ifwi version in this list
        ifwi_bin = list()

        if self.__build_target.get("valid_ifwi_version"):
            ifwi_bin.append(self.__build_target["valid_ifwi_version"])

        # specific methods
        if "legacy" in self.__configuration.flash.IFWI_CHECK:
            for f in self.__build_target["flash_list"]:
                local_flash_file = self.__flashFile.createFile(f)
                # getting file name
                ifwi_bin.extend([os.path.basename(i).replace(".bin", "") for i in
                                 self.getZipContentList(local_flash_file.get_localFlashFile(),
                                                        "(IFWI.*\.bin|capsule.*\.bin)")])
                # ifwi_bin: append FIRMWARE version from flash.xml
                if local_flash_file.get_pft_flash_file() == "":
                    ifwi_bin.extend(self.getVersionListFromXml(local_flash_file.get_localFlashFile(),
                                                               "flash.xml",
                                                               group="FIRMWARE"))
                else:
                    ifwi_bin.extend(self.getVersionListFromXml(local_flash_file.get_localFlashFile(),
                                                               flash_xml=local_flash_file.get_pft_flash_file(),
                                                               group="FIRMWARE"))
            # byt specific
            if self.__configuration.flash.UEFI:
                ifwi_bin = [single_ifwi.lower().replace("ifwi_", "") for single_ifwi in ifwi_bin]
                if ifwi_flashed:
                    try:
                        ifwi_flashed = ifwi_flashed.split()[1]
                    except:
                        ifwi_flashed = ifwi_flashed.split()[0]
        if "firmware-info" in self.__configuration.flash.IFWI_CHECK:
            firmware_info_file = ""
            firmware_info_filename = "firmware-info_{}.txt".format(
                re.sub(self.__configuration.download.MULTI_VARIANT_PREFIX, "", self.__configuration.board))
            for f in self.__build_target["flash_list"]:
                local_flash_file = self.__flashFile.createFile(f)
                if local_flash_file.get_file_type() in ["fastboot"]:
                    firmware_info_files = self.getZipContentList(
                        local_flash_file.get_localFlashFile(),
                        "^{}$".format(firmware_info_filename))
                    if len(firmware_info_files) != 1:
                        output = "zero or multiple {} file found in flash files".format(firmware_info_filename)
                        verdict = False
                        break
                    firmware_info_file = firmware_info_files[0]
            if not firmware_info_file:
                output = "cannot find {} file in flash files".format(firmware_info_filename)
                verdict = False
            elif not os.path.isfile(firmware_info_file):
                output = "file not found {}".format(firmware_info_file)
                verdict = False
            else:
                with open(firmware_info_file) as f:
                    firmware_info_content = f.read()
                prefix = re.search("IFWI Prefix.*: ([A-Za-z0-9_]+)", firmware_info_content)
                suffix = re.search("IFWI Suffix.*: ([A-Za-z0-9_]+)", firmware_info_content)
                if not prefix:
                    output = "prefix not found in {}".format(firmware_info_filename)
                    verdict = False
                elif not suffix:
                    output = "suffix not found in {}".format(firmware_info_filename)
                    verdict = False
                else:
                    ifwi_bin.append("_".join([prefix.group(1), suffix.group(1)]))

        # If ifwi/capsule bin file exists
        if verdict and ifwi_bin:
            ifwi_bin = [single_ifwi.replace(".", "").lower() for single_ifwi in ifwi_bin]

            # If matches
            if any(ifwi_flashed in single_ifwi for single_ifwi in ifwi_bin):
                self.__logger.printLog("INFO", log + "ifwi as expected: {} "
                                                     "(among: {})".format(ifwi_flashed,
                                                                          ", ".join(ifwi_bin)))
            # Else differs
            else:
                output = "actual: '{}', expected: {}".format(ifwi_flashed, ", ".join(ifwi_bin))
                verdict = False

        # Else no file to check: failure
        else:
            output = "no bin file found, cannot check ifwi"
            verdict = False
            if tag_check and ifwi_flashed and "valid_ifwi_version" not in self.__build_target:
                self.__build_target["valid_ifwi_version"] = ifwi_flashed
                self.__logger.printLog("DEBUG", log + "storing ifwi version: {}".format(ifwi_flashed))

        # check if WA before leaving
        if not verdict and self.__workaround.isWorkaround(data_type=self.__workaround.Function, output=output)[0]:
            verdict = True
        self.__output.appendOutput(output, verdict)
        return verdict

    ###
    # Check modem
    #
    def __modem_check(self):
        log = self.__info + " MODEM CHECK - "
        self.__logger.printLog("INFO", log + "start")
        verdict = True
        output = ""
        # Check modem gsm.version.baseband
        modem_flashed = self.__osManager.waitModem()[1].split("|")[0]
        if not modem_flashed:
            output = "gsm.version.baseband property is empty"
            verdict = False

        else:
            modem_bin_files = list()
            modem = None
            for f in self.__build_target["flash_list"]:
                local_flash_file = self.__flashFile.createFile(f)
                modem = self.getZipContentList(local_flash_file.get_localFlashFile(), "modem.*\.zip")
                if modem:
                    break

                # If modem zip file exists
            if modem:
                for m in modem:
                    modem_bin_files += self.getZipContentList(m)

                # If model binary file found
                if modem_bin_files:
                    modem_success = False
                    for binary in modem_bin_files:
                        with open(binary) as f:
                            self.__logger.printLog("INFO", log + "check if %s is in %s" % (modem_flashed, binary))
                            if modem_flashed in f.read():
                                modem_success = True
                                break
                    # If matches
                    if modem_success:
                        self.__logger.printLog("INFO", log + "modem version as expected: '%s'" % modem_flashed)
                    # If does not match
                    else:
                        output = "unexpected modem version: '%s'" % modem_flashed
                        verdict = False
                # Else no file to check: failure
                else:
                    output ="no bin file found, cannot check modem"
                    verdict = False

            # Else failure
            else:
                output = "no fastboot, cannot check modem"
                verdict = False

        # check if WA before leaving
        if not verdict and self.__workaround.isWorkaround(data_type=self.__workaround.Function, output=output)[0]:
            verdict = True
        self.__output.appendOutput(output, verdict)
        return verdict

    ###
    # Check serialno from cmdline
    #
    def __ssn_check(self):
        log = self.__info + " SERIAL NUMBER CHECK - "
        self.__logger.printLog("INFO", log + "start")
        verdict = True
        output = ""
        ssn = self.__device.getProperty("ro.serialno")
        # If no actual ssn
        if not ssn:
            output = "no serialno property is empty"
            verdict = False
        # Elif no ref ssn
        elif "ssn" not in self.__build_target or not self.__build_target["ssn"]:
            output = "no serialno in build"
            verdict = False
        # Elif actual and ref differ
        elif self.__build_target["ssn"] != ssn:
            output = "serialno failure, actual: %s, expected: %s" % (ssn, self.__build_target["ssn"])
            verdict = False

        # check if WA before leaving
        if not verdict and self.__workaround.isWorkaround(data_type=self.__workaround.Function, output=output)[0]:
            verdict = True
        self.__output.appendOutput(output, verdict)
        return verdict

    def __get_hexdump_value(self, index, offset, size):
        parameter = 32 + index * 24 + offset
        exec_status, output = self.__host.commandExecAdb("shell hexdump -s " + str(parameter) + " -n " + str(size) + " " + self.__block_osip)
        output = output.split(" ")
        if len(output) == 1 and output[0].startswith("*"):
            value = "0000"
        elif exec_status != 0 or "hexdump" in output:
            self.__logger.printLog("WARNING", self.__info + "failed to hexdump the osip header attribute of entry %s" % str(index))
            return False, None
        elif len(output) < 2:
            self.__logger.printLog("WARNING", self.__info + "output too short for entry %s" % str(index))
            return False, None
        elif len(output) < 3 and size > 2:
            self.__logger.printLog("WARNING", self.__info + "output too short for entry %s" % str(index))
            return False, None
        elif size > 2:
            # reconstruct value (little endian)
            value = str.upper(output[2]) + str.upper(output[1])
        else:
            value = str.upper(output[1])
        self.__logger.printLog("DEBUG", self.__info + "get_hexdump_value(%s, %s, %s) = '%s'" % (str(index), str(offset), str(size), value))
        return True, value

    # Inner method to read the OSIP header and get the info on the boot image location and size
    def __read_osip_header(self, target_os):
        # This values are hard coded in vendor/intel/hardware/libintelprov/update_osip.h
        os_attr = {"recovery.img": int("0x000C", 16), "droidboot.img": int("0x000E", 16), "main": int("0x0000", 16)}

        # Read the attribute of each entry is the osip header until we find the requested OS
        for i in range(0,7):
            # %02X format the output to a 2 digit hex number; 32(size of the osip header header) + X * 24(size of an osip header entry) + 20(offset of the attribute attribute); -n 1(size of attribute)
            # exec_status, output = self.__host.commandExec(("adb " + ADD_SSN_IN_COMMAND + "shell hexdump -e '\\\"%08X\\\"' -s $((32+" + str(i) + "*24+20)) -n 1 " + self.__block_osip), 3)
            status, output = self.__get_hexdump_value(i, 20, 1)
            if not status:
                self.__logger.printLog("WARNING", self.__info + "failed to hexdump entry %s" % str(i))
                return None, None
            # Apply a mask to ignore the LSB of the attribute that indicate is the image is signed or not
            current_attr = int(output, 16) & int("0x00FE", 16)
            # If the current osip entry match the researched OS img then return its offset and size
            if current_attr == os_attr[target_os]:
                # %02X format the output to a 8 digit hex number; 32(size of the osip header header) + X * 24(size of an osip header entry) + 4(offset of the logical_start_block attribute); -n 4(size of logical_start_block)
                # exec_status, output = self.__host.commandExec(("adb " + ADD_SSN_IN_COMMAND + "shell hexdump -e '\\\"%08X\\\"' -s $((32+" + str(i) + "*24+4)) -n 4 " + self.__block_osip), 3)
                status, output = self.__get_hexdump_value(i, 4, 4)
                if not status or "*" in output:
                    self.__logger.printLog("WARNING", self.__info + "failed to hexdump the osip header logical_start_block of entry %s" % str(i))
                    return None, None
                logical_start_block = int(output, 16)
                # %02X format the output to a 2 digit hex number; 32(size of the osip header header) + X * 24(size of an osip header entry) + 20(offset of the size_of_image attribute); -n 1(size of size_of_image)
                # exec_status, output = self.__host.commandExec(("adb " + ADD_SSN_IN_COMMAND + "shell hexdump -e '\\\"\%08X\\\"' -s $((32+" + str(i) + "*24+16)) -n 4 " + self.__block_osip), 3)
                status, output = self.__get_hexdump_value(i, 16, 4)
                if not status or "*" in output:
                    self.__logger.printLog("WARNING", self.__info + "failed to hexdump the osip header size_of_image of entry %s" % str(i))
                    return None, None
                size_of_image = int(output, 16)
                return logical_start_block, size_of_image

        # In case the researched OS isn't found
        return None, None

    # Inner method to get sha1 of flashed image
    def __get_sha1(self, block, offset, size):
        output = ""
        returnValue = None
        verdict = True
        of = "/data/dump"

        if not size:
            output = "failure: offset={0}, size={1}".format(offset, size)
            verdict = False
        else:
            size_512_block = int(float(size) / 512)
            size_diff = size - size_512_block * 512

            # Perform another dd as now the image contains the signature
            exec_status, ddOutput = self.__host.commandExecAdb('shell dd if={0} of={1} bs=512 skip={2} count={3}'.format(block, of, offset, size_512_block), 20)
            if exec_status != 0:
                output = "failure to dd skip=0 count={0} ({1})".format(size_512_block, ddOutput)
                verdict = False
            elif size_diff != 0:
                exec_status, ddOutput = self.__host.commandExecAdb('shell dd if={0} of={1} bs=1 skip={2} count={3} seek={4}'.format(block, of, size_512_block * 512, size_diff, size_512_block * 512), 20)
                if exec_status != 0:
                    output = "failure to dd skip={0} count={1} ({2})".format(size_512_block * 512, size_diff, ddOutput)
                    verdict = False
        if verdict:
            # Pull /data/dump.img on host because sha1sum command might not work on device
            if self.__logs.bootota_log_path:
                pulled_file = os.path.join(self.__logs.bootota_log_path, os.path.basename(of))
            else:
                pulled_file = os.path.join(tempfile.gettempdir(), os.path.basename(of))
            self.__misc.local_files_handler.addEntry(pulled_file)
            if not self.__logs.pull(of, target_path=os.path.dirname(pulled_file), push_to_server=False) or not os.path.isfile(pulled_file):
                output = "failure reading: {0} (file not present)".format(pulled_file)
                verdict = False
            else:
            # Compute sha1 then erase file
                returnValue = self.__get_sha1_img(pulled_file, skip_first_block=False)
            if os.path.isfile(pulled_file):
                os.remove(pulled_file)
        self.__output.appendOutput(output, verdict)
        return returnValue

    # Inner method to get sha1 of file
    @staticmethod
    def __get_sha1_img(path, skip_first_block=True):
        with open(path, 'rb') as f:
            h = hashlib.sha1()
            # skip first block
            if skip_first_block:
                f.seek(512)
            h.update(f.read())
        return h.hexdigest()

    # Inner method to get md5 of file
    @staticmethod
    def __get_md5_img(path, skip_first_block=True):
        with open(path, 'rb') as f:
            h = hashlib.md5()
            # skip first block
            if skip_first_block:
                f.seek(512)
            h.update(f.read())
        return h.hexdigest()

    # check image presence in flashfile
    def __checkImageInWorkdir(self, image_name, file_type_list=list()):
        if not file_type_list:
            file_type_list = ["fastboot"]
        img_list = list()
        for f in self.__build_target["flash_list"]:
            local_flash_file = self.__flashFile.createFile(f)
            if local_flash_file.get_file_type() not in file_type_list:
                self.__logger.printLog("DEBUG", "checkImageInWorkdir({1}): skipping '{0}' "
                                                "file type".format(local_flash_file.get_file_type(),
                                                                   image_name))
                continue
            img_list += self.getZipContentList(local_flash_file.get_localFlashFile(), image_name + "$")
        if len(img_list) == 0:
            self.__logger.printLog("WARNING", "checkImageInWorkdir({}): "
                                              "missing image from flash file".format(image_name))
            return False, ""
        else:
            if len(img_list) > 1:
                self.__logger.printLog("WARNING", "checkImageInWorkdir({0}): more that one image found, "
                                                  "selecting first one".format(image_name))
            returnElement = img_list[0]
            self.__logger.printLog("INFO", "checkImageInWorkdir({1}) selected: {0}".format(returnElement, image_name))
            return True, returnElement

    # generic check method for droidboot and recovery images
    def __genericImageCheck(self, image_name, partition_table):
        self.__logger.printLog("INFO", self.__info + "Flashed {0} Check: start".format(image_name.capitalize()))
        # check image on disk
        verdict = True
        output = ""
        status, image = self.__checkImageInWorkdir(image_name)
        if not status:
            output = "'{0}' not found".format(image_name)
            verdict = False
        flashed_sha1 = None
        partition_name = ""
        if image_name == "recovery.img":
            partition_name = "recovery"
        elif image_name == "droidboot.img":
            partition_name = "fastboot"
        # two types of image checks: osip and gpt
        if verdict and partition_table == "gpt":
            file_size = os.path.getsize(image)
            # get sha1 of flashed gpt partition
            flashed_sha1 = self.__get_sha1(self.__block_gpt + "/" + partition_name, 0, file_size)
        elif verdict and partition_table == "osip":
            # Dump values in OSIP
            offset, size = self.__read_osip_header(image_name)
            if None in (offset, size):
                output = "none dumped value"
                verdict = False
            else:
                # get sha1 of flashed osip partition
                flashed_sha1 = self.__get_sha1(self.__block_osip, offset, size * 512)

        # check sha1 was properly get
        if not flashed_sha1:
            output = "img: none sha1 value for '{0}' partition '{1}'".format(partition_table, image_name)
            verdict = False
        else:
            # compute sha1 of image on disk in workdir
            if partition_table == "gpt":
                source_sha1 = self.__get_sha1_img(image, skip_first_block=False)
            else:
                source_sha1 = self.__get_sha1_img(image)

            # print sha1 values for debug
            self.__logger.printLog("DEBUG", "{0} : sha1 of flashed '{1}'".format(flashed_sha1, partition_name))
            self.__logger.printLog("DEBUG", "{0} : sha1 of file '{1}'".format(source_sha1, image_name))

            # Check if difference exists
            if flashed_sha1 != source_sha1:
                output = "actual is {0}, expected is {1}".format(flashed_sha1, source_sha1)
                verdict =  False
        self.__output.appendOutput(output, verdict, argument="image={0}, partition={1}".format(image_name, partition_table))
        return verdict

    # specific osloader check
    def __osloader_check(self):
        self.__logger.printLog("INFO", self.__info + "Flashed Osloader Check: start")
        osloader_sha1 = None
        output = ""
        verdict, osloader_img = self.__checkImageInWorkdir("efilinux.*.efi", file_type_list=["blankphone", "fastboot"])
        if verdict:
            device_osloader = "/data/ESP/EFI/BOOT/bootx64.efi"
            status, ls_output = self.__host.commandExecAdb("shell ls {0}".format(device_osloader), 5)
            # if ESP not mounted
            if "No such file or directory" in ls_output or "not found" in ls_output or status != 0:
                self.__host.commandExecAdb("shell mkdir /data/ESP")
                status, mount_output = self.__host.commandExecAdb("shell mount -t vfat {0}/ESP /data/ESP".format(self.__block_gpt), 5)
                if status != 0:
                    output = "impossible to mount ESP at /data/ESP: {0}".format(mount_output)
                    verdict = False
                else:
                    self.__host.commandExecAdb("shell ls {0}".format(device_osloader), 3)
                    if self.__logs.bootota_log_path:
                        pulled_file = os.path.join(self.__logs.bootota_log_path, os.path.basename(device_osloader))
                    else:
                        pulled_file = os.path.join(tempfile.gettempdir(), os.path.basename(device_osloader))
                    self.__misc.local_files_handler.addEntry(pulled_file)
                    if not self.__logs.pull(device_osloader, target_path=os.path.dirname(pulled_file), push_to_server=False) or not os.path.isfile(pulled_file):
                        output = "failure reading: {0} (file not present)".format(pulled_file)
                        verdict = False
                    else:
                    # Compute sha1 then erase file
                        osloader_sha1 = self.__get_sha1_img(pulled_file, skip_first_block=False)
                        if os.path.isfile(pulled_file):
                            os.remove(pulled_file)
            # else already mounted
            else:
                if self.__logs.bootota_log_path:
                    pulled_file = os.path.join(self.__logs.bootota_log_path, os.path.basename(device_osloader))
                else:
                    pulled_file = os.path.join(tempfile.gettempdir(), os.path.basename(device_osloader))
                self.__misc.local_files_handler.addEntry(pulled_file)
                if not self.__logs.pull(device_osloader, target_path=os.path.dirname(pulled_file), push_to_server=False) or not os.path.isfile(pulled_file):
                    output = "failure reading: {0} (file not present)".format(pulled_file)
                    verdict = False
                else:
                # Compute sha1 then erase file
                    osloader_sha1 = self.__get_sha1_img(pulled_file, skip_first_block=False)
                    os.remove(pulled_file)
        else:
            output = "osloader not found in image"
            verdict = False

        if verdict:
            osloader_sha1_img = self.__get_sha1_img(osloader_img, skip_first_block=False)
            self.__logger.printLog("DEBUG", "{0} : sha1 of flashed osloader".format(osloader_sha1))
            self.__logger.printLog("DEBUG", "{0} : sha1 of efilinux.efi".format(osloader_sha1_img))
            if osloader_sha1 != osloader_sha1_img:
                output = "{0}: actual is {1}, expected is {2}".format(
                          os.path.basename(osloader_img),
                          osloader_sha1,
                          osloader_sha1_img)
                verdict = False

        # check if WA before leaving
        if not verdict and self.__workaround.isWorkaround(data_type=self.__workaround.Function, output=output)[0]:
            verdict = True
        self.__output.appendOutput(output, verdict)
        return verdict

    def __hashes_check(self):

        log = self.__info + " HASHES CHECK - "
        self.__logger.printLog("INFO", log + "start")

        output = ""

        # go to POS to launch command: fastboot oem get hashes
        if "bxtp" in self.__configuration.board and "lh" not in self.__configuration.board:
            verdict = self.__osManager.gotoFastbootBxtpMrb()
        else:
            verdict = self.__osManager.gotoPos()
        if not verdict:
            output = "failure to go to POS to get hashes"
        else:
            algorithm = self.__configuration.flash.CHECK_GET_HASHES.get("algorithm", "")
            hashes = self.__device.fastbootOemGetHashes(algorithm=algorithm)
            if not algorithm:
                algorithm = "sha1"
            self.__osManager.fastbootReboot(wait_main=False)
            if not hashes:
                verdict = False
                output = "failure to get hashes from fastboot command"
            else:
                failed_sha1 = list()
                for single_hash in self.__configuration.flash.CHECK_GET_HASHES.get("list", []):
                    if single_hash == "/recovery":
                        target_file = "^recovery.img"
                    elif single_hash == "/bootloader/loader.efi":
                        target_file = "^loader.efi"
                    elif single_hash == "/bootloader/EFI/BOOT/bootx64.efi":
                        target_file = "^loader.efi"
                    elif single_hash == "/boot":
                        target_file = "^boot.img"
                    elif single_hash == "/system":
                        target_file = "^system.img"
                    elif single_hash == "/bootloader":
                        if self.__configuration.flash.BXTP_ABL_BUILD == "gr_mrb_b1":
                            target_file = "^bootloader_gr_mrb_b1"
                        else:
                            target_file = "^bootloader_gr_mrb"
                    else:
                        img = None
                        target_file = None
                        self.__logger.printLog("INFO", log + "unknown hash name: {0}".format(single_hash))
                        failed_sha1.append(single_hash)
                    if target_file:
                        local_verdict, img = self.__checkImageInWorkdir(target_file)
                    else:
                        local_verdict = False
                    if local_verdict:
                        if single_hash == "/system":
                            # unsparse image
                            unsparsed_system = os.path.join(os.path.dirname(img), "unsparsed_system")
                            if os.path.exists(unsparsed_system):
                                os.remove(unsparsed_system)
                            if "windows" in os.environ.get('OS','').lower():
                                unsparsed_bin = os.path.abspath(os.path.join(os.path.dirname(__file__), "Extras", "simg2img.exe"))
                            else:
                                unsparsed_bin = os.path.abspath(os.path.join(os.path.dirname(__file__), "Extras", "simg2img"))
                            exec_status, _ = self.__host.commandExec("{0} {1} {2}".format(unsparsed_bin, img, unsparsed_system), 120)
                            if exec_status != 0:
                                output = "failure to unsparsed system image"
                                failed_sha1.append(single_hash)
                                continue
                            else:
                                img = unsparsed_system
                                self.__logger.printLog("DEBUG", log + "computing sha1sum of: {0}".format(img))
                        if algorithm in ["", "sha1", None]:
                            sha1_img = self.__get_sha1_img(img, skip_first_block=False)
                        elif algorithm == "md5":
                            sha1_img = self.__get_md5_img(img, skip_first_block=False)
                        flashed_sha1 = hashes.get(single_hash, "")
                        self.__logger.printLog("DEBUG", "{0} : sha1 of flashed {1}".format(flashed_sha1, single_hash))
                        self.__logger.printLog("DEBUG", "{0} : sha1 of {1} image".format(sha1_img, target_file.replace("^", "")))
                        if sha1_img != flashed_sha1:
                            output = "corrupted {} for {}, actual is {}, expected is {}".format(
                                algorithm,
                                single_hash,
                                flashed_sha1,
                                sha1_img)
                            failed_sha1.append(single_hash)
                    else:
                        self.__logger.printLog("INFO", log + "no flash files found for {0} in flashfiles (regexp={1})".format(single_hash, target_file))
                        if single_hash not in failed_sha1:
                            failed_sha1.append(single_hash)
                if failed_sha1:
                    verdict = False
                    output = "failure with hash checks for {0}".format(", ".join(failed_sha1))

        if "bxtp" in self.__configuration.board and "lh" not in self.__configuration.board:
            if self.__device.getProperty("sys.boot_completed") != "1":
                self.__osManager.rebootBxtpMrb()
        else:
            if not self.__osManager.waitOs("main"):
                self.__osManager.gotoMos()
            self.__device.adbRoot()
        self.__output.appendOutput(output, verdict)
        return verdict

    ###
    # Check flashed images (recovery/droidboot)
    #
    def __img_check(self, partition_table, recovery_check=True, fastboot_check=True, osloader_check=True, hashes_check=False):

        log = self.__info + " IMAGE CHECK - "
        self.__logger.printLog("INFO", log + "start")

        # Local verdict
        verdict = True
        output = ""


        # Root is mandatory
        if not self.__device.adbRoot():
            output = "failure to root device"
            self.__output.appendOutput(output, False)
            return False

        # Main partition
        self.__block_osip = "/dev/block/mmcblk0"
        self.__block_gpt = ""

        # on UEFI boards, check where are stored partitions
        _, mount = self.__host.commandExecAdb("shell cat /proc/mounts")
        # check cache mounting path to compute block path
        cache = re.findall("/dev/.*/cache", mount)
        if cache:
            try:
                self.__block_gpt = os.path.dirname(cache[0].split()[0])
            except:
                pass
        # if failure with cache, check system
        if not self.__block_gpt:
            system = re.findall("/dev/.*/system", mount)
            if system:
                try:
                    self.__block_gpt = os.path.dirname(system[0].split()[0])
                except:
                    pass
        # if failure with cache and system, fallback to default value
        if not self.__block_gpt:
            self.__block_gpt = "/dev/block/by-name"
            self.__logger.printLog("WARNING", log + "failure to compute path to partition. using default value: '{0}'".format(self.__block_gpt))

        failureList = list()
        if recovery_check:
            if not self.__genericImageCheck("recovery.img", partition_table):
                verdict = False
                failureList.append("recovery partition")

        if fastboot_check:
            if not self.__genericImageCheck("droidboot.img", partition_table):
                verdict = False
                failureList.append("fastboot partition")

        # osloader cannot be checked for eng variant
        if osloader_check and "eng" not in self.__globalConf.get("PROVISIONING_DATA", {}).get("build_variant", ""):
            if not self.__osloader_check():
                verdict = False
                failureList.append("osloader efi application")

        if hashes_check:
            if not self.__hashes_check():
                verdict = False
                failureList.append("hashes checks")

        if not verdict:
            output = "failed image checks: {0}".format(", ".join(failureList))

        if not verdict and self.__workaround.isWorkaround(data_type=self.__workaround.Function)[0]:
            verdict = True
        self.__output.appendOutput(output, verdict)
        return verdict

    def checkFlashedBuild(self, build_target, check_tag=True, check_ifwi=True,
                            check_modem=True, check_img=True, check_ssn=True):
        """ Check the flashed build by comparing build version, ifwi, modem...
            build_target must have "tag" (expected ro.build.version.incremental)
        """
        output = ""
        self.__info = "checkFlashedBuild({0}): ".format(build_target.get("tag", "no_tag"))
        self.__build_target = build_target
        self.__logger.printLog("INFO", self.__info + "start")

        currentOs = self.__osManager.getOs()
        if currentOs != "main":
            output = "device not in main (current os: {0})".format(currentOs)
            self.__output.appendOutput(output, False)
            return False

        verdict = True

        failureList = list()
        tag_check = False
        if check_tag:
            if not self.__tag_check():
                verdict = False
                failureList.append("tag number")
            else:
                tag_check = True

        if check_ifwi:
            if not self.__ifwi_check(tag_check):
                verdict = False
                failureList.append("ifwi version")

        if check_modem:
            if not self.__configuration.boot.MODEM:
                self.__logger.printLog("INFO", self.__info + "MODEM=False (board configuration), skip modem check")
            elif not self.__globalConf.get("MODEM", False):
                self.__logger.printLog("INFO", self.__info + "MODEM=False (campaign configuration), skip modem check")
            elif not self.__modem_check():
                verdict = False
                failureList.append("modem version")

        if check_ssn:
            if not self.__ssn_check():
                verdict = False
                failureList.append("serial number")

        if check_img:
            local_verdict = False
            for partition_type in self.__configuration.flash.PARTITION_TYPE.split(";"):
                if self.__img_check(partition_type,
                                        osloader_check=self.__configuration.flash.CHECK_OSLOADER,
                                        fastboot_check=self.__configuration.flash.CHECK_FASTBOOT,
                                        recovery_check=self.__configuration.flash.CHECK_RECOVERY,
                                        hashes_check=self.__configuration.flash.CHECK_GET_HASHES):
                    local_verdict = True
                    break
            if not local_verdict:
                failureList.append("image sha1")
                verdict = False

        if not verdict:
            self.__logger.printLog("WARNING", self.__info + "some checks have failed ({0})".format(", ".join(failureList)))
            output = "failed checks: {0}".format(", ".join(failureList))
            if not verdict and self.__workaround.isWorkaround(data_type=self.__workaround.Function)[0]:
                verdict = True

        argument = "tag={0}, check_tag={1}, check_ifwi={2}, check_modem={3}, check_img={4}, check_ssn={5}".\
            format(build_target.get("tag", "no_tag"), check_tag, check_ifwi, check_modem, check_img, check_ssn)
        self.__output.appendOutput(output, verdict, argument=argument)
        return verdict

    def checkFlashedBuildBxtpMrb(self, build_target, check_tag=True, check_ifwi=True,
                          check_modem=True, check_img=True, check_ssn=True):
        """ Check the flashed build by comparing build version, ifwi, modem...
            build_target must have "tag" (expected ro.build.version.incremental) for BXTP MRB board
        """
        output = ""
        self.__info = "checkFlashedBuildBxtpMrb({0}): ".format(build_target.get("tag", "no_tag"))
        self.__build_target = build_target
        self.__logger.printLog("INFO", self.__info + "start")

        if not self.__osManager.checkBootCompletedBxtpMrb():
            output = "device not in main "
            self.__output.appendOutput(output, False)
            return False

        verdict = True

        failureList = list()
        tag_check = False
        if check_tag:
            if not self.__tag_check():
                verdict = False
                failureList.append("tag number")
            else:
                tag_check = True

        if check_ifwi:
            if not self.__ifwi_check(tag_check):
                verdict = False
                failureList.append("ifwi version")

        if check_modem:
            if not self.__configuration.boot.MODEM:
                self.__logger.printLog("INFO", self.__info + "MODEM=False (board configuration), skip modem check")
            elif not self.__globalConf.get("MODEM", False):
                self.__logger.printLog("INFO", self.__info + "MODEM=False (campaign configuration), skip modem check")
            elif not self.__modem_check():
                verdict = False
                failureList.append("modem version")

        if check_ssn:
            if not self.__ssn_check():
                verdict = False
                failureList.append("serial number")

        if check_img:
            local_verdict = False
            for partition_type in self.__configuration.flash.PARTITION_TYPE.split(";"):
                if self.__img_check(partition_type,
                                    osloader_check=self.__configuration.flash.CHECK_OSLOADER,
                                    fastboot_check=self.__configuration.flash.CHECK_FASTBOOT,
                                    recovery_check=self.__configuration.flash.CHECK_RECOVERY,
                                    hashes_check=self.__configuration.flash.CHECK_GET_HASHES):
                    local_verdict = True
                    break
            if not local_verdict:
                failureList.append("image sha1")
                verdict = False

        if not verdict:
            self.__logger.printLog("WARNING", self.__info + "some checks have failed ({0})".format(", ".join(failureList)))
            output = "failed checks: {0}".format(", ".join(failureList))
            if not verdict and self.__workaround.isWorkaround(data_type=self.__workaround.Function)[0]:
                verdict = True

        argument = "tag={0}, check_tag={1}, check_ifwi={2}, check_modem={3}, check_img={4}, check_ssn={5}". \
            format(build_target.get("tag", "no_tag"), check_tag, check_ifwi, check_modem, check_img, check_ssn)
        self.__output.appendOutput(output, verdict, argument=argument)
        return verdict

    def checkFlashPartitionFreeSpace(self, build):
        # checks if there is enough space on the device for flashing

        verdict = True
        for flash_file in build["flash_list"]:
            local_flash_file = self.__flashFile.createFile(flash_file)

        self.__logger.printLog("INFO", "Local flash file zip: {}".format(local_flash_file.get_localFlashFile()))
        exec_status, output = self.__host.commandExec("du -sch {}".format(local_flash_file.get_localFlashFile()))
        output = output[0:output.rfind("home")-2]
        self.__logger.printLog("INFO", "Size of flash zip file {}".format(output))

        if output.endswith("G"):
            file_size = output.rstrip("G")
            file_size = float(file_size)
            file_size = float(file_size * 1000)
        else:
            file_size = output.rstrip("M")
            file_size = float(file_size)
        self.__logger.printLog("INFO", "Size of flash zip file {}M".format(file_size))

        data_info = self.__device.getPartitionInformation("data")
        if "Free" in data_info:
            free_space = data_info["Free"]
        else:
            free_space = data_info["Avail"]
        self.__logger.printLog("INFO", "Free space on data {}".format(free_space))
        if free_space.endswith("G"):
            data_free_space = free_space.rstrip("G")
            data_free_space = float(data_free_space)
            data_free_space = float(data_free_space * 1000)
        elif free_space.endswith("K"):
            data_free_space = free_space.rstrip("K")
            data_free_space = float(data_free_space)
            data_free_space = float(data_free_space / 1000)
        elif free_space.endswith("o"):
            data_free_space = free_space.rstrip("o")
            data_free_space = float(data_free_space)
            data_free_space = float(data_free_space / 1000000)
        else:
            data_free_space = free_space.rstrip("M")
            data_free_space = float(data_free_space)
        self.__logger.printLog("INFO", "Free space on data {}M".format(data_free_space))
        if file_size > data_free_space:
            verdict = False
            self.__logger.printLog("INFO", "File size greater than free space")
        else:
            self.__logger.printLog("INFO", "File size fits in available free space")

        return verdict

    def flashCapsule(self, build, check_ifwi=True):
        """ Flash capsule.bin in droidboot
        """
        log = "flash_capsule(): "
        output = ""
        verdict = True

        self.__logger.printLog("INFO", log + "Flash capsule method start")

        # Get capsule file from fastboot
        capsule_bin = self.getZipContentList(build["fastboot"], "capsule.*\.bin")
        if capsule_bin is None:
            output = "failed to retrieve capsule.bin (no fastboot file)"
            verdict =  False
        elif not capsule_bin:
            output = "failed to retrieve capsule.bin (not found)"
            verdict =  False

        # Go to POS
        if verdict and not self.__osManager.gotoPos():
            output = "failed to go to POS before capsule flashing"
            verdict =  False
        if not verdict:
            self.__output.appendOutput(output, False)
            return False

        # Flash capsule
        self.__logger.printLog("INFO", log + "Flash capsule with fastboot command")
        exec_status, flash_output = self.__host.commandExecFastboot("flash capsule " + str(capsule_bin[0]), 30)
        if exec_status != 0:
            output = "flash capsule command failure ({0})".format(flash_output)
            verdict =  False
        else:
            self.__host.commandExecFastboot("continue")
            if not self.__osManager.waitOs("main", timeout=500, loopTime=20):
                output = "failure to boot MOS after fastboot continue"
                verdict = False
            if verdict:
                verdict = self.checkFlashedBuild(build,
                                                check_tag=False,
                                                check_ifwi=check_ifwi,
                                                check_modem=False,
                                                check_img=False,
                                                check_ssn=False)
                if not verdict:
                    output = "failre to check BIOS after MOS completed"
        self.__output.appendOutput(output, verdict)
        return verdict

    def dediprog(self, binary, increaseBinSizeTo=0, timeout=600):
        """ Method to dediprog the firmware binary on a 8MB SPINOR
        """
        #from Lib.pupdr.PupdrLibrary.Tool.DEDIPROG import DEDIPROG as FLASH_DEDIPROG
        self.__logger.printLog("INFO", "dediprog({0}): start".format(binary))
        binary = str(binary)
        output = ""
        # Increase the binary size to 8MB filling with x00
        self.__dediprog.increaseBinSize(binary, increaseBinSizeTo)
        # wait that dediprog tools enumerates:
        verdict = self.__osManager.waitLsusbEnumeration(["0483:dada"], 60)
        if not verdict:
            self.__logger.printLog("WARNING", "enumeration failure, dediprog command may fail")

        # Apply the binary on the SPINOR flash
        verdict = self.__dediprog.flashBinary(binary, timeout=timeout)
        if not verdict:
            output = "dediprog({0}): FAILURE".format(binary)

        # force shutdown the board before exit to make sure BIOS is loaded to ram
        time.sleep(10)
        self.__osManager.forceShutdown()

        self.__output.appendOutput(output, verdict, argument="file={0}".format(os.path.basename(binary)))
        return verdict

    def getBuildProperties(self, zip_file, flash_xml="flash.xml", pft_info_file=False, isExit=False):
        """ Returns the dictionary of build properties present in flash.xml file
        """
        log = "get_build_properties(%s, %s): " % (zip_file, flash_xml)
        self.__logger.printLog("INFO", log + "start")
        build_info = dict()
        xml = None

        # support phoneflashtool json
        if pft_info_file:
            if not os.path.exists(zip_file):
                self.__logger.printLog("WARNING", "file does not exist: '%s'" % zip_file)
                return dict()
            with open(zip_file) as f:
                json_data = json.load(f)
            if not "buildProperties" in json_data:
                self.__logger.printLog("WARNING", "'buildProperties' entry missing in: '%s'" % zip_file)
                return dict()
            return json_data["buildProperties"]

        # Read flash.xml
        try:
            with open(self.getZipContentList(zip_file, flash_xml)[0]) as f:
                xml = f.read()
        except Exception as e:
            if isExit:
                self.__logger.printLog("WARNING", log + "failed to read xml: %s (%s)" % (zip_file, e))
                return dict()

        # Parse content
        try:
            build_properties = ET.fromstring(xml).getiterator("buildproperties")[0]
        except:
            self.__logger.printLog("WARNING", log + "'buildproperties' key not present in xml")
            return dict()
        for p in build_properties.getiterator("property"):
            build_info[p.get("name")] = p.get("value")

        if not build_info:
            self.__logger.printLog("WARNING", log + "Failed to read version in %s" % zip_file)
        else:
            self.__logger.printLog("INFO", log + "done")

        return build_info

    def getVersionListFromXml(self, zip_file, flash_xml, group, print_output=True):
        log = "getVersionListFromXml({0}, {1}): ".format(zip_file, flash_xml)
        version_list = list()
        if not flash_xml.endswith("xml"):
            self.__logger.printLog("WARNING", log +  "not an xml file ({0})".format(flash_xml))
            return version_list
        try:
            with open(self.getZipContentList(zip_file, flash_xml)[0]) as f:
                xml = f.read()
        except Exception as e:
            self.__logger.printLog("WARNING", log + "failure to read '{0}' from '{1}' ({2})".format(flash_xml, os.path.basename(zip_file), e))
            return list()

        # Parse content
        for code_group in ET.fromstring(xml).getiterator("code_group"):
            if code_group.get("name") in group.split():
                for version in code_group.getiterator("version"):
                    version_list.append(version.text)

        if print_output:
            if version_list:
                self.__logger.printLog("INFO", log + "found: {0}".format(", ".join(version_list)))
            else:
                self.__logger.printLog("INFO", log + "nothing found")

        return version_list

    def getVersionFromXml(self, zip_file, flash_xml="flash.xml", group="KERNEL BOOTLOADER OTA system UPDATE"):
        """ Returns the build version from withing flash.xml in zip file
        """
        log = "getVersionFromXml({0}, {1}): ".format(zip_file, flash_xml)
        # If empty argument
        if not zip_file or zip_file == "None":
            self.__logger.printLog("WARNING", log + "missing argument")
            return ""
        # xml file parsing
        self.__logger.printLog("INFO", log + "parsing {0}".format(flash_xml))

        version_list = self.getVersionListFromXml(zip_file, flash_xml, group, print_output=False)

        if not version_list:
            self.__logger.printLog("WARNING", log + "failure to read version in {0}".format(zip_file))
            return ""

        # remove double values
        version_list = list(set(version_list))
        if len(version_list) != 1:
            self.__logger.printLog("WARNING", log + "cannot select a version, first used ({0})".format(version_list))

        self.__logger.printLog("INFO", log + "version = {0}".format(version_list[0]))
        return version_list[0]

    def getFileSearch(self, path, flash_type, buildvariant=""):
        """ Returns a string list "file1 file2 ...".
            path         : local working directory path
            flash_type   : type of file (fastboot, ota, blankphone)
            buildvariant : full name of buildvariant
        """
        if not buildvariant:
            pattern = "[A-Za-z0-9_-]+{0}.*.zip".format(flash_type)
        elif "blankphone" in flash_type:
            pattern = ".*blankphone.(zip|json)"
        else:
            pattern = "%s-%s-.*(zip|json)" % (buildvariant, flash_type)

        files = self.__misc.getPathFile(pattern, path)
        if files:
            return " ".join(files)
        else:
            return ""

    def getUrlList(self, url):
        """ returns list of html elements href
        """
        self.__logger.printLog("INFO", "parse %s" % url)

        try:
            html_string = urllib2.urlopen(url).read()
            # Get reversed list of WW/daily (latest on top)
            list_href = sorted(set(re.findall('href="([^\"]*)"', html_string)),
                               reverse=True)

            try:
                list_href.remove("DO_NOT_USE/")
            except:
                pass
            try:
                list_href.remove("../")
            except:
                pass
            try:
                list_href.remove("old.latest/")
            except:
                pass
            try:
                list_href.remove("testTemporary/")
            except:
                pass

            if not list_href:
                raise
            return list_href

        except Exception as e:
            self.__logger.printLog("WARNING", "can't list elements at %s (%s)" % (url, e))
            return None

    def getZipContentList(self, zipname, pattern=".*", depth=0):
        """ Returns the path file within the zip that matches with regexp.
            Zipname can be a zip file or a file inside.
        """
        log = "getZipContentList({}): ".format(pattern)

        # Check if zipname exists
        if not os.path.isdir(zipname) and not os.path.isfile(zipname):
            self.__logger.printLog("WARNING", log + "{0} does not exist".format(zipname))
            return list()

        zipname = os.path.abspath(zipname)
        # check whether files was already unzipped
        if zipname in self.fileMatchingTable:
            output_directory = self.fileMatchingTable[zipname]
        else:
            # build path where to unzip, according to nesting
            relative_path = ""
            full_path = zipname.replace(".zip", "")
            for i in range(depth + 1):
                relative_path = os.path.join(os.path.basename(full_path), relative_path)
                full_path = os.path.dirname(full_path)
            output_directory = os.path.abspath(os.path.join(self.pupdrTempDirectory, relative_path))

            # append index in case of duplicated names
            if os.path.isdir(output_directory):
                index = 1
                while os.path.isdir(output_directory + "_" + str(index)):
                    index += 1
                output_directory = output_directory + "_" + str(index)

            # add zip file and output directory into local dictionary to avoid unzipping several times the same file
            self.fileMatchingTable[zipname] = output_directory

        if not os.path.isdir(output_directory):
            self.__logger.printLog("INFO", log + "unzip '{0}' in '{1}'".format(zipname, output_directory))
            z = self.__misc.unzip(zipname)
            z.extractall(output_directory)
            z.close()

        # Search the file
        ret = self.getDirectoryContentList(output_directory, pattern, depth)

        self.__logger.printLog("INFO", log + "content = {}".format(ret))
        return ret

    def getDirectoryContentList(self, directory, pattern, depth, recursive=True):
        log = "getDirectoryContentList({}): ".format(pattern)
        output_list = []
        if not os.path.isdir(directory):
            return output_list
        for f in os.listdir(directory):
            element = os.path.join(directory, f)
            if os.path.isfile(element):
                if re.search(pattern, f):
                    output_list.append(os.path.join(directory, f))
            if f.endswith(".zip"):
                output_list.extend(self.getZipContentList(element, pattern, depth=depth+1))
            if recursive and os.path.isdir(element):
                output_list.extend(self.getDirectoryContentList(element, pattern, depth=depth+1))

        self.__logger.printLog("INFO", log + "content = {}".format(output_list))
        return output_list

    def getCflasherTimeout(self, zip_file, flash_xml=None, config="recover"):
        """ Get total cflasher timeout from flash xml file
        """
        log = "getCflasherTimeout(): "
        total_timeout = 0
        default_timeout = 3333
        xml_file = None
        json_file = None

        # xml file parsing
        self.__logger.printLog("INFO", log + "start".format(flash_xml))
        if not flash_xml:
            self.__logger.printLog("INFO", log + "finding flash file (json or xml)")
            flash_xml = self.getZipContentList(zip_file, "flash.(json|xml)")
            if not flash_xml:
                self.__logger.printLog("WARNING", "no flash file found in: {0}".format(zip_file))
                return default_timeout
            flash_xml = os.path.basename(flash_xml[0])
        self.__logger.printLog("INFO", log + "parsing: {0}".format(flash_xml))
        if not config:
            config = "recover"
        try:
            filename = self.getZipContentList(zip_file, flash_xml)[0]
            self.__logger.printLog("INFO", log + "file found: {0}".format(filename))
            with open(filename) as f:
                if filename.endswith("xml"):
                    xml_file = ET.fromstring(f.read())
                elif filename.endswith("json"):
                    json_file = json.load(f)
        except Exception as e:
            self.__logger.printLog("WARNING", log + "failure to parse: {0} ({1})".format(flash_xml, e))
            return default_timeout

        # Read timeout/retry values
        if filename.endswith("xml"):
            for command in xml_file.getiterator("command"):
                try:
                    timeout = int(command.findall("timeout")[0].text) / 1000
                    total_timeout += timeout
                # Failed
                except Exception as e:
                    self.__logger.printLog("WARNING", log + "failure to compute timeout ({0})".format(e))
                    return default_timeout
        elif filename.endswith("json"):
            # find and parse all commands
            if "flash" in json_file and "commands" in json_file["flash"]:
                for command in json_file["flash"]["commands"]:
                    # check in command correspond to current flash configuration
                    if (("restrict" in command and command["restrict"] and config in command["restrict"]) or "restrict" not in command) and "timeout" in command:
                        # add command timeout to total flash timeout
                        try:
                            timeout = int(command["timeout"]) / 1000
                            total_timeout += timeout
                        # Failed
                        except Exception as e:
                            self.__logger.printLog("WARNING", log + "failure to compute timeout ({0})".format(e))
                            return default_timeout

        # Final
        if total_timeout == 0:
            self.__logger.printLog("WARNING", log + "using default timeout %ss" % default_timeout)
            return default_timeout
        else:
            self.__logger.printLog("INFO", log + "using timeout %ss" % total_timeout)
            return total_timeout

    def confirmOtaUpdate(self):
        """ Method to confirm in popup the ota update even if the battery == low
        """
        log = "confirmOtaUpdate(): "
        self.__logger.printLog("INFO", log + "confirm otaUpdate in popup low battery")
        # Check if the board is in MOS to avoid adb timeout
        if self.__osManager.getOs() == "main":
            # turn on screen
            self.__device.switchScreenState(True)
            # Go ahead even if low battery (KEYCODE_DPAD_RIGHT + KEYCODE_ENTER)
            self.__device.keyEvent(['KEYCODE_DPAD_RIGHT', 'KEYCODE_DPAD_RIGHT', 'KEYCODE_ENTER'])
            time.sleep(3)
            # Check again if the board is in MOS to avoid adb timeout
            if self.__osManager.getOs() == "main":
                # Confirm
                self.__device.keyEvent(['KEYCODE_DPAD_RIGHT', 'KEYCODE_ENTER'])
                time.sleep(3)
                self.__device.keyEvent(['KEYCODE_DPAD_RIGHT', 'KEYCODE_ENTER'])
            else:
                self.__logger.printLog("INFO", log + "keyEvent confirmation aborted - Device is not in MOS")
        else:
            self.__logger.printLog("INFO", log + "keyEvent for low battery aborted - Device is not in MOS")
        self.__output.appendOutput("", None)

    def changePartitionTbl(self, flash_files, partition, size):
        """ Change the partition size within partition.tbl file in blankphone.zip
        """
        log = "changePartitionTbl(): "
        verdict = True
        output = ""
        blankphone_dir = ""
        blankphone = ""
        partition = partition.replace("/", "")
        self.__logger.printLog("INFO", log + "change /%s size to %s"
                          % (partition, size))
        self.__build_target = flash_files

        partition_tbl = "partition.tbl"
        status, local_file = self.__checkImageInWorkdir(partition_tbl)
        if not status:
            output = "failure to find file: {0}".format(partition_tbl)
            verdict = False
        else:
            blankphone_dir = os.path.dirname(local_file)
            blankphone = None
            for f in flash_files["flash_list"]:
                local_flash_file = self.__flashFile.createFile(f)
                if local_flash_file.get_file_type() == "blankphone":
                    blankphone = local_flash_file.get_localFlashFile()
            # If blankphone does not exist
            if not blankphone:
                self.__logger.printLog("WARNING", "'blankphone' entry not found in: {0}".format(flash_files["flash_list"]))
                output = "blankphone file not found in FOTA dictionary"
                verdict = False
            # If blankphone does not exist
            if not os.path.isfile(blankphone):
                output = "missing file: {0}".format(blankphone)
                verdict = False

        if not verdict:
            self.__output.appendOutput(output, False)
            return False

        # remove unzipped blankphone
        if os.path.isdir(blankphone_dir):
            self.__logger.printLog("INFO", log + "removing original files: {0}".format(blankphone_dir))
            shutil.rmtree(blankphone_dir)
        local_directory = blankphone.replace(".zip", "")
        if os.path.isdir(local_directory):
            self.__logger.printLog("INFO", log + "removing original files: {0}".format(local_directory))
            shutil.rmtree(local_directory)

        # Update partition.tbl within zip file
        try:
            out = re.sub(r'(.*-s )[^ ]*(.* -l ' + partition + r')', r'\g<1>' + size + r'\2',
                         self.__misc.unzip(blankphone).read(partition_tbl))
            temp_tbl = os.path.join(tempfile.gettempdir(), partition_tbl)
            self.__misc.local_files_handler.addEntry(temp_tbl)
            f = file(temp_tbl, 'w')
            f.write(out)
            f.close()
            z = self.__misc.unzip(blankphone, 'a')
            z.write(temp_tbl, arcname=partition_tbl)
            z.close()

            # Final check
            local_verdict = False
            for l in self.__misc.unzip(blankphone).read(partition_tbl).split("\n"):
                if ("-l " + partition) in l:
                    if re.search("-s[ ]*%s .* -l %s" % (size, partition), l):
                        local_verdict = True
                        self.__logger.printLog("INFO", log + l)
                    else: self.__logger.printLog("WARNING", log + l)
                else: self.__logger.printLog("INFO", log + l)

            if not local_verdict:
                raise Exception("check failed")

            self.__logger.printLog("INFO", log + "complete")

        # Exception
        except Exception as e:
            output = "failure to update partition.tbl (error={0})".format(e)
            verdict = False

        self.__output.appendOutput(output, verdict)
        return verdict

    def checkBlankphoneId(self, build_1, build_2, key):
        """ Returns the verdict of the comparison between blankphone_id's
        """
        info = "check_blankphone_id(%s, %s): " % (build_1["tag"], build_2["tag"])
        blankphone_id_prop = "ro.blankphone_id"

        if key in build_1 and key in build_2:
            if build_1[key] and build_2[key]:
                flash_xml = "flash.xml"
                z_1 = self.getZipContentList(build_1[key], flash_xml)
                z_2 = self.getZipContentList(build_2[key], flash_xml)

                # If flash xml are present
                flash_xml = "flash.xml"
                if flash_xml in z_1 and flash_xml in z_2:

                    # Read xml values
                    blankphone_id_1 = ""
                    blankphone_id_2 = ""
                    with open(z_1[0]) as f: xml_1 = f.read()
                    with open(z_2[0]) as f: xml_2 = f.read()
                    for p in ET.fromstring(xml_1).getiterator("property"):
                        if p.get("name") == blankphone_id_prop:
                            blankphone_id_1 = p.get("value")
                    for p in ET.fromstring(xml_2).getiterator("property"):
                        if p.get("name") == blankphone_id_prop:
                            blankphone_id_2 = p.get("value")

                    if blankphone_id_1 and blankphone_id_2:
                        # If versions are identical
                        if blankphone_id_1 == blankphone_id_2:
                            self.__logger.printLog("INFO", info + "same version")
                            return True

                        # Else versions differ
                        else:
                            OUTPUT = info + "versions differ"
                            self.__logger.printLog("INFO", OUTPUT)
                            # Send a mail to warn about blankphone_id upgrade
                            To = ",".join(["tonyx.raveneau@intel.com",
                                            "vincentx.delord@intel.com",
                                            "eliseax.cornejo@intel.com"])
                            From = "PSI PUPDR and Flashing TLS <psi.pupdr.and.flashing.tls@intel.com>"
                            self.__misc.sendMail(subject = "blankphone_id upgrade detected !!",
                                      text    = "The forward flashing from %s to %s requires a mandatory blankphone !"
                                                % (build_1["tag"], build_2["tag"]), From=From, To=To)
                            return False

                # Else at least one file is absent
                else:
                    OUTPUT = info + "file absent"
                    self.__logger.printLog("INFO", OUTPUT)
                    return True
        else:
            return True

    def leaveFlashingSoftly(self, build, img_check=True, ifwi_check=True):
        """ flash device before leaving TC when device not recoverable
        """
        log = "leave_flashing_softly({0}): ".format(build["tag"])
        verdict = True
        output = ""

        currentOs = self.__osManager.getOs(check_charger=False)
        if currentOs == "main":
            self.__logger.printLog("INFO", log + "Board detected in MOS")
        elif any (single_os in currentOs for single_os in ("recovery", "main")):
            self.__logger.printLog("INFO", log + "Board not in MOS after flashing. Try to reboot the board in MOS")
            self.__osManager.gotoMos()
        else:
            self.__logger.printLog("INFO", log + "Board not in MOS after flashing and will unlikely go to MOS, skipping gotoMos")
        if self.checkFlashedBuild(build, check_img=img_check, check_ifwi=ifwi_check, check_ssn=False, check_modem=False):
            self.__logger.printLog("INFO", log + "nothing to do")
        else:
            if not self.flash(build, "force;blank"):
                verdict = False
                output = "failure to flash board before leaving TC"
        self.__output.appendOutput(output, verdict, argument="build={0}".format(build["tag"]))
        return verdict
