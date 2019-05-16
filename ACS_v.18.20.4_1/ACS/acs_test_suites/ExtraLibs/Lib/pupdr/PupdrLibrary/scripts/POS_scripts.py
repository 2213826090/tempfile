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
@summary: Pupdr Library - POS scripts
@since: 07/01/2015
@author: travenex
"""

import re
import os
import json
import time
import shutil
from .. import TestCaseModule
import Reboot_scripts
import Check_scripts
import KeyPress_scripts
import Usb_scripts
import Flash_scripts

class POS(object):

    __instance = None
    __testCase = None

    def __new__(cls):
        """
        Constructor that always returns the same instance
        """
        if not cls.__instance:
            cls.__instance = object.__new__(cls)
        return cls.__instance

    def __init__(self):
        pass

    @classmethod
    def init(cls, globalConf=None):
        cls.__globalConf = globalConf
        cls.__testCase = TestCaseModule.TestCaseModule()

    def GVBDeviceStateTestCase(self):
        TestCase = POS.GVBDeviceStateScript(self.__globalConf)
        TestCase()
        return TestCase.verdict, TestCase.output_dict

    def FastbootRebootTestCase(self):
        TestCase = POS.FastbootRebootScript(self.__globalConf)
        TestCase()
        return TestCase.verdict, TestCase.output_dict

    def FastbootRebootBootloaderTestCase(self):
        TestCase = POS.FastbootRebootBootloaderScript(self.__globalConf)
        TestCase()
        return TestCase.verdict, TestCase.output_dict

    def FastbootRebootBootloaderIocTestCase(self):
        TestCase = POS.FastbootRebootBootloaderIocScript(self.__globalConf)
        TestCase()
        return TestCase.verdict, TestCase.output_dict

    def PosPkpOffWithUsbTestCase(self):
        TestCase = POS.PkpInPosScript(self.__globalConf)
        TestCase(plug=True)
        return TestCase.verdict, TestCase.output_dict

    def PosPkpOffWithoutUsbTestCase(self):
        TestCase = POS.PkpInPosScript(self.__globalConf)
        TestCase(plug=False)
        return TestCase.verdict, TestCase.output_dict

    def FastbootComboKeyTestCase(self):
        TestCase = POS.FastbootComboKeyScript(self.__globalConf)
        TestCase()
        return TestCase.verdict, TestCase.output_dict

    def FRPUnlockRestrictedTestCase(self):
        TestCase = POS.FRPUnlockRestrictedScript(self.__globalConf)
        TestCase()
        return TestCase.verdict, TestCase.output_dict

    def SafeBootloaderFlashingTestCase(self):
        TestCase = POS.SafeBootloaderFlashingScript(self.__globalConf)
        TestCase()
        return TestCase.verdict, TestCase.output_dict

    def VerifyFromOtaFilesTestCase(self):
        TestCase = POS.VerifyFromOtaFilesScript(self.__globalConf)
        TestCase()
        return TestCase.verdict, TestCase.output_dict

    class FastbootRebootScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "POS REBOOT - "
        def runAllSteps(self):
            self.runStep(Reboot_scripts.Reboot.AdbRebootProvisioningStep, parameter={"check_ssn": True})
            self.runStep(Check_scripts.Check.PosCheckAfterRebootStep)
            self.runStep(POS.FastbootRebootStep, run_on_failure=True)
            self.runStep(Check_scripts.Check.MosCheckAfterRebootFromPosStep, run_on_failure=True)

    class GVBDeviceStateScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "GVB - "
        def runAllSteps(self):
            if not self.configuration.boot.GVB_ACTIVATION:
                output = "GVB not enabled"
                self.logger.printLog("INFO", output)
                self.skipping = True
                self.toggleVerdict(self.VALID)
                self.output_dict["valid_log"] = output

            self.runStep(Flash_scripts.Flash.DownloadStep, parameter={"create_but": True})
            # boot in POS
            self.runStep(Reboot_scripts.Reboot.AdbRebootProvisioningStep, parameter={"check_ssn": False})
            self.runStep(POS.CheckVerifiedBootStep, parameter={"bootstate": "GREEN"})
            # enable the unlock
            self.runStep(POS.EnableUnlockStep)
            # check the device state verified
            if self.configuration.boot.GVB_ACTIVATION == "unlocked":
                self.runStep(POS.GetDeviceStateStep, parameter={"state": "unlocked"})
            elif self.configuration.boot.GVB_ACTIVATION == "verified":
                self.runStep(POS.GetDeviceStateStep, parameter={"state": "verified"})
            else:
                self.runStep(POS.GetDeviceStateStep, parameter={"state": "locked"})
            self.runStep(POS.CheckVerifiedBootStep, parameter={"bootstate": "GREEN"})
            # change the device state to unlock
            self.runStep(POS.ChangeDeviceStateStep, parameter={"state":"unlock"})
            # check the device state to unlocked
            self.runStep(POS.GetDeviceStateStep, parameter={"state": "unlocked"})
            self.runStep(POS.CheckVerifiedBootStep, parameter={"bootstate": "GREEN"})
            # change the device state to lock
            self.runStep(POS.ChangeDeviceStateStep, parameter={"state": "lock"})
            # check the device state to locked
            self.runStep(POS.GetDeviceStateStep, parameter={"state": "locked"})
            if self.configuration.boot.GVB_ACTIVATION == "verified":
                # change the device state to verified
                self.runStep(POS.ChangeDeviceStateStep, parameter={"state": "verified"}, run_on_failure=True)
                # Check  the device state to verified
                self.runStep(POS.GetDeviceStateStep, parameter={"state": "verified"}, run_on_failure=True)
            elif self.configuration.boot.GVB_ACTIVATION == "unlocked":
                # change the device state to verified
                self.runStep(POS.ChangeDeviceStateStep, parameter={"state": "unlock"}, run_on_failure=True)
                # Check  the device state to verified
                self.runStep(POS.GetDeviceStateStep, parameter={"state": "unlocked"}, run_on_failure=True)
            self.runStep(POS.CheckVerifiedBootStep, parameter={"bootstate": "GREEN"})
            # Reboot MOS
            self.runStep(POS.FastbootRebootStep, parameter={"wait_main": False}, run_on_failure=True)
            # Wait for wipe data ROS and boot MOS
            self.runStep(Usb_scripts.Usb.PlugAndWaitOsStep,
                            parameter={"expectedOs":"main",
                                       "timeout": self.configuration.timeout.MOS_BOOT+self.configuration.timeout.ROS_BOOT+600+self.configuration.timeout.MOS_BOOT,
                                       "plug": False, "blind": True},
                         run_on_failure=True)
            if not self.verdict:
                self.runStep(Flash_scripts.Flash.LeaveFlashingSoftlyStep, parameter={"build": self.download.build_but},
                             impact_tc_verdict=False,
                             run_on_failure=True)

    class FastbootRebootBootloaderScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "POS REBOOT BOOTLOADER - "
        def runAllSteps(self):
            self.runStep(Reboot_scripts.Reboot.AdbRebootProvisioningStep, parameter={"check_ssn": True})
            self.runStep(POS.FastbootRebootBootloaderStep)
            self.runStep(Check_scripts.Check.PosCheckAfterRebootStep)
            self.runStep(POS.FastbootRebootStep, run_on_failure=True)
            self.runStep(Check_scripts.Check.MosCheckAfterRebootFromPosStep, run_on_failure=True)

    class FastbootRebootBootloaderIocScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "POS REBOOT BOOTLOADER IOC - "
        def runAllSteps(self):
            self.logger.printLog("INFO", "Disable init and final")
            self.enable_init = False
            self.enable_final = False
            self.runStep(Reboot_scripts.Reboot.RebootBxtpMrbStep)
            self.runStep(Reboot_scripts.Reboot.AdbRebootProvisioningIocStep)
            self.runStep(Reboot_scripts.Reboot.AdbRebootProvisioningIocStep)
            self.runStep(Check_scripts.Check.PosCheckAfterRebootStep)
            self.runStep(Reboot_scripts.Reboot.RebootBxtpMrbStep)
            self.runStep(Check_scripts.Check.MosCheckAfterRebootFromPosStep, run_on_failure=True)

    class PkpInPosScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "POS PKP - "
        def runAllSteps(self, plug=True):
            if not self.name:
                self.name = "POS PKP {0}S {1}- ".format(self.configuration.timeout.PKPOFF, "UNPLUG " if not plug else "")
            variant = self.device.getProperty("ro.build.type")
            self.runStep(Reboot_scripts.Reboot.AdbRebootProvisioningStep, parameter={"check_ssn": True})
            self.runStep(KeyPress_scripts.KeyPress.ForceShutdownStep, parameter={"plug": plug})
            self.runStep(Usb_scripts.Usb.UsbUnplugReplugStep, parameter={"duration": self.configuration.timeout.MOS_BOOT, "unplug": False, "replug": False})
            if self.configuration.boot.BOOT_COS_ON_FORCED_SHUTDOWN_PLUG and plug and variant != "user" and not self.configuration.boot.NOCOS:
                self.runStep(Usb_scripts.Usb.PlugAndGetOsStep, parameter={"expectedOs": "charger", "replug": not plug, "wait_charger": not plug})
            else:
                self.runStep(Usb_scripts.Usb.PlugAndGetOsStep, parameter={"expectedOs": "offline", "replug": not plug, "wait_charger": not plug})

    class FastbootComboKeyScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "POS COMBO KEY - "
        def runAllSteps(self):
            self.runStep(POS.FastbootComboKeyStep)

    class FRPUnlockRestrictedScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "FRP - "
        def runAllSteps(self):
            if self.device.getProperty("ro.build.type") != "user":
                output = "not a user build"
                self.logger.printLog("INFO", output)
                self.skipping = True
                self.toggleVerdict(self.VALID)
                self.output_dict["valid_log"] = output
            # boot in POS
            self.runStep(Reboot_scripts.Reboot.AdbRebootProvisioningStep, parameter={"check_ssn": False})
            # check unlock command is not allowed on user build
            self.runStep(POS.FRPUnlockRestrictedStep)

    class SafeBootloaderFlashingScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "SECURE FLASHING - "
        def runAllSteps(self):
            if not self.configuration.flash.UEFI:
                output = "not an UEFI board"
                self.logger.printLog("INFO", output)
                self.skipping = True
                self.toggleVerdict(self.VALID)
                self.output_dict["valid_log"] = output
            self.runStep(Flash_scripts.Flash.DownloadStep, parameter={"create_but": True})
            self.runStep(POS.BootloaderCorruptionStep, parameter={"build": self.download.build_but})
            self.runStep(POS.CheckBootloaderSafeFlashingStep, parameter={"build": self.download.build_but})

    class VerifyFromOtaFilesScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "Verify - "
        def runAllSteps(self):
            self.runStep(Flash_scripts.Flash.DownloadStep, parameter={"but_ota_files_needed": True})
            self.runStep(Reboot_scripts.Reboot.AdbRebootProvisioningStep, parameter={"check_ssn": True})
            self.runStep(POS.VerifyFromOtaFilesStep)

    class FastbootRebootStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Fastboot Reboot"
            self.description = "launch 'fastboot reboot' command to reboot the board from POS to MOS"
            self.defaultParameter = {"wait_main": True}
        def step(self, parameter):
            wait_main = parameter["wait_main"]
            if wait_main:
                self.description += " and wait for MOS"
            if not self.osManager.fastbootReboot(wait_main):
                self.verdict = False

    class FastbootRebootBootloaderStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Fastboot Reboot Bootloader"
            self.description = "launch 'fastboot reboot-bootloader' command to reboot the board in POS"
        def step(self, parameter):
            if not self.osManager.fastbootRebootBootloader():
                self.verdict = False

    class EnableUnlockStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Fastboot Enable Unlock"
            self.description = "Launch 'fastboot oem enable_oem_unlock' command and check command output"
        def step(self, parameter):
            exec_status, output = self.host.commandExecFastboot("oem enable_oem_unlock")
            if exec_status != 0 or "FAILED" in output:
                self.output = "failure to enable device unlocking with 'fastboot oem enable_oem_unlock' command"

    class GetDeviceStateStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Fastboot Getvar Device State"
            self.description = "launch 'fastboot getvar device_state' command and check command output"
            self.defaultParameter = {"state": "verified"}
        def step(self, parameter):
            state = parameter["state"]
            allowed_list = ("locked", "unlocked", "verified")
            if state not in allowed_list:
                self.output = "invalid expected state ('{0}' not in: {1})".format(state, ", ".join(allowed_list))
                self.verdict = False
            else:
                self.description = "launch 'fastboot getvar device_state' command and check '{}' state".format(state)
                exec_status, output = self.host.commandExecFastboot("getvar device-state")
                if exec_status != 0 or "FAILED" in output or not re.search("device-state: {0}".format(state), output):
                    exec_status, output = self.host.commandExecFastboot("getvar device_state")
                    if exec_status != 0 or "FAILED" in output or not re.search("device_state: {0}".format(state), output):
                        self.verdict = False
                        self.output = "failure with 'fastboot getvar device_state' output check, '{0}' state expected".format(state)
                if self.verdict:
                    self.logger.printLog("INFO", "'{0}' state successfully checked".format(state))

    class ChangeDeviceStateStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Change Device Security State"
            self.description = "launch fastboot lock/unlock/verified command to change board state"
            self.defaultParameter = {"state": "verified"}
        def step(self, parameter):
            state = parameter["state"]
            allowed_list = ("lock", "unlock", "verified")
            if state not in allowed_list:
                self.output = "invalid expected state ('{0}' not in: {1})".format(state, ", ".join(allowed_list))
                self.verdict = False
            else:
                self.description = "launch fastboot lock/unlock/verified command to change board into '{}' state".format(state)
                if self.configuration.boot.GVB_ACTIVATION.endswith("-M"):
                    exec_status, output = self.host.commandExecFastboot("flashing {0}".format(state), 180)
                else:
                    exec_status, output = self.host.commandExecFastboot("oem {0}".format(state), 180)
                if exec_status != 0 or "FAILED" in output:
                    self.verdict = False
                    self.output = "failure to change device state with command 'fastboot oem/flashing {0}'".format(state)

    class CheckVerifiedBootStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Check Verified Boot State"
            self.description = "launch adb shell getprop ro.boot.verifiedbootstate"
            self.defaultParameter = {"bootstate": "GREEN"}
        def step(self, parameter):
            bootstate = parameter["bootstate"]
            allowed_list = ("GREEN", "YELLOW", "ORANGE", "RED")
            if bootstate not in allowed_list:
                self.output = "invalid expected state ('{0}' not in: {1})".format(bootstate, ", ".join(allowed_list))
                self.verdict = False
            else:
                self.description = "launch adb shell getprop ro.boot.verifiedbootstate to check {} state".format(bootstate)
                exec_status, output = self.host.commandExecFastboot("getvar boot-state")
                if exec_status != 0 or "FAILED" in output:
                    self.verdict = False
                    self.output = "failure to launch the command 'fastboot getvar boot-state'"
                else:
                    formatted_output = list()
                    for line in output.split("\n"):
                        if not line.isspace():
                            formatted_output.append(line)
                    out = formatted_output[0].split(" ")
                    stateLine = list()
                    for element in out:
                        if element != "":
                            stateLine.append(element)
                    actual_boot_state = stateLine[-1:][0]
                    if actual_boot_state == bootstate:
                        self.verdict = True
                        self.output = "Expected boot state = actual boot state = {}".format(bootstate)
                    else:
                        self.verdict = False
                        self.output = "Expected boot state: {} does not match actual boot state: {}".format(bootstate, actual_boot_state)

    class FastbootFormatStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Format Partition"
            self.description = "format specified partition using 'fastboot format' command"
            self.defaultParameter = {"partition": ""}
        def step(self, parameter):
            partition = parameter["partition"]
            if not partition:
                self.output = "partition name empty"
                self.verdict = False
            else:
                self.description = "format '{}' partition using 'fastboot format' command".format(parameter)
                exec_status, output = self.host.commandExecFastboot("format {0}".format(partition), 90)
                if exec_status != 0 or "FAILED" in output:
                    self.verdict = False
                    self.output = "failure to format {0}".format(partition)

    class FastbootComboKeyStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Fastboot Combo Key"
            self.description = "use fastboot key combo to boot POS from OFF"
            self.defaultParameter = {}
        def step(self, parameter):
            current_os = self.osManager.getOs()
            if not current_os == "main":
                self.output = "starting step with wrong OS (expected='main' , actual='{0}')".format(current_os)
                self.verdict = False
            else:
                self.osManager.posCombo()
                self.verdict = self.osManager.waitOs("fastboot")

    class FRPUnlockRestrictedStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "FRP unlock device"
            self.description = "check unlock device is not allowed on user build"
            self.defaultParameter = {}
        def step(self, parameter):
            exec_status, output = self.device.gvbUnlock()
            if exec_status != 0 and "FAILED (remote: Unlocking device not allowed)" in output:
                self.verdict = True
            else:
                self.device.gvbLock()
                self.verdict = False
                self.output = "device could be unlocked"

    class BootloaderCorruptionStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Corrupt Bootloader"
            self.description = "generate corrupted bootloader"
            self.defaultParameter = {"build": {}}
        def step(self, parameter):
            build = parameter["build"]
            if not build or "flash_list" not in build:
                self.verdict = False
                self.output = "no build provided"
                return

            # find bootloader in flash files
            bootloader_list = []
            for f in build["flash_list"]:
                local_flash_file = self.flashFile.createFile(f)
                bootloader_list = self.flash.getZipContentList(local_flash_file.get_localFlashFile(), "^bootloader$")
                if bootloader_list:
                    break
            if len(bootloader_list) != 1:
                self.verdict = False
                self.output = "zero or multiple bootloader files found"
                return
            bootloader = bootloader_list[0]

            # extract manifest to get efi app list
            generation_path = self.logs.bootota_log_path
            manifest_path = os.path.join(generation_path, "manifest.txt")
            self.misc.local_files_handler.addEntry(manifest_path)
            self.logger.printLog("INFO", "get manifest from bootloader image at {0}".format(bootloader))
            status, output = self.host.commandExec("mcopy -i {0} ::/manifest.txt {1}".format(bootloader, generation_path))
            if status != 0:
                self.verdict = False
                self.output = "failure to extract manifest from {0}".format(bootloader)
                return
            if not os.path.isfile(manifest_path):
                self.verdict = False
                self.output = "extracted file not found {0}".format(manifest_path)
                return

            # get efi app list from manifest file
            found_efi_apps = list()
            with open(manifest_path) as f:
                for line in f.readlines():
                    if len(line.split("=")) == 2:
                        app_name = "::" + line.split("=")[1].replace("\\", "/")
                        app_name = re.search("[A-Za-z0-9\.:/]+", app_name).group(0)
                        found_efi_apps.append(app_name)
                        self.logger.printLog("DEBUG", "found {0}".format(found_efi_apps[-1]))
                    else:
                        self.logger.printLog("DEBUG", "nothing found from '{}'".format(line))

            # check mandatory elements found
            for mandatory_element in self.configuration.flash.BOOTLOADER_CONTENT:
                if mandatory_element not in found_efi_apps:
                    self.verdict = False
                    self.output = "mandatory element in bootloader not found: {}".format(mandatory_element)
                    return
                else:
                    self.logger.printLog("DEBUG", "checked mandatory element presence for {0}".format(mandatory_element))

            file_name = "pupdr_corrupted_bootloader"

            # delete older corrupted files if any found
            for element in os.listdir(generation_path):
                if element.startswith(file_name):
                    os.remove(os.path.join(generation_path, element))

            # generate corrupted bootloader
            corrupted_bootloaders = []
            local_app_path = os.path.join(generation_path, "corrupted_app.efi")
            self.misc.local_files_handler.addEntry(local_app_path)
            for index, single_efi_app in enumerate(found_efi_apps):
                target_bootloader = os.path.join(generation_path, file_name + str(index))
                self.misc.local_files_handler.addEntry(target_bootloader)
                self.logger.printLog("DEBUG", "extract {} app and generate corrupted bootloader: "
                                              "{}".format(single_efi_app,
                                                          os.path.basename(target_bootloader)))

                if os.path.exists(local_app_path):
                    os.remove(local_app_path)
                # extract app from bootloader
                exec_status, _ = self.host.commandExec("mcopy -i {0} {1} {2}".format(bootloader,
                                                                                     single_efi_app,
                                                                                     local_app_path))
                if exec_status != 0:
                    self.logger.printLog("WARNING", "failure with mcopy command, cannot extract content")
                    self.verdict = False
                    return

                # corrupt app
                exec_status, _ = self.host.commandExec("dd if=/dev/urandom of={} bs=1024 count=1 conv=notrunc".format(local_app_path))
                if exec_status != 0:
                    self.logger.printLog("WARNING", "failure with dd command, cannot corrupt app")
                    self.verdict = False
                    return

                # copy original bootloader
                shutil.copy(bootloader, target_bootloader)

                # remove app in bootloader
                exec_status, _ = self.host.commandExec("mdel -i {0} {1}".format(target_bootloader, single_efi_app))
                if exec_status != 0:
                    self.logger.printLog("WARNING", "failure with mdel command, cannot delete content content")
                    self.verdict = False
                    return

                # copy corrupted app in bootloader
                exec_status, _ = self.host.commandExec("mcopy -i {0} {1} {2}".format(target_bootloader,
                                                                                 local_app_path,
                                                                                 single_efi_app))
                if exec_status != 0:
                    self.logger.printLog("WARNING", "failure with mdel command, cannot delete content content")
                    self.verdict = False
                    return

                # add entry in dictionary
                corrupted_bootloaders.append({
                    "corrupted_bootloader_path": target_bootloader,
                    "corrupted_efi_app": single_efi_app
                })

            # store content in build
            build["corrupted_bootloaders"] = corrupted_bootloaders
            build["original_bootloader"] = bootloader

    class CheckBootloaderSafeFlashingStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Check Safe Flashing"
            self.description = "flash corrupted bootloader and check flashing commands fail then end up flashing valid bootloader"
            self.defaultParameter = {"build": {}}

        def exitTc(self, bootloader):
            self.logger.printLog("INFO", "returning to MOS before exiting TC")
            if bootloader:
                self.logger.printLog("INFO", "flashing original bootloader before leaving")
                self.host.commandExecFastboot("flash bootloader {0}".format(bootloader), 60)
            self.host.commandExecFastboot("format data", 90)
            self.device.gvbLock()
            self.osManager.fastbootReboot(wait_main=False)
            if not self.osManager.waitOs("main",
                                         waitWdExp=False,
                                         timeout=self.configuration.timeout.BOOT_TIMEOUT,
                                         blind=True):
                self.osManager.gotoMos()
            self.device.adbRoot()

        def step(self, parameter):
            build = parameter["build"]

            if not build or "corrupted_bootloaders" not in build \
                or not build["corrupted_bootloaders"] \
                or "original_bootloader" not in build:
                self.verdict = False
                self.output = "no build provided or missing 'corrupted_bootloaders', 'original_bootloader' entries"
                self.exitTc(None)
                return

            # go to POS
            if not self.osManager.gotoPos():
                self.verdict = False
                self.exitTc(build["original_bootloader"])
                return

            # unlock device
            exec_status, _ = self.device.gvbUnlock()
            if exec_status != 0:
                self.output = "failure to unlock device"
                self.verdict = False
                self.exitTc(build["original_bootloader"])
                return

            # flash all corrupted bootloader and check flashing fails
            error_list = []
            for corrupted_bootloader in build["corrupted_bootloaders"]:
                self.logger.printLog("INFO", "flashing corrupted bootloader and expect failed command")
                # press volume key after launching command because bios hangs upon corrupted efi app
                self.relayCard.runThread(["time.sleep(30)",
                                          "self.enableLine(self.relayConfiguration['VolumeUp'])",
                                          "time.sleep(0.7)",
                                          "self.disableLine(self.relayConfiguration['VolumeUp'])"])
                exec_status, out = self.host.commandExecFastboot("flash bootloader {0}".format(corrupted_bootloader["corrupted_bootloader_path"]), 60)
                if "TIMEOUT" in out:
                    self.logger.printLog("WARNING", "Command FAILED due to TIMEOUT")
                    self.verdict = False
                elif exec_status == 0:
                    error_log = "successfully flashed bootloader with corrupted {0}".format(corrupted_bootloader["corrupted_efi_app"])
                    error_list.append(error_log)
                    self.logger.printLog("WARNING", error_log)
            if error_list:
                self.verdict = False
                self.output = " - ".join(error_list)
                self.exitTc(build["original_bootloader"])
                return

            self.logger.printLog("INFO", "flashing original bootloader before leaving")
            exec_status, _ = self.host.commandExecFastboot("flash bootloader {0}".format(build["original_bootloader"]), 60)
            if exec_status != 0:
                self.output = "failure to flash original bootloader"
                self.verdict = False

            # exit POS and return to MOS
            self.exitTc("")

    class VerifyFromOtaFilesStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Verify From OTA Files"
            self.description = "Given a ota-files zipfile, connect to the device over Fastboot and verify that the installed system exactly matches that build"
        def step(self, parameter):
            workdir = self.globalConf.get("WORKDIR")
            info_path = os.path.join(workdir, "BUT", "ota.json")
            info_json_data = {}
            try:
                with open(info_path) as f:
                    info_json_data = json.load(f)
            except Exception as e:
                output = "failure to parse ota json output file {} (error={})".format(info_path, e)
                verdict = False
            zip_file = info_json_data.get("localFlashFile","")
            verify_path = os.path.join(zip_file.replace(".zip", ""), "verify")
            if not os.path.isdir(verify_path):
                self.output = "no verify directory found"
                self.verdict = False
                return
            self.host.commandExec("chmod 755 {}/fastboot".format(verify_path))
            self.host.commandExec("chmod 755 {}/verify_from_ota".format(verify_path))
            execute_path = os.getcwd()
            verify_execute_path = execute_path.replace("execute", "verify")
            if os.path.isdir(verify_execute_path):
                shutil.rmtree(verify_execute_path)
            shutil.copytree(verify_path, verify_execute_path, False)
            os.chdir(verify_execute_path)
            exec_status, output = self.host.commandExec("python verify_from_ota", 900)
            os.chdir(execute_path)
            shutil.rmtree(verify_execute_path)
            if "All tests completed successfully" in output:
                self.verdict = True
            else:
                self.verdict = False
