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

@organization: INTEL SSG
@summary: Pupdr Library - RMA scripts
@since: 26/05/2016
@author: chenyu
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
import POS_scripts
import thread
from .. import ConfigurationModule
from .. import DeviceModule
from .. import RelayCardModule

class RMA(object):

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
        cls.__configuration = ConfigurationModule.ConfigurationModule()

    def RMAUnlockRestrictedBTestCase(self, flash_files, force=True, build_tag_override=""):
        TestCase = RMA.RMAUnlockRestrictedBScript(self.__globalConf)
        TestCase(flash_files=flash_files, force=force, build_tag_override=build_tag_override)
        return TestCase.verdict, TestCase.output_dict

    def RMAUnlockRestrictedATestCase(self, flash_files, force=True, build_tag_override=""):
        TestCase = RMA.RMAUnlockRestrictedAScript(self.__globalConf)
        TestCase(flash_files=flash_files, force=force, build_tag_override=build_tag_override)
        return TestCase.verdict, TestCase.output_dict



    class RMAUnlockRestrictedBScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "RMA Unlock Restricted - "
        def runAllSteps(self, flash_files=list(), force=True, build_tag_override=""):
            #step 0: use fastboot key combo to boot POS from OFF
            self.runStep(RMA.AdbRebootFastbootStep)
            #step 1: fastboot flashing get_unlock_ability
            self.runStep(RMA.FastbootFlashingGetUnlockAbility1Step)
            #step 2:fastboot flashing unlock
            self.runStep(RMA.FastbootFlashingUnlockStep)
            #step 3:fastboot oem get-action-nonce
            if len(flash_files) == 1 and self.misc.isExpectedJsonFormat(
                    flash_files[0], ["build_target", "build_variant", "board_type",
                                     "url_buildinfo", "timeout","download_timeout"],
                    top_keys=["provisioning_properties"]):
                self.runStep(Flash_scripts.Flash.ProcessProvisioningJsonStep, parameter={"provisioning_json_file": flash_files[0]})
                self.runStep(Flash_scripts.Flash.DownloadStep, parameter={"download_rma_tools": True})
                self.runStep(RMA.FastbootOemGetActionNonceForceUnlockAndGenPkcs7Step)
            #step 4:fastboot getvar unlocked
            self.runStep(RMA.FastbootGetvarUnlockedYesStep)
            #step 5:fastboot reboot to MOS
            self.runStep(RMA.FastbootRebootStep)
            #step 6:Prepare device to be reprovisioned should enter dnx and ready flash
            self.runStep(RMA.AdbRebootFastbootStep)
            self.runStep(RMA.FastbootOemvarsDeleteTxtStep)
            if len(flash_files) == 1 and self.misc.isExpectedJsonFormat(
                    flash_files[0], ["build_target", "build_variant", "board_type",
                                     "url_buildinfo", "timeout","download_timeout"],
                    top_keys=["provisioning_properties"]):
                self.runStep(Flash_scripts.Flash.ProcessProvisioningJsonStep, parameter={"provisioning_json_file": flash_files[0]})
                self.runStep(Flash_scripts.Flash.DownloadStep, parameter={"download_rma_tools": False})

            else:
                self.runStep(Flash_scripts.Flash.BuildButFromFlashFilesStep, parameter={"flash_files": flash_files})
            if not force:
                flash_key = "recover"
                self.enable_init = False
            else:
                flash_key = "force;recover"
            self.runStep(Flash_scripts.Flash.FlashStep, parameter={"build": self.download.build_but,"flash_key": flash_key})
            if self.download.getBranchFromTag(str(self.download.build_but)) == "mmsd":
                self.runStep(Flash_scripts.Flash.CheckFlashBuildStep, parameter={"build": self.download.build_but,"check_tag": False,
                                                                      "check_ifwi": True, "check_modem": False,
                                                                      "check_img": True, "check_ssn": False,
                                                                      "build_tag_override": build_tag_override})
            elif self.download.getBranchFromTag(str(self.download.build_but)) != "demo":
                self.runStep(Flash_scripts.Flash.CheckFlashBuildStep, parameter={"build": self.download.build_but, "check_tag": True,
                                                                   "check_ifwi": True, "check_modem": False,
                                                                   "check_img": True, "check_ssn": False,
                                                                   "build_tag_override": build_tag_override})
            #flash end
            #setp 7:fastboot getvar unlocked
            self.runStep(RMA.AdbRebootFastbootStep)
            self.runStep(RMA.FastbootGetvarUnlockedNoStep)
            #step 8:fastboot flashing get_unlock_ability
            self.runStep(RMA.FastbootFlashingGetUnlockAbility1Step)
            #step 9:fastboot oem get-action-nonce force-unlock
            self.runStep(RMA.FastbootOemGetActionNonceForceUnlockStep)
            #step 10:fastboot flash action-authorization pkcs7.bin
            self.runStep(RMA.FastbootFlashActionAuthorizationPkcs7Step)
            #step 11:fastboot getvar unlocked
            self.runStep(RMA.FastbootGetvarUnlockedNoStep)
            #step 12:fastboot flashing unlock
            self.runStep(RMA.FastbootFlashingUnlockStep)

    class RMAUnlockRestrictedAScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "RMA Unlock Restricted - "
        def runAllSteps(self, flash_files=list(), force=True, build_tag_override=""):
            #step 0: adb reboot to fastboot
            self.runStep(RMA.AdbRebootFastbootStep)
            #step 1:fastboot oem get-action-nonce
            if len(flash_files) == 1 and self.misc.isExpectedJsonFormat(
                    flash_files[0], ["build_target", "build_variant", "board_type",
                                     "url_buildinfo", "timeout","download_timeout"],
                    top_keys=["provisioning_properties"]):
                self.runStep(Flash_scripts.Flash.ProcessProvisioningJsonStep, parameter={"provisioning_json_file": flash_files[0]})
                self.runStep(Flash_scripts.Flash.DownloadStep, parameter={"download_rma_tools": True})
                self.runStep(RMA.FastbootOemGetActionNonceForceUnlockAndGenPkcs7Step)
            #step 2:fastboot getvar unlocked
            self.runStep(RMA.FastbootGetvarUnlockedYesStep)
            #step 3:Generate a new bootloader policy to set the device in class A
            self.runStep(RMA.GenerateNewBootloaderPolicyOemvarsTxtFileStep)
            #step 4:fastboot flash oemvars bootloader_policy-oemvars.txt
            self.runStep(RMA.FastbootFlashOemvarsPolicyStep)
            #step 5:fastboot flashing lock.
            self.runStep(RMA.FastbootFlashingLockStep)
            self.runStep(RMA.FastbootGetvarUnlockedNoStep)
            #setp 6:fastboot flashing get_unlock_ability
            self.runStep(RMA.FastbootFlashingGetUnlockAbility2Step)
            #step 7:Enable OEM unlock option
            self.runStep(RMA.FastbootRebootStep)
            self.runStep(RMA.AdbEnableDevelopmentStep)
            self.runStep(RMA.AdbEnableOemUnlockStep)
            #step 8:fastboot flashing unlock
            self.runStep(RMA.AdbRebootFastbootStep)
            self.runStep(RMA.FastbootFlashingUnlockStep)
            #step 9:fastboot erase persistent to disable oem unlock
            self.runStep(RMA.FastbootRebootStep)
            self.runStep(RMA.AdbDisableOemUnlockStep)
            #step 10:fastboot oem get-action-nonce
            self.runStep(RMA.AdbRebootFastbootStep)
            self.runStep(RMA.FastbootOemGetActionNonceForceUnlockAndGenPkcs7Step)
            #step 11:fastboot getvar unlocked
            self.runStep(RMA.FastbootGetvarUnlockedYesStep)
            #step 12:Prepare device to be reprovisioned should enter dnx and ready flash
            self.runStep(RMA.FastbootOemvarsDeleteTxtStep)

            if len(flash_files) == 1 and self.misc.isExpectedJsonFormat(
                    flash_files[0], ["build_target", "build_variant", "board_type",
                                     "url_buildinfo", "timeout","download_timeout"],
                    top_keys=["provisioning_properties"]):
                self.runStep(Flash_scripts.Flash.ProcessProvisioningJsonStep, parameter={"provisioning_json_file": flash_files[0]})
                self.runStep(Flash_scripts.Flash.DownloadStep, parameter={"download_rma_tools": False})
            else:
                self.runStep(Flash_scripts.Flash.BuildButFromFlashFilesStep, parameter={"flash_files": flash_files})
            if not force:
                flash_key = "recover"
                self.enable_init = False
            else:
                flash_key = "force;recover"
            self.runStep(Flash_scripts.Flash.FlashStep, parameter={"build": self.download.build_but,"flash_key": flash_key})
            if self.download.getBranchFromTag(str(self.download.build_but)) == "mmsd":
                self.runStep(Flash_scripts.Flash.CheckFlashBuildStep, parameter={"build": self.download.build_but,"check_tag": False,
                                                                      "check_ifwi": True, "check_modem": False,
                                                                      "check_img": True, "check_ssn": False,
                                                                      "build_tag_override": build_tag_override})
            elif self.download.getBranchFromTag(str(self.download.build_but)) != "demo":
                self.runStep(Flash_scripts.Flash.CheckFlashBuildStep, parameter={"build": self.download.build_but, "check_tag": True,
                                                                   "check_ifwi": True, "check_modem": False,
                                                                   "check_img": True, "check_ssn": False,
                                                                   "build_tag_override": build_tag_override})
            #flash end
            #setp 13:fastboot getvar unlocked
            self.runStep(RMA.AdbRebootFastbootStep)
            self.runStep(RMA.FastbootGetvarUnlockedNoStep)
            #step 14.1:fastboot oem get-action-nonce
            self.runStep(RMA.FastbootOemGetActionNonceForceUnlockAndGenPkcs7Step)
            #step 14.2:fastboot getvar unlocked
            self.runStep(RMA.FastbootGetvarUnlockedYesStep)
            #step 15.1:fastboot flash oemvars bootloader_policy-oemvars.txt
            self.runStep(RMA.FastbootFlashOemvarsPolicyStep)
            #step 15.2:fastboot flashing lock
            self.runStep(RMA.FastbootFlashingLockStep)
            self.runStep(RMA.FastbootGetvarUnlockedNoStep)
            #setp 15.3:fastboot flashing get_unlock_ability
            self.runStep(RMA.FastbootFlashingGetUnlockAbility2Step)
            #step 15.4:Enable OEM unlock option
            self.runStep(RMA.FastbootRebootStep)
            self.runStep(RMA.AdbEnableDevelopmentStep)
            self.runStep(RMA.AdbEnableOemUnlockStep)
            #step 16:fastboot oem get-action-nonce force-unlock
            self.runStep(RMA.AdbRebootFastbootStep)
            self.runStep(RMA.FastbootOemGetActionNonceForceUnlockStep)
            #step 17:fastboot flash action-authorization pkcs7.bin
            self.runStep(RMA.FastbootFlashActionAuthorizationPkcs7Step)
            #setp 18:fastboot getvar unlocked
            self.runStep(RMA.FastbootGetvarUnlockedNoStep)
            #setp 19:fastboot flashing unlock
            self.runStep(RMA.FastbootFlashingUnlockStep)


    class AdbRebootFastbootStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "adb reboot fastboot"
            self.description = "Launch 'adb reboot fastboot' command and wait for pos mode"
        def step(self, parameter):
            current_os = self.osManager.getOs()
            if not current_os == "main":
                self.output = "starting step with wrong OS (expected='main' , actual='{0}')".format(current_os)
                self.verdict = False
            else:
                self.host.commandExecAdb("reboot fastboot")
                self.verdict = self.osManager.waitOs("fastboot")

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
                self.osManager.waitOs("main")

    class FastbootOemvarsDeleteTxtStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "fastboot flash oemvars"
            self.description = "Launch 'fastboot flash oemvars' command and wait for dnx mode"
        def step(self, parameter):
            self.host.commandExecFastboot("flashing unlock")
            self.host.commandExecFastboot("flash oemvars ../../library/Extras/blpolicy-delete.txt")
            exec_status, output = self.host.commandExecFastboot("oem reboot dnx")
            if exec_status != 0 or "FAILED" in output:
                self.output = output
                self.verdict = False
            else:
                self.output = output
                self.verdict = True

    class FastbootFlashingGetUnlockAbility1Step(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "fastboot flashing get_unlock_ability"
            self.description = "Launch 'fastboot flashing get_unlock_ability' command and check command output"
        def step(self, parameter):
            exec_status, output = self.host.commandExecFastboot("flashing get_unlock_ability")
            if exec_status != 0 or "FAILED" in output:
                self.output = output
                self.verdict = False
            elif "Unlock is disabled" in output:
                self.output = output
                self.verdict = True

    class FastbootFlashingGetUnlockAbility2Step(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "fastboot flashing get_unlock_ability"
            self.description = "Launch 'fastboot flashing get_unlock_ability' command and check command output"
        def step(self, parameter):
            exec_status, output = self.host.commandExecFastboot("flashing get_unlock_ability")
            if exec_status != 0 or "FAILED" in output:
                self.output = output
                self.verdict = False
            elif "does not permit" in output:
                self.output = output
                self.verdict = True

    class FastbootFlashingUnlockStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "fastboot flashing unlock"
            self.description = "Launch 'fastboot flashing unlock' command and check command output"
        def step(self, parameter):
            exec_status, output = self.host.commandExecFastboot("flashing unlock")
            if exec_status != 0 or "FAILED (remote: Unlock is not allowed)" in output:
                self.output = output
                self.verdict = True
            else:
                self.output = output
                self.verdict = False

    class FastbootFlashingLockStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "fastboot flashing lock"
            self.description = "Launch 'fastboot flashing lock' command and check command output"
        def fastbootFlashinglock(self):
            self.host.commandExec("fastboot flashing lock ")
        def fastbootlock(self):
            thread.start_new_thread(self.fastbootFlashinglock,())
            time.sleep(2)
            self.relayCard.upButtonPress(0.5)
            time.sleep(2)
            self.relayCard.powerButtonPress(0.5)
            time.sleep(2)
        def step(self, parameter):
            self.fastbootlock()
            i = 0
            exec_status, output = self.host.commandExecFastboot("getvar unlocked")
            x= (i < 3 ) and ( "unlocked: yes" in output )
            while x:
                  self.fastbootlock()
                  i += 1
                  exec_status, output = self.host.commandExecFastboot("getvar unlocked")
                  x= (i < 3 ) and ( "unlocked: yes" in output )
            if exec_status != 0 or "unlocked: yes" in output:
                  self.output = output
                  self.verdict = False
            else:
                  self.output = output
                  self.verdict = True



    class FastbootOemGetActionNonceForceUnlockAndGenPkcs7Step(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "fastboot oem get-action-nonce force-unlock"
            self.description = "Launch 'fastboot oem get-action-nonce force-unlock' command and check command output"
        def step(self, parameter):
            exec_status, output = self.host.commandExecFastboot("oem get-action-nonce force-unlock")
            if exec_status != 0 or "FAILED" in output:
                self.output = output
                self.verdict = False
            else:
                formatted_output=output.split(' ', 1 )[1];
                formatted_output=formatted_output.split('OKAY')[0];
                self.host.commandExec("chmod 777 ../../tools/rma/platform-rma-tools-linux/action-authorization")
                self.host.commandExec("chmod 777 ../../tools/rma/platform-rma-tools-linux/generate_blpolicy_oemvars.py")
                self.host.commandExec("chmod 777 ../../tools/rma/platform-rma-tools-linux/sign-efi-sig-list")
                self.host.commandExec("chmod 777 ../../tools/rma/platform-rma-tools-linux/openssl")
                exec_status, output = self.host.commandExec("../../tools/rma/platform-rma-tools-linux/action-authorization -O ../../tools/rma/OAK.x509.pem -K ../../tools/rma/OAK.pk8 -M "+formatted_output+" -F pkcs7.bin")
                if exec_status != 0 or "FAILED" in output:
                    self.output = output
                    self.verdict = False
                else:
                    exec_status, output = self.host.commandExecFastboot("flash action-authorization pkcs7.bin")
                    if exec_status != 0 or "FAILED" in output:
                        self.output = output
                        self.verdict = False
                    else:
                        self.output = output
                        self.verdict = True

    class FastbootGetvarUnlockedYesStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "fastboot getvar unlocked"
            self.description = "Launch 'fastboot getvar unlocked' command and check command output"
        def step(self, parameter):
            exec_status, output = self.host.commandExecFastboot("getvar unlocked")
            if exec_status != 0 or "unlocked: no" in output:
                self.output = output
                self.verdict = False
            else:
                self.output = output
                self.verdict = True

    class FastbootGetvarUnlockedNoStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "fastboot getvar unlocked"
            self.description = "Launch 'fastboot getvar unlocked' command and check command output"
        def step(self, parameter):
            exec_status, output = self.host.commandExecFastboot("getvar unlocked")
            if exec_status != 0 or "unlocked: yes" in output:
                self.output = output
                self.verdict = False
            else:
                self.output = output
                self.verdict = True

    class FastbootErasePersistentStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "fastboot erase persistent"
            self.description = "Launch 'fastboot erase persistent' command and check command output"
        def step(self, parameter):
            exec_status, output = self.host.commandExecFastboot("erase persistent")
            if exec_status != 0 or "FAILED" in output:
                self.output = output
                self.verdict = False
            else:
                self.output = output
                self.verdict = True

    class GenerateNewBootloaderPolicyOemvarsTxtFileStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "generate_blpolicy_oemvars.py -K odm"
            self.description = "Generate a new bootloader policy to set the device in class A"
        def step(self, parameter):
            exec_status, output = self.host.commandExec("../../tools/rma/platform-rma-tools-linux/generate_blpolicy_oemvars.py -K ../../tools/rma/odm -O ../../tools/rma/OAK.x509.pem -B 0x01 bootloader_policy-oemvars.txt")
            if exec_status != 0 or "FAILED" in output:
                self.output = output
                self.verdict = False
            else:
                self.output = output
                self.verdict = True

    class FastbootFlashOemvarsPolicyStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "fastboot flash oemvars"
            self.description = "Launch 'fastboot flash oemvars' command and check command output"
        def step(self, parameter):
            exec_status, output = self.host.commandExecFastboot("flash oemvars bootloader_policy-oemvars.txt")
            if exec_status != 0 or "error" in output:
                self.output = output
                self.verdict = False
            else:
                self.output = output
                self.verdict = True

    class FastbootOemReprovisionStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "fastboot oem reprovision"
            self.description = "Launch 'fastboot oem reprovision' command and check command output"
        def step(self, parameter):
            exec_status, output = self.host.commandExecFastboot("oem reprovision")
            if exec_status != 0 or "error" in output:
                self.output = output
                self.verdict = False
            else:
                self.output = output
                self.verdict = True


    class FastbootOemGetActionNonceForceUnlockStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "fastboot oem get-action-nonce force-unlock"
            self.description = "Launch 'fastboot oem get-action-nonce force-unlock' command and check command output"
        def step(self, parameter):
            exec_status, output = self.host.commandExecFastboot("oem get-action-nonce force-unlock")
            if exec_status != 0 or "FAILED" in output:
                self.output = output
                self.verdict = False
            else:
                self.output = output
                self.verdict = True


    class FastbootFlashActionAuthorizationPkcs7Step(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "fastboot flash action-authorization pkcs7.bin"
            self.description = "Launch 'fastboot flash action-authorization pkcs7.bin' command and check command output"
        def step(self, parameter):
            exec_status, output = self.host.commandExecFastboot("flash action-authorization pkcs7.bin")
            if exec_status != 0 or "0000001A" in output:
                self.output = output
                self.verdict = True
            else:
                self.output = output
                self.verdict = False

    class AdbEnableDevelopmentStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "adb enable Development"
            self.description = "Enable Development through adb command"
        def step(self, parameter):
            current_os = self.osManager.getOs()
            if not current_os == "main":
                self.output = "starting step with wrong OS (expected='main' , actual='{0}')".format(current_os)
                self.verdict = False
            else:
                output = self.host.commandExecAdb("shell dumpsys window displays")
                if "800x1280" in output:  #CHT  mrd
                    self.host.commandExecAdb("shell am start -n com.android.settings/com.android.settings.Settings")
                    self.host.commandExecAdb("shell input swipe 600 600  0 0")
                    self.host.commandExecAdb("shell input tap 600 1200")
                    self.host.commandExecAdb("shell input tap 600 800")
                    self.host.commandExecAdb("shell input tap 600 800")
                    self.host.commandExecAdb("shell input tap 600 800")
                    self.host.commandExecAdb("shell input tap 600 800")
                    self.host.commandExecAdb("shell input tap 600 800")
                    self.host.commandExecAdb("shell input tap 600 800")
                    self.host.commandExecAdb("shell input tap 600 800")
                elif "1200x1920" in output:  #CHT ffd
                    self.host.commandExecAdb("shell am start -n com.android.settings/com.android.settings.Settings")
                    self.host.commandExecAdb("shell input swipe 600 600  0 0")
                    self.host.commandExecAdb("shell input tap 800 1700")
                    self.host.commandExecAdb("shell input tap 800 1300")
                    self.host.commandExecAdb("shell input tap 800 1300")
                    self.host.commandExecAdb("shell input tap 800 1300")
                    self.host.commandExecAdb("shell input tap 800 1300")
                    self.host.commandExecAdb("shell input tap 800 1300")
                    self.host.commandExecAdb("shell input tap 800 1300")
                    self.host.commandExecAdb("shell input tap 800 1300")

    class AdbEnableOemUnlockStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "adb enable oem unlock"
            self.description = "Enable oem unlock through adb command"
        def step(self, parameter):
            current_os = self.osManager.getOs()
            if not current_os == "main":
                self.output = "starting step with wrong OS (expected='main' , actual='{0}')".format(current_os)
                self.verdict = False
            else:
                output = self.host.commandExecAdb("shell dumpsys window displays")
                if "800x1280" in output:  #CHT  mrd
                    self.host.commandExecAdb("shell am start -n com.android.settings/.DevelopmentSettings")
                    self.host.commandExecAdb("shell input tap 700 600")
                    self.host.commandExecAdb("shell input tap 700 800")
                elif "1200x1920" in output:  #CHT ffd
                    self.host.commandExecAdb("shell am start -n com.android.settings/.DevelopmentSettings")
                    self.host.commandExecAdb("shell input tap 800 800")
                    self.host.commandExecAdb("shell input tap 800 1200")

    class AdbDisableOemUnlockStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "adb disable oem unlock"
            self.description = "Disable oem unlock through adb command"
        def step(self, parameter):
            current_os = self.osManager.getOs()
            if not current_os == "main":
                self.output = "starting step with wrong OS (expected='main' , actual='{0}')".format(current_os)
                self.verdict = False
            else:
                output = self.host.commandExecAdb("shell dumpsys window displays")
                if "800x1280" in output:  #CHT  mrd
                    self.host.commandExecAdb("shell am start -n com.android.settings/.DevelopmentSettings")
                    self.host.commandExecAdb("shell input tap 700 600")
                elif "1200x1920" in output:  #CHT ffd
                    self.host.commandExecAdb("shell am start -n com.android.settings/.DevelopmentSettings")
                    self.host.commandExecAdb("shell input tap 800 800")


