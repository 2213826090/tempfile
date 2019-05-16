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
@summary: Pupdr Library - selfModule
@since: 11/17/2014
@author: travenex
"""

import re
import os
import time
import serial
import json
import tempfile
import LoggerModule
import HostModule
import RelayCardModule
import DeviceModule
import WorkaroundModule
import OutputModule
import MiscModule
import ConfigurationModule
import CampaignModule

class OsManagerModule(object):

    __instance = None
    waitOsDuration = None
    dnx_enumeration = None
    __globalConf = None
    __logger = None
    __host = None
    __relayCard = None
    __device = None
    __workaround = None
    __output = None
    __misc = None
    __configuration = None
    __campaign = None

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
        self.__output = OutputModule.OutputModule()
        self.__misc = MiscModule.MiscModule()
        self.__configuration = ConfigurationModule.ConfigurationModule()
        self.__campaign = CampaignModule.CampaignModule()
        self.waitOsDuration = 0
        self.dnx_enumeration = ""

    def getOs(self, check_charger=True):
        """Return the running OS name
        """

        # If DnX
        currentOs = self.isDnx()

        if not currentOs:
            devices = self.__device.adbDevices()[1]
            exec_status, fastboot = self.__host.commandExecFastboot("devices", timeout=8)
            osResult = re.search("(recovery|sideload|fastboot|device|unauthorized|bootloader)$", devices + fastboot)

            # If recovery, sideload, fastboot, devices
            if osResult:
                currentOs = osResult.group(1)
            else:
                currentOs = "offline"

            # main, charger, frozen, null (shell available but prop empty)
            if currentOs == "device":
                exec_status, adb_output = self.__host.commandExecAdb("shell echo alive")
                if exec_status != 0 or adb_output != "alive":
                    currentOs = "frozen"
                # Bootmode
                else:
                    currentOs = ""
                    start = time.time()
                    timeout = 2
                    while not currentOs and time.time() < start + timeout:
                        currentOs = self.__device.getProperty("ro.bootmode")
                        if not currentOs:
                            if "devices" not in self.__host.commandExecAdb("devices", 3)[1]:
                                currentOs = "unknown: state has changed"
                            else:
                                time.sleep(0.5)

                    # fallback for adb output:
                    if self.__configuration.boot.ADB_ENUMERATION_FALLBACK:
                        for fallback in self.__configuration.boot.ADB_ENUMERATION_FALLBACK:
                            if currentOs == fallback:
                                self.__logger.printLog("INFO", "getOs(): change name from '{0}' to '{1}'".format(currentOs, self.__configuration.boot.ADB_ENUMERATION_FALLBACK[fallback]))
                                currentOs = self.__configuration.boot.ADB_ENUMERATION_FALLBACK[fallback]

            # If currentOs value is still "" it means that getprop ro.bootmode does not respond
            if not currentOs:
                currentOs = "unknown: ro.bootmode is null"

        # check if os is ko
        if self.__configuration.boot.MOS_CHECK_SYS_BOOT_COMPLETED and currentOs == "main" and self.__device.getProperty("sys.boot_completed") != "1":
            currentOs = "main: boot-not-completed"
        elif self.__configuration.boot.MOS_CHECK_INIT_SVC_BOOTANIM and currentOs == "main" and self.__device.getProperty("init.svc.bootanim") != "stopped":
            currentOs = "main: bootanim-running"
            if self.__device.getProperty("service.bootanim.exit") == "1":
                currentOs = "main: bootanim-stuck"
        elif self.__configuration.boot.MOS_CHECK_INIT_SVC_ENCRYPT and currentOs == "main" and self.__device.getProperty("init.svc.encrypt") == "running":
            # get and display encryption progress if possible
            encryptionProgress = self.__device.getProperty("vold.encrypt_progress")
            if encryptionProgress:
                currentOs = "main: encryption ({0}%)".format(encryptionProgress)
            else:
                currentOs = "main: encryption"
        elif self.__configuration.boot.MOS_CHECK_DEV_BOOTCOMPLETE and currentOs == "main" and self.__device.getProperty("dev.bootcomplete") != "1":
            currentOs = "main: dev-boot-not-completed"
        elif self.__configuration.boot.MOS_CHECK_PATH_ANDROID and currentOs == "main" and "package:" not in self.__host.commandExecAdb("shell pm path android", 20)[1]:
            currentOs = "main: android-path-issue"
        elif self.__configuration.boot.MOS_CHECK_SYSTEMUI and currentOs == "main" and not self.__misc.checkProcess("systemui"):
            currentOs = "main: systemui-not-running"
        elif self.__configuration.boot.COS_CHECK_INIT_SVC_CHARGER_APP and currentOs == "charger" and self.__device.getProperty("init.svc.charger_app") != "running":
            currentOs = "charger-ko"
        elif self.__configuration.boot.SHELL_IN_ROS and self.__configuration.boot.ROS_CHECK_INIT_SVC_RECOVERY and currentOs == "recovery" and self.__device.getProperty("init.svc.recovery") != "running":
            currentOs = "recovery-ko"

        # fallback for final output:
        if isinstance(self.__configuration.boot.GET_OS_FALLBACK, str) and self.__configuration.boot.GET_OS_FALLBACK != "":
            for fallback in self.__configuration.boot.GET_OS_FALLBACK.split(";"):
                singleFallback = fallback.split(":")
                if len(singleFallback) != 2:
                    self.__logger.printLog("WARNING", "getOs(): invalid fallback: {0} ('<previous>:<new>' format expected)".format(fallback))
                else:
                    previousName = singleFallback[0]
                    newName = singleFallback[1]
                    if currentOs == previousName:
                        self.__logger.printLog("INFO", "getOs(): change name from '{0}' to '{1}'".format(currentOs, newName))
                        currentOs = newName

        # get charger type for debug
        if not any((self.__globalConf.get("DEFAULT_INIT"), self.__globalConf.get("TC_END_PHASE"))) and currentOs in ("main", "charger", "recovery") and check_charger:
            # skip if board in ROS and no shell in ROS
            if not (currentOs == "recovery" and self.__configuration.boot.SHELL_IN_ROS):
                self.__device.getChargerType()

        # currentOs is now read
        output = "device state is '{0}'".format(currentOs)
        self.__output.appendOutput(output, None)
        return currentOs

    def waitOs(self, expOs, timeout=None, start=None, loopTime=0, blind=False, frozenIsFail=True, pincode_decrypt=True, waitWdExp=True, reason=""):
        """ Wait for the board to end its booting sequence.
        It works for any OS (main, charger, recovery, fastboot, offline, sideload)
        multiple expected os is allowed: ie. recovery|offline
        """
        output = ""
        verdict = True

        if timeout is None:
            if self.__device.getOsExtentedTimeout(expOs) != 0:
                timeout = self.__device.getOsExtentedTimeout(expOs)
            else:
                timeout = self.__configuration.timeout.BOOT_TIMEOUT
        if not start:
            start = time.time()

        if not loopTime:
            loopTime = int(timeout/12)
            # limit between 3 seconds and 20 seconds
            if loopTime > 20:
                loopTime = 20
            elif loopTime < 3:
                loopTime = 3

        self.__logger.printLog("INFO", "Wait that boot of '{0}' completes (start={1}, timeout={2}s)...".format(expOs, time.strftime("%H:%M:%S", time.localtime(start)), timeout))

        if "|" in expOs:
            blind = True

        start_frozen_flag      = False
        ramdump_detected       = False
        freeze_timeout_failure = 10
        actualOs               = None
        start_frozen_time      = None

        # START while
        #
        latestGetOs = 0

        while True:
            actualOs = self.getOs(check_charger=False)
            latestGetOs = time.time()

            # Check Os
            if actualOs in expOs.split("|"):
                break

            # Compute delay for timeout break
            delay = time.time() - start
            if delay <= timeout:
                bar_number = int(delay/timeout*10.0)
                percent = int(delay/timeout*100.0)
                progress = "<{0:10}> {1}%".format("|"*bar_number, percent)
                self.__logger.printLog("INFO", "waitOs({0}): timeout in {1:.2f}s... {2}".format(expOs,
                                                                                                timeout-delay,
                                                                                                progress))

            # if ramdump detected, wait before resuming
            if actualOs == "ramdump" and not ramdump_detected:
                ramdump_detected = True
                extra_timeout = 90
                timeout += extra_timeout
                self.__logger.printLog("INFO", "ramdump detected, adding {0}s to waitOs timeout".format(extra_timeout))
                self.__misc.waitDelay("waiting for ramdump execution", extra_timeout)

            # If blind is True do not check os state compatibility
            if blind:
                pass

            # If state is frozen
            # and freeze state is still present after freeze_timeout_failure seconds
            elif actualOs == "frozen":
                if not start_frozen_flag:
                    start_frozen_flag = True
                    start_frozen_time = time.time()
                elif time.time() < start_frozen_time + freeze_timeout_failure:
                    pass

            # Board is expected to be either in expOs or offline and not another Os
            # This is only applicable when frozen state is allowed (WD TCs)
            # If board heads to "offline" state, failure is not applicable
            elif "offline" not in (expOs + actualOs) and \
                 "unknown:" not in actualOs          and \
                 "unauthorized" not in actualOs      and \
                 "dnx" not in actualOs               and \
                 [e for e in expOs.split("|") if e not in actualOs] \
                 and frozenIsFail:
                output = "'{0}'".format(actualOs)
                break

            # check timeout
            if delay >= timeout:
                output = "'{0}' (timeout expiration)".format(actualOs)
                self.__logger.printLog("WARNING", "waitOs({0}): {1}s timeout expired !".format(expOs, timeout))
                break
            # print message if looptime > 8 seconds to warn user
            if loopTime > 8:
                self.__misc.waitDelay("wait before next iteration", loopTime)
            else:
                time.sleep(loopTime)

            # TODO remove WA: bxt board replug
            if "bxt" in self.__configuration.board and "main" in expOs and not self.__device.adbDevices()[0]:
                self.__logger.printLog("DEBUG", "getOs(): WA BXT - adb not available, replugging usb")
                self.__relayCard.usbConnection(False)
                time.sleep(5)
                self.__relayCard.usbConnection(True)
                time.sleep(5)

        #
        # END while
        self.waitOsDuration = latestGetOs - start
        if self.waitOsDuration > timeout:
            self.waitOsDuration = timeout

        if actualOs in expOs.split("|"):
            # MAIN checks
            if actualOs == "main":
                failed_check_list = list()
                local_verdict, local_output = self.__device.checkPartitions(partition_list=self.__configuration.flash.PARTITIONS.split(";"))
                if not local_verdict:
                    failed_check_list.append(local_output)
                # wifi and modem checks are not done in Init and Final test case steps
                if not any((self.__globalConf.get("DEFAULT_INIT"), self.__globalConf.get("TC_END_PHASE"))):
                    # Wait modem
                    if not self.waitModem()[0]:
                        failed_check_list.append("modem failed to boot")
                    # Pupdr parameter: check wifi connection
                    if not self.__globalConf.get("SKIP_WIFI_CHECK", True):
                        self.__logger.printLog("INFO", "wait(main)=main, pupdrParamSkipWifiCheck = True")
                        if not self.waitWifi():
                            failed_check_list.append("wifi failed to connect")

                # Check verdict for main
                if failed_check_list:
                    verdict = False
                    output = "'main ({0})'".format(", ".join(failed_check_list))

        # if waitOs called with waitWdExp=True (default behavior) it waits for some time before resuming, in case the board reboots because of a crash
        else:
            if waitWdExp:
                self.__misc.waitDelay("wait watchdog expiration delay to enable crashlogs creation", self.__configuration.timeout.WATCHDOG_EXPIRATION)
            if self.__globalConf.get("HDK"):
                self.__relayCard.downButtonPress(1)
                time.sleep(60)
            verdict = False

        # Show properties
        if actualOs in ('main', 'recovery', 'charger'):
            self.__device.getChargerType()
            self.__device.getAllProperties()

        if verdict:
            output = "'{0}' ({1:.2f}s)".format(actualOs, self.waitOsDuration)
            local_output = ""
        else:
            local_output = output

        # store in json data and TCR
        if "offline" not in expOs and self.waitOsDuration < 2.0:
            self.__logger.printLog("DEBUG", "skipping data storing because of too short duration")
        elif self.__globalConf.get("DEFAULT_INIT"):
            self.__logger.printLog("DEBUG", "skipping data storing because Test Case init step")
        elif self.__globalConf.get("TC_END_PHASE"):
            self.__logger.printLog("DEBUG", "skipping data storing because Test Case final step")
        else:
            self.__campaign.Campaign_information.updateJsonWithWaitOsData(self.waitOsDuration, timeout, expOs.replace("|", "_or_"), actualOs, local_output, reason)
            if verdict:
                self.__campaign.Campaign_information.TCR_data_handler.addMetric(
                    "BOOTOTA_{0}_BOOT_TIME".format(expOs.split("|")[0].upper()),
                    self.waitOsDuration,
                    "second")

        if not verdict and self.__workaround.isWorkaround(data_type=self.__workaround.Function, output=output)[0]:
            verdict = True
        self.__output.appendOutput(output, verdict, argument="{0}, timeout={1}".format(expOs, str(timeout)))
        return verdict

    def waitModem(self, timeout=120):
        """ Wait modem boot to provision gsm.version.baseband
        """
        # do not wait for modem if MODEM var is False
        verdict = True

        if not self.__globalConf.get("MODEM", True):
            self.__output.appendOutput("MODEM var is False", True)
            return True, ""

        if not self.__configuration.boot.MODEM:
            self.__output.appendOutput("MODEM configuration var is False", True)
            return True, ""

        # Wait modem
        start = time.time()
        status = self.__device.getProperty("gsm.version.baseband")
        while time.time() - start < timeout and not status:
            self.__logger.printLog("INFO", "Modem timeout expiring in {0:.3f}s".format(timeout-(time.time()-start)))
            time.sleep(5)
            status = self.__device.getProperty("gsm.version.baseband")
        if not status:
            verdict = False

        output = "waitModem(): status='{0}'".format(status)
        self.__output.appendOutput(output, verdict)
        return verdict, status

    def waitWifi(self, timeout=60):
        """ Wait wifi connection
        """
        start = time.time()
        status = ""
        while time.time() - start < timeout and status not in ("ok", "failed"):
            time.sleep(5)
            # status is "ok" or "failed" or ""
            status = self.__device.getProperty("dhcp.wlan0.result")

        if status == "ok":
            verdict = True
        else:
            verdict = False

        output = "waitWifi(): status='{0}'".format(status)
        self.__output.appendOutput(output, verdict)
        return verdict

    def waitLsusbEnumeration(self, listValue, timeout=10, allMandatory=False):
        start = time.time()
        output = ""
        self.__logger.printLog("INFO", "waitLsusbEnumeration(): start waiting for: {0}".format(", ".join(listValue)))
        verdict = self.isLsusbEnumeration(listValue, allMandatory)
        while time.time() - start < timeout and not verdict:
            time.sleep(5)
            verdict = self.isLsusbEnumeration(listValue, allMandatory)
            if not verdict:
                self.__logger.printLog("INFO", "Lsusb expiring in %.3fs" % (timeout - (time.time() - start)))
        if not verdict:
            output = "enumeration of '{0}' failed (timeout={1}s)".format(", ".join(listValue), timeout)
        self.__output.appendOutput(output, verdict)
        return verdict

    def isLsusbEnumeration(self, listValue, allMandatory=False):
        verdict = False
        status, lsusb = self.lsusb()
        missingValues = listValue
        for element in listValue:
            if re.search(element, lsusb):
                self.__logger.printLog("INFO", "found: {0}".format(element))
                missingValues.remove(element)
                if not allMandatory:
                    verdict = True
                    break
        if not verdict:
            if missingValues:
                verdict = False
                self.__logger.printLog("WARNING", "isLsusbEnumeration(): elements not enumerating: {0}".format(", ".join(missingValues)))
            else:
                verdict = True
        return verdict

    def isDnx(self):
        """ Check is the device is in DnX mode
        """
        lsusb = self.lsusb()[1]

        element_list=["8087:0a65","8086:e005","8086:09ee"]
        # BXTP "8087:0a88"
        element_cse_list=["8087:0a82","8087:0a88"]
        if any(element in lsusb for element in element_list):
            output_os="dnx"
        #elif "8087:0a82" in lsusb:
        elif any(element_cse in lsusb for element_cse in element_cse_list):
            output_os="cse-dnx"
        else:
            output_os= ""
            self.__logger.printLog("WARNING", "isDnx(): no board in DNX found in lsusb")
        return output_os

    # Inner gotoMos method
    def __recover(self, log):
        output = ""
        if self.__globalConf.get("TC_END_PHASE", False) and not self.__globalConf.get("TC_VERDICT_BOOL", True):
            self.__misc.waitDelay("let the WD expire before shutting down", self.__configuration.timeout.MOS_BOOT)

        if "bxtp" in self.__configuration.board:
            if not self.shutDownBxtpMrb():
                verdict = False
                output = "failure to force shutdown the bxt board"
            else:
                verdict = True
                self.__logger.printLog("INFO", "Boot the board usinc IOC g command")
                usb2SerialConnection = serial.Serial(port='/dev/ttySerial2', baudrate=115200, timeout=0)
                usb2SerialConnection.write("g")
                starttime = time.time()
                while self.__device.getProperty("sys.boot_completed") != "1" and time.time() - starttime < 300:
                    self.__logger.printLog("INFO", "Waiting for board to boot (boot completed)")
                    time.sleep(.5)
                if not self.__device.getProperty("sys.boot_completed") == "1":
                    output = "board block, IOC command failed to boot the board in MOS for bxt"
                    verdict = False
        else:
            self.forceShutdown(plug=False)
            self.__relayCard.powerButtonPress(self.__configuration.timeout.PKPON)
            self.__relayCard.usbConnection(True)
            if self.__globalConf.get("DEFAULT_INIT", True):
                timeout = self.__configuration.timeout.BOOT_TIMEOUT
            else:
                timeout = None
            verdict = self.waitOs("main", timeout=timeout, waitWdExp=False)
            if not verdict:
                actualOs = self.getOs()
                if  actualOs in ("recovery", "fota") and "sf3g" in self.__configuration.board.lower():
                    verdict = self.adbRebootRecoveryClear()
                    if not verdict:
                        output = "failure to reboot in MOS from previous state"
                elif actualOs in ("charger", "recovery", "fota"):
                    verdict = self.adbReboot()
                    if not verdict:
                        output = "failure to reboot in MOS from previous state"
                else:
                    self.__logger.printLog("WARNING", log + "Board blocked ! As a last resort, try USB replugging to boot the board")
                    self.__relayCard.usbConnection(False)
                    time.sleep(3)
                    self.__relayCard.usbConnection(True)
                    time.sleep(1)
                    if not actualOs == "main":
                        if self.waitOs("charger", waitWdExp=False):
                            verdict = self.adbReboot()
                    if not verdict:
                        output = "board block, forced shutdown and power key press failed to boot the board in MOS"
        self.__output.appendOutput(output, verdict)
        return verdict

    def gotoMos(self):
        """ Force the device to boot in MOS from any other state.
        """
        log = "gotoMos(): "
        output = ""

        # If PUPDR.OsManagerModule.forceShutdown is disabled skip recover()
        allowForceShutdown = True
        if not self.__globalConf.get("FORCE_SHUTDOWN_IN_GOTO_MOS", True):
            self.__logger.printLog("INFO", log + "pupdrParamGotoMosForceShtn is " + str(self.__globalConf["FORCE_SHUTDOWN_IN_GOTO_MOS"]))
            allowForceShutdown = False

        self.__logger.printLog("INFO", log + "Ensure device is in MOS")
        currentOs = self.getOs()

        # If dnx let's finish to boot
        if currentOs == "dnx" and not self.__configuration.flash.DEDIPROG:
            self.__logger.printLog("INFO", log + "DnX OS detected")
            self.__misc.waitDelay("waiting MOS boot", self.__configuration.timeout.MOS_BOOT)
            currentOs = self.getOs()
        # If still DnX, probably booted from combo try a force shutdown
        # But if Dnx is due to failed blankphone it won't work
        if currentOs == "dnx" and allowForceShutdown:
            self.__recover(log)
            currentOs = self.getOs()

        if currentOs == "main":
            verdict = self.waitOs("main")
            if not verdict:
                self.__logger.printLog("WARNING", log + "failure with MOS checks, rebooting board with adb command")
                verdict = self.adbReboot()
                if not verdict:
                    output = "failure to reboot board in MOS"
        elif currentOs in ("recovery", "fota") and "sf3g" in self.__configuration.board.lower() and self.adbRebootRecoveryClear():
            verdict = True
        elif currentOs == "charger" and "sf3g" in self.__configuration.board.lower() and self.adbReboot():
            verdict = True
        elif currentOs in ("charger", "recovery", "fota") and self.adbReboot() and not "sf3g" in self.__configuration.board.lower():
            verdict = True
        elif currentOs == "fastboot" and self.fastbootReboot():
            verdict = True
        elif currentOs == "fastboot-adb" and self.adbReboot():
            verdict = True
        elif currentOs == "ptest" and self.adbRebootFromPtest():
            verdict = True
        elif allowForceShutdown:
            verdict = self.__recover(log)
            if not verdict:
                output = "recovery procedure failed to reboot the board in MOS"
        else:
            # a previous test case can leave the board powered off (PUNIT campaign)
            # need to try booting before failing even with force_shutdown=False
            self.__relayCard.powerButtonPress(self.__configuration.timeout.PKPON)
            verdict = self.waitOs("main")
            if not verdict:
                output = "power key press failed to reboot the board in MOS from OFF (force shutdown unactivated)"

        self.__output.appendOutput(output, verdict)
        return verdict

    # Inner gotoPos method
    def __pos(self, plug=False):
        verdict = True
        output = ""
        currentOs = self.getOs()
        # If already in POS
        if currentOs == "fastboot":
            pass
        # Elif another OS with adb (supports "-ko")
        elif [True for localOs in ("main", "charger", "recovery") if localOs in currentOs]:
            if not self.adbRebootBootloader():
                verdict = False
                output = "failure to reboot in POS from previous OS"
        elif currentOs == "offline" and "VolumeDown" in self.__relayCard.relayConfiguration:
            self.posCombo()
            self.__relayCard.usbConnection(True)
            verdict = self.waitOs("fastboot")
            if not verdict:
                output = "failure to boot POS from OFF with combo key"
        elif not "VolumeDown" in self.__relayCard.relayConfiguration:
            output = "'VolumeDown' not defined in Bench Configuration, unable to force POS boot from OFF"
            verdict = False
        else:
            output = "unknown OS"
            verdict = False

        self.__output.appendOutput(output, verdict)
        return verdict

    def gotoPos(self):
        """ Force the device to boot in POS from any other state.
        """
        log = "gotoPos(): "
        verdict = True
        output = ""
        self.__logger.printLog("INFO", log + "Ensure device is in POS")

        # Go to POS
        if not self.__pos():
            # Still not in POS (reboot POS failed or frozen)
            self.forceShutdown(plug=False)
            if not self.__pos(plug=True):
                actualOs = self.getOs()
                output = "failed to boot POS ({0})".format(actualOs)
                verdict = False

        self.__output.appendOutput(output, verdict)
        return verdict

    # Inner method to wait fastboot or adb shell
    def __waitFastbootOrShell(self, timeout, log):
        looptime = 1
        fastboot = False
        shell = False
        delay = 0
        start = time.time()
        # Wait fastboot or adb shell
        self.__logger.printLog("INFO", "wait fastboot or adb shell", frontContent=log)
        while not fastboot and not shell and delay <= timeout:
            time.sleep(looptime)
            delay = time.time() - start
            fastboot = True if "fastboot" in self.__host.commandExecFastboot("devices")[1] else False
            shell = True if re.search("ready|/system/bin/sh", self.__host.commandExecAdb("shell echo ready")[1]) else False
            self.__logger.printLog("INFO", "timeout in %.3fs" % (timeout - delay), frontContent=log)
        return fastboot, shell, delay

    # Inner method to reboot to DnX mode (if fails try combo then)
    def __rebootDnx(self, log):
        output = ""
        # Wait fastboot or adb shell
        fastboot, shell, delay = self.__waitFastbootOrShell(self.__configuration.timeout.MOS_BOOT, log)
        # If adb available
        if shell:
            verdict = self.adbRebootDnx()
            if not verdict:
                output = "'adb reboot dnx' command failed"
        # If fastboot available
        elif fastboot:
            verdict = self.fastbootOemRebootDnx()
            if not verdict:
                if not self.fastbootOemRebootDnx():
                    output = "'fastboot oem reboot dnx' command failed"
                    verdict = False
                    self.__relayCard.usbConnection(True)
                else:
                    verdict = True
        # If timeout reached
        else:
            output = "wait fastboot or adb shell timeout ({0}s)".format(delay)
            verdict = False

        # If DnX boot failed then try combo DnX boot
        if not verdict:
            self.forceShutdown()
            verdict = self.comboDnx()
            if not verdict:
                output = "dnx combo failed to boot the board"

        self.__output.appendOutput(output, verdict)
        return verdict

    def gotoDnx(self):
        """ Go to DnX mode
        """
        log = "gotoDnx():"
        self.__logger.printLog("INFO", log + "start")
        output = ""

        # If no DnX is present in this board
        if self.__configuration.boot.NODNX:
            self.__logger.printLog("WARNING", log + "no DnX exists")
            return False

        # Get current OS
        currentOs = self.getOs()

        # If already in DnX mode
        if currentOs == "dnx":
            self.__logger.printLog("INFO", log + "already in DnX mode")
            self.__output.appendOutput("", True)
            self.__checkDnxEnumeration()
            return True

        # If DnX if only manually booting
        if self.__configuration.flash.DEDIPROG:
            # If offline try combo DnX boot
            if self.getOs() == "offline" or self.__configuration.boot.DNX_BOOT_WITH_COMBO_ONLY:
                self.forceShutdown(plug=False)
                verdict = self.comboDnx()
                # If blocked it means that the volume is not wired
                if not verdict and not self.__configuration.boot.DNX_BOOT_WITH_COMBO_ONLY:
                    # try to boot any OS
                    self.gotoMos()
                    verdict = self.__rebootDnx(log)
                    if not verdict:
                        output = "failure to reboot dnx after power key press"
            else:
                verdict = self.__rebootDnx(log)
            # force USB replug
            self.__relayCard.usbConnection(True)

        # Else execute combo DnX boot
        else:
            verdict = self.comboDnx()
            if not verdict:
                output = "failure to boot dnx with combo buttons"

        # Confirm DnX mode
        if verdict:
            time.sleep(5)
            if self.getOs() != "dnx":
                output = "failed to confirm DnX mode"
                verdict = False
            else:
                self.__logger.printLog("INFO", "DnX confirmed")
                self.__checkDnxEnumeration()

        self.__output.appendOutput(output, verdict)
        return verdict

    def __checkDnxEnumeration(self):
        log = "checkDnxEnumeration() "
        # get dnx SSN:
        if self.__host.dnx_ssn:
            ssn_to_check = self.__host.dnx_ssn
        else:
            ssn_to_check = self.__host.serial_number
        if ssn_to_check:
            self.__logger.printLog("INFO", log + "checking {0} DNX enumeration".format(ssn_to_check))
            output = self.__host.commandExec("fastboot devices")[1]
            if re.search(ssn_to_check, output):
                self.__logger.printLog("INFO", log + "{0} DNX enumeration confirmed".format(ssn_to_check))
                self.dnx_enumeration = ssn_to_check
            else:
                self.__logger.printLog("INFO", log + "failure to confirm {0} DNX enumeration".format(ssn_to_check))
        else:
            self.__logger.printLog("INFO", log + "no SSN provided")

    def gotoS3(self, duration, waitBetweenCmd):
        """ Go to S3 mode and return verdict, effective time and output log
        """
        log_step = ""

        def get_pmu_states():
            time_index = None
            time_value = None
            residency_index = None
            residency_value = None
            self.__device.waitAdb(timeout=self.__configuration.timeout.MOS_BOOT, loopTime=5)
            # Read pmu states
            array_length = 13
            exec_status, output = self.__host.commandExecAdb("shell cat /d/mid_pmu_states | grep -ai -B%s ^s3" % str(array_length), 3)
            if exec_status != 0:
                self.__logger.printLog("INFO", log_step + " Cannot read pmu states (device not found)")
                return 0, 0, ""
            # Read wake_lock content
            exec_status, wake_lock = self.__host.commandExecAdb("shell cat /sys/power/wake_lock", 3)
            lines = output.strip("\n").replace(" ", "").split("\n")
            try:
                for line in lines:
                    l = line.split("\t")
                    if "residency" in line.lower():
                        residency_index = l.index([i for i in l if "residency" in i.lower()][0])
                    if "time" in line.lower():
                        time_index = l.index([i for i in l if "time" in i.lower()][0])
                    if line.lower().startswith("s3"):
                        if residency_index:
                            residency_value = l[residency_index]
                        if time_index:
                            time_value      = l[time_index]
                return float(time_value), float(residency_value), wake_lock
            except Exception as e:
                self.__logger.printLog("INFO", log_step + " Cannot read pmu states (%s)" % e)
                return 0, 0, ""


        # Read pmu states
        log_step = "Start !"
        pmu_start_time, pmu_start_residency, wake_lock_start = get_pmu_states()
        self.__logger.printLog("INFO", log_step + " pmu start values: time={0}s, residency={1}%"\
                                     .format(pmu_start_time, pmu_start_residency))

        # Switch off the screen if not done
        self.__device.switchScreenState(False)

        # SLEEP
        self.__relayCard.usbConnection(False)

        # Wait for entering in sleep mode
        self.__logger.printLog("INFO", log_step + " Wait for {0}s to enter in S3...".format(waitBetweenCmd))
        time.sleep(waitBetweenCmd)
        # Wait in sleep mode
        self.__logger.printLog("INFO", log_step + " Wait for {0}s in S3...".format(duration))
        time.sleep(duration)

        # WAKE UP
        log_step = "Wake up !"
        self.__logger.printLog("INFO", log_step)
        self.__relayCard.usbConnection(True)
        time.sleep(waitBetweenCmd)
        self.__device.switchScreenState(True)

        # CHECK
        # Read pmu states
        pmu_stop_time, pmu_stop_residency, wake_lock_stop = get_pmu_states()
        self.__logger.printLog("INFO", log_step + " pmu stop values: time={0}s, residency={1}%"\
                                     .format(pmu_stop_time, pmu_stop_residency))
        # Check effective duration in S3
        effective_duration = round(pmu_stop_time - pmu_start_time, 3)
        if effective_duration <= 0:
              verdict = False
              log, return_msg = "WARNING", log_step \
                                         + " Did not enter in S3 mode (wake_lock start='%s', stop='%s')" \
                                         % (wake_lock_start, wake_lock_stop)
        else:
            verdict = True
            log, return_msg = "INFO", "S3 wake-up after {0}s".format(effective_duration)
        self.__logger.printLog(log, return_msg)
        self.__output.appendOutput(return_msg, verdict)
        return verdict, effective_duration, return_msg

    def lsusb(self, timeout=3):
        """ Get usb info
        """
        # LINUX
        if os.name in ['posix']:
            lsusb_lib = "lsusb"

        # WINDOWS
        else:
            # Search lsusb.exe
            lsusb_lib = os.path.join(os.path.dirname(__file__), "lsusb.exe")
            # adding time for lsusb.exe to be executed on windows hosts
            timeout += 7

        # Execute
        exec_status, log = self.__host.commandExec(lsusb_lib, timeout)

        if os.name not in ['posix']:
            # Trimming the output to match linux
            log = log.replace(" ProductID", "")

        # return
        if exec_status != 0:
            verdict = False
            self.__logger.printLog("WARNING", "lsusb(): lsusb command failure")
            log = ""
        else:
            verdict = True

        return verdict, log

    # class rebootCmd():

    def __runRebootCommand(self, cmd, plug=True, shtn_duration=None, waitOffline=True):
        """ Execute a reboot command and check the status.
        Then return the time read at the command execution.
        """
        output = ""
        if not shtn_duration:
            shtn_duration = self.__configuration.timeout.SHUTDOWN_TIMEOUT

        self.__logger.printLog("INFO", "rebootCmd(): request is: {0}".format(cmd))
        if "adb" in cmd:
            self.__device.waitAdbDevices(timeout=self.__configuration.timeout.MOS_BOOT)

        # Start time
        self.__logger.printLog("INFO", "executing command: {0}".format(cmd))
        start = time.time()
        exec_status, _ = self.__host.commandExec(cmd, self.__configuration.timeout.REBOOT_COMMAND_TIMEOUT)
        # If command fails
        if exec_status != 0:
            output = "command failure (return code = {0})".format(exec_status)
            self.__output.appendOutput(output, False, argument=cmd)
            return start, False

        # If works unplugged
        if not plug:
            verdict = True
            self.__relayCard.usbConnection(False)
            time.sleep(shtn_duration)
        # Else plug
        elif waitOffline:
            verdict = self.waitOs("offline", timeout=shtn_duration, start=start)
            if not verdict:
                output = "board stayed ON after reboot request was launched"
        else:
            verdict = True

        self.__output.appendOutput(output, verdict, argument=cmd)
        return start, verdict

    def adbReboot(self, wait_main=True, pincode_decrypt=True, loopTime=3):
        """ Run a 'adb reboot' command and check the result
        Then it runs a self.waitOs() function
        """
        output = ""
        start, verdict = self.__runRebootCommand("adb " + self.__host.ssnExtra + "reboot")
        if verdict and wait_main:
            verdict = self.waitOs("main", start=start, pincode_decrypt=pincode_decrypt, loopTime=loopTime)
            if not verdict:
                output = "failure to reboot in MOS after 'adb reboot' command"
        elif not verdict:
            output = "failure to shutdown after 'adb reboot' command"
        self.__output.appendOutput(output, verdict)
        return verdict

    def adbRebootPtest(self, wait_main=True, loopTime=3):
        """ Run a 'adb reboot ptest' command and check the result
        Then it runs a self.waitOs() function
        """
        output = ""
        start, verdict = self.__runRebootCommand("adb " + self.__host.ssnExtra + "reboot ptest")
        if verdict and wait_main:
            verdict = self.waitOs("ptest", start=start, loopTime=loopTime)
            if not verdict:
                output = "failure to reboot in ptest after 'adb reboot ptest' command"
        elif not verdict:
            output = "failure to shutdown after 'adb reboot ptest' command"
        self.__output.appendOutput(output, verdict)
        return verdict

    def adbRebootFromPtest(self):
        """ Run a 'adb reboot ptest_clear' command
            Waits for boot completion
        """
        output = ""
        start, verdict = self.__runRebootCommand("adb " + self.__host.ssnExtra + "reboot ptest_clear")
        if not verdict:
            output = "failure to shutdown after 'adb reboot ptest_clear' command"
        else:
            verdict = self.waitOs("main", start=start)
            if not verdict:
                output = "failure to reboot in MOS after 'adb reboot ptest_clear' command"
        self.__output.appendOutput(output, verdict)
        return verdict

    def adbRebootSideloadAutoReboot(self):
        """ Run a 'adb reboot sideload-auto-reboot' command and check the result
        Then it runs a self.waitOs() function
        """
        output = ""
        start, verdict = self.__runRebootCommand("adb " + self.__host.ssnExtra + "reboot sideload-auto-reboot")
        if verdict:
            verdict = self.waitOs("sideload")
            if not verdict:
                output = "failure to reboot in sideload after 'adb reboot sideload-auto-reboot' command"
        elif not verdict:
            output = "failure to shutdown after 'adb reboot sideload-auto-reboot' command"
        self.__output.appendOutput(output, verdict)
        return verdict

    def adbRebootBootloader(self):
        """ Run a 'adb reboot bootloader' command
            Waits for boot completion
        """
        output = ""
        start, verdict = self.__runRebootCommand("adb " + self.__host.ssnExtra + "reboot {0}".format(self.__configuration.boot.FASTBOOT_TARGET))
        if not verdict:
            output = "failure to shutdown after 'adb reboot {0}' command".format(self.__configuration.boot.FASTBOOT_TARGET)
        else:
            verdict = self.waitOs("fastboot", start=start)
            if not verdict:
                output = "failure to reboot in POS after 'adb reboot {0}' command".format(self.__configuration.boot.FASTBOOT_TARGET)
        self.__output.appendOutput(output, verdict)
        return verdict

    def adbRebootRecovery(self):
        """ Run a 'adb reboot recovery' command
            Waits for boot completion
        """
        output = ""
        start, verdict = self.__runRebootCommand("adb " + self.__host.ssnExtra + "reboot recovery")
        if not verdict:
            output = "failure to shutdown after 'adb reboot recovery' command"
        else:
            verdict = self.waitOs("recovery", start=start)
            if not verdict:
                output = "failure to reboot in ROS after 'adb reboot recovery' command"
        self.__output.appendOutput(output, verdict)
        return verdict

    def adbRebootRecoveryClear(self):
        """ Run a 'adb reboot recovery_clear' command
            Waits for boot completion
        """
        output = ""
        start, verdict = self.__runRebootCommand("adb " + self.__host.ssnExtra + "reboot recovery_clear")
        if not verdict:
            output = "failure to shutdown after 'adb reboot recovery_clear' command"
        else:
            verdict = self.waitOs("main", start=start)
            if not verdict:
                output = "failure to reboot in MOS after 'adb reboot recovery_clear' command"
        self.__output.appendOutput(output, verdict)
        return verdict

    def adbRebootCharging(self):
        """ Run a 'adb reboot charging' command
            Waits for boot completion
        """
        output = ""
        start, verdict = self.__runRebootCommand("adb " + self.__host.ssnExtra + "reboot charging")
        if not verdict:
            output = "failure to shutdown after 'adb reboot charging' command"
        else:
            verdict = self.waitOs("charger", start=start)
            if not verdict:
                output = "failure to reboot in COS after 'adb reboot charging' command"
        self.__output.appendOutput(output, verdict)
        return verdict

    def fastbootReboot(self, wait_main=True):
        """ Run a 'fastboot reboot' command
            Waits for boot completion
        """
        # hard shutdown to reboot the board in fastboot
        output = ""
        start, verdict = self.__runRebootCommand("fastboot " + self.__host.fastbootSsnExtra + "reboot")
        if not verdict:
            output = "failure to shutdown after 'fastboot reboot' command"
        elif wait_main:
            verdict = self.waitOs("main", start=start)
            if not verdict:
                output = "failure to reboot in MOS after 'fastboot reboot' command"
        self.__output.appendOutput(output, verdict)
        return verdict

    def fastbootRebootBootloader(self):
        """ Run a 'fastboot reboot-bootloader' command
            Waits for boot completion
        """
        output = ""
        start, verdict = self.__runRebootCommand("fastboot " + self.__host.fastbootSsnExtra + "reboot-bootloader")
        if not verdict:
            output = "failure to shutdown after 'fastboot reboot-bootloader' command"
        else:
            verdict = self.waitOs("fastboot", start=start)
            if not verdict:
                output = "failure to reboot in POS after 'fastboot reboot-bootloader' command"
        self.__output.appendOutput(output, verdict)
        return verdict

    def peeknpoke(self):
        """ Run a 'adb shell peeknpoke p w cf9 6' command and check the result
            Then it runs a self.waitOs() function
        """
        output = ""
        start, verdict = self.__runRebootCommand("adb " + self.__host.ssnExtra + "shell peeknpoke p w cf9 6")
        if not verdict:
            output = "failure to shutdown after 'fastboot reboot' command"
        else:
            verdict = self.waitOs("main", start=start)
            if not verdict:
                output = "failure to reboot in MOS after 'fastboot reboot' command"
        self.__output.appendOutput(output, verdict)
        return verdict

    def gracefulShutdown(self, plug=True, intent=True, delay_time=10, force=False):
        """ Perform a graceful shutdown of the board.
        1. Plug the SDP
        2. Disconnect the adb session
        3. If "intent" is True, shutdown is done with an intent command, else it's done by pressing buttons
        4. If "plug" is False, the SDP is unplug at the end of the function
        """
        verdict = True
        output = ""
        if not self.__device.adbRoot():
            self.__output.appendOutput("failure to root device", False)
            return False

        #apk = AcsApk.AcsApk()

        # Use intent to shutdown
        if intent is True:
            intent_cmd = "am start -a android.intent.action.ACTION_REQUEST_SHUTDOWN"
            if not plug:
                # Push graceful shutdown script
                shell_script = "/data/graceful_shutdown.sh"
                # Prevent the board from going to S3
                self.__host.commandExecAdb("shell echo 'PUPDR_PACT_LOCK' > /sys/power/wake_lock")
                # Create the graceful_shutdown script
                local_file = os.path.join(tempfile.gettempdir(), "graceful_shutdown_{}.sh".format(self.__misc.getUser()))
                self.__misc.local_files_handler.addEntry(local_file)
                if os.path.exists(local_file):
                    os.remove(local_file)
                with open(local_file, "w+") as f:
                    f.write("echo 'Reboot script: Starting - wait for {0} seconds\\n' > /dev/kmsg;\n".format(delay_time))
                    f.write("sleep {0};\n".format(delay_time))
                    f.write("echo 'Reboot script: Sleep cmd over - launching intent\\n' > /dev/kmsg;\n")
                    f.write(intent_cmd + ";\n")
                    f.write("echo 'Reboot script: Finished - command launched\\n' > /dev/kmsg;")
                self.__host.commandExecAdb("push {0} {1}".format(local_file, shell_script))
                if os.path.isfile(local_file):
                    os.remove(local_file)
                # Make the script executable
                self.__host.commandExecAdb("shell chmod 777 {0}".format(shell_script))
                adbOutput = self.__host.commandExecAdb("shell cat {0}".format(shell_script))[1]
                if intent_cmd in adbOutput:
                    # Send command through adb and detach the shell
                    status = self.__host.commandExecAdb("shell nohup sh {0} > /data/nohup.out".format(shell_script))[0]
                    # unplug immediately
                    self.__relayCard.usbConnection(False)
                    if status != 0:
                        self.__logger.printLog("INFO", "nohup command failed to return properly")
                        # force verdict to True, because when command times out, board reboots properly anyway
                    self.__logger.printLog("INFO", "shutdown script launched")
                    if not force:
                        self.__misc.waitDelay("wait shutdown duration", self.__configuration.timeout.SHUTDOWN_TIMEOUT + int(delay_time))
                else:
                    output = "embedded script was not correctly written ({0})".format(adbOutput)
                    verdict = False
            # Else send through adb (rebootCmd will unplug if needed)
            else:
                self.__logger.printLog("INFO", "Start graceful shutdown through adb")
                start, verdict = self.__runRebootCommand("adb " + self.__host.ssnExtra + "shell " + intent_cmd, plug)
                if not verdict:
                    output = "failure with adn shutdown intent"

        # Use touch screen to shutdown
        else:
            self.__relayCard.usbConnection(True)
            self.__logger.printLog("INFO", "Starting manually graceful shutdown")
            # Wake up
            self.__device.switchScreenState(True)
            time.sleep(1)
            self.__relayCard.powerButtonPress(2)
            start, verdict = self.__device.keyEvent(['KEYCODE_ENTER',
                                                     'KEYCODE_ENTER',
                                                     'KEYCODE_DPAD_RIGHT',
                                                     'KEYCODE_ENTER'])
            if not verdict:
                output = "key event issues"
            else:
                # If not plugged
                if not plug:
                    self.__relayCard.usbConnection(False)
                    self.__misc.waitDelay("wait shutdown duration", self.__configuration.timeout.SHUTDOWN_TIMEOUT)
                # else
                else:
                    verdict = self.waitOs("offline", timeout=self.__configuration.timeout.SHUTDOWN_TIMEOUT)
                    if not verdict:
                        output = "failure to shutdown after adb keyEvent intents"
        self.__output.appendOutput(output, verdict, argument="plug={0}".format(str(plug)))
        return verdict

    def adbRebootDnx(self):
        """ Run a 'adb reboot dnx' command
            Waits for boot completion
        """
        output = ""
        start, verdict = self.__runRebootCommand("adb " + self.__host.ssnExtra + "reboot dnx", waitOffline=False)
        if not verdict:
            output = "failure to shutdown after 'adb reboot dnx' command"
        else:
            if "bxtp" in self.__configuration.board:
                verdict = self.waitOs("cse-dnx", start=start, blind=True)
            else:
                verdict = self.waitOs("dnx", start=start, blind=True)
            if not verdict:
                output = "failure to reboot in DNX after 'adb reboot dnx' command"
        if verdict:
            self.__checkDnxEnumeration()
        self.__output.appendOutput(output, verdict)
        return verdict

    def fastbootOemRebootDnx(self):
        """ Run a 'fastboot oem reboot dnx' command
            Waits for boot completion
        """
        output = ""
        start, verdict = self.__runRebootCommand("fastboot " + self.__host.ssnExtra + "oem reboot dnx", waitOffline=False)
        if not verdict:
            output = "failure to shutdown after 'fastboot oem reboot dnx' command"
        else:
            verdict = self.waitOs("dnx", start=start, blind=True)
            if not verdict:
                output = "failure to reboot in DNX after 'fastboot oem reboot dnx' command"
        if verdict:
            self.__checkDnxEnumeration()
        self.__output.appendOutput(output, verdict)
        return verdict

    # TODO: class button()

    def forceShutdown(self, plug=True, delay=None):
        """ Perform a force Shutdown on the board.
        - Plug the SDP if plug=True
        - Press the power button for "delay" seconds
        """
        verdict = True
        output = ""
        if delay is None:
            delay = self.__configuration.timeout.PKPOFF
        delay = float(delay)
        if delay < self.__configuration.timeout.PKPOFF:
            shtn_duration = self.__configuration.timeout.SHUTDOWN_TIMEOUT
        else:
            shtn_duration = 5
            #On imin_legacy Moorefield we must not force shutdown directly due to Hynyx emmc failure
            if self.__configuration.boot.GRACEFUL_BEFORE_HARD_SHUTDOWN and not plug:
                self.gracefulShutdown(plug=plug, intent=True, delay_time=5, force=True)
                #plug=False

        self.__logger.printLog("INFO", "Forcing shutdown")
        self.__relayCard.usbConnection(plug)
        self.__relayCard.powerButtonPress(delay)

        if plug:
            verdict =  self.waitOs("offline", timeout=shtn_duration)
            if not verdict:
                output = "force_shutdown(plug={0}, {1}s): failed".format(plug, delay)
        else:
            time.sleep(shtn_duration)
        self.__output.appendOutput(output, verdict, argument="plug={0}".format(str(plug)))
        return verdict

    def selectPosMenu(self, select=0):
        """ Perform the Vol Up and Vol Down Button combo needed to access the PosMenu
        """
        kOn = 0.1
        kOff = 0.2
        # Wake up fastboot
        self.__relayCard.enableLine(self.__relayCard.relayConfiguration["VolumeUp"])
        time.sleep(kOn)
        self.__relayCard.disableLine(self.__relayCard.relayConfiguration["VolumeUp"])
        time.sleep(2 * kOff)
        # Go to first item
        for x in range(0, 4):
            self.__relayCard.enableLine(self.__relayCard.relayConfiguration["VolumeUp"])
            time.sleep(kOn)
            self.__relayCard.disableLine(self.__relayCard.relayConfiguration["VolumeUp"])
            time.sleep(kOff)
        time.sleep(kOff)
        # Display menu
        fromJB = self.__misc.checkVersionFrom("4.1")
        if not fromJB:
            self.__relayCard.enableLine(self.__relayCard.relayConfiguration["SwitchOnOff"])
            self.__relayCard.enableLine(self.__relayCard.relayConfiguration["VolumeUp"])
            time.sleep(kOn)
            self.__relayCard.disableLine(self.__relayCard.relayConfiguration["VolumeUp"])
            self.__relayCard.disableLine(self.__relayCard.relayConfiguration["SwitchOnOff"])
            time.sleep(2 * kOff)
        # VoDOWN
        for x in range(0, select):
            self.__relayCard.enableLine(self.__relayCard.relayConfiguration["VolumeDown"])
            time.sleep(kOn)
            self.__relayCard.disableLine(self.__relayCard.relayConfiguration["VolumeDown"])
            time.sleep(kOff)
        time.sleep(kOff)
        # Select
        self.__relayCard.powerButtonPress(1)
        time.sleep(10)

    def select_ros_menu(self, select=None):
        """ Perform the Button combo needed to access the ROS Menu.
            'select' argument gives number of down to press before selecting item.
            If 'select' is None, only open or close ROS Menu.
        """
        kOn = 0.5
        kOff = 0.5
        # Vol down button
        wBtn = self.__relayCard.relayConfiguration["VolumeDown"]
        # Pwr button
        sBtn = self.__relayCard.relayConfiguration["SwitchOnOff"]
        self.__relayCard.enableLine(wBtn)
        time.sleep(kOff)
        # Vol up button
        self.__relayCard.enableLine(self.__relayCard.relayConfiguration["VolumeUp"])
        time.sleep(kOn)
        self.__relayCard.disableLine(self.__relayCard.relayConfiguration["VolumeUp"])
        time.sleep(kOff)
        self.__relayCard.disableLine(wBtn)
        time.sleep(2)
        if select is None:
            time.sleep(1)
            return
        # VoDOWN
        for x in range(0, select):
            self.__relayCard.enableLine(self.__relayCard.relayConfiguration["VolumeDown"])
            time.sleep(kOn)
            self.__relayCard.disableLine(self.__relayCard.relayConfiguration["VolumeDown"])
            time.sleep(kOff)
        # Select
        time.sleep(kOff)
        self.__relayCard.enableLine(sBtn)
        time.sleep(kOn)
        self.__relayCard.disableLine(sBtn)
        time.sleep(2)

    def comboBoot(self, btn1=0, btn2=0, btn3=0, delay=1, wait_dnx=False):
        """ Executes a combo button press to boot an OS
        """
        if btn1 != 0:
            self.__relayCard.enableLine(btn1)
            time.sleep(0.5)
        if btn2 != 0:
            self.__relayCard.enableLine(btn2)
            time.sleep(0.5)
        if btn3 != 0:
            self.__relayCard.enableLine(btn3)
            time.sleep(0.5)

        self.__relayCard.powerButtonPress(delay)
        time.sleep(0.5)

        if wait_dnx and self.__configuration.flash.DEDIPROG:
            if not self.waitOs("dnx", timeout=150, loopTime=5):
                self.__logger.printLog("WARNING", "failure to detect DNX before releasing buttons")

        if btn1 != 0:
            self.__relayCard.disableLine(btn1)
            time.sleep(0.5)
        if btn2 != 0:
            self.__relayCard.disableLine(btn2)
            time.sleep(0.5)
        if btn3 != 0:
            self.__relayCard.disableLine(btn3)
            time.sleep(0.5)
        self.__output.appendOutput("", None)

    def __launchComboSequence(self, log, sequence):
        self.__logger.printLog("INFO", log + "sequence = \n{}".format(json.dumps(sequence, indent=4)))
        for element in sequence:
            if not isinstance(element, dict):
                self.__logger.printLog("WARNING", log + "invalid element in dnx sequence '{}'".format(element))
                return False
            if element.get("action") == "relay":
                if element.get("relay_name") not in self.__relayCard.relayConfiguration:
                    self.__logger.printLog("WARNING", log + "invalid relay_name in dnx sequence '{}'".format(element))
                    return False
                if isinstance(element.get("relay_state"), bool):
                    if element["relay_name"] == "UsbHostPcConnect":
                        self.__logger.printLog("INFO", log + "{} USB connection".format(
                                "enabling" if element["relay_state"] else "disabling"))
                        self.__relayCard.usbConnection(element["relay_state"])
                    else:
                        if element["relay_state"]:
                            self.__logger.printLog("INFO", log + "pressing on '{}' relay".format(element["relay_name"]))
                            self.__relayCard.enableLine(self.__relayCard.relayConfiguration[element["relay_name"]])
                            time.sleep(0.5)
                        else:
                            self.__logger.printLog("INFO", log + "releasing '{}' relay".format(element["relay_name"]))
                            self.__relayCard.disableLine(self.__relayCard.relayConfiguration[element["relay_name"]])
                            time.sleep(0.5)
                elif isinstance(element.get("relay_state"), int):
                    self.__logger.printLog("INFO",
                                           log + "pressing '{}' relay for {} "
                                                 "seconds".format(element["relay_name"], element["relay_state"]))
                    self.__relayCard.enableLine(self.__relayCard.relayConfiguration[element["relay_name"]])
                    time.sleep(element["relay_state"])
                    self.__relayCard.disableLine(self.__relayCard.relayConfiguration[element["relay_name"]])
                    time.sleep(0.5)
                elif str(element.get("relay_state", "")) in ["PKPON", "PKPOFF"]:
                    if str(element["relay_state"]) == "PKPON":
                        duration = self.__configuration.timeout.PKPON
                    elif str(element["relay_state"]) == "PKPOFF":
                        duration = self.__configuration.timeout.PKPOFF
                    else:
                        duration = 0
                    self.__logger.printLog("INFO",
                                           log + "pressing '{}' relay for {} "
                                                 "seconds".format(element["relay_name"], duration))
                    self.__relayCard.enableLine(self.__relayCard.relayConfiguration[element["relay_name"]])
                    time.sleep(duration)
                    self.__relayCard.disableLine(self.__relayCard.relayConfiguration[element["relay_name"]])
                    time.sleep(0.5)
                else:
                    self.__logger.printLog("WARNING", log + "invalid 'relay_state' in:\n'{}'".format(json.dumps(element)))
                    return False
            elif element.get("action") == "os":
                if not isinstance(element.get("timeout"), int):
                    self.__logger.printLog("WARNING", log + "invalid 'timeout' in:\n'{}'".format(json.dumps(element)))
                    return False
                elif not element.get("os_name"):
                    self.__logger.printLog("WARNING", log + "invalid 'os_name' in:\n'{}'".format(json.dumps(element)))
                    return False
                self.waitOs(element["os_name"], timeout=element["timeout"], waitWdExp=False)
            elif element.get("action") == "sleep":
                if not isinstance(element.get("duration"), int):
                    self.__logger.printLog("WARNING", log + "invalid 'duration' in:\n'{}'".format(json.dumps(element)))
                    return False
                self.__logger.printLog("INFO", log + "waiting for {}s".format(element["duration"]))
                time.sleep(element["duration"])
            else:
                self.__logger.printLog("WARNING", log + "unknown action in:\n'{}'".format(json.dumps(element)))
                return False
        return True

    def __innerDnxCombo(self, log):
        dnx_sequence = self.__configuration.boot.DNX_COMBO
        if not isinstance(dnx_sequence, list):
            self.__logger.printLog("WARNING", log + "invalid DNX sequence '{}'".format(dnx_sequence))
            return
        if not self.__launchComboSequence(log, dnx_sequence):
            self.__logger.printLog("WARNING", log + "issue while executing combo sequence")

    def posCombo(self):
        log = "comboPos(): "
        pos_sequence = self.__configuration.boot.POS_COMBO
        if not isinstance(pos_sequence, list):
            self.__logger.printLog("WARNING", log + "invalid POS sequence '{}'".format(pos_sequence))
            return
        if not self.__launchComboSequence(log, pos_sequence):
            self.__logger.printLog("WARNING", log + "issue while executing combo sequence")

    # inner comboDnx function
    def __dnxButtonPressAction(self, log, combo_delay=0):
        self.__logger.printLog("INFO", log + "press combo buttons to boot DnX")
        if not combo_delay:
            combo_delay = 3
        if self.__configuration.flash.DEDIPROG:
            self.__innerDnxCombo(log)
        else:
            self.comboBoot(self.__relayCard.relayConfiguration["VolumeDown"], delay=combo_delay)
        time.sleep(1)
        output = ""
        verdict = self.waitOs("dnx", timeout=10, blind=True, waitWdExp=False)
        if not verdict:
            output = "failure to boot DNX after combo key from OFF"
        self.__output.appendOutput(output, verdict)
        return verdict

    def comboDnx(self):
        """ Force shutdown and combo button press to boot DnX mode
        """
        output = ""
        verdict = True
        log = "comboDnx(): "
        self.__logger.printLog("INFO", log + "start")
        currentOs = self.getOs()

        # If already in DnX mode
        if currentOs == "dnx":
            self.__output.appendOutput("", True)
            return True

        # Check that volume button can be controlled
        buttonList = ["VolumeUp", "VolumeDown"]
        if any (button not in self.__relayCard.relayConfiguration for button in buttonList):
            output = "button missing in relay card configuration ({0})".format(", ".join(buttonList))
            verdict = False
        else:
            if not self.__dnxButtonPressAction(log=log):
                self.__logger.printLog("WARNING", "First try failed. Force shutdown and retry because board was in idle state", frontContent=log)
                # forced shutdown the board in case it is in idle state
                if not self.forceShutdown(plug=False):
                    output = "failure to force shutdown"
                    verdict = False
                else:
                    verdict = self.__dnxButtonPressAction(log=log)
                    if not verdict:
                        output = "combo key failure"
                        self.forceShutdown()
        self.__output.appendOutput(output, verdict)
        return verdict

    def gotoCseDnx(self):
        output = ""
        log = "gotoCseDnx(): "
        self.__logger.printLog("INFO", log + "start")
        currentOs = self.getOs()

        if currentOs == "cse-dnx":
            self.__output.appendOutput("", True)
            return True

        buttonList = ["VolumeUp", "VolumeDown"]
        if any (button not in self.__relayCard.relayConfiguration for button in buttonList):
            output = "button missing in relay card configuration ({0})".format(", ".join(buttonList))
            verdict = False
        else:
            self.__relayCard.enableLine(self.__relayCard.VolumeDown)
            self.__relayCard.enableLine(self.__relayCard.VolumeUp)
            self.forceShutdown(plug=False, delay=13)
            self.__relayCard.powerButtonPress(self.__relayCard.configuration.timeout.PKPON)
            self.__relayCard.disableLine(self.__relayCard.VolumeDown)
            self.__relayCard.disableLine(self.__relayCard.VolumeUp)
            self.__relayCard.usbConnection(True)

            verdict = self.waitOs("cse-dnx", timeout=150, loopTime=5)
            if not verdict:
                output = "failure to detect the cse dnx before releasing buttons"

        self.__output.appendOutput(output, verdict)
        return verdict

    # inner isPinPage method
    def __check(self):
        search = re.search("(LockPassword|CryptKeeper|Keyguard)", self.__device.getDumpsysFocus()[1])
        if search:
            return True, search.group(1)
        else:
            return False, ""

    def isPinPage(self, switch_screen=False):
        """ Check if a PIN code page is on focus
        """
        log = "isPinPage():"
        self.__logger.printLog("INFO", "start", frontContent=log)

        self.__device.removeStartScreen()

        state, page = self.__check()
        # If in password page
        if switch_screen and not state:
            # Switch the screen OFF/ON
            self.__device.switchScreenState(False)
            self.__device.switchScreenState(True)
            state, page = self.__check()
            if not state:
                if self.adbReboot(pincode_decrypt=False):
                    self.__device.removeStartScreen()
                    state, page = self.__check()
        self.__logger.printLog("INFO", log + "%s, %s" % (state, page))
        if state:
            output = ""
        else:
            output = "failure, page is: {0}".format(page)
        self.__output.appendOutput(output, state)
        return state, page

    # inner enterPinCode method to enter PIN code
    def __enter(self, interval, pin):
        # Switch on the screen and unlock
        self.__device.switchScreenState(True)
        focus = self.__device.getDumpsysFocus()[1]
        if "CryptKeeper" in focus:
            self.__host.commandExecAdb("shell input text " + pin)
            time.sleep(interval)
            self.__device.keyEvent(["KEYCODE_ENTER"], interval)
            time.sleep(self.__configuration.timeout.MOS_BOOT)
        elif "TelephonyEventsNotifier/.AlertDialogActivity" in focus:
            output = "Telephony Event Notifier blocks pincode access through adb"
            self.__logger.printLog("WARNING", output)
        else:
            self.__host.commandExecAdb("shell input text " + pin)
            time.sleep(interval)
            self.__device.keyEvent(["KEYCODE_ENTER"], interval)
            time.sleep(1)
        self.__output.appendOutput("", None)

    def enterPinCode(self, pin="0000", wrong_pin="3333", interval=1, pin_expected=True):
        """ Enter the pin code
        """
        log = "enterPinCode(): "
        self.__logger.printLog("INFO", log + "start")
        time.sleep(interval)
        verdict = True
        output = ""

        # If no ADB
        if self.__device.waitAdb(timeout=5) != "1":
            output = "no adb"
            verdict = False
        # If not in password page => False
        elif not self.isPinPage()[0]:
            output = "not in a PIN page"
            verdict = False
        elif not pin_expected:
            pin = wrong_pin
            self.__logger.printLog("INFO", log + "Enter wrong PIN code %s" % pin)
        else:
            self.__logger.printLog("INFO", log + "Enter expected PIN code %s" % pin)
        if verdict:
            self.__enter(interval, pin)
        self.__output.appendOutput(output, verdict)
        return True

    def pincodeSetting(self, interval=1):
        """ Configure a Pin Code to unlock device
        """
        log = "pincodeSetting(): "
        self.__logger.printLog("INFO", log + "start")

        # If no ADB
        if self.__device.waitAdb(timeout=5) != "1":
            self.__logger.printLog("WARNING", log + "no adb")
            return False

        # Check if a pin code is already configured
        verdict, output = self.checkPincode()
        if verdict:
            self.__logger.printLog("INFO", log + "PIN code already activated")
            self.__output.appendOutput("", True)
            return True

        # Start to set PIN
        self.__device.keyEvent(["KEYCODE_MENU"], interval)
        self.__device.adbRoot()

        # Open PIN Password page and enter PIN
        self.__logger.printLog("INFO", log + "set PIN code")
        self.__host.commandExecAdb("shell am start -n com.android.settings/.ChooseLockPassword", 3)

        # Enter 3 times (start, new, confirm)
        verdict, output = self.confirmPincode(interval=1)
        if not verdict:
            self.__output.appendOutput(output, verdict)
            return False

        # Check if a pin code is now configured
        verdict, output = self.checkPincode()
        if not verdict:
            self.__output.appendOutput("PIN code check failed", verdict)
            return False

        self.__logger.printLog("INFO", "done", frontContent=log)
        time.sleep(3)
        self.__output.appendOutput(output, verdict)
        return verdict

    def confirmPincode(self, interval=1, times=3):
        """ Confirm Pin Code in security settings or encryption
        """
        log = "confirmPincode():"
        self.__logger.printLog("INFO", "start", frontContent=log)
        verdict = None
        output = ""

        # Enter many times (start, new, confirm)
        for i in range(times):
            verdict, output = False, "failed to confirm PIN code"

            if self.enterPinCode(interval=interval):

                if self.isPinPage()[0]:
                    self.__logger.printLog("INFO", "continue", frontContent=log)
                else:
                    verdict, output = True, "confirmed"
                    break
            else:
                break

        self.__output.appendOutput(output, verdict)
        return verdict, output

    def checkPincode(self, pin_expected=True):
        """ enter PIN code to decrypt the device
        """
        log = "check_pincode():"
        self.__logger.printLog("INFO", "start", frontContent=log)
        output = ""
        verdict = True
        # Switch screen state and check if pin page present
        state, page_before = self.isPinPage(switch_screen=True)
        if not state:
            output = "not in a PIN page"
            verdict = False
        # Wrong pin code: pin_expected allows to check that a wrong pincode does not unlock
        elif not pin_expected:
            verdict = self.enterPinCode(pin_expected=False)
            if not verdict:
                output = "Failed to enter wrong PIN code"
            # If focus window has changed it means that the pin code is passed in google (so unlocked)
            elif self.isPinPage()[1] != page_before:
                output = "wrong PIN code can unlock device!"
                verdict = False
        if verdict:
            # Enter right pincode
            verdict = self.enterPinCode()
            if not verdict:
                output = "Failed to enter PIN code"
            elif self.isPinPage()[1] == page_before:
                output = "still locked"
                verdict = False
        if verdict:
            self.__logger.printLog("INFO", "unlocked", frontContent=log)
        self.__output.appendOutput(output, state)
        return verdict, output

    def encryptPhone(self, set_pincode=True, encrypt_timeout=200):
        """ Start the encryption of the data partition
            In case the battery is lower that 80%, the encryption will not start
        """
        log = "encryptPhone():"
        self.__logger.printLog("INFO", "start", frontContent=log)
        interval = 1

        # If no ADB
        if self.__device.waitAdb(timeout=5) != "1":
            output = "No ADB"
            self.__output.appendOutput(output, False)
            return False, output

        # Check if already encrypted
        if self.__device.getProperty("ro.crypto.state") == "encrypted":
            output = "already encrypted"
            self.__output.appendOutput(output, True)
            return True, output

        # Check battery info (present =True, level>=80%, cf MIN_BATTERY_LEVEL in CryptKeeperSettings.java)
        exec_status, present, level = self.__device.getDumpsysBattery()
        if not exec_status:
            output = "failure to dumpsys battery"
            self.__output.appendOutput(output, False)
            return False, output
        if self.__misc.str2bool(present) is False and int(level) < 80:
            output = "battery check failed (present={0}, level={1})".format(present, level)
            self.__output.appendOutput(output, False)
            return False, output

        # Kill monkey process and setup a pin code
        self.__host.commandExecAdb("shell pkill -9 monkey")
        if set_pincode and not self.pincodeSetting():
            output = "failure to set PIN code"
            self.__output.appendOutput(output, False)
            return False, output

        # Go to Encrypt phone page and confirm
        self.__logger.printLog("INFO", log + "start encryption")
        self.__host.commandExecAdb("shell am start -a android.app.action.START_ENCRYPTION")
        self.__device.getDumpsysFocus()
        # Complex movements to be successful on tablet/phone
        encrypt = ["KEYCODE_TAB",
                   "KEYCODE_TAB",
                   "KEYCODE_MOVE_HOME",
                   "KEYCODE_DPAD_DOWN",
                   "KEYCODE_DPAD_DOWN",
                   "KEYCODE_ENTER",
                   "KEYCODE_BUTTON_START"]
        self.__device.keyEvent(encrypt, interval)
        # Confirm PIN code
        self.__logger.printLog("INFO", log + "confirm PIN code")
        # Enter 3 times (start, new, confirm)
        verdict, output = self.confirmPincode(interval=1)
        if not verdict:
            self.__output.appendOutput(output, False)
            return False, output
        self.__logger.printLog("INFO", log + "confirm encryption")
        self.__device.keyEvent(encrypt, interval)
        self.__device.getDumpsysFocus()

        # Wait reboot
        start = time.time()
        if not self.waitOs("offline", timeout=encrypt_timeout):
            if "CryptKeeperConfirm" in self.__device.getDumpsysFocus()[1]:
                output = log + "encryption hang"
            elif "CryptKeeper " in self.__device.getDumpsysFocus()[1]:
                output = log + "encryption still ongoing"
            else:
                output = log + "failed to confirm"
            self.__output.appendOutput(output, False)
            return False, output

        if not self.waitOs("main", loopTime=5, blind=True, pincode_decrypt=False):
            output = "failure to reboot"
            self.__output.appendOutput(output, False)
            return False, output
        total = time.time() - start

        # Check that encryption is applied
        if self.__device.getProperty("ro.crypto.state") != "encrypted":
            output = "failure to encrypt"
            self.__output.appendOutput(output, False)
            return False, output

        # Check that /data is not mounted (False expected)
        if self.__device.checkPartitions(partition_list=["/data"])[0]:
            output = "/data should not be mounted"
            self.__output.appendOutput(output, False)
            return False, output

        # Success
        output = log + "encryption done (%.3fs)" % total
        self.__logger.printLog("INFO", output)
        self.__output.appendOutput("", True)
        return True, output

    def decryptDataPartition(self, pin_expected=True):
        """ Enter PIN code to decrypt device and check /data and all other partition are mounted
        """
        log = "decryptDataPartition(): "

        # Check that encryption is applied
        if self.__device.getProperty("ro.crypto.state") != "encrypted":
            output = "not needed as unencrypted device"
            self.__logger.printLog("WARNING", log + output)
            self.__output.appendOutput(output, True)
            return True, output

        # Check if CryptKeeper page
        if "CryptKeeper" not in self.__device.getDumpsysFocus()[1]:
            output = "already decrypted"
            self.__output.appendOutput(output, True)
            return True, output

        # Decrypt: enter pin code (a wrong one firstly if pin_expected=False)
        verdict, output = self.checkPincode(pin_expected=pin_expected)
        if not verdict:
            self.__output.appendOutput(output, False)
            return False, output

        # Check that /data is not mounted (False expected)
        if not self.__device.checkPartitions(["/factory", "/(oem_)*config", "/cache", "/system", "/logs", "/data"])[0]:
            output = "/data should be mounted"
            self.__output.appendOutput(output, False)
            return False, output

        # Success
        self.__logger.printLog("INFO", output)
        self.__output.appendOutput("", True)
        return True, "decrypted"

    def checkAccessPartition(self, partition, write=True):
        """ check that the partition content can be read
        """
        log = "checkAccessPartition(%s):" % partition
        self.__logger.printLog("INFO", "start", frontContent=log)
        self.__device.adbRoot()

        # If not adb
        if self.__host.commandExecAdb("shell echo ready", 3)[1] != "ready":
            output = "no response from ADB"
            self.__output.appendOutput(output, False)
            return False, output

        output = list()
        verdict = True
        # Write
        if write:
            back = ""
            word = "try_access"
            local_file = "%s/write.txt" % partition
            cmds = ["adb " + self.__host.ssnExtra + "shell touch %s" % local_file,
                    "adb " + self.__host.ssnExtra + "shell echo %s > %s" % (word, local_file),
                    "adb " + self.__host.ssnExtra + "shell cat %s" % local_file]
            for cmd in cmds:
                back = self.__host.commandExec(cmd, 3)[1]
            if back != word:
                verdict = False
                output.append("cannot write into '{0}'".format(partition))
                self.__logger.printLog("WARNING", "; ".join(output), frontContent=log)

        # Read
        cmd = "adb " + self.__host.ssnExtra + "shell find %s/ -type f | while read -d '' -r file ; do cat '$file' > /dev/null ; done" % partition
        # If failed to read
        if self.__host.commandExec(cmd, 30)[1]:
            verdict = False
            output.append("possibly corruption")
            self.__logger.printLog("WARNING", "; ".join(output), frontContent=log)
        # Success
        else:
            self.__logger.printLog("INFO", log + "read files")

        returnOutput = "; ".join(output)
        self.__output.appendOutput(returnOutput, True)
        return verdict, returnOutput

    def fastboot2adb(self):
        """ Execute a fastboot oem fastboot2adb
        """
        verdict = True
        output = ""
        # If fastboot
        if self.getOs() == "fastboot":
            exec_status, cmdOutput = self.__host.commandExecFastboot("oem fastboot2adb", 10)
            # If command failed
            if exec_status != 0:
                output = "fastboot2adb command failed ({0})".format(str(cmdOutput))
                verdict = False
            # If adb not available
            elif self.__device.waitAdb() == "-1":
                output = "failure to get adb"
                verdict = False
        # Not in fastboot
        else:
            output = "not in fastboot"
            verdict = False
        if not verdict and self.__workaround.isWorkaround(data_type=self.__workaround.Function, output=output)[0]:
            verdict = True
        self.__output.appendOutput(output, verdict)
        return verdict, output

    def adb2fastboot(self):
        """ Execute a adb shell setprop sys.adb.config fastboot
        """
        verdict = True
        output = ""
        # If adb is in root
        if self.__device.adbRoot():
            exec_status, adbOutput = self.__host.commandExecAdb("shell setprop sys.adb.config fastboot")
            # If command failed
            if exec_status != 0:
                output = "adb2fastboot command failed ({0})".format(adbOutput)
                verdict = False
            else:
                timeout_fastboot = 30
                start = time.time()
                fastbootOutput = ""
                # wait for POS to enumerate in fastboot again instead of adb
                while fastbootOutput == "":
                    fastbootOutput = self.__host.commandExecFastboot("devices")[1]
                    time.sleep(1)
                    if time.time() - start > timeout_fastboot:
                        break
                # failure if fastboot command does not respond properly
                if self.getOs(check_charger=False) != "fastboot":
                    output = "failure to have board enumerating in fastboot after command was launched"
                    verdict = False
        # If device is unreachable
        else:
            output = "failure to root device before launching command"
            verdict = False
        if not verdict and self.__workaround.isWorkaround(data_type=self.__workaround.Function, output=output)[0]:
            verdict = True
        self.__output.appendOutput(output, verdict)
        return verdict, output

    def acceptEula(self):
        log = "acceptEula(): "
        self.__logger.printLog("INFO", log + "start")
        verdict = True
        output = ""

        if not self.__device.adbRoot():
            verdict = False
            output = "failure to root board before accepting EULA"
        else:
            exec_status = self.__device.setProp("persist.accept.eula", "yes");
            if not exec_status and not self.__device.setProp("persist.accept.eula", "yes"):
                verdict = False
                output = "failure with adb command to set/get EULA property"
            else:
                first_reboot_status = self.adbReboot()
                if not first_reboot_status and not self.adbReboot():
                    verdict = False
                    output = "failure to reboot after EULA"
        self.__output.appendOutput(output, verdict)
        return verdict

    def gotoFastbootBxtpLh(self):
        log = "gotoFastbootBxtpLh(): "
        self.__logger.printLog("INFO", log + "start")

        if not self.forceShutdown(plug=False):
            verdict = False
            output = "failure to force shutdown the board"
        else:
            self.__relayCard.usbConnection(False)
            if "abl" in self.__configuration.board:
                self.__logger.printLog("INFO", "J6B2 disabled")
                self.__relayCard.disableLine(self.__relayCard.relayConfiguration["J6B2_relay"])
                time.sleep(2)
            self.__relayCard.powerButtonPress(self.__configuration.timeout.PKPON)
            self.__relayCard.usbConnection(True)
            uartSerialConnection = serial.Serial(port='/dev/ttySerial0', baudrate=115200, timeout=0)
            starttime = time.time()
            exec_status, fastboot = self.__host.commandExecFastboot("devices", timeout=8)
            while not fastboot and time.time() - starttime < 60:
                self.__logger.printLog("INFO", "Waiting for fastboot mode")
                uartSerialConnection.write("\033[B")
                exec_status, fastboot = self.__host.commandExecFastboot("devices", timeout=8)
                time.sleep(.2)
            time.sleep(10)
            exec_status, fastboot = self.__host.commandExecFastboot("devices", timeout=8)
            if fastboot:
                verdict = True
                output = "Board successfully reached fastboot OS"
            else:
                verdict = False
                output = "Board OS is: {}".format(self.getOs())

        self.__output.appendOutput(output, verdict)
        return verdict

    def gotoFastbootBxtpMrb(self):
        log = "gotoFastbootBxtpMrb(): "
        self.__logger.printLog("INFO", log + "start")

        self.__relayCard.disableLine(self.__relayCard.relayConfiguration["SwitchOnOff"])
        self.__logger.printLog("INFO", "J6B2 enabled")
        self.__relayCard.enableLine(self.__relayCard.relayConfiguration["J6B2_relay"])
        time.sleep(2)
        counter = 0
        fastboot_success = False
        while not fastboot_success and counter < 3:
            if not self.shutDownBxtpMrb():
                verdict = False
                output = "failure to force shutdown the board"
            else:
                usb2SerialConnection = serial.Serial(port='/dev/ttySerial2', baudrate=115200, timeout=0)
                self.__relayCard.usbConnection(True)
                starttime = time.time()
                usb2SerialConnection.write("n2#")
                exec_status, fastboot = self.__host.commandExecFastboot("devices", timeout=2)
                while not fastboot and time.time() - starttime < 120:
                    self.__logger.printLog("INFO", "Waiting for fastboot mode")
                    exec_status, fastboot = self.__host.commandExecFastboot("devices", timeout=2)
                    time.sleep(.2)
                if fastboot:
                    fastboot_success = True
                else:
                    output = "Board did not enter fastbood mode. Try number: " + str(counter +1)
            counter += 1

            if fastboot_success:
                verdict = True
                output = "Board successfully entered fastboot mode"
            else:
                verdict = False
                output = "Board did not enter fastboot mode"

        self.__output.appendOutput(output, verdict)
        return verdict

    def gotoDebugBxtpMrb(self):
        log = "gotoDebugBxtpMrb(): "
        self.__logger.printLog("INFO", log + "start")

        self.__relayCard.disableLine(self.__relayCard.relayConfiguration["SwitchOnOff"])
        self.__logger.printLog("INFO", "J6B2 enabled")
        self.__relayCard.enableLine(self.__relayCard.relayConfiguration["J6B2_relay"])
        time.sleep(2)

        try:
            usb2SerialConnection = serial.Serial(port='/dev/ttySerial2', baudrate=115200, timeout=0)
            usb2SerialConnection.write("2l4")
            time.sleep(2)
            usb2SerialConnection.write("\r")
            time.sleep(2)

            verdict = True
            output = "Board successfully entered debug mode"
        except:
            verdict = False
            output = "Board did not enter debug mode"

        self.__output.appendOutput(output, verdict)
        return verdict

    def gotoFastbootBxtpMrbYocto(self):
        log = "gotoFastbootBxtpMrbYocto(): "
        self.__logger.printLog("INFO", log + "start")

        self.__relayCard.disableLine(self.__relayCard.relayConfiguration["SwitchOnOff"])

        self.__logger.printLog("INFO", "Shut down the board using IOC r command")
        usb2SerialConnection = serial.Serial(port='/dev/ttySerial2', baudrate=115200, timeout=0)
        usb2SerialConnection.write("r")
        time.sleep(20)

        self.__relayCard.usbConnection(False)
        self.__logger.printLog("INFO", "J6B2 disabled")
        self.__relayCard.disableLine(self.__relayCard.relayConfiguration["J6B2_relay"])
        time.sleep(2)

        usb2SerialConnection = serial.Serial(port='/dev/ttySerial2', baudrate=115200, timeout=0)
        self.__logger.printLog("INFO", "Boot the board usinc IOC g command")
        usb2SerialConnection.write("g")
        self.__relayCard.usbConnection(True)
        uartSerialConnection = serial.Serial(port='/dev/ttySerial3', baudrate=115200, timeout=0)
        starttime = time.time()
        exec_status, fastboot = self.__host.commandExecFastboot("devices", timeout=2)
        while not fastboot and time.time() - starttime < 120:
            self.__logger.printLog("INFO", "Waiting for fastboot mode")
            uartSerialConnection.write("\033[B")
            exec_status, fastboot = self.__host.commandExecFastboot("devices", timeout=2)
            time.sleep(.2)
        if fastboot:
            verdict = True
            output = "Board successfully reached fastboot OS"
        else:
            verdict = False
            output = "Board OS is: {}".format(self.getOs())

        self.__output.appendOutput(output, verdict)
        return verdict

    def shutDownBxtpMrb(self):
        log = "shutDownBxtpMrb(): "
        self.__logger.printLog("INFO", log + "start")

        self.__logger.printLog("INFO", "Shut down the board using IOC r command")
        usb2SerialConnection = serial.Serial(port='/dev/ttySerial2', baudrate=115200, timeout=0)
        usb2SerialConnection.write("r")
        time.sleep(15)
        if self.__device.adbDevices()[0]:
            usb2SerialConnection.write("r")
            time.sleep(15)
            if self.__device.adbDevices()[0]:
                verdict = False
                output = "failure to shut down bxtp mrb board"
            else:
                verdict = True
                output = "shut down bxtp mrb board success"
        else:
            verdict = True
            output = "shut down bxtp mrb board success"

        self.__output.appendOutput(output, verdict)
        return verdict

    def rebootAfterFlashBxtpLh(self):
        log = "rebootAfterFlashBxtpLh(): "
        self.__logger.printLog("INFO", log + "start")

        if not self.forceShutdown(plug=False):
            verdict = False
            output = "failure to force shutdown the board"
        else:
            self.__logger.printLog("INFO", "Change J6B2 relay state")
            if "abl" in self.__configuration.board:
                self.__relayCard.enableLine(self.__relayCard.relayConfiguration["J6B2_relay"])
                time.sleep(2)
            else:
                self.__relayCard.disableLine(self.__relayCard.relayConfiguration["J6B2_relay"])
                time.sleep(2)
            self.__relayCard.powerButtonPress(self.__configuration.timeout.PKPON)
            time.sleep(5)
            if "abl" in self.__configuration.board:
                self.__logger.printLog("INFO", "Send uart command boot mmc2:@0")
                uartSerialConnection = serial.Serial(port='/dev/ttySerial0', baudrate=115200, timeout=0)
                uartSerialConnection.write("boot mmc2:@0")
            time.sleep(30)
            starttime = time.time()
            #while not self.__device.adbDevices()[0] and time.time() - starttime < 300:
            while self.__device.getProperty("sys.boot_completed") != "1" and time.time() - starttime < 300:
                self.__logger.printLog("INFO", "Waiting for board to boot after flash")
                #time.sleep(.2)
                time.sleep(.5)
            #if self.__device.adbDevices()[0]:
            if self.__device.getProperty("sys.boot_completed") == "1":
                verdict = True
                output = "Board successfully rebooted after flash"
            else:
                verdict = False
                output = "Board reboot after flash failed"

        self.__output.appendOutput(output, verdict)
        return verdict

    def rebootBxtpMrb(self):
        log = "rebootBxtpMrb(): "
        self.__logger.printLog("INFO", log + "start")

        counter = 0
        reboot_success = False
        while not reboot_success and counter < 3:
            if not self.shutDownBxtpMrb():
                verdict = False
                output = "failure to force shutdown the board"
            else:
                self.__logger.printLog("INFO", "Boot the board usinc IOC g command")
                usb2SerialConnection = serial.Serial(port='/dev/ttySerial2', baudrate=115200, timeout=0)
                #usb3SerialConnection = serial.Serial(port='/dev/ttySerial3', baudrate=115200, timeout=0)
                usb2SerialConnection.write("g")
                starttime = time.time()
                while self.__device.getProperty("sys.boot_completed") != "1" and time.time() - starttime < 300:
                    self.__logger.printLog("INFO", "Waiting for board to boot (boot completed)")
                    # ------- Command used to connect on the serial port ------ #
                    #usb3SerialConnection.write("su\r\n")
                    time.sleep(.5)
                    #usb3SerialConnection.write("echo p > /sys/bus/platform/devices/intel-cht-otg.0/mux_state\r\n")
                    #time.sleep(10)
                if self.__device.getProperty("sys.boot_completed") == "1":
                    reboot_success = True
            counter += 1

            if reboot_success:
                verdict = True
                output = "Board successfully rebooted"

                if self.__device.getProperty("ro.boot.verifiedbootstate") == "green":
                    self.__logger.printLog("INFO", "ro.boot.verifiedbootstate is green")
                else:
                    verdict = False
                    self.__logger.printLog("WARNING", "ro.boot.verifiedbootstate is NOT green")
            else:
                verdict = False
                output = "Board reboot failed"

        self.__output.appendOutput(output, verdict)
        return verdict

    def rebootAfterFlashBxtpMrbYocto(self):
        log = "rebootAfterFlashBxtpMrbYocto(): "
        self.__logger.printLog("INFO", log + "start")

        self.__logger.printLog("INFO", "Shut down the board using IOC r command")
        usb2SerialConnection = serial.Serial(port='/dev/ttySerial2', baudrate=115200, timeout=0)
        usb2SerialConnection.write("r")
        time.sleep(20)

        self.__logger.printLog("INFO", "J6B2 enabled")
        self.__relayCard.enableLine(self.__relayCard.relayConfiguration["J6B2_relay"])
        time.sleep(2)

        self.__logger.printLog("INFO", "Boot the board usinc IOC g command")
        usb2SerialConnection.write("g")
        time.sleep(5)
        starttime = time.time()
        exec_status, out = self.__host.commandExec("ping 192.168.1.1 -c 1", 10)
        while "ttl=64" not in out and time.time() - starttime < 300:
            self.__logger.printLog("INFO", "Waiting for board to boot after flash")
            exec_status, out = self.__host.commandExec("ping 192.168.1.1 -c 1", 10)
            time.sleep(.2)
        if "ttl=64" in out:
            verdict = True
            output = "Board successfully rebooted after flash"
        else:
            verdict = False
            output = "Board reboot after flash failed"
        self.__host.commandExec("ip a")

        self.__output.appendOutput(output, verdict)
        return verdict

    def checkBootCompletedBxtpMrb(self):
        log = "checkBootCompletedBxtpMrb(): "
        self.__logger.printLog("INFO", log + "start")

        if self.__device.getProperty("sys.boot_completed") == "1":
            verdict = True
            output = "Boot completed; device in main OS"
        else:
            verdict = False
            output = "Boot not completed yet"

        self.__output.appendOutput(output, verdict)
        return verdict
