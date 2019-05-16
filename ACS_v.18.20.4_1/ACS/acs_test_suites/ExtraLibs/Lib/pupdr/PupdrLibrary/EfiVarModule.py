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
@summary: Pupdr Library - EfiVarModule
@since: 11/17/2014
@author: travenex
"""

import LoggerModule
import HostModule
import RelayCardModule
import DeviceModule
import WorkaroundModule
import MiscModule
import OsManagerModule
import FlashFileModule
import OutputModule

class EfiVarModule(object):

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
    __RSCI_dict = None
    efi_vars_matching = None
    __flashFile = None
    __output = None

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
        self.__output = OutputModule.OutputModule()
        # table to match uefi var with offset in RCSI_table
        self.__RSCI_dict = dict()
        self.__RSCI_dict["wake_source"] = 36
        self.__RSCI_dict["reset_source"] = 37
        self.__RSCI_dict["reset_type"] = 38
        self.__RSCI_dict["shutdown_source"] = 39
        self.__RSCI_dict["reset_extra_information"] = None
        # table to match uefi var to name in sysfs
        self.efi_vars_matching = dict()
        self.efi_vars_matching["wake_source"] = "WakeSource"
        self.efi_vars_matching["reset_source"] = "ResetSource"
        self.efi_vars_matching["reset_type"] = "ResetType"
        self.efi_vars_matching["shutdown_source"] = "ShutdownSource"
        self.efi_vars_matching["mode"] = "LoaderEntryLast"
        self.efi_vars_matching["watchdog"] = "WdtCounter"

    def __isUniq(self, keyw, missing_ok=False):
        """ Find the number of occurrence of an EFI var by its name (multiple GUID)
            If missing_ok is True the function returns True if the efi var is missing
        """
        log = ""
        verdict = True

        # Check the uniqueness of the efi variable, same variable can be present with 2 different GUID
        exec_status, efi_var_list = self.__host.commandExecAdb("shell ls /sys/firmware/efi/vars/")
        occurrence = efi_var_list.count(keyw)

        self.__logger.printLog("DEBUG", "Occurrence found : %s" % str(occurrence))

        if occurrence == 0 and not missing_ok:
            log = "%s efi variable not present" % keyw
            self.__logger.printLog("WARNING", log)
            verdict = False

        elif occurrence > 1:
            log = "Multiple occurrence of efi variable %s" % keyw
            self.__logger.printLog("WARNING", log)
            verdict = False

        return verdict, log, occurrence

    def setEfiVar(self, keyw, value):
        """ Set the value of an EFI var on EDK2 BIOS
          This method is dangerous ! make sure you know what you are doing
        """
        # determine the value type
        if type(value) == int:
            value_type = "int"
            value = str(value)
        elif type(value) == str:
            value_type = "string"
        else:
            return False, "Wrong value type in set_efi_var %s" % keyw

        # Get occurrence
        exec_status, log, occurrence = self.__isUniq(keyw, missing_ok=True)

        # if the efi var requested is unique
        if exec_status:
            if occurrence == 0:
                # get the GUID of the namespace (ref. WakeSource)
                exec_status, efi_var_full_name = self.__host.commandExecAdb("shell ls /sys/firmware/efi/vars/ | grep -a WakeSource")
            else:
                # get the efi var GUID
                exec_status, efi_var_full_name = self.__host.commandExecAdb("shell ls /sys/firmware/efi/vars/ | grep -a %s" % keyw)
            guid = "-".join(efi_var_full_name.split("-")[1:])
            self.__logger.printLog("DEBUG", "Constructed GUID %s" % guid)
            # write the value in the uefi var
            exec_status, log = self.__host.commandExecAdb("shell uefivar -g " + guid
                                                     + " -n " + keyw
                                                     + " -t " + value_type
                                                     + " -s " + value)
            if exec_status != 0:
                verdict = False
            else:
                verdict = True
        else:
            verdict = False

        return verdict, log

    def getHexdumpValue(self, keyw):
        exec_status, output = self.__host.commandExecAdb("shell hd /sys/firmware/efi/vars/" + keyw + "-*/data")
        if exec_status != 0 or "hd" in output:
            self.__logger.printLog("WARNING", "failed to hexdump {0}".format(keyw))
            return False, None
        character_list = list()
        sub_list= list()
        output_value = str()
        for element in output.split("\n")[:-1]:
            for sub_element in element.split(" ")[1:-2]:
                if sub_element:
                    character_list.append(sub_element)
        if len(character_list) == 0:
            self.__logger.printLog("WARNING", "no element in character_list")
            return False, None
        elif len(character_list) == 1:
            output_value = character_list[0].lower()
        else:
            for i in range(len(character_list)/2):
                sub_list.append(character_list[2*i+1] + character_list[2*i])
            for element_list in sub_list:
                if element_list != "0000":
                    output_value += chr(int(element_list,16))
        self.__logger.printLog("DEBUG", "getHexdumpValue({0}) = '{1}'".format(keyw, output_value))
        return True, output_value

    def getOctalDumpValue(self, keyw):
        exec_status, output = self.__host.commandExecAdb("shell od -x /sys/firmware/efi/vars/" + keyw + "-*/data")
        if exec_status != 0 or "od" in output:
            self.__logger.printLog("WARNING", "failure to hexdump {0}".format(keyw))
            return False, None
        character_list = list()
        sub_list= list()
        output_value = str()
        for element in output.split("\n"):
            for sub_element in element.split(" ")[1:]:
                if sub_element:
                    character_list.append(sub_element)
        if len(character_list) == 0:
            self.__logger.printLog("WARNING", "no element in character_list")
            return False, None
        elif len(character_list) == 1:
            output_value = character_list[0].lower()
        else:
            for i in range(len(character_list)/2):
                sub_list.append(character_list[2*i+1] + character_list[2*i])
            for element_list in sub_list:
                if element_list != "0000":
                    output_value += element_list.replace("\r", "")
        self.__logger.printLog("DEBUG", "getHexdumpValue({0}) = '{1}'".format(keyw, output_value))
        return True, output_value

    def getEfiVar(self, keyw):
        """ Read the efi variable in the sysfs of the phone.
        """
        exec_status, log, occurrence = self.__isUniq(keyw)

        # if the efi var is unique
        if exec_status:
            exec_status, efi_var = self.getHexdumpValue(keyw)

            if not exec_status:
                output = "failed getHexdumpValue(%s)" % keyw
                self.__logger.printLog("WARNING", output)
                return False, output
        else:
            if keyw == "WdtCounter" and occurrence == 0:
                # If there hasn't been any WD expiration since the FW flash the var doesn't exist. It is equivalent to WdtCounter = 00
                return True, "00"
            else:
                return False, log

        return True, efi_var

    def getRSCITable(self, table_location):
        RSCI_table = list()
        verdict = True
        # execute od command to dump full RSCI table
        exec_status, output = self.__host.commandExecAdb("shell od {0} -x".format(table_location), timeout=10)
        parsing = "od"
        if exec_status != 0 or "od" in output:
            self.__logger.printLog("WARNING", "failure to od file {0}, trying hd command (error={1})".format(table_location, output))
            exec_status, output = self.__host.commandExecAdb("shell hd {0}".format(table_location))
            parsing = "hd"
            if exec_status != 0 or "hd" in output:
                self.__logger.printLog("WARNING", "failure to od and hd file {0} (error={1})".format(table_location, output))
                verdict = False
        if verdict:
            # compute formatted RSCI table in python list
            sub_list=list()
            if parsing == "od":
                for line in (l for l in output.split("\n") if l.startswith("0000")):
                        for element in line.split(" ")[1:]:
                            if element != " ":
                                for i in range(0, len(element) - 1, 2):
                                    sub_list.append(element[i:i+2])
                if len(sub_list) == 0:
                    self.__logger.printLog("WARNING", "no element in sub_list")
                    return False, None
                elif len(sub_list) == 1:
                    RSCI_table = sub_list
                else:
                    for i in range(len(sub_list)/2):
                        RSCI_table.append(sub_list[2*i+1])
                        RSCI_table.append(sub_list[2*i])
            elif parsing == "hd":
                for line in (l for l in output.split("\n") if l.startswith("0000")):
                    for element in line.split(" ")[1:-2]:
                        if element != " ":
                            RSCI_table.append(element)
            # get value if index is valid
            self.__logger.printLog("DEBUG", "formatted RSCI output: {0}".format(" ".join(RSCI_table)))
        return RSCI_table

    def getRSCIValue(self, stringId, table_content):
        # check in name exist in dictionary
        if stringId not in self.__RSCI_dict:
            self.__logger.printLog("WARNING", "unknown uefi name: {0}".format(stringId))
            return False, None
        # check if table not empty
        if len(table_content) == 0:
            self.__logger.printLog("WARNING", "empty RSCI table")
            return False, None

        if stringId == "reset_extra_information":
            outputValue = bin(int(table_content[44],16)+16*int(table_content[45],16)+16*16*int(table_content[46],16)+16*16*16*int(table_content[47],16)).split("0b")[-1]
            self.__logger.printLog("INFO", "getRSCIValue({0}, indexes 44 to 47) = {1}".format(stringId, outputValue))
            return True, outputValue
        elif len(table_content) > self.__RSCI_dict[stringId]:
            outputValue = table_content[self.__RSCI_dict[stringId]]
            self.__logger.printLog("INFO", "getRSCIValue({0}, index={2}) = {1}".format(stringId, outputValue, self.__RSCI_dict[stringId]))
            return True, outputValue
        else:
            self.__logger.printLog("WARNING", "invalid index, cannot get value (index={0}, length={1})".format(self.__RSCI_dict[stringId], len(table_content)))
            return False, None

    @staticmethod
    def formatRsciValue(nameId, value):
        value = value.upper()
        output_value = value + " (unknown)"
        if nameId == "wake_source":
            if value == "00":
                return "00 (not applicable)"
            elif value == "01":
                return "01 (battery inserted)"
            elif value == "02":
                return "02 (usb charger inserted)"
            elif value == "03":
                return "03 (acdc charger inserted)"
            elif value == "04":
                return "04 (power button pressed)"
            elif value == "05":
                return "05 (rtc timer)"
            elif value == "06":
                return "06 (battery reached ia threshold)"
        elif nameId == "reset_source":
            if value == "00":
                return "00 (not applicable)"
            elif value == "01":
                return "01 (os initiated)"
            elif value == "02":
                return "02 (forced)"
            elif value == "03":
                return "03 (fw update)"
            elif value == "04":
                return "04 (kernel watchdog)"
            elif value == "05":
                return "05 (security watchdog)"
            elif value == "06":
                return "06 (security initiated)"
            elif value == "07":
                return "07 (pmc watchdog)"
            elif value == "08":
                return "08 (ec watchdog)"
            elif value == "09":
                return "09 (pmic watchdog)"
            elif value == "0A":
                return "0A (pmc reset timeout)"
            elif value == "0B":
                return "0B (short power loss)"
            elif value == "0C":
                return "0C (platform specific)"
            elif value == "FF":
                return "FF (unknown)"
        elif nameId == "reset_type":
            if value == "00":
                return "00 (not applicable)"
            elif value == "01":
                return "01 (warm reset)"
            elif value == "02":
                return "02 (cold reset)"
            elif value == "07":
                return "07 (global reset)"
        elif nameId == "shutdown_source":
            if value == "00":
                return "00 (not applicable)"
            elif value == "01":
                return "01 (power button override)"
            elif value == "02":
                return "02 (battery removal)"
            elif value == "03":
                return "03 (vcrit)"
            elif value == "04":
                return "04 (thermtrip)"
            elif value == "05":
                return "05 (pmictemp)"
            elif value == "06":
                return "06 (systemp)"
            elif value == "07":
                return "07 (battemp)"
            elif value == "08":
                return "08 (sysuvp)"
            elif value == "09":
                return "09 (sysovp)"
            elif value == "0A":
                return "0A (security watchdog)"
            elif value == "0B":
                return "0B (security initiated)"
            elif value == "0C":
                return "0C (pmc watchdog)"
            elif value == "0D":
                return "0D (ec watchdog)"
            elif value == "0E":
                return "0E (platform watchdog)"
        elif nameId == "reset_extra_information":
            if value == "0":
                return "0 (not applicable)"
            elif value == "1":
                return "1 (ISH Sram ECC (Req0) global reset)"
            elif value == "10":
                return "10 (ISH watchdog expiration (Req1))"
            elif value == "100":
                return "100 (ISH misc (Req2) global reset)"
            elif value == "1000":
                return "1000 (CSE MIA internal)"
            elif value == "10000":
                return "10000 (CSE MIA shutdown)"
            elif value == "100000":
                return "100000 (CSE SRAM ECC)"
            elif value == "1000000":
                return "1000000 (CSE watchdog timeout)"
            elif value == "10000000":
                return "10000000 (CSE firmware indication that next reset should be global reset)"
            elif value == "100000000":
                return "100000000 (CSE firmware global reset)"
            elif value == "1000000000":
                return "1000000000 (PMC FW timeout)"
            elif value == "10000000000":
                return "10000000000 (PMC watchdog expired)"
            elif value == "100000000000":
                return "100000000000 (PMC firmware error)"
            elif value == "1000000000000":
                return "1000000000000 (Microcode shutdown special cycle)"
            elif value == "10000000000000":
                return "10000000000000 (Non-critical thermal trip)"
            elif value == "100000000000000":
                return "100000000000000 (DFX global reset trigger)"
            elif value == "1000000000000000":
                return "1000000000000000 (Save/restore module completion count error)"
        elif nameId == "fw_error_code":
            if value == "0x1311":
                return value + " (PMC_RESET_ENTRY_IP_RESET_PREP_ACK_TIMEOUT_ERROR)"
            elif value == "0x0b11":
                return value + " (PMC_CSE_CLD_RST_IP_RESET_PREP_ACK_TIMEOUT_ERROR)"
            elif value == "0x136b":
                return value + " (PMC_RESET_ENTRY_CSE_RESET_WARN_ACK_TIMEOUT_ERROR)"
            elif value == "0x076b":
                return value + " (PMC_BOOT_CSE_RESET_WARN_ACK_TIMEOUT_ERROR)"
            elif value == "0x0b6b":
                return value + " (PMC_CSE_CLD_RST_CSE_RESET_WARN_ACK_TIMEOUT_ERROR)"
            elif value == "0x1310":
                return value + " (PMC_RESET_ENTRY_PUNIT_RESET_WARN_ACK_TIMEOUT_ERROR)"
            elif value == "0x104c":
                return value + " (PMC_RESET_ENTRY_PLATFORM_EVENT_LOGGED)"
            elif value == "0x17b1":
                return value + " (PMC_RT_EVENT_LOOP_ACK_SX_MSG_TIMEOUT_ERROR)"
        return output_value

    def isRsciEntry(self, name):
        return name in self.__RSCI_dict