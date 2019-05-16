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

@organization: INTEL CCG PSI
@summary: Pupdr Library
@since: 03/11/2015
@author: travenex
"""

import LoggerModule
import HostModule
import RelayCardModule
import DeviceModule
import OsManagerModule
import WorkaroundModule
import MiscModule
import FlashFileModule
import FlashModule
import LogsModule
import EfiVarModule
import DediprogModule
import OutputModule
import BootDataModule
import DownloadModule
import ResetIrqModule
import WatchdogModule
import CampaignModule
import TestingModule
import ReportModule
import EventsModule
import ConfigurationModule
import scripts.COS_scripts
import scripts.MOS_scripts
import scripts.Watchdog_scripts
import scripts.KeyPress_scripts
import scripts.POS_scripts
import scripts.RMA_scripts
import scripts.ROS_scripts
import scripts.DNX_scripts
import scripts.Flash_scripts
import scripts.Shell_scripts
import scripts.Time_scripts
import scripts.Debug_scripts

class Pupdr(object):

    __instance = None
    __version = "Release_2017WW10.5.14h40 - Update branch name master for O and n_master for N"
    __framework_push = "ACS Library Rebase Release_2017WW12.5.14h05"
    __globalConf = None
    Logger = None
    Misc = None
    Configuration = None
    Workaround = None
    Host = None
    RelayCard = None
    Output = None
    Device = None
    Campaign = None
    OsManager = None
    Dediprog = None
    FlashFile = None
    Logs = None
    EfiVar = None
    Flash = None
    ResetIrq = None
    BootData = None
    Watchdog = None
    Download = None
    Events = None
    Report = None
    Testing = None
    COS_scripts = None
    MOS_scripts = None
    Debug_scripts = None
    Watchdog_scripts = None
    KeyPress_scripts = None
    POS_scripts = None
    RMA_scripts = None
    ROS_scripts = None
    DNX_scripts = None
    Flash_scripts = None
    Shell_scripts = None
    Time_scripts = None

    def __new__(cls):
        """
        Constructor that always returns the same instance
        """
        if not cls.__instance:
            cls.__instance = object.__new__(cls)
        return cls.__instance

   # def __init__(self):


    def init(self, conf):
        # global conf
        self.__globalConf = conf

        # create all modules

        # create logger
        isExternal = self.__globalConf.get("EXTERNAL_LOGGER")
        LoggerModule.LoggerModule().init(self.__globalConf, externalLogger=isExternal)
        self.Logger = LoggerModule.LoggerModule()
        # create misc
        MiscModule.MiscModule().init(self.__globalConf)
        self.Misc = MiscModule.MiscModule()
        # create configs
        ConfigurationModule.ConfigurationModule().init(self.__globalConf)
        self.Configuration = ConfigurationModule.ConfigurationModule()
        # create workaround
        WorkaroundModule.WorkaroundModule().init(self.__globalConf)
        self.Workaround = WorkaroundModule.WorkaroundModule()
        # create host
        isExternal = self.__globalConf.get("EXTERNAL_LOCAL_EXEC", False)
        HostModule.HostModule().init(self.__globalConf, externalCmdExec=isExternal)
        self.Host = HostModule.HostModule()
        # create relay card
        RelayCardModule.RelayCardModule().init(self.__globalConf, externalRelayCard=self.__globalConf.get("EXTERNAL_RELAY_CARD"))
        self.RelayCard = RelayCardModule.RelayCardModule()
        # create report
        ReportModule.ReportModule().init(self.__globalConf)
        self.Report = ReportModule.ReportModule()
        # create output
        OutputModule.OutputModule().init(self.__globalConf)
        self.Output = OutputModule.OutputModule()
        # create device
        DeviceModule.DeviceModule().init(self.__globalConf)
        self.Device = DeviceModule.DeviceModule()
        # create campaign
        CampaignModule.CampaignModule().init(self.__globalConf)
        self.Campaign = CampaignModule.CampaignModule()
        # create OsManager
        OsManagerModule.OsManagerModule().init(self.__globalConf)
        self.OsManager = OsManagerModule.OsManagerModule()
        # create dediprog
        DediprogModule.DediprogModule().init(self.__globalConf)
        self.Dediprog = DediprogModule.DediprogModule()
        # create FlashFile
        FlashFileModule.FlashFileModule().init(self.__globalConf)
        self.FlashFile = FlashFileModule.FlashFileModule()
        # create logs
        LogsModule.LogsModule().init(self.__globalConf)
        self.Logs = LogsModule.LogsModule()
        # create efi var
        EfiVarModule.EfiVarModule().init(self.__globalConf)
        self.EfiVar = EfiVarModule.EfiVarModule()
        # create Flash
        FlashModule.FlashModule().init(self.__globalConf)
        self.Flash = FlashModule.FlashModule()
        # create reset irq
        ResetIrqModule.ResetIrqModule().init(self.__globalConf)
        self.ResetIrq = ResetIrqModule.ResetIrqModule()
        # create boot data
        BootDataModule.BootDataModule().init(self.__globalConf)
        self.BootData = BootDataModule.BootDataModule()
        # create watchdog
        WatchdogModule.WatchdogModule().init(self.__globalConf)
        self.Watchdog = WatchdogModule.WatchdogModule()
        # create download
        DownloadModule.DownloadModule().init(self.__globalConf)
        self.Download = DownloadModule.DownloadModule()
        # create testing
        TestingModule.TestingModule().init(self.__globalConf)
        self.Testing = TestingModule.TestingModule()
        # create DntEvents
        EventsModule.EventsModule().init(self.__globalConf)
        self.Events = EventsModule.EventsModule()

        # scripts
        scripts.COS_scripts.COS().init(self.__globalConf)
        self.COS_scripts = scripts.COS_scripts.COS()
        scripts.MOS_scripts.MOS().init(self.__globalConf)
        self.MOS_scripts = scripts.MOS_scripts.MOS()
        scripts.Watchdog_scripts.Watchdog().init(self.__globalConf)
        self.Watchdog_scripts = scripts.Watchdog_scripts.Watchdog()
        scripts.KeyPress_scripts.KeyPress().init(self.__globalConf)
        self.KeyPress_scripts = scripts.KeyPress_scripts.KeyPress()
        scripts.POS_scripts.POS().init(self.__globalConf)
        self.POS_scripts = scripts.POS_scripts.POS()
        scripts.RMA_scripts.RMA().init(self.__globalConf)
	self.RMA_scripts = scripts.RMA_scripts.RMA()
        scripts.ROS_scripts.ROS().init(self.__globalConf)
        self.ROS_scripts = scripts.ROS_scripts.ROS()
        scripts.DNX_scripts.DNX().init(self.__globalConf)
        self.DNX_scripts = scripts.DNX_scripts.DNX()
        scripts.Flash_scripts.Flash().init(self.__globalConf)
        self.Flash_scripts = scripts.Flash_scripts.Flash()
        scripts.Shell_scripts.Shell().init(self.__globalConf)
        self.Shell_scripts = scripts.Shell_scripts.Shell()
        scripts.Time_scripts.Time().init(self.__globalConf)
        self.Time_scripts = scripts.Time_scripts.Time()
        scripts.Debug_scripts.Debug().init(self.__globalConf)
        self.Debug_scripts = scripts.Debug_scripts.Debug()

        # print Library version
        self.Logger.printLog("INFO", "BOOT/OTA LIBRARY Code Version: " + self.__version)
        self.__globalConf["BOOTOTA_LIBRARY_CODE_VERSION"] = self.__version
        if self.__framework_push:
            self.Logger.printLog("INFO", "FRAMEWORK Code Version: " + self.__framework_push)
            self.__globalConf["FRAMEWORK_CODE_VERSION"] = self.__framework_push
