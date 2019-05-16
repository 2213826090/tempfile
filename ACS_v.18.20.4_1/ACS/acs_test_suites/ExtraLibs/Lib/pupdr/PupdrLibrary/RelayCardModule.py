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
@summary: Pupdr Library - RelayCardModule
@since: 11/17/2014
@author: travenex
"""

import os
import time
import threading
import LoggerModule
import HostModule
import OutputModule
import ConfigurationModule

class RelayCardModule(object):

    __instance = None
    globalConf = None
    __logger = None
    __host = None
    __output = None
    configuration = None
    cardConfiguration = None
    relayConfiguration = None
    __delayBetweenCommands = None
    __relay = None
    SwitchOnOff = None
    UsbHostPcConnect = None
    VolumeUp = None
    VolumeDown = None
    Dediprog = None
    J6B2_relay = None
    PowerSupply = None
    __allow_usb_toggle_event = None
    __usb_connected = None

    def __new__(cls):
        """
        Constructor that always returns the same instance
        """
        if not cls.__instance:
            cls.__instance = object.__new__(cls)
        return cls.__instance

    def init(self, globalConf=None, externalRelayCard=None):
        self.globalConf = globalConf
        self.__logger = LoggerModule.LoggerModule()
        self.__host = HostModule.HostModule()
        self.__output = OutputModule.OutputModule()
        self.configuration = ConfigurationModule.ConfigurationModule()
        self.cardConfiguration = self.globalConf.get("RELAY_CARD_CONFIGURATION", dict())
        self.relayConfiguration = self.cardConfiguration.get("relays", dict())
        self.__delayBetweenCommands = 1
        self.__allow_usb_toggle_event = False
        self.__usb_connected = "unknown"
        if externalRelayCard:
            self.__relay = externalRelayCard
        else:
            self.__relay = self.__internalRelayCard()
            self.__relay.setConfig(self.cardConfiguration)
            if self.relayConfiguration:
                self.__relay.setDefaultState()
            else:
                self.relayConfiguration["SwitchOnOff"] = -1
                self.relayConfiguration["UsbHostPcConnect"] = -1
                self.relayConfiguration["VolumeUp"] = -1
                self.relayConfiguration["VolumeDown"] = -1
                self.relayConfiguration["Dediprog"] = -1
                self.relayConfiguration["PowerSupply"] = -1
                self.relayConfiguration["J6B2_relay"] = -1
        self.SwitchOnOff = self.relayConfiguration.get("SwitchOnOff", -1)
        self.UsbHostPcConnect = self.relayConfiguration.get("UsbHostPcConnect", -1)
        self.VolumeUp = self.relayConfiguration.get("VolumeUp", -1)
        self.VolumeDown = self.relayConfiguration.get("VolumeDown", -1)
        self.Dediprog = self.relayConfiguration.get("Dediprog", -1)
        self.PowerSupply = self.relayConfiguration.get("PowerSupply", -1)
        self.J6B2_relay = self.relayConfiguration.get("J6B2_relay", -1)

    def usbConnection(self, state):
        self.__allow_usb_toggle_event = True
        if state and (self.__usb_connected is True):
            self.__logger.printLog("INFO", "Plug USB skipped as already plugged in")
            self.__output.appendOutput("nothing to do", None)
        elif state:
            self.__logger.printLog("INFO", "Plug USB")
            self.__relay.enable_line(self.UsbHostPcConnect)
            self.__output.appendOutput("plugged", None)
            self.__usb_connected = True
        elif not state and (self.__usb_connected is False):
            self.__logger.printLog("INFO", "Unplug USB skipped as already unplugged")
            self.__output.appendOutput("nothing to do", None)
        else:
            self.__logger.printLog("INFO", "Unplug USB")
            self.__relay.disable_line(self.UsbHostPcConnect)
            self.__output.appendOutput("unplugged", None)
            self.__usb_connected = False
        time.sleep(self.__delayBetweenCommands)
        self.__allow_usb_toggle_event = False

    def buttonPress(self, button, duration):
        self.__logger.printLog("INFO", "Press %s for %ss" % (button, duration))
        self.__relay.enable_line(self.relayConfiguration[button])
        time.sleep(duration)
        self.__relay.disable_line(self.relayConfiguration[button])
        time.sleep(self.__delayBetweenCommands)

    def powerButtonPress(self, duration):
        self.buttonPress("SwitchOnOff", duration)
        self.__output.appendOutput("{0}s press".format(duration), None)

    def upButtonPress(self, duration):
        self.buttonPress("VolumeUp", duration)
        self.__output.appendOutput("{0}s press".format(duration), None)

    def downButtonPress(self, duration):
        self.buttonPress("VolumeDown", duration)
        self.__output.appendOutput("{0}s press".format(duration), None)

    def enableLine(self, relay):
        if relay == self.UsbHostPcConnect and not self.__allow_usb_toggle_event:
            self.__logger.printLog("INFO", "not allowed to toggle USB event, please use usbConnection method exclusively")
            return
        self.__relay.enable_line(relay)

    def disableLine(self, relay):
        if relay == self.UsbHostPcConnect and not self.__allow_usb_toggle_event:
            self.__logger.printLog("INFO", "not allowed to toggle USB event, please use usbConnection method exclusively")
            return
        self.__relay.disable_line(relay)

    def powerSupply(self, state):
        if "PowerSupply" not in self.relayConfiguration:
            self.__logger.printLog("DEBUG", "powerSupply(): 'PowerSupply' entry not found in relayConfiguration")
        else:
            if state:
                self.__logger.printLog("INFO", "Plug power supply")
                self.__relay.enable_line(self.relayConfiguration["PowerSupply"])
                self.__output.appendOutput("connect", None)
            else:
                self.__logger.printLog("INFO", "Unplug power supply")
                self.__relay.disable_line(self.relayConfiguration["PowerSupply"])
                self.__output.appendOutput("disconnect", None)
            time.sleep(self.__delayBetweenCommands)

    def replugDediprog(self, duration=2):
        self.__logger.printLog("INFO", "Replug dediprog")
        self.__relay.disable_line(self.relayConfiguration["Dediprog"])
        time.sleep(duration)
        self.__relay.enable_line(self.relayConfiguration["Dediprog"])
        time.sleep(10)
        self.__output.appendOutput("", None)

    # Method that group the process list
    def __process(self, processList):
        for proc in processList:
           eval(proc)

    def runThread(self, process_list):
        """ Run processes list in background
        """
        try:
            threading.Thread(target=self.__process, args=(process_list,)).start()
        except Exception as e:
            log = "run_thread(%s): failed to run process list (%s)" \
                  % (process_list, e)
            self.__logger.printLog("WARNING", log)
            return False

        return True

    # internal class to manage relay card
    class __internalRelayCard():
        def __init__(self):
            self.__logger = LoggerModule.LoggerModule()
            self.__host = HostModule.HostModule()
            self.__config = None
            self.__port = None
            self.__defaultState = None
            self.__wiringTable = None
            self.__relays = None
            self.__delay = 1

        def setConfig(self, conf):
            self.__config = conf
            self.__port = self.__config.get("ComPort")
            self.__defaultState = self.__config.get("DefaultStates")
            self.__wiringTable = self.__config.get("WiringTable")
            self.__relays = self.__config.get("relays")
            if self.__port:
                self.__host.commandExec("stty -F %s 19200 cs8 cstopb -parenb" % self.__port)
            self.printConfig()

        def printConfig(self):
            self.__logger.printLog("INFO", "RelayCardModule init: relay card configuration {0}".format(str(self.__config)))

        def setDefaultState(self):
            defaultReverted = self.__defaultState[::-1]
            wireReverted = self.__wiringTable[::-1]
            cpt = 0
            for _ in defaultReverted:
                cpt += 1
                cmd = "open"
                if (not bool(int(wireReverted[cpt - 1])) and not bool(int(defaultReverted[cpt - 1]))) or \
                    (bool(int(wireReverted[cpt - 1])) and bool(int(defaultReverted[cpt - 1]))):
                    cmd = "close"
                self.__sendCommand(cpt, cmd)
            pass

        def enable_line(self, intId):
            if intId == -1:
                self.__logger.printLog("WARNING", "cannot use relay card, bad configuration")
                return False
            wireReverted = self.__wiringTable[::-1]
            self.__logger.printLog("INFO", "Enable Relay %s" % str(intId))
            if bool(int(wireReverted[intId - 1])):
                self.__sendCommand(intId, "close")
            else:
                self.__sendCommand(intId, "open")

            #RELAY State should be checked
            return True

        def disable_line(self, intId):
            if intId == -1:
                self.__logger.printLog("WARNING", "cannot use relay card, bad configuration")
                return False
            wireReverted = self.__wiringTable[::-1]
            self.__logger.printLog("INFO", "Disable Relay %s" % str(intId))
            if bool(int(wireReverted[intId - 1])):
                self.__sendCommand(intId, "open")
            else:
                self.__sendCommand(intId, "close")

            #RELAY State should be checked
            return True

        def __sendCommand(self, id, strCmd):
            value = 68 + id
            if strCmd is not "open":
                value += 10
            #print str(int(value)) + " " + strCmd
            cmd = "echo " + chr(value).lower() + " > " + self.__port
            #print cmd
            os.system(cmd)