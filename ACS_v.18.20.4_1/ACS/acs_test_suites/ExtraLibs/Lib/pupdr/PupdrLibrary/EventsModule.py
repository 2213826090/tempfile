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
@summary: Pupdr Library - DntEventsModule
@since: 18/05/2016
@author: costindx
"""

from datetime import datetime
import HostModule
import ConfigurationModule
import LoggerModule

class EventsModule(object):

    __instance = None
    __globalConf = None
    __host = None
    __logger = None

    def __new__(cls):
        """
        Constructor that always returns the same instance
        """
        if not cls.__instance:
            cls.__instance = object.__new__(cls)
        return cls.__instance

    def init(self, globalConf):
        self.__globalConf = globalConf
        self.__host = HostModule.HostModule()
        self.__logger = LoggerModule.LoggerModule()
        self.__configuration = ConfigurationModule.ConfigurationModule()

    class Event(object):
        def __init__(self, eventLevel, eventId, eventTime, eventType, eventRoot = None):
            self.level = eventLevel
            self.id = eventId
            self.time = eventTime
            self.etype = eventType
            self.root = eventRoot
            self.__logger = LoggerModule.LoggerModule()
            self.__host = HostModule.HostModule()

        def get_filelist(self, host):
            files = []
            exec_status, adbOutput = host.commandExecAdb("shell ls {}".format(self.root))
            if exec_status == 0:
                files = adbOutput.splitlines(False)
            return files

        def has_files(self, host, outputMod, *r_files):
            files = self.get_filelist(host)
            verdict = True
            for rf in r_files:
                if not any(f.find(rf) != -1 for f in files):
                    self.__logger.printLog("ERROR", "Not found {}".format(rf))
                    verdict = False
            return verdict

    def __get_from_history_content(self, content):
        eventList = []
        for hystLine in content.splitlines(False):
            if len(hystLine) > 0 and (not hystLine.startswith('#')):
                fields = filter(None,hystLine.split(" "))
                if len(fields) > 4:
                    ev = self.Event(fields[0], fields[1], datetime.strptime(fields[2], '%Y-%m-%d/%H:%M:%S'),fields[3],fields[4])
                    eventList.append(ev)
                elif len(fields) > 3:
                    ev = self.Event(fields[0], fields[1], datetime.strptime(fields[2], '%Y-%m-%d/%H:%M:%S'),fields[3])
                    eventList.append(ev)
        return eventList

    def get_from_dev_history(self):
        eventList = []
        exec_status, adbOutput = self.__host.commandExecAdb("shell cat {}/history_event".format(self.__configuration.logs.LOG_PATH))
        if exec_status == 0:
            eventList = self.__get_from_history_content(adbOutput)
        return eventList

    def remove_duplicates(self, in_list, from_list):
        for event_in in in_list:
            for event_from in from_list:
                if event_in.id == event_from.id:
                    from_list.remove(event_from)

    def get_last_event(self, hist, event_lvl):
        for event in reversed(hist):
            if event.level == event_lvl:
                return event
        return None

    if __name__ == '__main__':
        pass