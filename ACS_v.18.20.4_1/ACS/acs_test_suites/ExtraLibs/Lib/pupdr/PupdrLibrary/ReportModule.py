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
@summary: Pupdr Library - ReportModule
@since: 11/17/2014
@author: travenex
"""

import os
import LoggerModule
import HostModule
import MiscModule

class ReportModule(object):

    __instance = None
    __globalConf = None
    __logger = None
    __host = None
    __misc = None
    __data_push_handler = None
    short_retention = None
    long_retention = None

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
        self.__misc = MiscModule.MiscModule()
        self.__data_push_handler = self.__globalConf.get("EXTERNAL_DATA_PUSH")
        self.short_retention = "SHORT"
        self.long_retention = "LONG"
        # checking class properly implemented
        if not self.__data_push_handler:
            self.__logger.printLog("WARNING", "ReportModule init: no report push mechanism enabled")
        else:
            try:
                self.__data_push_handler.resultPush
            except AttributeError as e:
                self.__logger.printLog("WARNING", "ReportModule init: missing 'resultPush' in class methods for "
                                                  "external report push (error={})".format(e))
                self.__data_push_handler = None
            else:
                try:
                    self.__data_push_handler.attachmentPush
                except AttributeError as e:
                    self.__logger.printLog("WARNING", "ReportModule init: missing 'attachmentPush' in class methods for "
                                                      "external report push (error={})".format(e))
                    self.__data_push_handler = None

    def TCRDataPush(self, custom_dict):
        log = "TCRDataPush(): "
        self.__logger.printLog("INFO", log + "start")
        if not self.__data_push_handler:
            self.__logger.printLog("WARNING", log + "no push mechanism, skipping")
            return
        if not isinstance(custom_dict, dict):
            self.__logger.printLog("WARNING", log + "invalid input, no data pushed")
        else:
            try:
                self.__data_push_handler.resultPush({"bootOta": custom_dict})
                self.__logger.printLog("INFO", log + "push successful")
            except Exception as e:
                self.__logger.printLog("WARNING", log + "failure with TCR push (error={0})".format(e))

    def TCRMetricsPush(self, custom_list):
        log = "TCRMetricsPush(): "
        self.__logger.printLog("INFO", log + "start")
        if not self.__data_push_handler:
            self.__logger.printLog("WARNING", log + "no push mechanism, skipping")
            return
        if not isinstance(custom_list, list):
            self.__logger.printLog("WARNING", log + "invalid input, no data pushed")
        else:
            try:
                self.__data_push_handler.resultPush({"metrics": custom_list})
                self.__logger.printLog("INFO", log + "push successful")
            except Exception as e:
                self.__logger.printLog("WARNING", log + "failure with TCR push (error={0})".format(e))

    def TCRAttachmentPush(self, local_file, file_name="", retention="LONG"):
        log = "TCRAttachmentPush(): "
        self.__logger.printLog("INFO", log + "starting for {}".format(local_file))
        if not self.__data_push_handler:
            self.__logger.printLog("WARNING", log + "no push mechanism, skipping")
            return
        if not os.path.isfile(local_file):
            self.__logger.printLog("WARNING", log + "invalid file input, no pushed attachment")
        else:
            if not file_name or not isinstance(file_name, str):
                file_name = os.path.basename(local_file)
            if retention not in [self.long_retention, self.short_retention]:
                self.__logger.printLog("DEBUG", log + "invalid input, setting short retention")
                retention = self.short_retention
            try:
                self.__data_push_handler.attachmentPush(local_file, file_name, retention)
                self.__logger.printLog("INFO", log + "push successful")
            except Exception as e:
                self.__logger.printLog("WARNING", log + "failure with TCR push (error={0})".format(e))