#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=missing-docstring, invalid-name, unused-argument
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

@organization: INTEL SQE
@summary: Save on HOST, DUT logs on request: tombstones, dmesg, anr, logcat
@since: 18/09/15
@author: mceausu
"""

import os
import subprocess

from Device.Module.DeviceModuleBase import DeviceModuleBase
from acs_test_scripts.Device.Module.Common.Debug.IDebugModule import IDebugModule
from UtilitiesFWK.Utilities import Verdict
from Device.DeviceManager import DeviceManager
import UtilitiesFWK.Utilities as Util


class ExtendedDebugModule(IDebugModule, DeviceModuleBase):
    """
    Implement a module to handle DUT on demand log extract
    """
    DMESG_FILE_NAME = "dmesg"
    ANR_FILE_NAME = "anr"
    TOMBSTONE_FILE_NAME = "tombstone"
    LOGCAT_FILE_NAME = "logcat"
    ADB_ANR_TRACES = "/data/anr/traces.txt"
    ADB_TOMBSTONES = "/data/tombstones/tombstone_00"

    def __init__(self):
        super(ExtendedDebugModule, self).__init__()
        self._retrieve_dmesg = None
        self._retrieve_anr_traces = None
        self._retrieve_tombstone = None
        self._retrieve_logcat = None
        self._only_for_failed_tests = None
        self._retrieve_data = None
        self._device_id = Util.AcsConstants.NOT_AVAILABLE
        self._verdict = Util.Verdict()
        self.result_folder_path = None
        self.result_file_name = None
        self.anrs_number = None
        self.tombs_number = None
        self.tombs_list = None
        self.tc_verdict = None

    def _set_configuration(self):
        """
        Retrieve and set parameters from module configuration
        :return:
        """
        self._retrieve_dmesg = self.configuration.get_value("retrieveDmesg", "True", "str_to_bool")
        self._retrieve_anr_traces = self.configuration.get_value("retrieveAnrTraces","True","str_to_bool")
        self._retrieve_tombstone = self.configuration.get_value("retrieveTombstone", "True", "str_to_bool")
        self._retrieve_logcat = self.configuration.get_value("retrieveLogcat", "True", "str_to_bool")
        self._only_for_failed_tests = self.configuration.get_value("OnlyForFailedTests", "True", "str_to_bool")
        self._retrieve_data = self._retrieve_dmesg or self._retrieve_anr_traces or self._retrieve_tombstone
        if self._retrieve_anr_traces:
            self.anrs_number = self._get_anr_number(self.ADB_ANR_TRACES)
        if self._retrieve_tombstone:
            self.tombs_number = self._get_tombstone_number(self.ADB_TOMBSTONES)

    def init(self, device_serial_number):
        """
        Initialize crash module

        :type device_serial_number: str
        :param device_serial_number:  ssn of the device
        :type cache_folder: str
        :param cache_folder:  cache folder path

        :rtype: bool
        :return: initialization status
        """
        self._set_configuration()
        if device_serial_number in (None, "None", ""):
            self._retrieve_data = False
            message = "Invalid serial number value : '{0}'".format(device_serial_number)
            self.logger.error(message)
        else:
            self._device_id = device_serial_number
        if self._retrieve_logcat:
            return self.LOGCAT_FILE_NAME
        return None

    def _create_log_filename(self, log_file_name):
        """
        :param log_file_name: log file name
        """
        file_path = os.path.join(self.result_folder_path, log_file_name)
        return file_path

    def _extract_dmesg(self):
        """
        Extract dmesg from DUT
        """
        if self._retrieve_dmesg:
            dmesg_file_path = self._create_log_filename(self.DMESG_FILE_NAME)
            logfile = open(dmesg_file_path, "wr")
            _cmd = "adb -s {0} shell dmesg".format(self._device_id)
            process = subprocess.Popen(_cmd, shell=True, stdout=logfile)
            process.wait()
            logfile.close()

    def _get_anr_number(self, log_type):
        """
        :return: /data/anr/traces.txt file timestamp
        """
        result, msg = self.device.run_cmd("adb shell ls -l %s" % log_type, 5)
        if result == Util.Global.SUCCESS and 'No such file or directory' not in msg:
            msg_list = msg.split()
            anr_number = msg_list[4].strip() + "_" + msg_list[5].strip()
            self.logger.debug("ANR traces file creation time= %s" % anr_number)
            return anr_number
        return None

    def _extract_anr(self):
        """
        Extract anr from DUT
        file_number - number of anr traces files to be retrieved from DUT
        """
        if self._retrieve_anr_traces:
            anr_file_path = self._create_log_filename(self.ANR_FILE_NAME)
            try:
                self.device.pull(self.ADB_ANR_TRACES, anr_file_path, timeout=20)
            except Exception, err:
                self.logger.debug("Exception was generated while extracting ANR traces log file: %s" %err)
                pass

    def _get_tombstone_number(self, log_type):
        """
        :return: the number of data/tombstones files number
        """
        result, msg = self.device.run_cmd("adb shell ls -l %s"%os.path.dirname(log_type), 5)
        try:
            if result == Util.Global.SUCCESS and msg and msg.strip():
                self.tombs_list = msg.strip().split('\n')
                self.logger.debug("Tombstones logs from: %s files number= %s"
                                  %(os.path.dirname(log_type), len(self.tombs_list)))
                return len(self.tombs_list)
        except Exception, err:
            self.logger.debug("Exception was generated while extracting Tombstones Number: %s" %err)
            pass
        return None

    def _extract_tombstone(self):
        """
        Extract tombstone from DUT
        file_number - number of tombstones files to be retrieved from DUT
        """
        if self._retrieve_tombstone:
            tombstone_file_path = self._create_log_filename(self.TOMBSTONE_FILE_NAME)
            try:
                tombs = self.tombs_list[self.tombs_number-1].split()[6].strip()
                self.device.pull(os.path.join(os.path.dirname(self.ADB_TOMBSTONES), tombs),
                                 tombstone_file_path, timeout=20)
            except Exception, err:
                self.logger.debug("Exception was generated while extracting Tombstones log file: %s" %err)
                pass

    def cleanup(self):
        """
        Clean debug log files and folders if needed
        """
        tc_fail_log_file = None
        if self._retrieve_logcat:
            tc_fail_log_file = self._create_log_filename(self.LOGCAT_FILE_NAME)
        if self.tc_verdict in self._verdict.PASS and self._only_for_failed_tests:
            if tc_fail_log_file and os.path.isfile(tc_fail_log_file):
                os.remove(tc_fail_log_file)
        if os.path.isdir(self.result_folder_path) and not os.listdir(self.result_folder_path):
            os.removedirs(self.result_folder_path)

    def dump(self, verdict, dump_location):
        """
        Dump all DUT debug information
        :type verdict: str
        :param verdict: test status
        :type dump_location: str
        :param dump_location: directory where data will be stored
        """
        self.result_folder_path = dump_location
        anrs_number = None
        tombs_number = None
        self.tc_verdict = verdict
        fail_test = verdict in self._verdict.FAIL and self._only_for_failed_tests
        if self._retrieve_data and dump_location \
            and fail_test or not self._only_for_failed_tests:
            self._extract_dmesg()
            if self._retrieve_anr_traces:
                anrs_number = self._get_anr_number(self.ADB_ANR_TRACES)
            if self._retrieve_tombstone:
                tombs_number = self._get_tombstone_number(self.ADB_TOMBSTONES)
            if (anrs_number and not self.anrs_number) or \
                (anrs_number and self.anrs_number and anrs_number not in self.anrs_number):
                self.logger.debug("ANR traces file was updated. Extracting it on HOST")
                self.anrs_number = anrs_number
                self._extract_anr()
            if (tombs_number and not self.tombs_number) or \
                (tombs_number and self.tombs_number and tombs_number > self.tombs_number):
                self.logger.debug("New tombstone file was generated. Extracting it on HOST")
                self.tombs_number = tombs_number
                self._extract_tombstone()
        return 0
