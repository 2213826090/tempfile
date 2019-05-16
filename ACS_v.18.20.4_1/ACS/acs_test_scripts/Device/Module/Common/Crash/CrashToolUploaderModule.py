#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=missing-docstring, invalid-name, unused-argument
"""
@copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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

@organization: INTEL MCG AMPS
@summary: Module that implement CrashToolUploader library for device crash management
@since: 7/1/14
@author: cbonnard
"""

import json
import yaml
import re
import os

from CrashEvent import CrashEvent
from Core.PathManager import Paths, absjoin
from acs_test_scripts.Device.Module.Common.Crash.ICrashModule import ICrashModule
from Device.Module.DeviceModuleBase import DeviceModuleBase
from acs_test_scripts.Equipment.EquipmentManager import EquipmentManager
from UtilitiesFWK.Utilities import global_to_bool
from UtilitiesFWK.Utilities import internal_shell_exec
from UtilitiesFWK.Utilities import Global
from UtilitiesFWK.Utilities import Verdict
from UtilitiesFWK.CommandLine import CommandLine


class CrashToolUploaderModule(ICrashModule, DeviceModuleBase):
    """
        Implements Crash feature based on CrashToolUploader implementation
    """
    JSON_PATTERN = "\[\n  {.*\}\n\]"
    CRASHTOOLUPLOADER_URI = "https://mcg-depot.intel.com/artifactory/psi-tools/crashtooluploader/"
    CRASHTOOLUPLOADER_PATH = absjoin(Paths.TEST_SCRIPTS, "Lib/CrashToolUploader")
    CRASHTOOLUPLOADER_NAME = "CrashToolUploader.jar"
    CRASHTOOLUPLOADER_DEFAULT_PATH = absjoin(CRASHTOOLUPLOADER_PATH, CRASHTOOLUPLOADER_NAME)
    CRASHTOOLUPLOADER_CMD = "java -jar {0} {1} -appname ACS"
    JSON_EVENTS_FILE = "events.json"
    SSN_OPTION = " -ssn"
    EVENTS_OPTION = " -getEvents"
    UPLOAD_OPTION = " -crashServerPushUrl"
    FETCH_OPTION = " -eventLogDir"
    UPLOAD_EVENT_OPTION = " -events"
    FETCH_ONLY_OPTION = " -fetchOnly"
    GET_DEVICE_INFO = " -getDeviceInfo"

    _proceeded_events = dict()
    _first_event = dict()
    _ignore_events = dict()

    def __init__(self):
        super(CrashToolUploaderModule, self).__init__()
        self._device_serial_number = None
        self._cache_folder = None
        self._disable_clota = None
        self._server_address = None
        self._server_port = None
        self._retrieve_data = None
        self._json_pattern_expr = None
        self._debug_mode = None

        self.__adb_option = ""
        adb_full_path = CommandLine.which("adb")
        if adb_full_path:
            self.__adb_option = " -adbpath \"{0}\"".format(adb_full_path)

        self._crashtooluploader_cmd = self.CRASHTOOLUPLOADER_CMD.format(self.CRASHTOOLUPLOADER_DEFAULT_PATH,
                                                                        self.__adb_option)

    def _download_crashtooluploader(self):
        """
        Download CrashToolUploader from official url and return the destination path.

        :return: path to CrashToolUploader. In case of errors returns an empty string
        """
        crashtooluploader_path = ""
        try:
            art_mgr = EquipmentManager().get_artifact_manager("ARTIFACT_MANAGER")
            crashtooluploader_path = art_mgr.get_artifact(self.CRASHTOOLUPLOADER_NAME, self.CRASHTOOLUPLOADER_URI)
        except Exception as artmgr_exception:
            self.logger.debug("Unable to download {0} ! ({1})".format(self.CRASHTOOLUPLOADER_NAME,
                                                                      str(artmgr_exception)))
            self.logger.debug("Use crashtooluploader from {0} !".format(self.CRASHTOOLUPLOADER_DEFAULT_PATH))
            crashtooluploader_path = self.CRASHTOOLUPLOADER_DEFAULT_PATH
        return crashtooluploader_path

    def _set_configuration(self):
        """
        Retrieve and set parameters from module configuration

        :return:
        """

        self._disable_clota = self.configuration.get_value("disableCLOTA", "True", "str_to_bool")
        retrieve_device_log_on_critical = self.configuration.get_value("retrieveDeviceLogOnCriticalFailure", "False",
                                                                       "str_to_bool")
        self._server_upload = self.configuration.get_value("uploadCrashToServer", "False", "str_to_bool") or \
                              retrieve_device_log_on_critical
        self._host_fetch = self.configuration.get_value("fetchCrashToHost", "False", "str_to_bool")
        self._server_address = self.configuration.get_value("serverAddress", "crashtool.iind.intel.com")
        self._server_port = self.configuration.get_value("serverPort", "4002")
        self._retrieve_data = self._server_upload or self._host_fetch
        self._debug_mode = self.configuration.get_value("debugMode", "False", "str_to_bool")
        self._nb_max_event_to_proceed = self.configuration.get_value("nbCrashLogProcessingLimit", 20, int)

        crashtooluploader_binary_path = self._download_crashtooluploader()
        if crashtooluploader_binary_path and os.path.isfile(crashtooluploader_binary_path):
            self._crashtooluploader_cmd = self.CRASHTOOLUPLOADER_CMD.format(crashtooluploader_binary_path,
                                                                            self.__adb_option)
        else:
            self._retrieve_data = False
            message = "Invalid CrashToolUploader file: '{0}'".format(crashtooluploader_binary_path)
            self.logger.error(message)
        if self._debug_mode:
            self._crashtooluploader_cmd = "{0} {1}".format(self.CRASHTOOLUPLOADER_CMD, "-d")

    def _run_crashtooluploader_cmd(self, cmd):
        """
        Run sync cmd line of CrashToolUploader

        :return:
        """
        if self._device_serial_number != "not_set":
            cmd = "{0}{1} {2}".format(cmd,
                                      self.SSN_OPTION,
                                      self._device_serial_number)
        try:
            # Timeout is high here because sometimes, we need to upload a modem panic event + BP log (~600MB)
            result, output = internal_shell_exec(cmd, 900, log_stdout=False, silent_mode=(not self._debug_mode))
        except Exception as e:
            message = "Error while running CrashToolUploader command: {0}.\n{1}".format(cmd, e)
            result = Global.FAILURE
            output = message
            self.logger.warning(message)

        return result, output

    def _convert_json_dict(self, json_input):
        """
        Convert all unicode elements of a dictionary loaded from JSON.
        This aims to print to the stdout a 'json' dictionary without 'u' char
        before each string.
        No conversion is performed if input data is not a dictionary.
        @rtype: dictionary
        @return: the input dictionary converted.
        """
        object_dict = {}
        if isinstance(json_input, dict):
            for key in json_input:
                value = str(json_input[key])
                key = str(key)
                object_dict[key] = value

        return object_dict

    def init(self, device_serial_number, cache_folder):
        """
        Initialize crash module from crash info library

        :type device_serial_number: str
        :param device_serial_number:  ssn of the device
        :type cache_folder: str
        :param cache_folder:  cache folder path

        :rtype: bool
        :return: initialization status
        """
        self._set_configuration()
        self._cache_folder = cache_folder
        self._json_pattern_expr = re.compile(self.JSON_PATTERN)
        if device_serial_number in (None, "None", ""):
            device_serial_number = "not_set"

        self._device_serial_number = device_serial_number
        CrashToolUploaderModule._first_event[self._device_serial_number] = None
        CrashToolUploaderModule._proceeded_events[self._device_serial_number] = list()
        CrashToolUploaderModule._ignore_events[self._device_serial_number] = list()
        return True

    def disable_clota(self):
        """
        Disable Crash Log Over the Air on device

        :rtype: bool, str
        :return: disabling status
        """
        status = False
        if self._disable_clota:
            if self.device.is_available():
                result, message = self.device.run_cmd(cmd="adb shell setprop persist.crashreport.disabled 1",
                                                      timeout=1, force_execution=True)
                status = global_to_bool(result)
            else:
                message = "Disabling CLOTA not possible, no adb connection to the device"
                self.logger.error(message)
        else:
            message = "clota feature disabled by user"
            self.logger.warning(message)
        return status

    def enable_clota(self):
        """
        Enable Crash Log Over the Air on device

        :rtype: bool, str
        :return: disabling status
        """
        status = False
        if self._disable_clota:
            if self.device.is_available():
                result, message = self.device.run_cmd("adb shell setprop persist.crashreport.disabled 0", 1)
                status = global_to_bool(result)
            else:
                message = "Enabling CLOTA not possible, no adb connection to the device"
                self.logger.error(message)
        else:
            message = "clota feature disabled by user"
            self.logger.warning(message)
        return status

    def _filter_events(self, events):
        """
            Filter events from events of device to obtain only relevant events
        :rtype: list
        :return: list of events to proceed
        """
        results = list()
        try:
            ref_event = CrashToolUploaderModule._first_event[self._device_serial_number]
            events_to_handle = [event for event in events if not ref_event or
                                isinstance(event._id, basestring) and isinstance(ref_event._id, basestring) and
                                event._id and ref_event._id and
                                int(event._id) >= int(ref_event._id)
                                and event.eventName == "CRASH"]
            for event in events_to_handle:
                if event.eventID not in CrashToolUploaderModule._proceeded_events.get(self._device_serial_number, []):
                    results.append(event)
                    CrashToolUploaderModule._proceeded_events[self._device_serial_number].append(event.eventID)
        except (AttributeError, ValueError) as e:
            self.logger.error("Missing event data: cannot process this event ({0})".format(e))
        return results

    def list(self):
        """
        List crash events to be processed

        :rtype: list
        :return: list of crash events
        """
        all_events = self._list()
        # store the first known event of the campaign to ignore previous ones
        if all_events and not CrashToolUploaderModule._first_event[self._device_serial_number]:
            CrashToolUploaderModule._first_event[self._device_serial_number] = all_events[0]
        results = self._filter_events(all_events)
        return sorted(results)

    def _list(self):
        """
        List crash events available on device

        :rtype: list
        :return: list of crash events
        """
        message = ""
        events_list = []
        if self.device.is_available():
            json_file = absjoin(self._cache_folder, self.JSON_EVENTS_FILE)
            list_cmd = "{0}{1} {2}".format(self._crashtooluploader_cmd,
                                           self.EVENTS_OPTION,
                                           json_file)
            result, output = self._run_crashtooluploader_cmd(list_cmd)
            status = global_to_bool(result)
            if status:
                try:
                    with open(json_file) as data_file:
                        events = json.loads(data_file.read())
                except IOError:
                    message = "Failed to list events: no json data file produced by CrashToolUploader"
                    self.logger.error(message)
                else:
                    for event in events:
                        if self._device_serial_number:
                            event["ssn"] = self._device_serial_number
                        events_list.append(CrashEvent(event.get("eventID", ""),
                                                      event.get("eventType", ""),
                                                      event.get("date", ""),
                                                      event.get("crashdir", ""),
                                                      event.get("crashtoolUrl", ""),
                                                      event))
            else:
                self.logger.error(output)
        else:
            message = "Failed to list events: no adb connection established with the device"
            self.logger.error(message)

        return events_list

    def fetch(self, event_id=None, verdict=None):
        """
        Fetch crashes from device to host

        :param verdict: Verdict of the test case

        :param event_id:

        :rtype: bool
        :return: fetch status
        """

        status, message = False, "Unexpected error"
        if self._retrieve_data:
            msg = None
            if self._server_upload and self._host_fetch:
                msg = "Fetching crash events to: {0}, uploading to: {1}".format(
                    self._cache_folder, self._server_address)
            elif self._server_upload:
                msg = "Uploading crash events to: {0}".format(self._server_address)
            elif self._host_fetch:
                msg = "Fetching crash events to: {0}".format(self._cache_folder)
            if msg:
                self.logger.info(msg)
            try:
                event_list = []
                if event_id:
                    event_list.append(event_id)
                else:
                    event_list = self._list()
                crash_processed = 0
                for event in event_list:
                    try:
                        if event.get("eventID") not in CrashToolUploaderModule._ignore_events[self._device_serial_number]:
                            crash_processed += 1
                            if event.crashdir:
                                msg = "Processing crash event {0}:{1}".format(event.eventID, event.eventType)
                                self.logger.info(msg)
                            status, error_msg = self._retrieve_event(event.eventID)
                            if status:
                                if event.crashdir and self._cache_folder and self._host_fetch:
                                    # check if pull succeed
                                    if os.path.isdir(self._cache_folder):
                                        msg = "Crash event {0} saved to {1}".format(
                                            event.eventID, self._cache_folder)
                                        self.logger.info(msg)
                                    else:
                                        msg = "Crash event {0} logdir creation failed{1}".format(event.eventID,
                                                                                                 self._cache_folder)
                                        self.logger.error(msg)
                            else:
                                msg = "Crash event {0} retrieval failed : {1}".format(
                                    event.eventID, error_msg)
                                self.logger.error(msg)
                                CrashToolUploaderModule._ignore_events[
                                    self._device_serial_number].append(event.eventID)

                    except AttributeError as ex:
                        self.logger.error("Crash event {0} key not found".format(ex))
                        if event.get("_id"):
                            CrashToolUploaderModule._ignore_events[self._device_serial_number].append(event.eventID)

                    finally:
                        if crash_processed > self._nb_max_event_to_proceed > 0:
                            # Stop processing event
                            self.logger.info(
                                "Max crash event processing reached ({0}), stop processing.".format(
                                    self._nb_max_event_to_proceed))
                            break
            except (KeyboardInterrupt, SystemExit):
                raise
            except Exception as ex:  # pylint: disable=W0703
                self.logger.debug("Crash event: {0}".format(ex))
        else:
            status = True
            self.logger.debug("Fetch / Upload disabled")
        return status

    def _retrieve_event(self, event_id):
        """
        Retrieve crashes from device to host or server

        :type event_id: str
        :param event_id: identifier of event

        :rtype: bool, str
        :return: fetch status and path to the crash event
        """
        status, message = False, "undefined error"
        cmd = None
        if self.device.is_available():
            if self._server_upload and self._host_fetch:
                # fetch&upload
                cmd = "{0}{1} {2}:{3}{4} \"{5}\"{6} {7}".format(self._crashtooluploader_cmd,
                                                                self.UPLOAD_OPTION,
                                                                self._server_address,
                                                                self._server_port,
                                                                self.FETCH_OPTION,
                                                                self._cache_folder,
                                                                self.UPLOAD_EVENT_OPTION,
                                                                event_id)
            elif self._server_upload:
                # upload only
                cmd = "{0}{1} {2}:{3} \"{4}\"{5} {6}".format(self._crashtooluploader_cmd,
                                                             self.UPLOAD_OPTION,
                                                             self._server_address,
                                                             self._server_port,
                                                             self._cache_folder,
                                                             self.UPLOAD_EVENT_OPTION,
                                                             event_id)
            elif self._host_fetch:
                # fetch only
                cmd = "{0} {1} \"{2}\"{3} {4}{5}".format(self._crashtooluploader_cmd,
                                                         self.FETCH_OPTION,
                                                         self._cache_folder,
                                                         self.UPLOAD_EVENT_OPTION,
                                                         event_id,
                                                         self.FETCH_ONLY_OPTION)
            if cmd:
                result, message = self._run_crashtooluploader_cmd(cmd)
                status = global_to_bool(result)
        else:
            message = "Retreiving crashes not possible, no adb connection to the device"

        return status, message

    def upload(self):
        """
        Upload crashes from device to server

        :rtype: bool
        :return: upload status
        """
        status, message = False, "Unexpected error"

        return status, message

    def get_device_info(self):
        """
        Return device info dictionary

        :rtype: tuple status & dict
        :return: device information with its status
        """

        status = Verdict.PASS
        device_info_dict = {}

        self.logger.info("Download crash info with CrashToolUploader")
        list_cmd = "{0}{1}".format(self._crashtooluploader_cmd, self.GET_DEVICE_INFO)

        result, output = self._run_crashtooluploader_cmd(list_cmd)
        if result != Global.SUCCESS:
            self.logger.error("Crash info not retrieved - command={0}, result={1}, output={2}".format(list_cmd,
                                                                                                      result,
                                                                                                      output))
            status = Verdict.FAIL
        else:
            if output:
                # Deserialize json output
                try:
                    # In this case the device is returned but with some errors
                    # Try to extract the error and return device info
                    search_pattern = r'(?P<cmd_error>(.*\n)*)(?P<device_info>\{(.*\n)+\})'
                    search_str = re.match(search_pattern, output)
                    if search_str:
                        device_info_dict = yaml.load(search_str.group('device_info'))
                        error_msg = search_str.group('cmd_error')
                        if error_msg:
                            self.logger.error("CrashToolUploader: Deserialize json output failure:\n"
                                              "error_msg=\n{0}".format(error_msg))
                            device_info_dict.get("device", {}).update(
                                {"crashToolUploaderError": error_msg.splitlines()})
                            status = Verdict.INTERRUPTED
                        else:
                            self.logger.info("CrashToolUploader: Deserialize json output results success:\n"
                                         "device_info=\n{0}".format(device_info_dict))
                    else:
                        self.logger.warning("CrashToolUploader: Cannot find device info in output:\n{0}".format(output))
                        status = Verdict.INTERRUPTED

                except Exception as e:
                    self.logger.error("Cannot parse CrashToolUploader (exception={0}) output:\n{1}".format(e, output))
                    status = Verdict.BLOCKED
            else:
                self.logger.warning("Cannot parse CrashToolUploader output (it is empty)")
                status = Verdict.INTERRUPTED

        return status, device_info_dict
