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
@summary: Module that implement Crash Info library for device crash management
@since: 7/1/14
@author: cbonnard
"""
import os

from acs_test_scripts.Device.Module.Common.Crash.CrashEvent import CrashEvent
from acs_test_scripts.Device.Module.Common.Crash.ICrashModule import ICrashModule
from Device.Module.DeviceModuleBase import DeviceModuleBase
from Device.Model.AndroidDevice.CrashInfo.CrashInfo import CrashInfo, CrashInfoException
from UtilitiesFWK.Utilities import global_to_bool
from UtilitiesFWK.Utilities import AcsConstants
from UtilitiesFWK.Utilities import Verdict

# pylint: disable=E1002


class CrashInfoModule(ICrashModule, DeviceModuleBase):

    """
        Implements Crash feature based on CrashInfo implementation
    """
    _proceeded_events = dict()
    _first_event = dict()
    _ignore_events = dict()

    def __init__(self):
        super(CrashInfoModule, self).__init__()
        self._crash_info = None
        self._cache_folder = None
        self._disable_clota = None
        self._crash_lib_silent_mode = None
        self._server_address = None
        self._server_port = None
        self._retrieve_data = None
        self._device_id = AcsConstants.NOT_AVAILABLE

    def _set_configuration(self):
        """
        Retrieve and set parameters from module configuration

        :return:
        """
        self._disable_clota = self.device.config.get_value("disableCLOTA", "True", "str_to_bool")
        retrieve_device_log_on_critical = self.device.config.get_value("retrieveDeviceLogOnCriticalFailure", "False",
                                                                       "str_to_bool")
        retrieve_device_crash_log = self.device.config.get_value("retrieveDeviceCrashLog", "True", "str_to_bool")
        self._crash_lib_silent_mode = self.device.config.get_value("crashServerSilentMode", "True", "str_to_bool")
        self._server_address = self.device.config.get_value("crashServer", "crashtool.iind.intel.com")
        self._server_port = self.device.config.get_value("crashServerPort", "4002")
        self._retrieve_data = retrieve_device_crash_log or retrieve_device_log_on_critical
        self._log_detail_level = self.device.config.get_value("crashLogDetailLevel", 0, int)
        self._nb_max_event_to_proceed = self.device.config.get_value("nbCrashLogProcessingLimit", 20, int)

    def _list(self, detail_level=0):
        """
        List crash events all available on device

        :param detail_level: define level of info for events

        :rtype: list
        :return: list of crash events
        """
        list = []
        list = self._crash_info.get_event(detail_level)
        # convert all event from dict to CrashEvent object
        list = [CrashEvent(event.get("eventID", AcsConstants.NOT_AVAILABLE),
                           event.get("eventType", AcsConstants.NOT_AVAILABLE),
                           event.get("date", AcsConstants.NOT_AVAILABLE),
                           event.get("crashdir", AcsConstants.NOT_AVAILABLE),
                           event.get("crashtoolUrl", AcsConstants.NOT_AVAILABLE),
                           event)
                for event in list]
        return list

    def _filter_crash_method(self, events):
        results = list()
        ref_event = CrashInfoModule._first_event[self._device_id]
        events_to_handle = [event for event in events if not ref_event or
                            int(event._id) >= int(ref_event._id)
                            and event.eventName == "CRASH"]
        for event in events_to_handle:
            if event.eventId not in self._proceeded_events.get(self._device_id, []):
                results.append(event)
                self._proceeded_events[self._device_id].append(event.eventId)
        return results

    def init(self, device_serial_number, cache_folder):
        """
        Initialize crash module from crash info library

        :type device_serial_number: str
        :param device_serial_number:  ssn of the device
        :type cache_folder: str
        :param cache_folder:  cache folder path

        :rtype: bool, str
        :return: initialization status and error message
        """
        status = False
        self._set_configuration()
        self._cache_folder = cache_folder

        if device_serial_number in (None, "None", ""):
            self._retrieve_data = False
            message = "Invalid serial number value : '{0}'".format(device_serial_number)
            self.logger.error(message)
        else:
            self._device_id = device_serial_number
            CrashInfoModule._first_event[self._device_id] = None
            CrashInfoModule._proceeded_events[self._device_id] = list()
            CrashInfoModule._ignore_events[self._device_id] = list()
            try:
                self._crash_info = CrashInfo(self._crash_lib_silent_mode, device_serial_number, prog_name="CRASHINFO",
                                             logger=self.logger)
            except CrashInfoException as ex:
                self.logger.error(ex)
                self._crash_info = None

            if self._crash_info:
                status = True
                message = "Initialization of CrashInfo succeeded"
                self.logger.debug(message)
            else:
                self.logger.error("Initialization of CrashInfo failed")
        return status

    def disable_clota(self):
        """
        Disable Crash Log Over the Air on device

        :rtype: bool
        :return: disabling status
        """
        status = False
        if self._disable_clota:
            if self.device.is_available():
                result, message = self.device.run_cmd("adb shell setprop persist.crashreport.disabled 1", 1)
                status = global_to_bool(result)
                if status:
                    self.logger.debug("CLOTA disabled")
                else:
                    self.logger.error("CLOTA failed to be disabled: {0}" .format(message))

            else:
                message = "CLOTA failed to be disabled: no adb connection established with the device"
                self.logger.debug(message)
        return status

    def enable_clota(self):
        """
        Enable Crash Log Over the Air on device

        :rtype: bool
        :return: disabling status
        """
        status = False
        if self._disable_clota:
            if self.device.is_available():
                result, message = self.device.run_cmd("adb shell setprop persist.crashreport.disabled 0", 1)
                status = global_to_bool(result)
                if status:
                    self.logger.debug("CLOTA enabled")
                else:
                    self.logger.debug("CLOTA failed to be enable: {0}".format(message))
            else:
                message = "CLOTA failed to be enable, no adb connection to the device"
                self.logger.debug(message)
        return status

    def list(self):
        """
        List crash events available on device

        :rtype: list
        :return: list of crash events
        """
        all_events = self._list(detail_level=self._log_detail_level)
        # store the first known event
        if all_events and not CrashInfoModule._first_event[self._device_id]:
            CrashInfoModule._first_event[self._device_id] = all_events[0]
        results = self._filter_crash_method(events=all_events)
        return sorted(results)

    def fetch(self, event_id=None, verdict=None):
        """
        Fetch event logs from the device, if any and retrieve them on local host
        :param verdict: Verdict of the test case

        :type event_id: str
        :param event_id: optional identifier of event

        :rtype: bool, str
        :return: fetch status and error message
        """
        status = True
        if self._retrieve_data:
            msg = "Fetching crash events to: {0}, uploading to: {1}".format(self._cache_folder, self._server_address)
            self.logger.info(msg)
            try:
                if event_id:
                    status, _, _ = self._fetch_event(event_id)
                else:
                    # get the list of recent events and their upload status,
                    # i.e., whether they've been already sent to the logserver or not
                    # Dump event logs from the device
                    crash_processed = 0
                    for event in self._list(detail_level=1):
                        try:
                            if (event.get("_id") not in self._ignore_events[self._device_id] and
                                    (event.get("uploaded") == "0" or (
                                            event.get("logsuploaded") == "0" and event.get("crashdir")))):
                                crash_processed += 1
                                # send logs & update the upload flag
                                if event.crashdir:
                                    msg = "Processing crash event {0}:{1}".format(event.eventId, event.type)
                                    self.logger.info(msg)
                                status, log_path, error_msg = self._fetch_event(event._id)
                                if status:
                                    # upload event succeed
                                    if event.crashdir and log_path:
                                        # check if pull succeed
                                        if os.path.isdir(log_path):
                                            msg = "Crash event {0} saved to {1}".format(event.eventId, log_path)
                                            self.logger.info(msg)
                                        else:
                                            msg = "Crash event {0} failed to create logdir {1}".format(event.eventId,
                                                                                                       log_path)
                                            self.logger.error(msg)
                                else:
                                    msg = "Crash event {0} failed to fetch&upload : {1}".format(
                                        event.eventId, error_msg)
                                    self.logger.error(msg)
                                    CrashInfoModule._ignore_events[self._device_id].append(event._id)
                        except AttributeError as ex:
                            self.logger.error("Processing crash event: {0} key not found".format(ex))
                            if event.get("_id"):
                                CrashInfoModule._ignore_events[self._device_id].append(event._id)
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
                self.logger.debug("Processing crash event: {0}".format(ex))
        else:
            status = True
            self.logger.debug("Fetch&Upload disabled")
        return status

    def _fetch_event(self, event_id):
        """
        Fetch crashes from device to host

        :type event_id: str
        :param event_id: identifier of event

        :rtype: bool, str
        :return: fetch status and path to the crash event
        """
        status, log_path = False, ""

        status, log_path, error_msg = self._crash_info.pull_event(event_id,
                                                                  self._cache_folder,
                                                                  self._server_address,
                                                                  self._server_port)

        return status, log_path, error_msg

    def upload(self, event_id=None):
        """
        Upload crashes from host to server

        :type event_id: str
        :param event_id: identifier of event

        :rtype: bool, str
        :return: upload status and error msg
        """
        # Upload is done whole fetching by CrashInfo
        return True

    def get_device_info(self):
        """
        Return device info dictionary

        :rtype: tuple status & dict
        :return: device information with its status
        """
        return Verdict.PASS, {}
