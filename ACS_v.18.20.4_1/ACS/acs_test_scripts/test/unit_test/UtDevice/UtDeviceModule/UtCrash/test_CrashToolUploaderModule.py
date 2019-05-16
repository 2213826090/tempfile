#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=missing-docstring, invalid-name, unused-argument
from mock import MagicMock
from mock import patch
from mock import PropertyMock

from unit_test_fwk.UTestBase import UTestBase
from acs_test_scripts.Device.Module.Common.Crash.CrashToolUploaderModule import CrashToolUploaderModule
from UtilitiesFWK.AttributeDict import AttributeDict
from UtilitiesFWK.Utilities import Global


class UTCrashModule(UTestBase):

    def __get_crash_list(self, crash_mod):
        crash_json = """
            [
              {
                "_id": "10",
                "eventID": "b8e3539d02eee1e00ff3",
                "eventName": "CRASH",
                "date": "2014-08-26/08:16:13",
                "eventType": "JAVACRASH",
                "crashdir": "",
                "crashtoolUrl": "http://crashtool.iind.intel.com/crashtoolStrutsWEB/event.do?action=display&eventId=b8e3539d02eee1e00ff3"
              },
              {
                "_id": "11",
                "eventID": "c727160ee73300433949",
                "eventName": "CRASH",
                "date": "2014-08-26/08:16:28",
                "eventType": "JAVACRASH",
                "crashdir": "",
                "crashtoolUrl": "http://crashtool.iind.intel.com/crashtoolStrutsWEB/event.do?action=display&eventId=c727160ee73300433949"
              },
              {
                "_id": "12",
                "eventID": "16aeba2eed7a76a85fa9",
                "eventName": "CRASH_NOT",
                "date": "2014-08-26/08:57:15",
                "eventType": "JAVACRASH",
                "crashdir": "",
                "crashtoolUrl": "http://crashtool.iind.intel.com/crashtoolStrutsWEB/event.do?action=display&eventId=16aeba2eed7a76a85fa9"
              }
            ]
        """

        # mock json reading
        my_mock = MagicMock()
        with patch('__builtin__.open', my_mock):
            manager = my_mock.return_value.__enter__.return_value
            manager.read.return_value = crash_json
            crash_listed = crash_mod.list()

        return crash_listed

    def __get_crash_module(self, device=MagicMock()):
        crash_mod = CrashToolUploaderModule()
        crash_mod.device = device
        crash_mod.logger = MagicMock()
        return crash_mod

    @patch.object(CrashToolUploaderModule, "_download_crashtooluploader")
    def test_enable_clota_when_disableCLOTA_deactivated(self, mock_download_crastooluploader):
        """
        'disableCLOTA' set to False in device config
        Check that enable_clota do nothing on the phone
        """
        mock_download_crastooluploader.return_value = ""

        device = MagicMock()
        config = PropertyMock(return_value=AttributeDict({"disableCLOTA": "False",
                                                          "retrieveDeviceLogOnCriticalFailure": "False",
                                                          "uploadCrashToServer": "True",
                                                          "crashServer": "crashtool.iind.intel.com",
                                                          "crashServerPort": "4002"
                                                          }))
        type(device).config = config
        device.run_cmd.return_value = 0, "cmd ok"
        crash_mod = self.__get_crash_module(device=device)
        crash_mod.init(device_serial_number="ACSPHONE1234567", cache_folder="")
        results = crash_mod.enable_clota()
        self.assertEqual(device.run_cmd.call_count, 0)
        self.assertEqual(results, False)

    @patch.object(CrashToolUploaderModule, "_download_crashtooluploader")
    def test_disable_clota_when_disableCLOTA_deactivated(self, mock_download_crastooluploader):
        """
        'disableCLOTA' set to False in device config
        Check that 'disable_clota' do nothing on the phone
        """
        mock_download_crastooluploader.return_value = ""

        device = MagicMock()
        config = PropertyMock(return_value=AttributeDict({"disableCLOTA": "False",
                                                          "retrieveDeviceLogOnCriticalFailure": "False",
                                                          "uploadCrashToServer": "True",
                                                          "crashServer": "crashtool.iind.intel.com",
                                                          "crashServerPort": "4002"
                                                          }))
        type(device).config = config
        device.run_cmd.return_value = 0, "cmd ok"
        crash_mod = self.__get_crash_module(device=device)
        crash_mod.init(device_serial_number="ACSPHONE1234567", cache_folder="")
        results = crash_mod.disable_clota()
        self.assertEqual(device.run_cmd.call_count, 0)
        self.assertEqual(results, False)

    @patch.object(CrashToolUploaderModule, "_download_crashtooluploader")
    def test_enable_clota_when_disableCLOTA_activated(self, mock_download_crastooluploader):
        """
        'disableCLOTA' set to True in device config
         Check that 'enable_clota' will run a cmd on the device
        """
        mock_download_crastooluploader.return_value = ""

        device = MagicMock()
        config = PropertyMock(return_value=AttributeDict({"disableCLOTA": "True",
                                                          "retrieveDeviceLogOnCriticalFailure": "False",
                                                          "uploadCrashToServer": "True",
                                                          "crashServer": "crashtool.iind.intel.com",
                                                          "crashServerPort": "4002"
                                                          }))
        type(device).config = config
        device.run_cmd.return_value = 0, "cmd ok"
        crash_mod = self.__get_crash_module(device=device)
        crash_mod.init(device_serial_number="ACSPHONE1234567", cache_folder="")
        results = crash_mod.enable_clota()
        self.assertGreaterEqual(device.run_cmd.call_count, 1)
        self.assertEqual(results, True)

    @patch.object(CrashToolUploaderModule, "_download_crashtooluploader")
    def test_disable_clota_when_disableCLOTA_activated(self, mock_download_crastooluploader):
        """
        'disableCLOTA' set to True in device config
        Check that 'disable_clota' call will run a cmd on the device
        """
        mock_download_crastooluploader.return_value = ""

        device = MagicMock()
        config = PropertyMock(return_value=AttributeDict({"disableCLOTA": "True",
                                                          "retrieveDeviceLogOnCriticalFailure": "False",
                                                          "uploadCrashToServer": "True",
                                                          "crashServer": "crashtool.iind.intel.com",
                                                          "crashServerPort": "4002"
                                                          }))
        type(device).config = config
        device.run_cmd.return_value = 0, "cmd ok"
        crash_mod = self.__get_crash_module(device=device)
        crash_mod.init(device_serial_number="ACSPHONE1234567", cache_folder="")
        results = crash_mod.disable_clota()
        self.assertGreaterEqual(device.run_cmd.call_count, 1)
        self.assertEqual(results, True)

    @patch.object(CrashToolUploaderModule, "_download_crashtooluploader")
    def test_list_when_uploadCrashToServer_deactivated(self, mock_download_crastooluploader):
        """
        'uploadCrashToServer' set to False in device config
        Check that 'list' do nothing
        """
        mock_download_crastooluploader.return_value = ""

        device = MagicMock()
        config = PropertyMock(return_value=AttributeDict({"disableCLOTA": "True",
                                                          "retrieveDeviceLogOnCriticalFailure": "False",
                                                          "uploadCrashToServer": "False",
                                                          "crashServer": "crashtool.iind.intel.com",
                                                          "crashServerPort": "4002"}))
        type(device).config = config
        crash_mod = self.__get_crash_module(device=device)
        crash_mod.init(device_serial_number="ACSPHONE1234567", cache_folder="")
        self.assertEqual(crash_mod.list(), [])

    @patch.object(CrashToolUploaderModule, "_download_crashtooluploader")
    @patch.object(CrashToolUploaderModule, "_run_crashtooluploader_cmd")
    def test_list_when_uploadCrashToServer_activated(self, mock_run_cmd, mock_download_crastooluploader):
        """
        'uploadCrashToServer' set to True in device config
        Check that 'list' return CRASH events
        """
        mock_download_crastooluploader.return_value = ""
        mock_run_cmd.return_value = Global.SUCCESS, ""

        device = MagicMock()
        config = PropertyMock(return_value=AttributeDict({"disableCLOTA": "True",
                                                          "retrieveDeviceLogOnCriticalFailure": "False",
                                                          "uploadCrashToServer": "True",
                                                          "crashServerSilentMode": "True",
                                                          "crashServer": "crashtool.iind.intel.com",
                                                          "crashServerPort": "4002"
                                                          }))
        type(device).config = config

        crash_mod = self.__get_crash_module(device=device)
        crash_mod.init(device_serial_number="ACSPHONE1234567", cache_folder="")
        crash_listed = self.__get_crash_list(crash_mod)
        self.assertNotEqual(crash_listed, [])
        self.assertEqual(len(crash_listed), 2)
        for event in crash_listed:
            self.assertTrue(event["_id"] in ["10", "11"])

    @patch.object(CrashToolUploaderModule, "_download_crashtooluploader")
    @patch.object(CrashToolUploaderModule, "_run_crashtooluploader_cmd")
    def test_fetch_event_id_when_uploadCrashToServer_deactivated(self, mock_run_cmd, mock_download_crastooluploader):
        """
        'uploadCrashToServer' set to False in device config
        Check that 'fetch' do nothing
        """
        mock_download_crastooluploader.return_value = ""
        mock_run_cmd.return_value = Global.SUCCESS, ""

        device = MagicMock()
        config = PropertyMock(return_value=AttributeDict({"disableCLOTA": "True",
                                                          "retrieveDeviceLogOnCriticalFailure": "False",
                                                          "uploadCrashToServer": "False",
                                                          "crashServerSilentMode": "True",
                                                          "crashServer": "crashtool.iind.intel.com",
                                                          "crashServerPort": "4002"
                                                          }))
        type(device).config = config
        crash_mod = self.__get_crash_module(device=device)
        crash_mod.init(device_serial_number="ACSPHONE1234567", cache_folder="")
        self.assertTrue(crash_mod.fetch(event_id=10))

    @patch.object(CrashToolUploaderModule, "_download_crashtooluploader")
    @patch.object(CrashToolUploaderModule, "_run_crashtooluploader_cmd")
    def test_fetch_event_id_when_uploadCrashToServer_activated(self, mock_run_cmd, mock_download_crastooluploader):
        """
        'uploadCrashToServer' set to False in device config
        Check that 'fetch' do nothing
        """
        mock_download_crastooluploader.return_value = ""
        mock_run_cmd.return_value = Global.SUCCESS, ""

        device = MagicMock()
        config = PropertyMock(return_value=AttributeDict({"disableCLOTA": "True",
                                                          "retrieveDeviceLogOnCriticalFailure": "False",
                                                          "uploadCrashToServer": "True",
                                                          "crashServerSilentMode": "True",
                                                          "crashServer": "crashtool.iind.intel.com",
                                                          "crashServerPort": "4002"
                                                          }))
        type(device).config = config
        crash_mod = self.__get_crash_module(device=device)
        crash_mod.init(device_serial_number="ACSPHONE1234567", cache_folder="")
        self.assertTrue(crash_mod.fetch(event_id=10))

    @patch.object(CrashToolUploaderModule, "_download_crashtooluploader")
    @patch.object(CrashToolUploaderModule, "_run_crashtooluploader_cmd")
    @patch.object(CrashToolUploaderModule, "_list")
    def test_fetch_when_uploadCrashToServer_activated(self, mock_list, mock_run_cmd, mock_download_crastooluploader):
        """
        'uploadCrashToServer' set to True in device config
        Check that 'fetch' fetch all event
        """
        mock_download_crastooluploader.return_value = ""
        mock_run_cmd.return_value = Global.SUCCESS, ""

        device = MagicMock()
        config = PropertyMock(return_value=AttributeDict({"disableCLOTA": "True",
                                                          "retrieveDeviceLogOnCriticalFailure": "False",
                                                          "uploadCrashToServer": "True",
                                                          "crashServerSilentMode": "True",
                                                          "crashServer": "crashtool.iind.intel.com",
                                                          "crashServerPort": "4002"
                                                          }))
        type(device).config = config
        crash_mod = self.__get_crash_module(device=device)
        crash_mod.init(device_serial_number="ACSPHONE1234567", cache_folder="")
        mock_list.return_value = self.__get_crash_list(crash_mod)
        self.assertTrue(crash_mod.fetch())
