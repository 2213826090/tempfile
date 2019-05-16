#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=missing-docstring, invalid-name, unused-argument
from mock import MagicMock
from mock import patch
from mock import PropertyMock

from unit_test_fwk.UTestBase import UTestBase
from acs_test_scripts.Device.Module.Common.Crash.CrashInfoModule import CrashInfoModule
from UtilitiesFWK.AttributeDict import AttributeDict


class UTCrashModule(UTestBase):

    def setUp(self):
        self.__crash_lib_mock = patch("acs_test_scripts.Device.Module.Common.Crash.CrashInfoModule.CrashInfo")
        self.__crash_lib = self.__crash_lib_mock.start()()

    def tearDown(self):
        self.__crash_lib_mock.stop()

    def __get_crash_module(self, device=MagicMock()):
        crash_mod = CrashInfoModule()
        crash_mod.device = device
        crash_mod.logger = MagicMock()
        return crash_mod

    def test_enable_clota_when_disableCLOTA_deactivated(self):
        """
        'disableCLOTA' set to False in device config
        Check that enable_clota do nothing on the phone
        """
        device = MagicMock()
        config = PropertyMock(return_value=AttributeDict({"disableCLOTA": "False",
                                                          "retrieveDeviceLogOnCriticalFailure": "False",
                                                          "retrieveDeviceCrashLog": "True",
                                                          "crashServerSilentMode": "True",
                                                          "crashServer": "crashtool.iind.intel.com",
                                                          "crashServerPort": "4002"
                                                          }))
        type(device).config = config
        device.run_cmd.return_value = 0, "cmd ok"
        flash_mod = self.__get_crash_module(device=device)
        flash_mod.init(device_serial_number="ACSPHONE1234567", cache_folder="")
        flash_mod.enable_clota()
        self.assertEqual(device.run_cmd.call_count, 0)

    def test_disable_clota_when_disableCLOTA_deactivated(self):
        """
        'disableCLOTA' set to False in device config
        Check that 'disable_clota' do nothing on the phone
        """
        device = MagicMock()
        config = PropertyMock(return_value=AttributeDict({"disableCLOTA": "False",
                                                          "retrieveDeviceLogOnCriticalFailure": "False",
                                                          "retrieveDeviceCrashLog": "True",
                                                          "crashServerSilentMode": "True",
                                                          "crashServer": "crashtool.iind.intel.com",
                                                          "crashServerPort": "4002"
                                                          }))
        type(device).config = config
        device.run_cmd.return_value = 0, "cmd ok"
        flash_mod = self.__get_crash_module(device=device)
        flash_mod.init(device_serial_number="ACSPHONE1234567", cache_folder="")
        flash_mod.disable_clota()
        self.assertEqual(device.run_cmd.call_count, 0)

    def test_enable_clota_when_disableCLOTA_activated(self):
        """
        'disableCLOTA' set to True in device config
         Check that 'enable_clota' will run a cmd on the device
        """
        device = MagicMock()
        config = PropertyMock(return_value=AttributeDict({"disableCLOTA": "True",
                                                          "retrieveDeviceLogOnCriticalFailure": "False",
                                                          "retrieveDeviceCrashLog": "True",
                                                          "crashServerSilentMode": "True",
                                                          "crashServer": "crashtool.iind.intel.com",
                                                          "crashServerPort": "4002"
                                                          }))
        type(device).config = config
        device.run_cmd.return_value = 0, "cmd ok"
        flash_mod = self.__get_crash_module(device=device)
        flash_mod.init(device_serial_number="ACSPHONE1234567", cache_folder="")
        flash_mod.enable_clota()
        self.assertGreaterEqual(device.run_cmd.call_count, 1)

    def test_disable_clota_when_disableCLOTA_activated(self):
        """
        'disableCLOTA' set to True in device config
        Check that 'disable_clota' call will run a cmd on the device
        """
        device = MagicMock()
        config = PropertyMock(return_value=AttributeDict({"disableCLOTA": "True",
                                                          "retrieveDeviceLogOnCriticalFailure": "False",
                                                          "retrieveDeviceCrashLog": "True",
                                                          "crashServerSilentMode": "True",
                                                          "crashServer": "crashtool.iind.intel.com",
                                                          "crashServerPort": "4002"
                                                          }))
        type(device).config = config
        device.run_cmd.return_value = 0, "cmd ok"
        flash_mod = self.__get_crash_module(device=device)
        flash_mod.init(device_serial_number="ACSPHONE1234567", cache_folder="")
        flash_mod.disable_clota()
        self.assertGreaterEqual(device.run_cmd.call_count, 1)

    def test_list_when_retrieveDeviceCrashLog_deactivated(self):
        """
        'retrieveDeviceCrashLog' set to False in device config
        Check that 'list' do no
        """
        device = MagicMock()
        config = PropertyMock(return_value=AttributeDict({"disableCLOTA": "True",
                                                          "retrieveDeviceLogOnCriticalFailure": "False",
                                                          "retrieveDeviceCrashLog": "False",
                                                          "crashServerSilentMode": "True",
                                                          "crashServer": "crashtool.iind.intel.com",
                                                          "crashServerPort": "4002"
                                                          }))
        type(device).config = config
        flash_mod = self.__get_crash_module(device=device)
        flash_mod.init(device_serial_number="ACSPHONE1234567", cache_folder="")
        self.assertEqual(flash_mod.list(), [])

    def test_list_when_retrieveDeviceCrashLog_activated(self):
        """
        'retrieveDeviceCrashLog' set to True in device config
        Check that 'list' return CRASH events
        """
        device = MagicMock()
        config = PropertyMock(return_value=AttributeDict({"disableCLOTA": "True",
                                                          "retrieveDeviceLogOnCriticalFailure": "False",
                                                          "retrieveDeviceCrashLog": "True",
                                                          "crashServerSilentMode": "True",
                                                          "crashServer": "crashtool.iind.intel.com",
                                                          "crashServerPort": "4002"
                                                          }))
        type(device).config = config
        # add 3 elements: 2 CRASH only
        crash1 = dict(_id=10,
                      eventName="CRASH",
                      date="2014-07-09/10:39:44",
                      eventId=10,
                      type="IPANIC",
                      crashdir="/home/acs/my_log")
        crash2 = dict(_id=50,
                      eventName="CRASH",
                      date="2014-07-09/10:41:37",
                      eventId=50,
                      type="ANR",
                      crashdir="/home/acs/my_log1")
        crash3 = dict(_id=55,
                      eventName="CRASH_NOT",
                      date="2014-07-09/10:42:57",
                      eventId=55,
                      type="IPANIC",
                      crashdir="/home/acs/my_log2")

        self.__crash_lib.get_event.return_value = [crash1,
                                                   crash2,
                                                   crash3]
        flash_mod = self.__get_crash_module(device=device)
        flash_mod.init(device_serial_number="ACSPHONE1234567", cache_folder="")
        crash_listed = flash_mod.list()
        self.assertNotEqual(crash_listed, [])
        for event in crash_listed:

            self.assertTrue(event["_id"]in [crash1["_id"], crash2["_id"]])
            self.assertFalse(dict(event) == crash3)

    def test_fetch_event_id_when_retrieveDeviceCrashLog_deactivated(self):
        """
        'retrieveDeviceCrashLog' set to False in device config
        Check that 'fetch' do nothing
        """
        device = MagicMock()
        config = PropertyMock(return_value=AttributeDict({"disableCLOTA": "True",
                                                          "retrieveDeviceLogOnCriticalFailure": "False",
                                                          "retrieveDeviceCrashLog": "False",
                                                          "crashServerSilentMode": "True",
                                                          "crashServer": "crashtool.iind.intel.com",
                                                          "crashServerPort": "4002"
                                                          }))
        type(device).config = config
        flash_mod = self.__get_crash_module(device=device)
        flash_mod.init(device_serial_number="ACSPHONE1234567", cache_folder="")
        self.assertTrue(flash_mod.fetch(event_id=10))
        self.assertEqual(self.__crash_lib.pull_event.call_count, 0)

    def test_fetch_event_id_when_retrieveDeviceCrashLog_activated(self):
        """
        'retrieveDeviceCrashLog' set to True in device config
        Check that 'fetch' fetch the specific id
        """
        device = MagicMock()
        config = PropertyMock(return_value=AttributeDict({"disableCLOTA": "True",
                                                          "retrieveDeviceLogOnCriticalFailure": "False",
                                                          "retrieveDeviceCrashLog": "True",
                                                          "crashServerSilentMode": "True",
                                                          "crashServer": "crashtool.iind.intel.com",
                                                          "crashServerPort": "4002"
                                                          }))
        type(device).config = config
        flash_mod = self.__get_crash_module(device=device)
        flash_mod.init(device_serial_number="ACSPHONE1234567", cache_folder="")
        self.__crash_lib.pull_event.return_value = True, "/home/acs/my_log1", "no error"
        self.assertTrue(flash_mod.fetch(event_id=10))
        self.assertGreaterEqual(self.__crash_lib.pull_event.call_count, 1)

    def test_fetch_when_retrieveDeviceCrashLog_activated_and_no_event(self):
        """
        'retrieveDeviceCrashLog' set to True in device config
        Check that 'fetch' fetch all event
        """
        device = MagicMock()
        config = PropertyMock(return_value=AttributeDict({"disableCLOTA": "True",
                                                          "retrieveDeviceLogOnCriticalFailure": "False",
                                                          "retrieveDeviceCrashLog": "True",
                                                          "crashServerSilentMode": "True",
                                                          "crashServer": "crashtool.iind.intel.com",
                                                          "crashServerPort": "4002"
                                                          }))
        type(device).config = config
        flash_mod = self.__get_crash_module(device=device)
        flash_mod.init(device_serial_number="ACSPHONE1234567", cache_folder="")
        self.__crash_lib.get_event.return_value = []
        self.assertTrue(flash_mod.fetch())
        self.assertGreaterEqual(self.__crash_lib.get_event.call_count, 1)
