# -*- coding:utf-8 -*-

'''
@summary: Android SDK API test, using instrumentation.
@since: 06/30/2016
@author: Lijin Xiong
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.androidframework.fetch_resources import resource
from testlib.androidframework.sdk_api_test_impl import SDK_API_Impl

class SDK_API(UIATestBase):

    def setUp(self):
        super(SDK_API, self).setUp()
        self.__test_name = __name__
        print "Setup: %s" % self.__test_name
        self._api_test = SDK_API_Impl()
        resource.disable_app_verification()
        _apk_path = resource.get_resource_from_atifactory\
        ("tests.tablet.artifactory.conf", "SDK_API", "api_test")
        g_common_obj.adb_cmd_common("install -r %s" % _apk_path)


    def test_AcquireLock(self):
        self._api_test.run_sdk_api_test("PowerManagerTestsDriver#testAcquireLock")

    def test_AllocateMemoryFile(self):
        self._api_test.run_sdk_api_test("MemoryFileDriver#testAllocateMemoryFile")

    def test_BoardString(self):
        self._api_test.run_sdk_api_test("BuildTestsDriver#testBoardString")

    def test_BootloaderString(self):
        self._api_test.run_sdk_api_test("BuildTestsDriver#testBootloaderString")

    def test_BrandString(self):
        self._api_test.run_sdk_api_test("BuildTestsDriver#testBrandString")

    def test_CpuAbi2String(self):
        self._api_test.run_sdk_api_test("BuildTestsDriver#testCpuAbi2String")

    def test_CpuAbiString(self):
        self._api_test.run_sdk_api_test("BuildTestsDriver#testCpuAbiString")

    def test_CreateCanvas(self):
        self._api_test.run_sdk_api_test("CanvasTestsDriver#testCreateCanvas")

    def test_DataDirectory(self):
        self._api_test.run_sdk_api_test("EnvironmentTestsDriver#testDataDirectory")

    def test_DeviceString(self):
        self._api_test.run_sdk_api_test("BuildTestsDriver#testDeviceString")

    def test_Dimensions(self):
        self._api_test.run_sdk_api_test("CanvasTestsDriver#testDimensions")

    def test_DisplayString(self):
        self._api_test.run_sdk_api_test("BuildTestsDriver#testDisplayString")

    def test_DownloadCacheDirectory(self):
        self._api_test.run_sdk_api_test("EnvironmentTestsDriver#testDownloadCacheDirectory")

    def test_ExternalStorageDirectory(self):
        self._api_test.run_sdk_api_test("EnvironmentTestsDriver#testExternalStorageDirectory")

    def test_ExternalStorageState(self):
        self._api_test.run_sdk_api_test("EnvironmentTestsDriver#testExternalStorageState")

    def test_FileObserver(self):
        self._api_test.run_sdk_api_test("EnvironmentTestsDriver#testFileObserver")

    def test_FingerprintString(self):
        self._api_test.run_sdk_api_test("BuildTestsDriver#testFingerprintString")

    def test_HardwareString(self):
        self._api_test.run_sdk_api_test("BuildTestsDriver#testHardwareString")

    def test_HostString(self):
        self._api_test.run_sdk_api_test("BuildTestsDriver#testHostString")

    def test_IdString(self):
        self._api_test.run_sdk_api_test("BuildTestsDriver#testIdString")

    def test_ManufacturerString(self):
        self._api_test.run_sdk_api_test("BuildTestsDriver#testManufacturerString")

    def test_ModelString(self):
        self._api_test.run_sdk_api_test("BuildTestsDriver#testModelString")

    def test_ProductString(self):
        self._api_test.run_sdk_api_test("BuildTestsDriver#testProductString")

    def test_RadioVersionString(self):
        self._api_test.run_sdk_api_test("BuildTestsDriver#testRadioVersionString")

    def test_ReadMemoryFile(self):
        self._api_test.run_sdk_api_test("MemoryFileDriver#testReadMemoryFile")

    def test_RootDirectory(self):
        self._api_test.run_sdk_api_test("EnvironmentTestsDriver#testRootDirectory")

    def test_ScreenOn(self):
        self._api_test.run_sdk_api_test("PowerManagerTestsDriver#testScreenOn")

    def test_SerialString(self):
        self._api_test.run_sdk_api_test("BuildTestsDriver#testSerialString")

    def test_TagsString(self):
        self._api_test.run_sdk_api_test("BuildTestsDriver#testTagsString")

    def test_TimeString(self):
        self._api_test.run_sdk_api_test("BuildTestsDriver#testTimeString")

    def test_TypeString(self):
        self._api_test.run_sdk_api_test("BuildTestsDriver#testTypeString")

    def test_UserString(self):
        self._api_test.run_sdk_api_test("BuildTestsDriver#testUserString")

    def test_VibratePattern(self):
        self._api_test.run_sdk_api_test("VibratorTestsDriver#testVibratePattern")

    def test_VibrateTime(self):
        self._api_test.run_sdk_api_test("VibratorTestsDriver#testVibrateTime")

    def test_WriteMemoryFile(self):
        self._api_test.run_sdk_api_test("MemoryFileDriver#testWriteMemoryFile")
