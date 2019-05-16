'''
Created on Mar 20, 2014

@author: vbpeters
'''
from mock import patch
from mock import Mock
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from acs_test_scripts.TestStep.Device.Apps.Concurrency.RunSplitScreenGearsVideoRecord import RunSplitScreenGearsVideoRecord
from ErrorHandling.DeviceException import DeviceException
from Core.TestStep.TestStepContext import TestStepContext

activity = "Gfx_VideoCapture"
triglogApplist = [ "ActivityManager: Start proc com.example.gfx_videocapture for activity com.example.gfx_videocapture/.Gfx_VideoCapture",
                   "OpenGLRenderer: Enabling debug mode 0",
                   "Camera_HAL: atom_preview_enabled"]

#Theese messages represent errors that can be catched in stagefright.so
triglogMsglist =  ["OMXVideoEncoder: Failed to encode frame",
                   "The Input buf size is 0 or buf is NULL, return with error"]

class Test(UTestTestStepBase):
    _mock_trigger_strings_found = []
#     _mock_trigger_status_match_iter = 0
#     _mock_trigger_status_iterations = 0

    def setUp(self):
        UTestTestStepBase.setUp(self)
        patch("acs_test_scripts.Utilities.HttpDownloaderUtil.os.makedirs").start()
        self._sut = None
        self._context = TestStepContext()

    def _create_sut(self):
        self._sut = RunSplitScreenGearsVideoRecord(None, None, None, Mock())
        self._sut._device.get_device_logger.return_value.get_message_triggered_status = Mock()
        self._sut._device.get_device_logger.return_value.get_message_triggered_status.side_effect = self._mock_get_msg_trig_status

    def _mock_get_msg_trig_status(self, message):
        if message in self._mock_trigger_strings_found:
            return message
        else:
            return ""

    def test_fail_to_start_app(self):
        self._create_sut()
        self._sut._pars.duration = 1
        self._sut._pars.app_package = "com.example.gfx_videocapture"
        self._sut._pars.activity = activity
        self._mock_trigger_strings_found = []
        with self.assertRaises(DeviceException):
            self._sut.run(self._context)

    def test_fail_app(self):
        self._create_sut()
        self._sut._pars.duration = 1
        self._sut._pars.app_package = "com.example.gfx_videocapture"
        self._sut._pars.activity = activity
        self._mock_trigger_strings_found = triglogMsglist
        with self.assertRaises(DeviceException):
            self._sut.run(self._context)

    def test_pass_result(self):
        self._create_sut()
        self._sut._pars.duration = 1
        self._sut._pars.app_package = "com.example.gfx_videocapture"
        self._sut._pars.activity = activity
        self._mock_trigger_strings_found = triglogApplist
        self._sut.run(self._context)

