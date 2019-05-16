# @PydevCodeAnalysisIgnore
# pylint: disable=E0602,W0212,C0103,C0111
"""
:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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

:organization: INTEL NDG sw
:summary: Unit test module
:since: 21/10/14
:author: dpierrex
"""

from mock import MagicMock

from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.TestStep.Device.System.Settings.GetDateTime import GetDateTime
from acs_test_scripts.TestStep.Device.System.Settings.GetRtcMode import GetRtcMode
from acs_test_scripts.TestStep.Device.System.Settings.SetDateTime import SetDateTime
from UtilitiesFWK.Utilities import Global
from Core.TestStep.TestStepContext import TestStepContext


GOOD_DATE = "2014-10-21"
GOOD_TIME = "22:43:56"
BAD_DATE = "2014/10/21"
BAD_TIME = "22.43.56"
DEFAULT_SAVE_VALUE = "my_save_value"
class DateTimeTestCase(UTestTestStepBase):

    def setUp(self):
        UTestTestStepBase.setUp(self)
        self._context = TestStepContext()

    def test_setdatetime_with_bad_time_format(self):
        """
         Check that exception is raised when bad time format is given
        """
        teststep = self._create_sut_set({"DATE": GOOD_DATE, "TIME": BAD_TIME})
        teststep._system_api.set_date_and_time.return_value = (Global.SUCCESS, "")
        with self.assertRaisesRegexp(AcsConfigException, "Bad time format"):
            teststep.run(self._context)

    def test_setdatetime_with_bad_date_format(self):
        """
         Check that exception is raised when bad date format is given
        """
        teststep = self._create_sut_set({"DATE": BAD_DATE, "TIME": GOOD_TIME})
        teststep._system_api.set_date_and_time.return_value = (Global.SUCCESS, "")
        with self.assertRaisesRegexp(AcsConfigException, "Bad date format"):
            teststep.run(self._context)

    def test_setdatetime_with_failure(self):
        """
         Check that exception is raised when set_date_and_time fail
        """
        teststep = self._create_sut_set({"DATE": GOOD_DATE, "TIME": GOOD_TIME})
        teststep._system_api.set_date_and_time.return_value = (Global.FAILURE, "fail")
        with self.assertRaises(DeviceException):
            teststep.run(self._context)

    def fixme_test_getdatetime_with_bad_output_format(self):
        """
         Check that exception is raised when bad output format is given
        """
        teststep = self._create_sut_get_date_time({"VALUE": DEFAULT_SAVE_VALUE, "DATE_FORMAT": "Y-m-d"})
        teststep._system_api.get_date_and_time.return_value = (Global.SUCCESS, "2014-10-09 19:11:07")
        with self.assertRaises(AcsConfigException):
            teststep.run(self._context)

    def test_getdatetime_with_device_exeption(self):
        """
         Check that exception is raised when set_date_and_time fail
        """
        teststep = self._create_sut_get_date_time({"VALUE": DEFAULT_SAVE_VALUE})
        teststep._system_api.get_date_and_time.return_value = (Global.FAILURE, "date: invalid date")
        with self.assertRaises(DeviceException):
            teststep.run(self._context)

    def test_getrtcmode_with_rtc_in_utc(self):
        """
         Check that returned value is "0" when rtc is in utc
        """
        teststep = self._create_sut_get_rtc({"VALUE": DEFAULT_SAVE_VALUE})
        teststep._phonesystem_api.get_rtc_mode.return_value = False
        teststep.run(self._context)
        self.assertEquals("0", self._context.get_info(DEFAULT_SAVE_VALUE))

    def test_getrtcmode_with_rtc_in_local(self):
        """
         Check that returned value is "1" when rtc is in local
        """
        teststep = self._create_sut_get_rtc({"VALUE": DEFAULT_SAVE_VALUE})
        teststep._phonesystem_api.get_rtc_mode.return_value = True
        teststep.run(self._context)
        self.assertEquals("1", self._context.get_info(DEFAULT_SAVE_VALUE))

    def fixme_test_getrtcmode_with_bad_cmd_answer(self):
        """
         Check that exception is raised when command return nothing
        """
        teststep = self._create_sut_get_rtc({"VALUE": DEFAULT_SAVE_VALUE})
        teststep._device._internal_exec.return_value = (Global.SUCCESS, "")
        with self.assertRaises(DeviceException):
            teststep.run(self._context)

    def _create_sut_get_rtc(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """
        sut = GetRtcMode(None, None, test_step_pars, MagicMock())
        return sut

    def _create_sut_get_date_time(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """
        sut = GetDateTime(None, None, test_step_pars, MagicMock())
        return sut

    def _create_sut_set(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """
        sut = SetDateTime(None, None, test_step_pars, MagicMock())
        return sut
