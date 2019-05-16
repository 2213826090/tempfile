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

:organization: INTEL MCG PSI
:summary: This file implements selection of the sim in DSDS
:since: 15/03/2016
:author: nowelchx
"""

from UtilitiesFWK.Utilities import Global
from ErrorHandling.DeviceException import DeviceException
from uiautomator import Device
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase

class LiveDsdsSeletion(UseCaseBase):

    """
    Live DSDS sim selection.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        UseCaseBase.__init__(self, tc_name, global_config)

        self._test_timeout=10
        #getting the value from xml
        self.preffered_sim=self._tc_parameters.get_param_value("SIM")
        self.d = Device(self._device.retrieve_serial_number())
        # Retrieve phone system APIs
        self._voicecall_api = self._device.get_uecmd("VoiceCall")

        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")
#------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """
        #using the Usecase run test
        UseCaseBase.run_test(self)
        #checking the sim to be selected as default
        if self.preffered_sim=="SIM1":
            verdict,msg=self._select_first_sim()
            return verdict,msg
        if self.preffered_sim=="SIM2":
            verdict,msg=self._select_second_sim()
            return verdict,msg

#-------------------------------------------------------------------------------
    def _select_first_sim(self):
        """
        To select the first sim
        """
        #To Clear the Screen
        self._device.run_cmd("adb shell input keyevent 82",self._test_timeout,
                                                  force_execution=True)
        #To pop up the dailog box to set the default sim
        statusintent=self._voicecall_api.get_sim_select(dialog=3,sim=0)
        self._logger.info(statusintent)

        if statusintent == None:
            err_msg=" The option dialog is not reached"
            raise DeviceException(DeviceException.OPERATION_SET_ERROR, err_msg)
            return Global.FAILURE,err_msg
        #To check the dailog box appearerd or not and click to set it
        if self.d(resourceId='android:id/button1').wait.exists(timeout=20):
            responseClick=self.d(resourceId='android:id/button1').click()
            self._logger.info("Default sim set successfully")
        else:
            err_msg = "Failed to set "
            raise DeviceException(DeviceException.OPERATION_SET_ERROR, err_msg)
            return Global.FAILURE,err_msg

        if responseClick==True:
            self.d.press.home()
            return Global.SUCCESS,"sim is selected successfully"

#-------------------------------------------------------------------------------
    def _select_second_sim(self):
        """
        To select the second sim
        """
        #To Clear the Screen
        self._device.run_cmd("adb shell input keyevent 82",self._test_timeout,
                                                  force_execution=True)
        #To pop up the dailog box to set the default sim
        statusintent=self._voicecall_api.get_sim_select(dialog=3,sim=1)
        self._logger.info(statusintent)

        if statusintent == None:
            err_msg=" The option dialog is not reached"
            raise DeviceException(DeviceException.OPERATION_SET_ERROR, err_msg)
            return Global.FAILURE,err_msg
        #To check the dailog box appeared or not and click to set it
        if self.d(resourceId='android:id/button1').wait.exists(timeout=20):
            responseClick=self.d(resourceId='android:id/button1').click()
            self._logger.info("Default sim set successfully")
        else:
            err_msg = "Failed to set "
            raise DeviceException(DeviceException.OPERATION_SET_ERROR, err_msg)
            return Global.FAILURE,err_msg

        if responseClick==True:
            self.d.press.home()
            return Global.SUCCESS,"sim is selected successfully"