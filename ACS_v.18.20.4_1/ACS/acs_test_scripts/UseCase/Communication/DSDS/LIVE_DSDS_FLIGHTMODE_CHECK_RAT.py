"""
:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
The source code contained or described herein and all documents related
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
:summary: This file implements the RAT check after flight mode
:since: 19/04/2016
:author: nowelchx
"""

import time
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.DeviceException import DeviceException
from uiautomator import Device
from UtilitiesFWK.Utilities import Global

class LiveFlightModeCheckRatDsds(UseCaseBase):

    """
    RAT check after flight mode, test steps :
    - Camp on a specific RAT
    - set flight mode ON
    - set flight mode OFF
    - check device is camping on the same RAT after flight mode
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCaseBase init function
        UseCaseBase.__init__(self, tc_name, global_config)
        self.d = Device(self._device.retrieve_serial_number())

        self._network_pref = \
            self._tc_parameters.get_param_value("PREFERRED_NETWORK_TYPE",
                                                None)
        self._test_timeout=10
        # Get TC parameters
        self._registration_timeout = \
            int(self._dut_config.get("registrationTimeout"))

        # Arbitrary value for timeout in seconds
        self._flight_mode_timeout = 30

        # Instantiate UE Command categories
        self._voicecall_api = self._device.get_uecmd("VoiceCall")
        self._networking_api = self._device.get_uecmd("Networking")
        self._modem_api = self._device.get_uecmd("Modem")
#------------------------------------------------------------------------------

    def set_up(self):
        """
        Test setup
        """
        UseCaseBase.set_up(self)

        # Disable flight mode, if set
        if (self._networking_api.get_flight_mode() is "1"):
            self._networking_api.set_flight_mode("off")

        return Global.SUCCESS, "No errors"
#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """

        UseCaseBase.run_test(self)

        # Enable flight mode on the device
        self._networking_api.set_flight_mode("on")

        self._logger.info("Sleeping %d seconds to ensure modem OFF state"
                        % self._flight_mode_timeout)
        time.sleep(self._flight_mode_timeout)

        # Disable flight mode on the device
        self._networking_api.set_flight_mode("off")

        self._logger.info("Sleeping %d seconds to ensure modem ON state"
                        % self._flight_mode_timeout)
        time.sleep(self._flight_mode_timeout)

        # Check the DUT is camped on a compatible network with the expected
        # preferred network.
        self._setDefaultSim()
        time.sleep(10)
        camped=self._modem_api.check_rat_for_dsds_network(self._registration_timeout)
        self._logger.info(camped)
        self._setDefaultSecondSim()
        time.sleep(10)
        camped_on=self._modem_api.check_rat_for_dsds_network(self._registration_timeout)
        self._logger.info(camped_on)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        Tear down function
        """
        UseCaseBase.tear_down(self)

        return Global.SUCCESS, "No errors"
#--------------------------------------------------------------------------------------------------
    def _setDefaultSim(self):

        self._device.run_cmd("adb shell input keyevent 82",self._test_timeout,
                                                  force_execution=True)
        time.sleep(8)
        #To pop up the dialog box for selecting the sim
        statusintent=self._voicecall_api.get_sim_select(dialog=3,sim=0)
        self._logger.info(statusintent)
        #To check dailog is reached or not
        if statusintent == {}:
            err_msg=" The option dialog is not reached"
            raise DeviceException(DeviceException.OPERATION_SET_ERROR, err_msg)
            return Global.FAILURE,err_msg
        #Waiting for the dailog box
        if self.d(resourceId='android:id/title_template').wait.exists(
                    timeout=5000):
            #clicking the default sim
            self.d(resourceId='android:id/button1').click()
            self._logger.info("Default sim is selected")
            return Global.SUCCESS,"Default sim is selected"
        else:
            err_msg="Dialog is not opening"
            raise DeviceException(DeviceException.OPERATION_SET_ERROR, err_msg)
            return Global.FAILURE,err_msg


#-------------------------------------------------------------------------------------------------
    def _setDefaultSecondSim(self):

        self._device.run_cmd("adb shell input keyevent 82",self._test_timeout,
                                                  force_execution=True)
        time.sleep(8)
        #To pop up the dialog box for selecting the sim
        statusintent=self._voicecall_api.get_sim_select(dialog=3,sim=1)
        self._logger.info(statusintent)
        #To check dailog is reached or not
        if statusintent == {}:
            err_msg=" The option dialog is not reached"
            raise DeviceException(DeviceException.OPERATION_SET_ERROR, err_msg)
            return Global.FAILURE,err_msg
        #Waiting for the dailog box
        if self.d(resourceId='android:id/title_template').wait.exists(
                    timeout=5000):
            #clicking the default sim
            self.d(resourceId='android:id/button1').click()
            self._logger.info("Default sim is selected")
            return Global.SUCCESS,"Default sim is selected"
        else:
            err_msg="Dialog is not opening"
            raise DeviceException(DeviceException.OPERATION_SET_ERROR, err_msg)
            return Global.FAILURE,err_msg