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

@organization: INTEL MCG PSI
@summary: This file implements the reboot from MOS to MOS UC
@since: 03/03/2014
@author: Chenghua Yang
"""

from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
import time


class LabDutRebootWhenMbOff(UseCaseBase):

    """
    This Use Case reboots the DUT when Radio OFF (mobile broadband off)
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # Get UECmdLayer for Data Use Cases
        self._networking_api = self._device.get_uecmd("Networking")
        self._modem_api = self._device.get_uecmd("Modem")

        # Initialize some attributes
        self._boot_timeout = None
        self._settledown_duration = None

        # Get bootTimeout value either from parameter file or
        # default value from phone catalog
        if self._tc_parameters.get_param_value("BOOT_TIMEOUT") != "":
            self._boot_timeout = \
                int(self._tc_parameters.get_param_value("BOOT_TIMEOUT"))

        # Get settledown duration value either from parameter file or
        # default value from phone catalog
        if self._tc_parameters.get_param_value("SETTLEDOWN_DURATION") != "":
            self._settledown_duration = \
                int(self._tc_parameters.get_param_value("SETTLEDOWN_DURATION"))

        # Get UECmdLayer for Data Use Cases
        self._modem_api = self._device.get_uecmd("Modem")

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """

        # Run UC base run_test
        UseCaseBase.run_test(self)

        # Initialize the return code and message of the test
        return_code = Global.FAILURE
        return_message = "An error occurred."

        # turn off mobile broadband
        self._networking_api.reset_mobile_broadband_mode(0)

        time.sleep(5)
        mode_before_reboot = str(self._modem_api.get_modem_power_status())
        self._logger.info ("Starting reboot - Mobile Broadband before reboot is %s" % mode_before_reboot)

        # Restart the phone in MOS
        rebooted = self._device.reboot(self._settledown_duration )
        if rebooted:
            return_code = Global.SUCCESS
            return_message = "Board rebooted successfully."
        else:
            return_code = Global.FAILURE
            return_message = "An error occurred when rebooting the board."

        self._logger.info("Start To Detect Modom connection")
        result = self._modem_api.detect_modem_connection()
        if result == True :
            self._logger.info( "Modem is connected and ready to use" )
        else:
            self._logger.info( "There is no modem connected" )
            return_code = Global.FAILURE
            return_message = "Board was not rebooted properly."
            return return_code, return_message

        # check if the mobile broadband stays off after reboot
        current_mode = str(self._modem_api.get_modem_power_status())
        if mode_before_reboot == current_mode:
                self._logger.info ("Mobile Broadband before reboot is %s and after reboot is %s" % (mode_before_reboot, current_mode) )
                return_code = Global.SUCCESS
                return_message = "Board rebooted successfully and Mobile Broadband before reboot is %s and after reboot is %s" % (mode_before_reboot, current_mode)

        # Return the verdict and the message
        return return_code, return_message
