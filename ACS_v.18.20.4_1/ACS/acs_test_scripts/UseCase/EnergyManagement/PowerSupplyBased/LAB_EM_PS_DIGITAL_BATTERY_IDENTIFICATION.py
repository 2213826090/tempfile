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
:summary: Identification of the digital battery
:author: jvauchex
:since: 18/08/2014
"""
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import Global


class LabEmPsDigitalBatteryIdentification(EmUsecaseBase):

    """
    Lab Energy Management class.
    """
    DEDICATED_BENCH = "POWER_SUPPLY_BENCH"

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LAB_EM_BASE Init function
        EmUsecaseBase.__init__(self, tc_name, global_config)

        # Read BATTERY_TYPE from TC parameters
        self.__battery_type = self._tc_parameters.get_param_value("BATTERY_TYPE", self.phone_info["BATTERY"]["BATTID_TYPE"])

        # Redefine initial value for setting USBDIO:
        # - BatteryType
        self.em_core_module.io_card_init_state["BatteryType"] = self.__battery_type

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # Call the LabEmBasePS Setup function
        EmUsecaseBase.set_up(self)

        # if the battery type is emulator, we need to reboot the board
        # in order to take in account new battery emulator  mode
        self._device.reboot()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        """
        # Call LAB_EM_BASE_PS Run function
        EmUsecaseBase.run_test_body(self)

        # Call UECmd in order to verify the DMESG logs
        uecmd_result = self.em_api.is_digital_battery_valid()

        # Verify the verdict
        if not uecmd_result:
            txt = "the Battery charging profile may have not been load in the board"
            self._logger.error(txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, txt)
        else:
            txt = "Digital Battery is Valid"
            self._logger.info(txt)

        return Global.SUCCESS, "the battery charging profile was well loaded"
