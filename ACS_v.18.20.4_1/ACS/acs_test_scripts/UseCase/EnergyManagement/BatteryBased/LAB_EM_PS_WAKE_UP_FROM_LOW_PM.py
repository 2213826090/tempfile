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
:summary: Platform shall wake up from low power mode upon USB insertion : check screen wake up
:author: jortetx
:since: 05/05/2014
"""
from ErrorHandling.DeviceException import DeviceException
import time

from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from ErrorHandling.AcsConfigException import AcsConfigException


class LabEmWakeUpFromLowPm(EmUsecaseBase):

    """
    Lab Energy Management class.
    """
    DEDICATED_BENCH = "BATTERY_BENCH"

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LAB_EM_BASE Init function
        EmUsecaseBase.__init__(self, tc_name, global_config)
        # Read charger type from TC parameters
        self.__charger = self._tc_parameters.get_param_value("CHARGER")
        # sleeping time to stabilize the platform
        self.__time_to_sleep = self._tc_parameters.get_param_value("TIME_TO_SLEEP", default_cast_type=int)
        # waiting offset for scheduled command
        self.__waitingoffset = 20

    def set_up(self):
        """
        Initialize the test:
        """
        EmUsecaseBase.set_up(self)

        # Check if charger type is supported by your io card
        if self.__charger not in self._io_card.SUPPORTED_DEVICE_TYPE:
            txt = "io card does not support cable type %s " % self.__charger
            self._logger.error(txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, txt)
        # board should sleep asap
        self.phonesystem_api.set_screen_timeout(self.usb_sleep + self.__waitingoffset)
        self.phonesystem_api.set_phone_lock(0)
        return Global.SUCCESS, "No errors"

    def run_test_body(self):
        """
        Execute the test
        """
        # launch the scheduled command
        scheduled_task_id = self.phonesystem_api.get_screen_status("scheduled", self.__time_to_sleep + self.usb_sleep + self.__waitingoffset)
        # turn off screen
        self.phonesystem_api.sleep_screen()
        # unplug charger
        self._device.disconnect_board()
        self._io_card.remove_cable(self.__charger)
        # wait 180s to stabilize the device
        self._logger.info("waiting %ss to let the DUT go in idle mode" % self.__time_to_sleep)
        time.sleep(self.__time_to_sleep)
        # plug the usb charger
        self._io_card.simulate_insertion(self.__charger)
        time_during_insertion = self.usb_sleep + self.__waitingoffset + 30
        self._logger.info("waiting %ss to let the board see the cable insertion" % time_during_insertion)
        time.sleep(time_during_insertion)
        # disconnecting charger
        self._logger.info("Disconnecting %s" % self.__charger)
        self._io_card.remove_cable(self.__charger)
        time.sleep(self.usb_sleep)
        # reconnect pc host to check results
        self._logger.info("reconnecting pc host cable to get measured data")
        self._io_card.usb_host_pc_connector(True)
        time.sleep(self.usb_sleep)
        # reconnect board to acs
        self._device.connect_board()

        # read the scheduled task value
        return_state = self.phonesystem_api.get_screen_status("read", scheduled_task_id)

        if return_state:
            msg = "Device waked up upon %s insertion " % self.__charger
        else:
            msg = "Device is still in sleeping mode after %s insertion " % self.__charger
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        return Global.SUCCESS, msg

    def tear_down(self):
        """
        End and dispose the test
        """
        EmUsecaseBase.tear_down(self)
        # retrieve logs
        self.get_application_logs()
        self.phonesystem_api.set_screen_timeout(60)
        # clean schedule created object
        self.phonesystem_api.clean_daemon_files()
        return Global.SUCCESS, "No errors"
