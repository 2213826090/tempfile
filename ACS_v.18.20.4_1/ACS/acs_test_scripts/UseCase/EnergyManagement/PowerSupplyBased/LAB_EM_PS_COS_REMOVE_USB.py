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
:summary: Remove USB Charger when board is in COS
:author: jvauchex
:since: 07/16/2014
"""
import time
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException


class LabEmPsCosRemoveUsb(EmUsecaseBase):

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
        self.__charger_type = self._tc_parameters.get_param_value("CHARGER_TYPE")
        self.__cos_shutdown_timeout_value = int(self._tc_parameters.get_param_value("COS_SHUTDOWN_TIMEOUT_VALUE", 120))
        self.__mos_boot_timeout = int(self._tc_parameters.get_param_value("MOS_BOOT_TIMEOUT", 60))
        self.__cos_boot_timeout = int(self._tc_parameters.get_param_value("COS_BOOT_TIMEOUT", 120))
        self.__screen_off_wait = int(self._tc_parameters.get_param_value("SCREEN_OFF_WAIT", 300))
        self.__cos_battery_voltage = float(self._tc_parameters.get_param_value("COS_BATTERY_VOLTAGE"))
        self.__current_range_value = float(self._tc_parameters.get_param_value("CURRENT_RANGE_VALUE", 0.002))

        # Redefine initial value for setting USBDIO:
        # - Platform
        self.em_core_module.io_card_init_state["Platform"] = "OFF"

        # Set initial value for setting Power Supply VBATT:
        # - VoltageLevel = VBATT
        self.em_core_module.eqp_init_state["BATT"]["VoltageLevel"] = self.__cos_battery_voltage

    def run_test_body(self):
        """
        Execute the test
        """
        # Call LAB_EM_BASE_PS Run function
        EmUsecaseBase.run_test_body(self)

        # Plug the right USB Charger
        self._io_card.simulate_insertion(self.__charger_type)

        # Wait the Boot of the board => COS Expected
        self._logger.info("Wait %ss for booting board" % self.__cos_boot_timeout)
        mode = self._device.get_boot_mode()
        end_time = time.time() + self.__cos_boot_timeout
        while (time.time() < end_time) and (mode == "UNKNOWN"):
            mode = self._device.get_boot_mode()
            time.sleep(1)

        # Check if the board boots in COS
        if mode != "UNKNOWN":

            if mode == "COS":

                # Wait the screen off
                self._logger.info("Wait %d the screen off..." % self.__screen_off_wait)
                time.sleep(self.__screen_off_wait)

                # Disconnect the board
                self._logger.info("Disconnect the usb charger ...")
                self._device.disconnect_board()
                self._io_card.usb_connector(False)

                # Wait the board shutdown during the timeout
                self._logger.info("Wait %ss for shutdown" % self.__cos_shutdown_timeout_value)
                time.sleep(self.__cos_shutdown_timeout_value)

                # Measure current from Vbatt
                meas_ibatt = self.em_core_module.get_battery_current()
                # store value in list for later comparison
                if ((meas_ibatt > self.__current_range_value) and (meas_ibatt < -(self.__current_range_value))):
                    txt = "Current value is not in the expected range (%s < i < -%s) "\
                          % (str(self.__current_range_value), str(self.__current_range_value))
                    self._logger.error(txt)
                    raise DeviceException(DeviceException.OPERATION_FAILED, txt)
            else:
                txt = "Board boot mode ( %s ) is not correct !" % mode
                self._logger.error(txt)
                raise AcsConfigException(AcsConfigException.TIMEOUT_REACHED, txt)

        else:
            txt = "Board does not boot in COS before the COS Timeout"
            self._logger.error(txt)
            raise AcsConfigException(AcsConfigException.TIMEOUT_REACHED, txt)

        return self._error.Code, self._error.Msg
