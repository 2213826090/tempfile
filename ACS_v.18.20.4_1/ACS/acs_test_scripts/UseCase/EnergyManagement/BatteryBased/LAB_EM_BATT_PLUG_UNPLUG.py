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
:summary: Energy Management / Control a charger plug and unplug on a bench
         without powersupply and check battery & charger register value.
:author: dbatut, vgombert
:since: 16/05/2013
"""
import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class LabEmBattPlugUnplug(EmUsecaseBase):

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

        # Read CHARGER_TYPE from TC parameters
        self.__charger_type = \
            str(self._tc_parameters.get_param_value("CHARGER_TYPE"))
        # Read PLUG_TIMEOUT from TC parameters
        self.__action_timeout = \
            int(self._tc_parameters.get_param_value("ACTION_TIMEOUT"))
        # Read UNPLUG_TIMEOUT from TC parameters
        self.__action_order = \
            str(self._tc_parameters.get_param_value("ACTION_ORDER")).strip()
        self.__action = None

        # init ac charger status
        self.__is_ac_charger_plugged = False
        # init timing offset due to embedded uecmd com
        self.__uecmd_com_timing = 16
        # declare uc name here to ease inheritance
        self._uc_name = "LAB_EM_BATT_PLUG_UNPLUG"

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Execute the set up
        """
        # Call the UseCaseBase Setup function
        EmUsecaseBase.set_up(self)

        # Check if charger type is supported by your io card
        if self.__charger_type not in self._io_card.SUPPORTED_DEVICE_TYPE:
            txt = "io card does not support cable type %s " \
                % self.__charger_type
            self._logger.error(txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, txt)

        if self.__action_order not in ["PLUG;UNPLUG", "UNPLUG;PLUG"]:
            txt = "wrong action order %s " % self.__action_order
            self._logger.error(txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, txt)
        else:
            self.__action = filter(None, self.__action_order.split(";"))

        # get em targets
        self._em_targets = self._target_file.parse_energy_management_targets(
            self._uc_name, self._tc_parameters.get_params_as_dict(),
            self._device.get_phone_model())

        # load targets in order to measure iteration
        self._em_meas_verdict.load_target(self._em_targets, self.tcd_to_test)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        """
        # Call LAB_EM_BASE Run function
        EmUsecaseBase.run_test_body(self)

        # schedule operations
        pid1 = self.em_api.get_msic_registers("scheduled",
                                               self.__action_timeout + self.__uecmd_com_timing)
        time1 = time.time() + self.__action_timeout + self.__uecmd_com_timing
        pid2 = self.em_api.get_msic_registers("scheduled",
                                               self.__action_timeout * 2 + self.__uecmd_com_timing)
        time2 = time.time() + self.__action_timeout * 2 + \
            self.__uecmd_com_timing

        # disconnect the board during test
        self._device.disconnect_board()

        # check that timing control is enough good
        if time.time() > time1:
            raise DeviceException(DeviceException.TIMEOUT_REACHED,
                                   "the time of the embedded msic register measure was reached before"
                                   " the action was done. You need to increase the parameter TIMEOUT")

        # do the first action ( plug or unplug )
        self.__charger_action(self.__action[0])

        # wait some time before check measure
        self._logger.info("Wait %s s before the first measure" %
                          str(time1 + 5 - time.time()))
        while time.time() < time1 + 5:
            time.sleep(1)

        # check that timing control is enough good
        if time.time() > time2:
            raise DeviceException(DeviceException.TIMEOUT_REACHED,
                                   "the time of the embedded msic register measure was reached before"
                                   " the action was done. You need to increase the parameter TIMEOUT")

        # do the second action ( plug or unplug )
        self.__charger_action(self.__action[1])

        # wait some time before check measure
        self._logger.info("Wait %s s before second measure" %
                          str(time2 + 5 - time.time()))
        while time.time() < time2 + 5:
            time.sleep(1)

        # reconnect and get info
        self._io_card.usb_connector(False)
        self._io_card.usb_host_pc_connector(True)
        time.sleep(self.usb_sleep)
        self._device.connect_board()
        self.em_core_module.check_board_connection()

        # Read Platform OS and compare with expected values
        msic_registers = self.em_api.get_msic_registers("read", pid1)
        self._meas_list.add_dict("EMINFO_DURING_" + str(self.__action[0]), msic_registers,
                                 msic_registers["TIME_STAMP"][0])

        # Read Platform OS and compare with expected values
        msic_registers = self.em_api.get_msic_registers("read", pid2)
        self._meas_list.add_dict("EMINFO_DURING_" + str(self.__action[1]), msic_registers,
                                 msic_registers["TIME_STAMP"][0])

        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets)
        self._em_meas_verdict.judge(ignore_blocked_tc=True)
        self._meas_list.clean()

        return self._em_meas_verdict.get_current_result_v2()

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        EmUsecaseBase.tear_down(self)

        # clean generated measure on the board
        if self.is_board_and_acs_ok():
            self.phonesystem_api.clean_daemon_files()

        # check for setting the board as before the setup
        if self.__is_ac_charger_plugged is True:
            self._io_card.ac_charger_connector(False)
            self.__is_ac_charger_plugged = False

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def __charger_action(self, action):
        """
        do the second test action : plug or unplug the good charger type
        """
        if action == "PLUG":
            # plug the charger type
            if self.__charger_type == "AC_CHARGER":
                self._io_card.usb_connector(False)
                self.__is_ac_charger_plugged = True
                self._io_card.ac_charger_connector(False)
            else:
                self._io_card.simulate_insertion(self.__charger_type)
        elif action == "UNPLUG":
            # unplug the charger type
            self._io_card.usb_connector(False)
            if self.__charger_type == "AC_CHARGER":
                self.__is_ac_charger_plugged = False
                self._io_card.ac_charger_connector(False)
