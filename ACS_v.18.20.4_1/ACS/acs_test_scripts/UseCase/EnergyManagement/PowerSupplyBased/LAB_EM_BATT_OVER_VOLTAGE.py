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
:summary: Energy Management battery over voltage Use case (set & exit battery over voltage condition)
:author: dbatut
:since: 11/08/2011
:last vgomberx: 29/07/2015
"""
import time
import os
from UtilitiesFWK.Utilities import Global, str_to_bool
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from acs_test_scripts.Utilities.EMUtilities import XMLMeasurementFile
from ErrorHandling.DeviceException import DeviceException


class LabEmBattOverVoltage(EmUsecaseBase):

    """
    Lab Energy Management class.
    :TODO: change this usecase to be a voltage change UC, meaning that it can test voltage drop too.
    """
    DEDICATED_BENCH = "POWER_SUPPLY_BENCH"

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call Init function
        EmUsecaseBase.__init__(self, tc_name, global_config)

        # Read parameters from tc
        self.__vbatt_lo = self._tc_parameters.get_param_value("VBATT_LOW", default_cast_type=float)
        self.__vbatt_hi = self._tc_parameters.get_param_value("VBATT_HIGH" , default_cast_type=float)
        self.__charger_to_plug = self._tc_parameters.get_param_value("CHARGE_TYPE_DURING_TEST")
        self.__max_time_high = self._tc_parameters.get_param_value("MAX_TIME_TO_SEE_CHANGE_FOR_VBATT_HIGH", default_cast_type=int)
        self.__max_time_low = self._tc_parameters.get_param_value("MAX_TIME_TO_SEE_CHANGE_FOR_VBATT_LOW", default_cast_type=int)
        self.__set_board_in_idle = self._tc_parameters.get_param_value("IDLE_STATE", default_cast_type=str_to_bool)
        self.__idle_time = self._tc_parameters.get_param_value("TIME_TO_WAIT_AFTER_IDLE_STATE", default_value=0, default_cast_type=int)

        # load targets in order to compute verdict
        self._em_targets = self._target_file.parse_energy_management_targets(
            "LAB_EM_BATT_OVER_VOLTAGE", self._tc_parameters.get_params_as_dict(), self._device.get_phone_model())
        self._em_meas_verdict.load_target(self._em_targets, self.tcd_to_test)

        # Set initial value for setting Power Supply VBATT:
        # - VoltageLevel
        self.em_core_module.eqp_init_state["BATT"]["VoltageLevel"] = self.__vbatt_lo
        # measurement file
        meas_file_name = os.path.join(self._saving_directory, "EM_meas_report.xml")
        self.__em_meas_tab = XMLMeasurementFile(meas_file_name)

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test:
        """
        EmUsecaseBase.set_up(self)
        # Set screen timeout to 30 minutes
        self.phonesystem_api.set_screen_timeout(60 * 60)
        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        Execute the test
        """
        # Call LAB_EM_BASE Run function
        EmUsecaseBase.run_test_body(self)
        self.em_api.clean_autolog()
        log_delay = 30
        extra_time_due_to_idle = 0
        if self.__set_board_in_idle == True:
            extra_time_due_to_idle = 20

        # compute test duration by adding some extra time to let the board see charge insertion
        test_duration = self.__max_time_low + self.__max_time_high + self.__idle_time + extra_time_due_to_idle + 30
        # choose function to put in logger
        self.em_api.add_fct_to_auto_logger(self.em_api.AUTOLOG_UEVENT, "sequenced")
        self.em_api.set_autolog_duration(test_duration)
        # start  non persistent autolog with a short period of data polling
        self.em_api.start_auto_logger(log_delay, 5, "sequenced")

        #---    SWITCH CHARGER         ------#
        start_time = time.time()
        self._device.disconnect_board()
        self.em_core_module.plug_charger(self.__charger_to_plug, ext_ps=True)
        time.sleep(self.usb_sleep)
        remaning_time = log_delay - (time.time() - start_time)
        if remaning_time > 0 :
            self._logger.info("wait %ss to sync measurement with the test beginning" % remaning_time)
            time.sleep(remaning_time)

        #--- VOLTAGE CAPACITY JUMP CASE ------#
        self.__voltage_jump()

        # sync the end of the test to avoid having measurement when data cable is plug back
        remaning_time = test_duration + log_delay + 50 - (time.time() - start_time)
        if remaning_time > 0 :
            self._logger.info("wait %ss to sync measurement with the test end" % remaning_time)
            time.sleep(remaning_time)

        #--- CONNECT DATA AND COLLECT EM INFO --#
        self._io_card.usb_host_pc_connector(True)
        time.sleep(self.usb_sleep)
        self._device.connect_board()
        if self.is_board_and_acs_ok():
            self.em_api.stop_auto_logger()
            # parse autolog response and reset them
            msic_list = self.em_api.get_autolog_msic_registers()
            self.em_api.clean_autolog()

            meas_length = len(msic_list)
            test_step = 0
            test_dict = ["EM_INFO_REF", "EM_INFO_HIGH", "EM_INFO_LOW"]

            for i in range(meas_length):
                try:
                    # get battery/charger info
                    msic_tuple = msic_list[i]
                    if len(msic_tuple) > 1:
                        msic_dict = msic_tuple[1]
                        #--- search for a good battery health proof as reference state --#
                        if test_step < len(test_dict):

                            self._meas_list.add_dict(test_dict[test_step], msic_dict)
                            if self._em_meas_verdict.test_list(self._meas_list, self._em_targets):
                                self._em_meas_verdict.compare_list(self._meas_list, self._em_targets, clean_meas_list=True)
                                test_step += 1

                            # case we reach the end of the measurement, we commit the last test step
                            if (i + 1) == meas_length:
                                self._em_meas_verdict.compare_list(self._meas_list, self._em_targets, clean_meas_list=True)

                        # store result on xml
                        self.__em_meas_tab.add_dict_measurement(msic_dict)
                finally:
                    self.__em_meas_tab.add_measurement(
                        [self.get_time_tuple(), (self._em_cst.COMMENTS,
                            "RUNTEST:battery voltage change")])
                    # switch meas to next meas
                    self.__em_meas_tab.switch_to_next_meas()

        # generate em verdict
        self._em_meas_verdict.judge()
        self._meas_list.clean()
        return self._em_meas_verdict.get_current_result_v2()

    def __voltage_jump(self):
        """
        this is when the vbatt suddenly jump from one value to another
        """
        # turn board in idle and ensure that it was done
        if self.__set_board_in_idle:

            idle_try = 2
            while idle_try > 0:
                ibatt_before_idle = self.em_core_module.get_battery_current()
                self._io_card.press_power_button(0.3)
                time.sleep(5)
                ibatt_after_idle = self.em_core_module.get_battery_current()
                if ibatt_after_idle[0] < ibatt_before_idle[0]:
                    break
                else:
                    self._logger.warning("it seems that the board did not went in idle : IBATT before %s, IBATT after %s" % (ibatt_before_idle, ibatt_after_idle))
                    idle_try -= 1
                    if idle_try <= 0:
                        tmp_txt = "board failed to be seen in idle : IBATT before %s, IBATT after %s" % (ibatt_before_idle, ibatt_after_idle)
                        self._logger.error(self +tmp_txt)
                        raise DeviceException(DeviceException.INVALID_DEVICE_STATE, tmp_txt)

            self._logger.info("Waiting %ss to go in idle mode" % self.__idle_time)
            # wait that board go in idle
            time.sleep(self.__idle_time)

        #------ get some reference values ---#
        ibatt_ref = self.em_core_module.get_battery_current()
        icharger_ref = self.em_core_module.get_charger_current(self._io_card.WALL_CHARGER)
        # store value for comparison
        self._meas_list.add("IBATT_REF", ibatt_ref)
        self._meas_list.add("ICHARGER_REF", icharger_ref)

        start_time = time.time()
        #---- set the vbatt to HIGH value ----#
        ibatt_high, icharger_high = self.__change_voltage(self.__vbatt_hi, ibatt_ref, self.__max_time_high)
        # store value for comparison
        self._meas_list.add("IBATT_HIGH", ibatt_high)
        self._meas_list.add("ICHARGER_HIGH", icharger_high)
        remaning_time = self.__max_time_high - (time.time() - start_time)
        if remaning_time > 0 :
            self._logger.info("wait %ss to let the board see the first voltage change" % remaning_time)
            time.sleep(remaning_time)

        start_time = time.time()
        #---- set the vbatt back to LOW value ----#
        ibatt_low, icharger_low = self.__change_voltage(self.__vbatt_lo, ibatt_high, self.__max_time_low)
        # store value for comparison
        self._meas_list.add("IBATT_LOW", ibatt_low)
        self._meas_list.add("ICHARGER_LOW", icharger_low)
        remaning_time = self.__max_time_low - (time.time() - start_time)
        if remaning_time > 0 :
            self._logger.info("wait %ss to let the board see the first voltage change" % remaning_time)
            time.sleep(remaning_time)

        self._em_meas_verdict.compare_list(self._meas_list, self._em_targets, clean_meas_list=True)

    def __change_voltage(self, vbatt, ref_ibatt, time_to_wait):
        """
        factorized code
        """
        self.em_core_module.set_battery_voltage(vbatt)
        # monitor a change on ibatt
        end_time = time.time() + time_to_wait
        while time.time() < end_time:
            # Measure current from Vbatt
            ibatt = self.em_core_module.get_battery_current()
            # if the product of the ibatt is negative , it means a drastically change has been done
            # and thus we can stop the measurement here
            if ibatt[0] * ref_ibatt[0] < 0:
                break
            time.sleep(1)

        # Measure ICHARGER
        icharger = self.em_core_module.get_charger_current(self._io_card.WALL_CHARGER)
        self._logger.info("VBATT set to voltage %s : IBatt = %s , ICharger= %s " % (vbatt, ibatt, icharger))
        return ibatt, icharger
