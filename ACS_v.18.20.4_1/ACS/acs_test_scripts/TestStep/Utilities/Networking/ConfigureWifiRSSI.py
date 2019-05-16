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

:organization: INTEL MCG PSI
:summary: This file implements Test Step for Configure RSSI
:since 01/12/2014
:author: jfranchx
"""
import time
import numpy
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.Equipment.ConfigurableAP.Cisco1250.Cisco1250 import Cisco1250
from acs_test_scripts.Utilities.NetworkingUtilities import AcsWifiFrequencies


class ConfigureWifiRSSI(DeviceTestStepBase):
    """
    Implements Configure RSSI class
    """

    # All values between 1 and 100 are available on Asus AC66U. We will just try some values.
    ASUS_AC66U_POWER_VALUES = [1, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100]
    MINIMUM_TIME_BEFORE_MEASURE = 6.0
    TIME_TO_MEASURE_RSSI = 30.0
    MAX_LOOP_ATTENUATOR = 10

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Constructor
        """
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._equipment_manager = None
        self._configurable_ap_parameters = None
        self._rf_attenuator_parameters = None

        self._configurable_ap = None
        self._rf_attenuator = None
        self._rf_attenuator_min_value = None
        self._rf_attenuator_max_value = None

        self._networking_api = self._device.get_uecmd("Networking")
        self._min_rssi = None
        self._max_rssi = None
        self._ssid = None
        self._loop_attenuator = 0

    #------------------------------------------------------------------------------

    def run(self, context):
        """
        Run the step
        """
        DeviceTestStepBase.run(self, context)
        self._equipment_manager = self._factory.create_equipment_manager()
        self._configurable_ap_parameters = self._global_conf.benchConfig.get_parameters(self._pars.configurable_ap)
        self._configurable_ap = self._equipment_manager.get_configurable_ap(self._pars.configurable_ap)
        if self._pars.rf_attenuator is not None:
            self._rf_attenuator_parameters = self._global_conf.benchConfig.get_parameters(self._pars.rf_attenuator)
            self._rf_attenuator = self._equipment_manager.get_rf_attenuator(self._pars.rf_attenuator)
        else:
            msg = "RF_ATTENUATOR not defined - No attenuator used for the Test Step"
            self._logger.debug(msg)

        self._min_rssi = int(self._pars.rssi_min)
        self._max_rssi = int(self._pars.rssi_max)

        if self._configurable_ap is None:
            msg = "CONFIGURABLE_AP %s not found - Check your bench config" % self._pars.configurable_ap
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        self._ssid = self._configurable_ap_parameters.get_param_value("SSID")
        if self._ssid is None:
            msg = "Can't get SSID of configurable ap - Check bench config"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG, msg)

        if self._configurable_ap_parameters.get_param_value("Model") == "CISCO_1250":
            if self._pars.standard in AcsWifiFrequencies.WIFI_STANDARD_5G:
                power_values = Cisco1250.POWER_VALUES_5G
            else:
                power_values = Cisco1250.POWER_VALUES_2G
        elif self._configurable_ap_parameters.get_param_value("Model") == "ASUS_AC66U":
            power_values = self.ASUS_AC66U_POWER_VALUES
        else:
            msg = "EQT Model %s unknown - Only CISCO_1250 and ASUS_AC66U supported" % \
                  self._configurable_ap_parameters.get_param_value("Model")
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        if self._pars.rf_attenuator is not None and self._rf_attenuator is None:
            self._logger.error("RF_ATTENUATOR %s not found - Check your bench config" % self._pars.rf_attenuator)
            self._logger.error("TS will try to configure RSSI without RF attenuator")

        # Check if RSSI is already OK
        if self._control_rssi(self.TIME_TO_MEASURE_RSSI):
            self._logger.debug("RSSI already OK - nothing to configure")
            return

        # Initiate connection to equipments
        self._logger.debug("RSSI not OK - start RSSI configuration")
        self._configurable_ap.init()

        rssi_check = False
        try:
            # Configure power on AP
            for power in power_values:
                self._configurable_ap.set_wifi_power(self._pars.standard, power)

                if self._rf_attenuator is None:
                    result = self._control_rssi(self.TIME_TO_MEASURE_RSSI)
                else:
                    self._loop_attenuator = 0
                    self._rf_attenuator.init()
                    self._rf_attenuator_min_value = self._rf_attenuator.get_min_attn()
                    self._rf_attenuator_max_value = self._rf_attenuator.get_max_attn()
                    result = self._attenuator_process();
                if result:
                    rssi_check = True
                    break
        finally:
            # Close the connection to equipments
            self._configurable_ap.release()
            if self._rf_attenuator is not None:
                self._rf_attenuator.release()

        if not rssi_check:
            # Bad RSSI configuration, raise an exception
            msg = "Can't configure RSSI between %sdBm and %sdBm" % (self._min_rssi, self._max_rssi)
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, msg)

    def _attenuator_process(self):
        """
        Configure attenuator to get the correct RSSI

        :rtype: bool
        :return: True if RSSI is OK, else False
        """
        # Check if attenuation possibilities are in range of RSSI margin
        self._rf_attenuator.set_attenuation(self._rf_attenuator_min_value)
        time.sleep(self.MINIMUM_TIME_BEFORE_MEASURE)
        if self._networking_api.get_wifi_rssi(self._ssid) < self._min_rssi:
            self._logger.debug("Can't configure RSSI with this AP configuration")
            return False
        self._rf_attenuator.set_attenuation(self._rf_attenuator_max_value)
        time.sleep(self.MINIMUM_TIME_BEFORE_MEASURE)
        if self._networking_api.get_wifi_rssi(self._ssid) > self._max_rssi:
            self._logger.debug("Can't configure RSSI with this AP configuration")
            return False

        result_at = self._loop_attenuator_configuration(self._rf_attenuator_min_value, self._rf_attenuator_max_value)
        if result_at:
            return True
        return False

    def _loop_attenuator_configuration(self, attenuation_min_value, attenuation_max_value):
        """
        Function to configure RSSI with attenuator.

        :type attenuation_min_value: int
        :param attenuation_min_value: minimum attenuation value
        :type attenuation_max_value: int
        :param attenuation_max_value: maximum attenuation value
        :rtype: bool
        :return: True if RSSI is OK, else False
        """
        self._loop_attenuator += 1
        if self._loop_attenuator > self.MAX_LOOP_ATTENUATOR:
            msg = "Can't configure attenuator - Stop loop"
            self._logger.debug(msg)
            return False

        cmd_average = int(round((attenuation_min_value + attenuation_max_value)/2))
        self._rf_attenuator.set_attenuation(cmd_average)
        time.sleep(self.MINIMUM_TIME_BEFORE_MEASURE)
        check_control = self._control_rssi(self.TIME_TO_MEASURE_RSSI)
        if check_control:
            return True

        current_rssi = self._networking_api.get_wifi_rssi(self._ssid)
        if current_rssi > self._max_rssi:
            # Need to reduce RSSI -> raise attenuation
            result = self._loop_attenuator_configuration(cmd_average, attenuation_max_value)
        elif current_rssi < self._min_rssi:
            # Need to raise RSSI -> reduce attenuation
            result = self._loop_attenuator_configuration(attenuation_min_value, cmd_average)
        else:
            # Retry
            result = self._loop_attenuator_configuration(attenuation_min_value, attenuation_max_value)
        return result

    def _control_rssi(self, timer):
        """
        Check the RSSI from DUT during a given time.
        :type timer: int
        :param timer: time for control
        :rtype: boolean
        :return: return True if DUT RSSI is ok, else False
        """
        rssi_values = []
        init_time = time.time()
        while time.time() - init_time < timer:
            tmp_value = self._networking_api.get_wifi_rssi(self._ssid)
            rssi_values.append(tmp_value)
        rssi_check_value = numpy.median(rssi_values)

        if rssi_check_value in range(self._min_rssi, self._max_rssi):
            return True
        return False
