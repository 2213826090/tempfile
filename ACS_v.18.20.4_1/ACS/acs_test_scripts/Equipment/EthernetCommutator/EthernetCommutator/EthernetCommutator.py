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
:summary: relay card implementation
:since: 07/08/2013
:author: apairex
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Equipment.EthernetCommutator.Interface.IEthernetCommutator import IEthernetCommutator
from acs_test_scripts.Equipment.IOCards.USBRly08.USBRly08 import USBRly08
import time


class EthernetCommutator(USBRly08, IEthernetCommutator):

    """
    Class that implements Ethernet Commutator equipment
    """

    _CORPORATE_NETWORK = "corporate_network"
    _BENCH_NETWORK = "bench_network"
    _TIME2WAIT_AFTER_POWER_UP = 2

    def __init__(self, name, model, eqt_params, bench_params):
        """
        Constructor
        :type name: str
        :param name: the bench configuration name of the equipment
        :type model: str
        :param model: the model of the equipment
        :type eqt_params: dict
        :param eqt_params: the dictionary containing equipment parameters
        :type bench_params: dict
        :param bench_params: the dictionary containing equipment bench parameters
        """
        USBRly08.__init__(self, name, model, eqt_params, bench_params)
        IEthernetCommutator.__init__(self)
        self._bench_params = bench_params

        # Check for mandatory parameters
        self._bench_params.get_param_value("ComPort")
        self._relay_state_for_bench_network = self._bench_params.get_param_value("RelayStatesForBenchNetwork")
        self._time2wait_after_commutation = self._bench_params.get_param_value("time2waitAfterCommutation")

        self._relay_state_for_bench_network = str(self._relay_state_for_bench_network).upper()

        self._current_network = None
        self._init_done = False

    def init(self):
        """
        Initializes the equipment.
        This function power up the Equipment, if controlled by a master IOCard
        and initialized the USBRLY08 card embedded into the Ethernet Commutator box.
        """
        if not self._init_done:
            # Power up the IOCard inside the EthernetCommutator, if required
            USBRly08.init(self)

            # Check bench config parameters validity
            if self._relay_state_for_bench_network not in ["ON", "OFF"]:
                msg = "Wrong parameter for RelayStatesForBenchNetwork: " + self._relay_state_for_bench_network
                self._logger.error(msg)
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

            if not str(self._time2wait_after_commutation).isdigit():
                msg = "Wrong parameter for time2waitAfterCommutation: " + self._time2wait_after_commutation
                self._logger.error(msg)
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)
            self._time2wait_after_commutation = int(self._time2wait_after_commutation)

            if self._bench_params.get_param_value("IOCard_master"):
                # Need to wait for the embedded IOCard to power up
                time.sleep(self._TIME2WAIT_AFTER_POWER_UP)

            #  The initialization has to be done only once during a given campaign
            self._init_done = True

    def activate_corporate_network(self):
        """
        Plug the ACS host computer to the Ethernet CORPORATE network
        """
        if self._current_network == self._CORPORATE_NETWORK:
            self._logger.debug("Already connected to the CORPORATE network")
            return

        if self._relay_state_for_bench_network == "ON":
            self.disable_line(self._ALL_LINES)
        else:
            self.enable_line(self._ALL_LINES)

        self._current_network = self._CORPORATE_NETWORK

        self._logger.debug("Waiting %d seconds for the CORPORATE Ethernet connection to establish")
        time.sleep(self._time2wait_after_commutation)

    def activate_bench_network(self):
        """
        Plug the ACS host computer to the Ethernet BENCH network
        """
        if self._current_network == self._BENCH_NETWORK:
            self._logger.debug("Already connected to the BENCH network")
            return

        if self._relay_state_for_bench_network == "ON":
            self.enable_line(self._ALL_LINES)
        else:
            self.disable_line(self._ALL_LINES)

        self._current_network = self._BENCH_NETWORK

        self._logger.debug("Waiting %d seconds for the BENCH Ethernet connection to establish")
        time.sleep(self._time2wait_after_commutation)
