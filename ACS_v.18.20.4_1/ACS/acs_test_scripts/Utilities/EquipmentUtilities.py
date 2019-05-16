#!/usr/bin/env python
# -*- coding: utf-8 -*-

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
:summary:
:since: 1/2/14
:author: galagnox
"""
from ErrorHandling.AcsBaseException import AcsBaseException


class NetworkSimulatorIP:
    """
    Structure that represent Network Simulator IP addresses
    """

    def __init__(self, logger=None):
        self._ns_number = 1
        self._logger = logger
        self._ns_ip_lan1 = None
        self._ns_ip_lan2 = None
        self._ns_ip_dut1 = None
        self._ns_ip_dut2 = None
        self._ns_ip_dns1 = None
        self._ns_ip_dns2 = None
        self._ns_ip_sub_net_mask = None
        self._ns_ip_default_gateway = None

    def setup_config(self, global_config, network_simulator_number=1):
        """
        Set all the class variables

        :type global_config: dict
        :param global_config: Global config
        :type network_simulator_number: int
        :param network_simulator_number: Network simulator number in bench config

        :rtype:
        :return: None
        """
        self._ns_number = network_simulator_number
        try:
            if str(self._ns_number).isdigit():
                # Read NETWORK_SIMULATOR%d from BenchConfig.xml
                ns_node = global_config.benchConfig.get_parameters("NETWORK_SIMULATOR%d" % self._ns_number)
            else:
                raise AcsBaseException(AcsBaseException.INVALID_PARAMETER, "Wrong parameter Network Simulator (digit).")

            self._ns_ip_lan1 = ns_node.get_param_value("IP_Lan1")
            self._ns_ip_lan2 = ns_node.get_param_value("IP_Lan2")
            self._ns_ip_dut1 = ns_node.get_param_value("DUT_IP_Address")
            self._ns_ip_dut2 = ns_node.get_param_value("DUT_IP_Address2", "")
            self._ns_ip_dns1 = ns_node.get_param_value("DNS1")
            self._ns_ip_dns2 = ns_node.get_param_value("DNS2")
            self._ns_ip_sub_net_mask = ns_node.get_param_value("Subnet_Mask")
            self._ns_ip_default_gateway = ns_node.get_param_value("Default_Gateway")
        except AcsBaseException as acs_exception:
            if self._logger is not None:
                self._logger.error("Failed to construct NetworkSimulatorIP object: %s" % str(acs_exception))
            raise

    def get_config(self):
        """
        Return a list of Network Simulator parameters:
        IP lan 1 and 2, DUT IP 1 and 2, DNS 1 and 2, and sub-net mask

        :rtype: tuple
        :return: list of Network Simulator IP addresses
        """
        var_list = (self._ns_ip_lan1, self._ns_ip_lan2, self._ns_ip_dut1, self._ns_ip_dut2, self._ns_ip_dns1,
                    self._ns_ip_dns2, self._ns_ip_sub_net_mask, self._ns_ip_default_gateway)
        return var_list


def get_nw_sim_bench_name(capability, global_config, logger):
    """
    Returns the bench name of the network simulator having the wanted capability

    :type capability: str
    :param capability: cellular capability can be either 2G|3G|4G

    :type global_config: Dictionary
    :param global_config: Global configuration of ACS

    :type logger: object
    :param logger: Instance used to log messages

    :rtype: str
    :return: bench name as typed in bench config file
    """
    bench_name = ""
    ns1_model = ""
    ns2_model = ""
    logger.info("Looking for a network simulator supporting %s" % capability)
    try:
        # Read NETWORK_SIMULATOR1 from BenchConfig.xml
        ns1_node = global_config.benchConfig.get_parameters("NETWORK_SIMULATOR1")
        # Retrieve the model of the equipment
        ns1_model = ns1_node.get_param_value("Model")
        logger.info("NETWORK_SIMULATOR1 found: %s" % ns1_model)
    except:
        logger.warning("NETWORK_SIMULATOR1 not found in Bench Config file")
        pass

    try:
        # Read NETWORK_SIMULATOR2 from BenchConfig.xml
        ns2_node = global_config.benchConfig.get_parameters("NETWORK_SIMULATOR2")
        # Retrieve the model of the equipment
        ns2_model = ns2_node.get_param_value("Model")
        logger.info("NETWORK_SIMULATOR2 found: %s" % ns2_model)
    except:
        logger.warning("NETWORK_SIMULATOR2 not found in Bench Config file")
        pass

    if capability in ["2G", "3G"]:
        if ns1_model in ["AGILENT_8960", "RS_CMU200", "RS_CMW500"]:
            bench_name = "NETWORK_SIMULATOR1"
            ns_model = ns1_model
        elif ns2_model in ["AGILENT_8960", "RS_CMU200", "RS_CMW500"]:
            bench_name = "NETWORK_SIMULATOR2"
            ns_model = ns2_model
        else:
            raise AcsBaseException(AcsBaseException.CRITICAL_FAILURE, "No 2G/3G Network Simulator available in Bench config!!!")
    if capability == "4G":
        if ns1_model in ["AGILENT_E6621A", "RS_CMW500"]:
            bench_name = "NETWORK_SIMULATOR1"
            ns_model = ns1_model
        elif ns2_model in ["AGILENT_E6621A", "RS_CMW500"]:
            bench_name = "NETWORK_SIMULATOR2"
            ns_model = ns2_model
        else:
            raise AcsBaseException(AcsBaseException.CRITICAL_FAILURE, "No 4G Network Simulator available in Bench config!!!")
    logger.info("Network simulator chosen is %s which is a %s" % (bench_name, ns_model))
    return bench_name
