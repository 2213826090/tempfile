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
:summary: virtual interface of LTE data functionalities for LTE cellular network
simulators
:since: 27/04/2012
:author: lvacheyx
.. note:: BZ3071 - PING MO over LTE network
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException

# pylint: disable=W0613


class IData4G(object):

    """
    IData4G class: virtual interface of LTE data functionalities for LTE cellular
    network simulators.
    """
    def __getattr__(self, attr):
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED, "%s" % attr)

    def clear_all_dut_ip_addresses(self):
        """
        Clears all configured DUT IP addresses and sets them to " "
        """
        # Agilent8960 has two DUT IP addresses
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_dut_primary_dns(self, ip_addr):
        """
        Sets DUT primary DNS address.
        :type ip_addr: str
        :param ip_addr: the IP address to set.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_dut_secondary_dns(self, ip_addr):
        """
        Sets DUT secondary DNS IP address.
        :type ip_addr: str
        :param ip_addr: the IP address to set.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def ue_detach(self):
        """
        Sends the specified DETACH message contained in the scenario

        .. note:: This command is relevant only for AgilentE6621A
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_network_type(self):
        """
        Returns the expected network type
        :rtype: str
        :return: the expected network type
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_connection_setup(self, conn_setup):
        """
        To initiate a connection setup
        :type conn_setup: str
        :param conn_setup: the state expected. Possible values :
                - "CONN" : Initiate a mobile terminated connection setup
                - "HAND" : Initiate a handover
                - "DET" : Initiate a Detach procedure
                - "SMS" : Send a SMS
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def configure_eps_bearer_config_2(self, apn, ip_version, ip_addr):
        """
        Sets the APN, IP version and IP address for the Default EPS Bearer Config #2

        :type apn: str
        :param apn: The Access Point Name to be configured on equipment
        :type ip_version: str
        :param ip_version: version to use when establishing the connection
        Possible values: IPV4, IPV6, IPV4V6
        :type ip_addr: str
        :param ip_addr: the value of the IPV4 or IPV6 address

        :rtype: None
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)
