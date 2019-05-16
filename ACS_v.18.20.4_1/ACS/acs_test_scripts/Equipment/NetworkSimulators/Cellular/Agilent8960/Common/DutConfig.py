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

:organization: INTEL QCTV
:summary: Agilent 8960 specific actions for DUT configuration
:since: 22/09/2014
:author: jduran4x
"""

from acs_test_scripts.TestStep.Utilities.Visa import VisaObject


class DutConfig(VisaObject):

    """
    Test mode 3G implementation for RS CMU200
    """

    def __init__(self, visa):
        """
        Constructor
        :type visa: VisaInterface
        :param visa: the PyVisa connection
        """
        VisaObject.__init__(self, visa)

    def set_dut_ip_address(self, ip_num, ip_addr):
        """
        Sets the DUT IP address.
        :type ip_num: integer
        :param ip_num: number of the IP address to set (1 to 4).
        :type ip_addr: str
        :param ip_addr: the IP address to set.
        """
        self._visa.write("CALL:MS:IP:ADDR%d '%s'" % (ip_num, ip_addr))

    def clear_all_dut_ip_addresses(self):
        """
        Clears all configured DUT IP addresses and sets them to " "
        during the clear process, the network simulator may complain because of no IP addresses assigned
        """
        # Agilent8960 has four DUT IP addresses
        for ip_num in range(1, 5):
            self.set_dut_ip_address(ip_num, "")
            self._logger.info("the network simulator ip_addresses are temporary cleared, "
                              "please ignore the previous warning displayed")

    def get_dut_ip_address(self, ip_num):
        """
        Gets the DUT IP address.
        :type ip_num: integer
        :param ip_num: the number of the IP address to return (1 to 4).
        :rtype: str
        :return: the IP address of the DUT.
        """
        return self._visa.query("CALL:MS:IP:ADDR%d?" % ip_num)

    def set_dut_primary_dns(self, ip_addr):
        """
        Sets DUT primary DNS address.
        :type ip_addr: str
        :param ip_addr: the IP address to set.
        """
        self._visa.write("CALL:MS:DNSS:PRIM:IP:ADDR '%s'" % ip_addr)

    def get_dut_primary_dns(self):
        """
        Gets DUT primary DNS address.
        :rtype: str
        :return: the IP address of the primary DNS of the DUT.
        """
        return self._visa.query("CALL:MS:DNSS:PRIM:IP:ADDR?")

    def set_dut_secondary_dns(self, ip_addr):
        """
        Sets DUT secondary DNS address.
        :type ip_addr: str
        :param ip_addr: the IP address to set.
        """
        self._visa.write("CALL:MS:DNSS:SEC:IP:ADDR '%s'" % ip_addr)

    def get_dut_secondary_dns(self):
        """
        Gets DUT secondary DNS address.
        :rtype: str
        :return: the IP address of the device under test secondary DNS
        """
        return self._visa.query("CALL:MS:DNSS:SEC:IP:ADDR?")
