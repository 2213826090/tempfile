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
:summary: Cell 4G implementation for Anritsu M8475A using visa interface
:since: 20/03/2015
:author: gcharlex
"""
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.AnritsuM8475A.Tech4G.Data4G import Data4G
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Interface.ICell4G import ICell4G
from acs_test_scripts.TestStep.Utilities.Visa import VisaObject
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.AnritsuM8475A.Common.CellCommon import CellCommon


class Cell4G(ICell4G, CellCommon):

    """
    Cell LTE implementation for Anritsu M8475A
    """

    def __init__(self, visa, number=1):
        """
        Constructor
        :type visa: visaInterface
        :param visa: the PyVisa connection

        :type number: int
        :param number: cell number
        """
        CellCommon.__init__(self, visa, number)
        self.__data = Data4G(visa)
        self.tech = "LTE"

    def __del__(self):
        """
        Destructor
        """
        VisaObject.__del__(self)

    def get_data(self):
        """
        Access to LTE data interface.
        :rtype: IData4G
        :return: the LTE data object.
        """
        return self.__data

    def is_dut_registered(self, dut_imsi=None):
        """
        Test if DUT is synchronized with equipment
        :type dut_imsi: str
        :param dut_imsi: IMSI retrieved from CDK.
        :rtype: boolean
        :return: true if the DUT is synchronized with the equipment, false otherwise
        """
        gpib_cmd = "BTSCONNSTAT? RegistrationPs,{0}".format(self._name)
        output = self._visa.query(gpib_cmd)
        return output == "ON"
