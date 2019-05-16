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
:summary: common methods to cell 2G / 3G for Agilent 8960 cellular network simulator
:since: 24/04/2015
:author: gcharlex
"""
from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.TestStep.Utilities.Visa import VisaObject


class CellCommon(VisaObject):

    """
    Cell implementation for Anritsu M8475A
    """

    MAX_LAC_VALUE = 65535

    def __init__(self, visa, number=1):
        """
        Constructor
        :type visa: visaInterface
        :param visa: the PyVisa connection

        :type number: int
        :param number: cell number
        """
        VisaObject.__init__(self, visa)
        self._data = None
        self._name = "BTS" + str(number)

    def __del__(self):
        """
        Destructor
        """
        VisaObject.__del__(self)
        del self._data

    def get_cell_name(self):
        """
        Return cell name
        :rtype: str
        :return: cell name
        """
        return self._name

    def __error_check(self, err, msg):
        """
        Error checking and warning reporting
        :raise TestEquipmentException: if err < 0
        """
        if err < 0:
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, msg)
        elif err > 0:
            self.get_logger().warning(msg)

    def get_logger(self):
        """
        Gets the logger of the equipment, i.e root's one
        """
        return self._logger

    def get_data(self):
        """
        Access to 2G data interface.
        :rtype: IData2G
        :return: the 2G data object.
        """
        return self._data
