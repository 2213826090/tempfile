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
:summary: virtual interface for NFC robot
:since:09/10/2012
:author: lpastor
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException

# pylint: disable=W0613


class INFCRobot(object):

    """
    Virtual interface for NFC robot
    """

    def positioning(self, xCoordinate, yCoordinate, zCoordinate, timer):
        """
        Change robot position
        :type xCoordinate: str
        :param xCoordinate: X-axis coordinate. Use "null" to move along Z axis
        :type yCoordinate : str
        :param yCoordinate: Y-axis coordinate. Use "null" to move along Z axis
        :type zCoordinate : str
        :param zCoordinate: Z-axis coordinate. Use "null" to move in XY plan
        :type timer : str
        :param timer: time in seconds to stay in position before coming back at origin. Use "null" to keep the position
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def retrieve_position(self):
        """
        return a dictionnary with the current robot position
        :return: position : dictionnary
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)
