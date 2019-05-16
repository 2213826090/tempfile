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
:summary: interface for temperature chamber equipment implementation
:since: 14/03/2012
:author: vgombert
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException

# pylint: disable=W0613


class ITemperatureChamber(object):

    """
    ITemperatureChamber class.
    """

    def init(self):
        """
        Initializes the equipment. Final state: the equipment is ready to use.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def release(self):
        """
        Releases the connection with the equipment and all resources allocated
        to its use.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_regulation(self, state):
        """
        toggle the regulation state, allowed the chamber
        to adjust and keep it's temperature to the wanted one.

        :param state: False to stop to regulate the temperature
                      True to start to regulate the temperature
        :type state: boolean
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_regulation(self):
        """
        Check the regulation state.

        :rtype: boolean
        :return: true if regulation is running, false otherwise
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_temperature(self, value):
        """
        Set the temperature.

        :param value: temperature value in Celsius.
        :type value: float or int
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_temperature(self):
        """
        Get the temperature.

        :rtype: float or int
        :return: temperature value in Celsius
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def is_connected(self):
        """
        check connection with the temperature chamber.

        :rtype: boolean
        :return: true if connection is established, false otherwise
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_measurement_unit(self):
        """
        Return the measurement unit used currently by this equipment.

        :rtype: Str
        :return: measurement unit
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)
