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
:summary: virtual interface with gps network simulator
:since:18/01/2012
:author: ssavrimoutou
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException

# pylint: disable=W0613


class IGPSNetSim(object):

    """
    Virtual interface for gps network simulators
    """

    def init(self):
        """
        Initializes the equipment and establishes the connection.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def release(self):
        """
        Releases equipment resources and close connection.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_status(self):
        """
        Get the status of the equipment

        :rtype: str
        :return: Status returned by the equipment
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def send_command(self, command, check_status=None, max_retry=None, delay=None):
        """
        Send the command to the spirent. If a status is awaited and not received, try to send again the command.
        10 times failed command sending is failed.

        :type command: str
        :param command: String representing a well formed command.

        :type check_status: List of str
        :param check_status: String or a list of str representing the status to check once the command is sent

        :type max_retry: integer
        :param max_retry: Maximum retries before returning Failure status.

        :type delay: float
        :param delay: Delay to wait before reading the response

        :rtype: int
        :return: return Global.SUCCESS or Global.FAILURE if an error occurred
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def perform_full_preset(self):
        """
        Resets all equipment parameters to defaults.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_rf_power_level(self, power_level):
        """
        Set the RF power level

        :type power_level: integer
        :param power_level: the value RF power level to set
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def load_configuration_file(self, filename):
        """
        Loads equipment configuration/scenario from a filename.

        :type filename: str
        :param filename: the configuration file to load
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def start_test(self):
        """
        Start test on equipment using loaded file scenario.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def stop_test(self):
        """
        Stop test on equipment using loaded file scenario.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)
