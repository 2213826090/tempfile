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
:summary: data 3G implementation for RS CMU200 cellular network simulator
:since: 05/04/2011
:author: ymorel
"""

import time
from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Interface.IData3G import IData3G


class Data3G(IData3G):

    """
    Data 3G implementation for RS CMU200
    """

    def __init__(self, root):  # pylint: disable=W0231
        """
        Constructor
        :type root: weakref
        :param root: a weak reference on the root class (RsCmu200)
        """
        IData3G.__init__(self)
        self.__root = root

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

    def get_root(self):
        """
        Get the root object of the equipment
        :rtype: RsCmu200
        :return: the root object of the equipment
        """
        return self.__root

    def get_logger(self):
        """
        Gets the logger
        """
        return self.get_root().get_logger()

    def check_data_connection_state(self, state, timeout=0, blocking=True, cell_id=None):
        """
        Checks that the data connection is set at the required state
        before the given timeout. If timeout is <= 0, only one test is performed.
        :raise TestEquipmentException: the required status has not been reached before the timeout
        :type state: str
        :param state: the expected state. Possible values:
            - "OFF"
            - "IDLE"
            - "ATT"
            - "CIPR"
            - "CONN"
            - "DIPR"
        :type timeout: integer
        :param timeout: allowed time to reach expected state
        :type blocking: boolean
        :param blocking: boolean to know if the function raises an error
        or simply return true or false if the status is reached or not
        :rtype: boolean
        :return: True if state is reached, else returns False
        :type cell_id : str
        :param cell_id: cell used for the test. Possible values:
            - "A"
            - "B"
        .. warning:: This parameter is only used in 4G (LTE)
        """

        attached = False
        timer = timeout
        self.get_logger().info(
            "Check data connection is %s before %d seconds",
            state,
            timeout)

        current_state = self.__root.query_command("SENSe:SIGNalling:PSWitched:STATe?", "WCDMA")

        while (timer > 0) and (current_state != state):
            if state == "ATT" and current_state == "CONN":
                attached = True
                break
            time.sleep(1)
            current_state = \
                self.__root.query_command("SENSe:SIGNalling:PSWitched:STATe?", "WCDMA")
            timer -= 1

        if current_state != state and not attached:
            if blocking:
                # Failed to reach desired state
                msg = "Failed to reach %s data state!" % state
                self.get_logger().error(msg)
                raise TestEquipmentException(TestEquipmentException.TIMEOUT_REACHED, msg)
            else:
                # Failed to reach desired state (Test failed no TestEquipmentException raised)
                self.get_logger().error("Failed to reach %s data state!", state)

            return False
        else:
            self.get_logger().info("Data connection is %s and has been reached in %d seconds" % (current_state,
                                                                                                 timeout - timer))
            return True

    def get_network_type(self):
        """
        Returns the expected network type
        :rtype: str
        :return: the expected network type
        """
        network_type = "UNKNOWN"
        return network_type
