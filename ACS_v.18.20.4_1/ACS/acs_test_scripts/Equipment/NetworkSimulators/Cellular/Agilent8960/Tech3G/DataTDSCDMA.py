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
:summary: data 3G TD-SCDMA implementation for Agilent 8960
:since: 24/07/2014
:author: mbrisbax
"""

import time
from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Agilent8960.Tech3G.Data3G import Data3G


class DataTDSCDMA(Data3G):
    """
    Data 3G TD-SCDMA implementation for Agilent 8960
    """

    def get_data_connection_status(self):
        """
        Gets data connection status.
        :rtype: str
        :return: the data connection status. Possible returned values:
            - "OFF": Default returned value
            - "IDLE"
            - "ATTACHING"
            - "ATTACHED"
            - "ATTACHED_INCOMPLETE"
            - "DETACHING"
            - "IDLE"
            - "OFF" => Default returned value
            - "DETACHED"
            - "PDP_ACTIVATING"
            - "PDP_ACTIVE"
            - "PDP_DEACTIVATING"
        """
        current_state = "IDLE"
        gmm_state = self.get_root().query_command("CALL:STATus:GMM?")
        data_state = self.get_root().query_command("CALL:STATus:DATA?")
        if gmm_state == "ATT" and data_state == "IDLE":
            current_state = "ATTACHED"
        elif gmm_state == "ATT" and data_state == "PDP":
            current_state = "PDP_ACTIVE"
        self.get_logger().debug("Data state is %s " % current_state)
        return current_state

    def check_data_connection_state(self, state, timeout=0, blocking=True, cell_id=None):
        """
        Checks that the data connection is set at the required sate
        before the given timeout. If timeout is <= 0, only one test is performed.
        :raise TestEquipmentException: the required status has not been reached before the timeout
        :type state: str
        :param state: the expected state. Possible values:
            - "ATTACHED"
            - "PDP_ACTIVE"
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
        :attention: This parameter is only used in 4G (LTE)
        """
        attached = False
        timer = timeout
        self.get_logger().info(
            "Check data connection is %s before %d seconds",
            state,
            timeout)
        current_state = self.get_data_connection_status()

        while (timer > 0) and (current_state != state):
            if state == "ATTACHED" and current_state == "PDP_ACTIVE":
                attached = True
                break
            time.sleep(1)
            current_state = self.get_data_connection_status()
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
            self.get_logger().info("Data connection is %s and has been reached in %d seconds" % (current_state, int(timeout) - int(timer)))
            return True

    def get_network_type(self):
        """
        Returns the expected network type

        :rtype: str
        :return: the expected network type
        """
        # GPRS384 |PSDHsdpa64 |PHSPa
        # Read the GPRS RAB value
        rab = self.get_root().query_command("CALL:SERVice:GPRS:RAB?")
        self.get_logger().debug("GPRS Radio Access Bearer is %s", rab)

        network_type = ""
        # For network types, Check if GPRS Radio Access Bearer is set for WCDMA, HSDPA, HSUPA or HSPA
        # Five different RAB for WCDMA
        # Then the RBTEST Channel type gives the network type between HSUPA, HSPA and HSPA Plus
        if (rab == "GPRS384"):
            network_type = "WCDMA"

        # Check if HSDPA category is reported
        elif (rab == "PSDH64") or (rab == "PHSP"):
            network_type = "HSPA"

        else:  # network RAT is wrong
            msg = " Network RAT (%s) does not match any known equipment RAT !" \
                % network_type
            self.get_logger().warning(msg)

        return network_type
