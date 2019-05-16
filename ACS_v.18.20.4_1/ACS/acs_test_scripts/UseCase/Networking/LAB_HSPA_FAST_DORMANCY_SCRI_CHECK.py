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
:summary: This file implements HSPA Fast dormancy feature validation
:author: hbianx
:since:27/03/2013
"""

import time
import os
from LAB_HSPA_BASE import LabHspaBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class LabHspaFastDormancyScriCheck(LabHspaBase):

    """
    Check HSPA FAST DORMANCY SCRI feature
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LAB_HSPA_BASE Init function
        LabHspaBase.__init__(self, tc_name, global_config)

        self._ns_fast_dormancy = "enable"
        self._timeout_between_commands = 5

        # Read the ftp file name from UseCase xml Parameter
        if self._direction == "DL":
            self._ftp_filename = os.path.join(
                self._ftp_path,
                self._tc_parameters.get_param_value("DL_FILENAME"))
        elif self._direction == "UL":
            # Read the UL_FILE value from UseCase xml Parameter
            self._ftp_filename = os.path.join(
                self._ftp_path,
                self._tc_parameters.get_param_value("UL_FILENAME"))

        # Read the XFER_TIMEOUT from UseCase xml Parameter
        self._xfer_timeout = \
            self._tc_parameters.get_param_value("XFER_TIMEOUT")
        if self._xfer_timeout is not None and \
                str(self._xfer_timeout).isdigit():
            self._xfer_timeout = int(self._xfer_timeout)
        else:
            self._xfer_timeout = None
        # Timeout to wait for RRC switch from RRC_DCH to Idle
        self._rrc_idle_timeout = \
            self._tc_parameters.get_param_value("RRC_IDLE_TIMEOUT")
        if self._rrc_idle_timeout is not None and \
                str(self._rrc_idle_timeout).isdigit():
            self._rrc_idle_timeout = int(self._rrc_idle_timeout)
        else:
            self._rrc_idle_timeout = None
        # inactivity timer for RRC state change
        self._rrc_inactivity_timer = \
            self._tc_parameters.get_param_value("RRC_INACTIVITY_TIMER")
        if self._rrc_inactivity_timer is not None and \
                str(self._rrc_inactivity_timer).isdigit():
            self._rrc_inactivity_timer = int(self._rrc_inactivity_timer)
        else:
            self._rrc_inactivity_timer = None

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Initialize the test
        """

        LabHspaBase.set_up(self)

        if self._xfer_timeout is None:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                   "XFER_TIMEOUT should be int")
        if self._rrc_idle_timeout is None:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                   "RRC_IDLE_TIMEOUT should be int")
        if self._rrc_inactivity_timer is None:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                   "RRC_INACTIVITY_TIMER should be int")
        # We should stop all the applications which reques the data from the netwrok
        app_list = ["com.google.android.youtube"]
        for app in app_list:
            cmd = "adb shell am force-stop %s" % app
            self._device.run_cmd(cmd, 3)

        return Global.SUCCESS, self._error.Msg

    def run_test(self):
        """
        Execute the test
        """
        test_result = Global.FAILURE
        # Call LAB_HSPA_BASE Run function
        LabHspaBase.run_test(self)

        time.sleep(self._wait_btwn_cmd)

        self._networking_api.ftp_xfer(self._direction,
                                      self._server_ip_address,
                                      self._username,
                                      self._password,
                                      self._ftp_filename,
                                      self._xfer_timeout,
                                      self._device.get_ftpdir_path())

        # TODO: For the moment, we can't stop all the google data services
        # and ntp service, we have to wait the service stop itself
        time.sleep(60)

        timer = 0
        # PDP should always stay activate and RRC should pass to IDLE state before timeout
        while timer < self._rrc_idle_timeout:
            data_connection_status = self._ns_data_3g.get_data_connection_status()
            rrc_states = self._ns_data_3g.get_rrc_states()
            states_msg = "data_connection_status: %s rrc_states: %s after %d " % \
                (data_connection_status,
                 rrc_states,
                 timer)
            self._logger.info(states_msg)
            if data_connection_status != "PDP_ACTIVE":
                self._error.Msg = "Failed to reach PDP_ACTIVE data state"
                raise DeviceException(DeviceException.OPERATION_FAILED, self._error.Msg)
            if rrc_states == "IDLE":
                self._error.Msg = states_msg
                test_result = Global.SUCCESS
                break
            time.sleep(1)
            timer += 1

        return test_result, self._error.Msg
