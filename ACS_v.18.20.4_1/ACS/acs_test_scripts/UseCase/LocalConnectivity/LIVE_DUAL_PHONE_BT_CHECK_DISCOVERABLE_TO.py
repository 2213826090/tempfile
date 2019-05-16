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
:summary: This file implements the LIVE_DUAL_PHONE_BT_CHECK_DISCOVERABLE_TO
:author: cmichelx
:since:03/08/2012
"""

import time
import math
from LIVE_DUAL_PHONE_BT_BASE import LiveDualPhoneBTBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class LiveDualPhoneBTCheckDiscoverableTO(LiveDualPhoneBTBase):

    """
    Live BT Check Discoverable TimeOut test.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LiveBTBase init function
        LiveDualPhoneBTBase.__init__(self, tc_name, global_config)

        # Read DISCOV_MODE from test case xml file
        self._discov_mode = \
            self._tc_parameters.get_param_value("DISCOV_MODE")

        # Read DISCOV_TIMEOUT from test case xml file
        self._discov_timeout = \
            self._tc_parameters.get_param_value("DISCOV_TIMEOUT")



#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # Call LiveBTBase set_up function
        LiveDualPhoneBTBase.set_up(self)

        # Format _discov_mode
        self._discov_mode = str(self._discov_mode).lower()

        # Format _discov_timeout
        self._discov_timeout = int(self._discov_timeout)

        if self._discov_mode not in ("on", "1", "off", "0"):
            msg = "Set wrong discov_mode. could only be on(1) or off(0)"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """

        LiveDualPhoneBTBase.run_test(self)

        # Flush bt scanned devices list.
        # Doing so makes sure if the DUT is in the list is because it's been found now
        self._bt_api2.flush_bt_scanned_devices()

        if self._discov_mode in ("on", "1"):
            # step 1: Enable discoverable mode
            start_time = int(time.time())
            self._bt_api.set_bt_discoverable('both', self._discov_timeout)
            start_time2 = math.ceil(time.time())
            delta_time = 1

            if self._discov_timeout != 0:
                self._logger.info("Start waiting for the end of %d sec timeout"
                                  % self._discov_timeout)
                # step 2: time < timeout, Check DUT is visible by 2nd phone
                while delta_time < self._discov_timeout:
                    time.sleep(1)
                    if not delta_time % 20:
                        self._logger.info("timeout in %d sec"
                                          % int(self._discov_timeout - delta_time))

                    if ((not delta_time % 50) and
                        (delta_time < self._discov_timeout) and
                            (not self._bt_api2.bt_find_device(self._phone1_addr))):
                        msg = "DUT is not visible"
                        self._logger.error(msg)
                        raise DeviceException(DeviceException.OPERATION_FAILED, msg)
                    delta_time = math.ceil(time.time() - start_time)

                # step 3: time > timeout, Check DUT no more visible
                delta_time = int(time.time() - start_time2)
                while delta_time < self._discov_timeout:
                    time.sleep(1)

                    delta_time = math.ceil(time.time() - start_time2)

                # Before scanning again (and expecting not to find the device)
                # flush bt scanned devices list.
                # Doing so makes sure if the DUT is in the list is because it's been found now
                self._bt_api2.flush_bt_scanned_devices()

                if self._bt_api2.bt_find_device(self._phone1_addr):
                    msg = "DUT is still visible"
                    self._logger.error(msg)
                    raise DeviceException(DeviceException.OPERATION_FAILED, msg)
            else:
                # Set screen always on and unlock phone (Since LLP , DUT need to be unlocked to be visible)
                self._phone_system_api.set_phone_screen_lock_on("on")
                self._phone_system_api.set_phone_lock(0)

                max_time = 300
                self._logger.info("timeout is to never")
                self._logger.info("Start waiting for %d sec" % max_time)
                # timeout = 0  equals never timeout
                # step 2: time < 5min, Check DUT is visible by 2nd phone
                while int(delta_time) < max_time:
                    time.sleep(1)
                    delta_time = int(time.time() - start_time)
                    if not int(delta_time) % 20:
                        self._logger.info("end of wait = %d sec"
                                          % int(max_time - delta_time))
                    if ((not delta_time % 50) and
                            (not self._bt_api2.bt_find_device(self._phone1_addr))):
                        msg = "DUT is not visible"
                        self._logger.error(msg)
                        raise DeviceException(DeviceException.OPERATION_FAILED, msg)

                # Before scanning again (and expecting to find the device)
                # flush bt scanned devices list.
                # Doing so makes sure if the DUT is in the list is because it's been found now
                self._bt_api2.flush_bt_scanned_devices()

                # step 3: Check after 5minutes that DUT is still visible
                if not self._bt_api2.bt_find_device(self._phone1_addr):
                    msg = "DUT is not visible"
                    self._logger.error(msg)
                    raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        elif self._discov_mode in ("off", "0"):
            self._bt_api.set_bt_discoverable('none', 0)

            time.sleep(self._wait_btwn_cmd)
            if self._bt_api2.bt_find_device(self._phone1_addr):
                msg = "phone in wrong discoverable mode"
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        return Global.SUCCESS, "No errors"
