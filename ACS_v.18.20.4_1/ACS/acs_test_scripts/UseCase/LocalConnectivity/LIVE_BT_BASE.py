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
:summary: This file implements the LIVE BT Base UC
:since: 29/09/2010
:author: skgurusX
"""
import time

from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.DeviceException import DeviceException


class LiveBTBase(UseCaseBase):

    """
    Live BT Test base class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        UseCaseBase.__init__(self, tc_name, global_config)

        # Get UECmdLayer
        self._bt_api = self._device.get_uecmd("LocalConnectivity")

        # get networking cmd.
        self._networking_api = self._device.get_uecmd("Networking")

        # Should we set flight mode?
        self._original_flight_mode = 0
        self._use_flightmode = self._tc_parameters.get_param_value("FLIGHT_MODE")
        if str(self._use_flightmode).lower() in ["1", "on", "true", "yes"]:
            self._use_flightmode = 1
        else:
            self._use_flightmode = 0

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Initialize the test
        """
        UseCaseBase.set_up(self)

        # Get original flight mode
        self._original_flight_mode = self._networking_api.get_flight_mode()
        # Flight mode
        if self._use_flightmode != self._original_flight_mode:
            self._networking_api.set_flight_mode(self._use_flightmode)
            time.sleep(self._wait_btwn_cmd)
            if self._use_flightmode != self._networking_api.get_flight_mode():
                msg = "set flight mode failure"
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        self._logger.info("Reset the Bluetooth adapter")
        self._bt_api.bt_reset_device()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def tear_down(self):
        """
        End and dispose the test
        """
        UseCaseBase.tear_down(self)

        time.sleep(self._wait_btwn_cmd)

        # Recover to initial phone mode
        self._logger.info("Recover DUT to original flight mode")
        self._networking_api.set_flight_mode(self._original_flight_mode)
        time.sleep(self._wait_btwn_cmd)

        self._logger.info("Close Adapter devices")
        self._bt_api.set_bt_power("off")

        time.sleep(self._wait_btwn_cmd)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def _exception_reader(self, exception_queue, threads_running, timeout=3600):
        """
        Read the queue gave in parameter and raise an exception when a message arrive.
        The exception is raised only when all threads are dead.
        The function loops until all threads are alive or the timeout is done.

        :type exception_queue : Queue Object
        :param exception_queue : queue which receive message from threads
        :type threads_running : Thread list Object
        :param threads_running : list of all threads running during the reading
        type timeout : integer
        :param timeout : time limit before the function stops itself
        """

        current_exception = None
        start = time.time()

        while (start + timeout) > time.time():
            time.sleep(0.5)
            if current_exception is None and not exception_queue.empty():
                current_exception = exception_queue.get(True, 3)

            threads_alive = False
            for element in threads_running:
                if element is not None and element.isAlive():
                    threads_alive = True
                    break
            if not threads_alive:
                if current_exception is not None:
                    self._logger.error(current_exception.get_specific_message())
                    raise DeviceException(current_exception.get_generic_error_message(),
                                          current_exception.get_specific_message())
                break
