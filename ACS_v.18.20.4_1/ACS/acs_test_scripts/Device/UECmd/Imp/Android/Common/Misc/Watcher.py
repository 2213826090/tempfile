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
:summary: This file implements the Watcher UEcmd for Android phone
:since: 22/02/2013
:author: jduran4x
"""

from threading import Timer
from acs_test_scripts.Device.UECmd.Imp.Android.Common.Base import Base
from acs_test_scripts.Device.UECmd.Interface.Misc.IWatcher import IWatcher
from ErrorHandling.AcsConfigException import AcsConfigException


class Watcher(Base, IWatcher):

    """
    :summary: Watcher UEcommands operations for Android platforms
    using an C{Intent} based communication to the I{DUT}.
    """

    def __init__(self, device):
        """
        Constructor.
        """
        Base.__init__(self, device)
        IWatcher.__init__(self, device)

        self.__max_number_of_pdp_contexts = 0
        self.__pdpcontext_timer = None
        self.__pdpcontext_period = 1
        self.__is_pdpcontext_timer_started = False
        self._rmnet_device_path = "/sys/devices/virtual/net/"

    def start_watch_pdp_contexts(self, period):
        """
        Starts the watcher of Pdp Context number

        :param: period
        :type: int

        :return: None
        :raise AcsBaseException: if the command failed
        """
        self._logger.info("Starting PDP context watcher...")
        if not str(period).isdigit() or period < 0:
            error_msg = "Cannot start PDP context watcher ! (period parameter shall be a integer > 0)"
            raise AcsConfigException(AcsConfigException.INSTANTIATION_ERROR, error_msg)

        if self.__pdpcontext_timer:
            try:
                if self.__pdpcontext_timer.isAlive():
                    self.__pdpcontext_timer.cancel()
            except Exception as ex:  # pylint: disable=W0703
                self._logger().debug("Cannot start PDP context watcher ! ({0})".format(ex))
            finally:
                self.__aplog_timer = None

        # Initialize the pull period for other occurrences
        self.__pdpcontext_period = int(period)

        # Start the application log timer
        self.__start_pdpcontext_timer(1)

    def __start_pdpcontext_timer(self, period):
        """
        Starts the watcher of Pdp Context number

        :param: period
        :type: int

        :return: None
        """

        try:
            # Start timer to retrieve pdp context number
            self.__pdpcontext_timer = Timer(period, self.__retrieve_pdp_context_number, [True])
            self.__pdpcontext_timer.name = "Watcher:PDPContextWatcher"
            self.__pdpcontext_timer.daemon = True
            self.__pdpcontext_timer.start()

            self.__is_pdpcontext_timer_started = True

        except Exception as ex:  # pylint: disable=W0703
            self._logger.debug("Cannot start PDP context watcher ! ({0})".format(ex))

    def __retrieve_pdp_context_number(self, relaunch_timer=False):
        """
        Fetch application logs from devices, if any.
        """
        try:
            output_cmd = self._exec("adb shell ls {0} | grep rmnet".format(self._rmnet_device_path))
            if output_cmd:
                current_pdp_context = len(output_cmd.splitlines())

                if current_pdp_context > self.__max_number_of_pdp_contexts:
                    self.__max_number_of_pdp_contexts = current_pdp_context

            # Relaunch timer if needed
            if self.__is_pdpcontext_timer_started and relaunch_timer:
                self.__start_pdpcontext_timer(self.__pdpcontext_period)

            elif self.__pdpcontext_timer and self.__pdpcontext_timer.isAlive():
                self.__pdpcontext_timer.cancel()
                self.__pdpcontext_timer = None

        except (KeyboardInterrupt, SystemExit):
            # Skip pulling and stop properly the application log mechanism
            self._logger.debug("Received ctrl-c when retrieving PDP context number !")
            self.stop_watch_pdp_contexts()
            raise

    def stop_watch_pdp_contexts(self):
        """
        Stops the watcher of Pdp Context number

        :return: None
        :raise AcsBaseException: if the command failed
        """
        self._logger.info("Stopping PDP context watcher...")

        # Disable timer to retrieve application logs
        try:
            if self.__pdpcontext_timer and self.__pdpcontext_timer.isAlive():
                self.__pdpcontext_timer.cancel()
            self.__pdpcontext_timer = None
        except Exception as ex:  # pylint: disable=W0703
            self._logger().debug("Cannot stop PDP context watcher ! ({0})".format(ex))

        self.__is_pdpcontext_timer_started = False

    def get_pdp_contexts_number(self):
        """
        retrieves the max number of pdp contexts activated simultaneously

        :rtype: int
        :return: the number of pdp contexts (i.e. rmnet devices)
        :raise AcsBaseException: if the command failed
        """
        # In case the number of pdp contexts is equal to 0, try to get it.
        if self.__max_number_of_pdp_contexts == 0:
            self.__retrieve_pdp_context_number()

        return self.__max_number_of_pdp_contexts
