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
:summary: This file is the implementation of
            LIVE_SYSTEM_MANAGE_TIME_AUTO_UPDATE use case
:since: 04/17/2013
:author: jreynaux
"""

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global, str_to_bool
from datetime import datetime
import time
from ErrorHandling.AcsConfigException import AcsConfigException


class LiveSystemManageTimeAutoUpdate(UseCaseBase):

    """
    Class LiveSystemManageTimeAutoUpdate.
    """

    DEFAULT_YEAR = "2000"
    DEFAULT_MOUTH = "01"
    DEFAULT_DAY = "01"
    DEFAULT_HOUR = "00"
    DEFAULT_MINUTES = "00"
    DEFAULT_SECONDS = "00"

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        self._initial_pdp_context_status = None
        self._use_given_datetime = False

        (self._year, self._month, self._day) = ("0", "0", "0")
        (self._hour, self._minutes, self._seconds) = ("0", "0", "0")

        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # Get TC Parameters (optionals)
        self._test_date = self._tc_parameters.get_param_value("TEST_DATE")
        self._test_time = self._tc_parameters.get_param_value("TEST_TIME")
        self._time_to_wait = \
            self._tc_parameters.get_param_value("TIME_TO_WAIT",
                                                default_value=5,
                                                default_cast_type=int)

        # Allowed time gap in minutes
        self._time_gap = int(self._tc_parameters.get_param_value("TIME_GAP"))

        # Read the parameter indicating whether we shall deactivate
        # data during the test or not.
        deactivate_data = self._tc_parameters.\
            get_param_value("DEACTIVATE_DATA")
        if deactivate_data not in ("", None):
            self._deactivate_data = str_to_bool(deactivate_data)
        else:
            self._deactivate_data = False

        # Get UECmdLayer
        self._phone_system_api = self._device.get_uecmd("PhoneSystem")
        self._system_api = self._device.get_uecmd("System")
        self._networking_api = self._device.get_uecmd("Networking")

        self._initial_flight_mode_state = None

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Execute the test
        """
        # Call UseCase base setup function
        UseCaseBase.set_up(self)

        # Recording initial state before starting the test to put it back when test is over
        self._initial_flight_mode_state = self._networking_api.get_flight_mode()

        # Set flight mode to wanted state (OFF, 0)
        self._networking_api.set_flight_mode("off")

        # Deactivate data if requested
        if self._deactivate_data:
            # First store the initial PDP context status
            # pylint: disable=W0212
            self._initial_pdp_context_status = \
                self._networking_api._get_pdp_context_status()
            # pylint: enable=W0212
            # Check whether we had to change the PDP context status
            if self._initial_pdp_context_status in ("0", "1"):
                # Then deactivate PDP context status
                self._networking_api.deactivate_pdp_context()

        # Getting usable date and time
        if self._test_date not in ("", None):
            self._logger.info("Getting usable date from TC parameters")
            (self._year, self._month, self._day) = \
                self._get_date_time_from_input(self._test_date)
            self._use_given_datetime = True
        else:
            self._logger.info("Using default date.")
            (self._year, self._month, self._day) = (self.DEFAULT_YEAR,
                                                    self.DEFAULT_MOUTH,
                                                    self.DEFAULT_DAY)

        if self._test_time not in ("", None):
            self._logger.info("Getting usable time from TC parameters")
            (self._hour, self._minutes, self._seconds) = \
                self._get_date_time_from_input(self._test_time)
            self._use_given_datetime = self._use_given_datetime and True
        else:
            self._logger.info("Using default time.")
            (self._hour, self._minutes, self._seconds) = (self.DEFAULT_HOUR,
                                                          self.DEFAULT_MINUTES,
                                                          self.DEFAULT_SECONDS)

        # Get current auto_time and set it to false if Enabled
        if self._phone_system_api.get_auto_time() is True:
            self._phone_system_api.set_auto_time(0)

        # Configure a date on the device
        input_date = self._year + "/" + self._month + "/" + self._day
        input_time = self._hour + ":" + self._minutes + ":" + self._seconds
        self._logger.info("Set Initial date and time to: " + input_date + " " + input_time)
        self.configure_time(self._year, self._month, self._day,
                           self._hour, self._minutes, self._seconds)

        time.sleep(self._wait_btwn_cmd)

        # Check initial date and time are correctly set before enabling time auto update
        # Get the date and time on DUT
        (ye, mo, da, ho, mi, se) = self._phone_system_api.get_current_time()
        # Mind the Gap
        (status, output) = self._check_date_time(ye, mo, da, ho, mi, se)

        # Log if failure
        if status == Global.FAILURE:
            output = "Set Initial date and time Failed!"
            self._logger.error(output)

        return status, output

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        # Call UseCase base Run function
        UseCaseBase.run_test(self)

        time.sleep(self._wait_btwn_cmd)

        self._logger.info("Run Manage Time test ...")

        # Set auto_time on dut
        self._phone_system_api.set_auto_time(1)

        # Wait a while to left dut update its date and time
        self._logger.info("Wait for %d seconds" % self._time_to_wait)
        time.sleep(self._time_to_wait)

        # Get current date and time if needed.
        self._logger.info("Retrieve current date and time from host ...")
        (self._year, self._month, self._day) = \
            self._get_current_host_date()

        (self._hour, self._minutes, self._seconds) = \
            self._get_current_host_time()

        # Get the date and time on DUT
        (ye, mo, da, ho, mi, se) = self._phone_system_api.get_current_time()

        # Mind the Gap
        (status, output) = self._check_date_time(ye, mo, da, ho, mi, se)

        return status, output

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        # Call use case base tear_down function
        UseCaseBase.tear_down(self)

        # Activate data if it has been disabled previously
        # (restore previous phone state).
        if self._deactivate_data:
            # Check whether we had to change the PDP context status
            if self._initial_pdp_context_status in ("0", "1"):
                # Activate PDP context
                self._networking_api.activate_pdp_context()

        time.sleep(self._wait_btwn_cmd)

        # Set flight mode back to initial state
        if self._initial_flight_mode_state not in ("", None):
            self._networking_api.set_flight_mode(self._initial_flight_mode_state)

        self._logger.info("Quit Manage Time auto-update test...")

        return self._error.Code, "No errors"

    @staticmethod
    def _get_date_time_from_input(formatted_datetime):
        """
        Return the date or time as list from given formatted input.

        :rtype: tuple
        :return: Each element of splitted input str.
        """
        splitted_date = formatted_datetime.split(".")
        if len(splitted_date) is not 3:
            raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG,
                                     "Invalid given DateTime in UC parameter")
        return splitted_date[0], splitted_date[1], splitted_date[2]

    def _get_current_host_date(self):
        """
        Return the date as list from current.

        :rtype: tuple
        :return: The year, month and day of month of now.
        """
        now = datetime.now()
        self._logger.debug("Getting current date from: " + str(now))
        return str(now.year), str(now.month), str(now.day)

    def _get_current_host_time(self):
        """
        Return the date as list from given formatted input.

        :rtype: tuple
        :return: The hour, minutes and second of now.
        """
        now = datetime.now()
        self._logger.debug("Getting current time from: " + str(now))
        return str(now.hour), str(now.minute), str(now.second)

    def _check_date_time(self, year, month, day, hour, mins, seconds):
        """
        Check that the given date and time are relevant
        according the stored one

        :type year:str
        :param year: The year to be used for check.

        :type month:str
        :param month: The month to be used for check.

        :type day:str
        :param day: The day to be used for check.

        :type hour:str
        :param hour: The hour to be used for check.

        :type mins:str
        :param mins: The minutes to be used for check.

        :type seconds:str
        :param seconds: The seconds to be used for check.

        :rtype: tuple
        :return: The status and output log
        """
        reference_date = self._year + "." + self._month + "." + self._day
        self._logger.info("Reference date: %s" % str(reference_date))

        input_date = year + "." + month + "." + day
        self._logger.info("DUT given date: %s" % str(input_date))

        if datetime.strptime(reference_date, "%Y.%m.%d") != \
                datetime.strptime(input_date, "%Y.%m.%d"):
            return Global.FAILURE, "The date are not similar"

        reference_time = self._hour + "." + self._minutes + "." + self._seconds
        self._logger.info("Reference time: %s" % str(reference_time))

        input_time = hour + "." + mins + "." + seconds
        self._logger.info("DUT given time: {0}".format(input_time))

        delta = datetime.strptime(input_time, "%H.%M.%S") \
            - datetime.strptime(reference_time, "%H.%M.%S")
        delta_in_seconds = delta.total_seconds()
        self._logger.debug("Delta of {0} seconds (tolerance of {1}s)".format(delta_in_seconds, self._time_gap))

        if delta_in_seconds < 0:
            self._logger.warning("DUT time is earlier than reference time ! ({0}s)".format(delta_in_seconds))

        # If delta is less than gap and positive (because reference time retrieved BEFORE dut time), it OK
        if abs(delta_in_seconds) < self._time_gap:
            self._logger.debug("The compared date and time are similar !")
        else:
            return (Global.FAILURE, "The time is not the same as now " +
                    "(at %s seconds gap)" % str(self._time_gap))

        return Global.SUCCESS, "The compared date and time are similar"

    def configure_time(self, year="1970", month="01", day="01",
                       hour="00", minutes="00", seconds="00"):
        """
        Configure the DUT date and time with given values.

        :type year: str
        :param year: The year to be set on DUT

        :type month: str
        :param month: The month to be set on DUT

        :type day: str
        :param day: The day to be set on DUT

        :type hour: str
        :param hour: The hour to be set on DUT

        :type minutes: str
        :param minutes: The minutes to be set on DUT

        :type seconds: str
        :param seconds: The seconds to be set on DUT

        :rtype: None
        """
        structure_datetime = datetime(int(year), int(month), int(day), int(hour), int(minutes), int(seconds))
        self._system_api.set_date_and_time(structure_datetime)
