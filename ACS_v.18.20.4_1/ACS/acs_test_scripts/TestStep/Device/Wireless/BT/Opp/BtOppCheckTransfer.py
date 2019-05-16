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
:summary: This file implements a Test Step that Check OPP transfer
:since:18/12/2013
:author: fbongiax
"""

import time
from UtilitiesFWK.Utilities import split_and_strip
from acs_test_scripts.TestStep.Device.Wireless.BT.Constants import Constants
from acs_test_scripts.TestStep.Device.Wireless.BT.Base import BtBase
from acs_test_scripts.TestStep.Device.Wireless.BT.Opp.BtOppTransfer import OppTransfer
from acs_test_scripts.Utilities.CommunicationUtilities import ConfigsParser


class BtOppCheckTransfer(BtBase):
    """
    Implements the test step to check OPP transfer
    """
    _DEFAULT_OFFSET_TIMEOUT_SECS = 120
    _DEFAULT_POLL_SLEEP_SECS = 1

    _ALL_COMPLETED = "all_completed"
    _ANY_STARTED = "any_started"
    _ANY_CANCELLED = "any_cancelled"

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Constructor
        """

        BtBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._files = {}
        self._num_expected_files = 0
        self._time_out_offset = self._DEFAULT_OFFSET_TIMEOUT_SECS
        self._poll_sleep = self._DEFAULT_POLL_SLEEP_SECS
        # If no time out related arguments are passed, timeout by default is time_out_offset
        self._time_out = self._time_out_offset
        self._throughput_targets = None

        if not self._pars.expected_state:
            self._pars.expected_state = self._ALL_COMPLETED

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """

        try:
            if self._pars.throughput_margin and int(self._pars.throughput_margin) != 0:  # get target if needed
                self._throughput_targets = ConfigsParser("BT_Throughput_Targets").parse_bt_targets(
                    self._device.get_phone_model(), "OPP")
            return self._do_run(context)
        finally:
            self._clean_up()

    def _clean_up(self):
        """
        Removes file transfer info
        """
        self._files.clear()

    def _do_run(self, context):
        """
        Executes the run logic
        """
        BtBase.run(self, context)

        self._setup_timeout_value()

        self._logger.info("File transfer is supposed to complete in %d seconds" % self._time_out)

        file_names = split_and_strip(self._pars.files, Constants.FILE_NAME_SEPARATOR)
        self._num_expected_files = len(file_names)

        self._logger.info("Expect to receive %d files" % self._num_expected_files)

        self._reach_expected_state()

        self._save_file_size_if_needed(context)
        self._save_file_chksum_if_needed(None, file_names, context)
        self._validate_throughput_if_needed()


    def  _setup_timeout_value(self):
        """
        Sets up the time out based on the different possibilities
        """
        if self._pars.expected_files_size is not None:
            # Calculate timeout based on total size of files to be transferred (passed as parameter)
            self._time_out = int(self._pars.expected_files_size) / Constants.OPP_SPEED_FOR_TIMEOUT + \
            self._time_out_offset
        elif self._pars.timeout:
            self._time_out = self._pars.timeout

    def _reach_expected_state(self):
        """
        Polls until expected state is reached or an error occurs (time out or failure)
        """
        keep_going = True
        initial_time = time.time()
        while keep_going:
            raw_data = self._api.bt_opp_check_service_raw()
            self._update_files(raw_data)
            if self._pars.expected_state == self._ALL_COMPLETED:
                keep_going = not self._are_all_completed()
            elif self._pars.expected_state == self._ANY_STARTED:
                keep_going = not self._is_any_started()
            elif self._pars.expected_state == self._ANY_CANCELLED:
                keep_going = not self._is_any_cancelled()
            else:
                self._raise_device_exception("Invalid value for EXPECTED_STATE parameter: %s" \
                                             % self._pars.expected_state)
            if keep_going:
                self._stop_if_transfer_failed()
                if (time.time() - initial_time) >= self._time_out:
                    if not raw_data:
                        self._raise_device_exception("OPP Check list is empty, Transfer seems not to be started")

                    self._raise_device_exception("File(s) haven't been completely downloaded before timeout expired")

                time.sleep(self._poll_sleep)

    def _validate_throughput_if_needed(self):
        """
        Validate throughput if needed
        """
        if self._pars.throughput_margin is None or int(self._pars.throughput_margin) == 0:
            return

        threshold = self._throughput_targets.ul_target.value
        threshold = threshold * (100 - int(self._pars.throughput_margin)) / 100

        for key in self._files:
            file_info = self._files[key]
            self._logger.info("Check average %s throughput %01.1f is greater or equal than threshold %01.1f"
                              % (file_info.file_name, file_info.throughput, threshold))
            if not self._files[key].is_throughput_above(threshold):
                self._raise_device_exception("Throughput wasn't good enough for %s transfer" \
                                             % self._files[key].file_name)

    def _stop_if_transfer_failed(self):
        """
        If any of the transfer failed stop
        """
        for key in self._files:
            if self._files[key].is_cancelled():
                self._raise_device_exception("At least one of the transfer failed")

    def _raise_error_if_timed_out(self, initial_time, timeout):
        """
        Raise an error if time out expired
        """
        if (time.time() - initial_time) >= timeout:
            self._raise_device_exception("File(s) haven't been completely downloaded before timeout expired")

    def _update_files(self, raw_data):
        """
        Update files information
        """

        for file_info in raw_data:
            data_id = file_info['id']
            if file_info['address'] == self._pars.bdaddr:
                if not data_id in self._files.keys():
                    file_obj = OppTransfer()
                    self._files[data_id] = file_obj
                else:
                    file_obj = self._files[data_id]

                file_obj.update(file_info)
                self._logger.info("Stats: %s", str(file_obj))

    def _are_all_completed(self):
        """
        Return true if all the files have been downloaded
        """
        if len(self._files) == 0:
            return False

        for key in self._files:
            if not self._files[key].is_completed():
                return False
        return len(self._files) == self._num_expected_files

    def _is_any_started(self):
        """
        Return true if any of the transfer has started
        """
        if len(self._files) == 0:
            return False

        for key in self._files:
            if self._files[key].is_running():
                return True
        return False

    def _is_any_cancelled(self):
        """
        Return true if any of the transfer has been cancelled
        """
        if len(self._files) == 0:
            return False

        for key in self._files:
            if self._files[key].is_cancelled():
                return True
        return False

    def _save_file_size_if_needed(self, context):
        """
        Save the sum of all downloaded file sizes
        """
        if self._pars.save_info_as:
            size = 0
            for key in self._files:
                size += self._files[key].downloaded_size

            context.set_nested_info([self._pars.save_info_as, Constants.OPP_INFO_FILE_SIZE], size)

    def _save_file_chksum_if_needed(self, path, filenames, context):
        """
        Save files checksum in the context if needed
        """
        if self._pars.save_info_as:
            items = self._api.bt_opp_get_files_checksum(path, filenames)
            context.set_nested_info([self._pars.save_info_as, Constants.OPP_INFO_FILE_CHKSUM], str(items))
