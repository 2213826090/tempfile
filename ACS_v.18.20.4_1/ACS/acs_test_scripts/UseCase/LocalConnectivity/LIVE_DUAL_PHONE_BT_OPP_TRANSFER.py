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
:summary: Send file using BT OPP transfer between DUT and reference phone
:since: 12/07/2012
:author: apairex

"""

import posixpath
import time
from Device.DeviceManager import DeviceManager
from acs_test_scripts.Device.UECmd.UECmdTypes import BtOppDir
from LIVE_DUAL_PHONE_BT_BASE import LiveDualPhoneBTBase
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from acs_test_scripts.Utilities.CommunicationUtilities import ConfigsParser
import acs_test_scripts.Utilities.NetworkingUtilities as NetworkingUtil
from UtilitiesFWK.Utilities import Global, str_to_bool_ex
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class LiveDualPhoneBTOppTransfer(LiveDualPhoneBTBase):

    """
    Live Dual Phone BT OPP transfer test.
    """
    # Time to wait for the transfer to occur in bytes/second
    BT_OPP_SPEED_FOR_TIMEOUT = (1024.0 * 1024.0 / (1.5 * 9.0))
    FILE_NAME_SEPARATOR = ","
    FILE_TABLE_STATUS = "status"
    FILE_TABLE_SIZE = "size"
    # THROUGPUT value index for the tuple returned by _throughput_calc()
    INSTANT = 0
    AVERAGE = 1
    # Direction type
    TP_DIR = "direction"
    UPLOAD = "UL"
    DOWNLOAD = "DL"
    # index name for self._tp_meas
    CURTIME = "current_time"
    STARTTIME = "starttime"
    TIMESTAMP = "timestamp"

    # status value
    ST_WAITING_ACCEPT = "waiting_accept"
    ST_DOWNLOADING = "downloading"
    ST_DOWNLOADED = "downloaded"

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LiveDualPhoneBTBase.__init__(self, tc_name, global_config)

        # Get device_config and phone system apis
        self._dut2_config = DeviceManager().get_device_config("PHONE2")
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")
        self._phonesystem2_api = self._phone2.get_uecmd("PhoneSystem")

        # Get mandatory TC Parameters
        self._direction = str(self._tc_parameters.get_param_value("DIRECTION")).upper()
        self._filename = str(self._tc_parameters.get_param_value("FILENAME"))

        # Get optional TC Parameters
        self._is_bidirectional = \
            str(self._tc_parameters.get_param_value("BOTH_DIRECTIONS", "")).lower()
        self._tp_enable = \
            str(self._tc_parameters.get_param_value("THROUGHPUT_ENABLE", "")).lower()
        self._tp_margin = \
            str(self._tc_parameters.get_param_value("ERROR_MARGIN", ""))

        # Will contain self._tp_margin value as an integer
        # If self._tp_margin is empty, by default _throughtput_margin will be 0.
        self._throughtput_margin = 0

        # Format _is_bidirectional to boolean
        self._is_bidirectional = str_to_bool_ex(self._is_bidirectional)
        if self._is_bidirectional is None:
            self._is_bidirectional = False

        # Format _tp_enable to boolean
        self._tp_enable = str_to_bool_ex(self._tp_enable)

        # Initialize sender/receiver pointers
        self._sender_api = None
        self._sender_add = None
        self._sender_device = None
        self._sender_phonesys_api = None
        self._receiver_api = None
        self._receiver_add = None
        self._receiver_device = None
        self._receiver_phonesys_api = None

        # Get the Multimedia folder name
        self._multimedia_path1 = self._device.multimedia_path
        self._multimedia_path2 = DeviceManager().get_device("PHONE2").multimedia_path
        self._fullpath_filenames = ""
        self._fp_filename_other_dir = ""

        # Initialize other protected attributes
        self._timeout1 = 0
        self._timeout2 = 0

        # Split the file names and strip each of them
        self._file_list = [x.strip() for x in self._filename.split(self.FILE_NAME_SEPARATOR)]

        # Initialize variables for average throughput calculation
        self._throughput_targets = None
        self._tp_meas = {}
        self._tp_meas_bidir = {}

    def set_up(self):
        """
        Initialize the test
        """
        LiveDualPhoneBTBase.set_up(self)

        # Check BT addresses validity
        if not NetworkingUtil.is_valid_mac_address(self._phone1_addr):
            msg = "Wrong MAC address for PHONE1 [%s]" % self._phone1_addr
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        if not NetworkingUtil.is_valid_mac_address(self._phone2_addr):
            msg = "Wrong MAC address for PHONE2 [%s]" % self._phone2_addr
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # Set sender/receiver APIs and variables
        if self._direction == self.UPLOAD:
            self._sender_api = self._bt_api
            self._sender_add = self._phone1_addr
            self._sender_device = self._device
            self._sender_phonesys_api = self._phonesystem_api
            self._receiver_api = self._bt_api2
            self._receiver_add = self._phone2_addr
            self._receiver_device = self._phone2
            self._receiver_phonesys_api = self._phonesystem2_api
            (self._fullpath_filenames, expected_size1) = self \
                ._process_files(self._multimedia_path1, self._file_list, False)

            (self._fp_filename_other_dir, expected_size2) = self \
                ._process_files(self._multimedia_path2, self._file_list, True)
        elif self._direction == self.DOWNLOAD:
            self._sender_api = self._bt_api2
            self._sender_add = self._phone2_addr
            self._sender_device = self._phone2
            self._sender_phonesys_api = self._phonesystem2_api
            self._receiver_api = self._bt_api
            self._receiver_add = self._phone1_addr
            self._receiver_device = self._device
            self._receiver_phonesys_api = self._phonesystem_api
            (self._fullpath_filenames, expected_size1) = self \
                ._process_files(self._multimedia_path2, self._file_list, False)

            (self._fp_filename_other_dir, expected_size2) = self \
                ._process_files(self._multimedia_path1, self._file_list, True)
        else:
            msg = "Invalid DIRECTION parameter [%s]" % self._direction
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        if expected_size1 != expected_size2:
            # same files, hence must have same size!!
            msg = "Files size are supposed to be the same!!"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        self._timeout1 = int(expected_size1 / self.BT_OPP_SPEED_FOR_TIMEOUT) + 20
        self._timeout2 = int(expected_size2 / self.BT_OPP_SPEED_FOR_TIMEOUT) + 20

        # Set receiver discoverable by sender
        self._receiver_api.set_bt_discoverable("connectable", 0)
        # First Scan devices around to speed-up run_test
        self._sender_api.bt_scan_devices()

        # unlock screen and set display ON
        # Mandatory prior to use bt_opp_send_file UECmd
        self._sender_phonesys_api.display_on()
        self._sender_phonesys_api.set_phone_lock(0)

        # Process the bidirectional transfer
        if self._is_bidirectional:
            # Set sender discoverable by receiver
            self._sender_api.set_bt_discoverable("connectable", 0)
            self._receiver_api.bt_scan_devices()
            self._receiver_phonesys_api.display_on()
            self._receiver_phonesys_api.set_phone_lock(0)

        # Check throughput data validity
        if self._tp_enable:
            msg = "Throughput measurement is enabled"
            self._logger.info(msg)
            # Read the throughput targets
            self._throughput_targets = ConfigsParser("BT_Throughput_Targets").\
                parse_bt_targets(self._device.get_phone_model(), "OPP")

            # Verify validity of margin parameter
            if self._tp_margin.isdigit():
                self._throughtput_margin = int(self._tp_margin)
            else:
                msg = "ERROR_MARGIN parameter is not valid: <%s>" % self._tp_margin
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

            # Initialize Average tp measure
            self._reset_tp_meas(self._file_list, self._direction, self._is_bidirectional)

        return Global.SUCCESS, "No errors"

    def run_test(self):
        """
        Execute the test
        """
        LiveDualPhoneBTBase.run_test(self)

        # If necessary, remove the file on the destination device
        self._file_cleanup()

        # Clean the notification list
        self._receiver_api.bt_opp_clean_notification_list()
        self._sender_api.bt_opp_clean_notification_list()

        # Request for the First transfer to be registered
        self._sender_api.bt_opp_send_file(self._fullpath_filenames,
                                          self._receiver_add)

        # Process the bidirectional transfer
        if self._is_bidirectional:
            # Wait for the transfer start
            self._wait_for_transfer("downloading", self._receiver_api,
                                    self._sender_add, True, self._timeout1)

            # Request for the second transfer to be registered
            self._receiver_api.bt_opp_send_file(self._fp_filename_other_dir,
                                                self._sender_add)

        # Wait for the transfer First to complete
        self._wait_for_transfer("downloaded", self._receiver_api,
                                self._sender_add, False, self._timeout1)

        # Process the bidirectional transfer
        if self._is_bidirectional:
            # Wait for the second transfer to complete
            self._wait_for_transfer("downloaded", self._sender_api,
                                    self._receiver_add, False, self._timeout2)

        self._logger.info("Compare files' checksum on RX phone with the sent ones")
        self._check_files_checksum(self._sender_api, self._multimedia_path1,
                                   self._receiver_api,
                                   "Received files on RX phone are different from Sent ones"
                                   " (or don't exist)")

        if self._is_bidirectional in ["1", "yes", "true"]:
            self._logger.info("Compare files' checksum on TX phone with the sent ones")
            self._check_files_checksum(self._receiver_api, self._multimedia_path2,
                                       self._sender_api,
                                       "Received files on TX phone are different from Sent ones"
                                       " (or don't exist)")

        # Throughput measurement
        if self._tp_enable:
            # Compare average throughput with Threshold
            self._compare_tp_threshold()
            # Reset self._tp_meas value in case of multiple b2bIteration.
            self._reset_tp_meas(self._file_list, self._direction, self._is_bidirectional)

        return Global.SUCCESS, "No errors"

    def tear_down(self):
        """
        End and dispose the test
        """

        # to have a log "We enter into the tear_down"
        UseCaseBase.tear_down(self)

        # Clean the notification list
        if self._receiver_api is not None:
            self._receiver_api.bt_opp_clean_notification_list()
        if self._sender_api is not None:
            self._sender_api.bt_opp_clean_notification_list()

        self._file_cleanup()

        # lock screen and set display OFF
        self._sender_phonesys_api.set_phone_lock(1)
        self._sender_phonesys_api.display_off()

        # Process the bidirectional transfer
        if self._is_bidirectional:
            self._receiver_phonesys_api.set_phone_lock(1)
            self._receiver_phonesys_api.display_off()

        # Switch OFF BT at the end
        LiveDualPhoneBTBase.tear_down(self)

        return Global.SUCCESS, "No errors"

    def _file_cleanup(self):
        """
        If it exists, remove the file on the destination phone
        """
        for file_name in self._file_list:
            self._receiver_api.bt_opp_init(file_name)
            if self._is_bidirectional:
                self._sender_api.bt_opp_init(file_name)

    def _check_files_status(self, files_table, status, any_file):
        """
        :type files_table: dict
        :param files_table: a table containing file_name/{status, size}
        :type status: str
        :param status: the status to compare the file names' status to
        :type any_file: bool
        :param any_file: if true, one file reaching the status is enough to return true
        :return: True if all the file names' status are equal to status, False otherwise
        """

        # when any_file == False, method returns false as soon as one of the files is in
        # the passed state
        # when any_file == True, method returns true if at least one of the file is in the
        # passed state
        result = not any_file

        for key in files_table:
            if not any_file:
                if files_table[key][self.FILE_TABLE_STATUS] != status:
                    result = False
                    break
            else:
                result = result or \
                    (files_table[key][self.FILE_TABLE_STATUS] == status)
                if result:
                    break

        return result

    def _wait_for_transfer(self, waiting_status, receiver_api, receiver_addr, any_file, timeout=None):
        """
        Wait for the transfer to reach a given status. Trigger the message in the DUT logs.

        Return a BLOCK exception in case status 'downloading' not catch in time

        :type waiting_status: The status to wait before continue
        :param waiting_status: value can be : "waiting_accept", "downloading" or "downloaded"
        :type receiver_api: The BT Local Connectivity API
        :param receiver_api: the api (receiver or sender) to check the notification list
        :type receiver_addr: The BT receiver address
        :param receiver_addr: the receiver BT address (receiver or sender)
        :type any_file: bool
        :param any_file: if true wait for any file to reach the status. if false
                         all files need to reach the status
        :type timeout: int
        :param timeout: timeout to exit the function if the message
                        has not been caught
        :return: Nothing
        """
        self._logger.info("Waiting for the file to transfer (%s sec max)"
                          % str(timeout))
        if any_file:
            self._logger.debug("At least one file must reach %s status" % waiting_status)
        else:
            self._logger.debug("All the files must reach %s status" % waiting_status)

        now = time.time()

        # Populate the table with all the files and status empty
        files = {}
        self._logger.debug("Cleared files table (size: %d)" % len(files))
        for file_name in self._file_list:
            files[file_name] = {self.FILE_TABLE_STATUS: '', self.FILE_TABLE_SIZE: 0}

        # waiting file download completed
        status_reached = False
        while (not status_reached) and (timeout is None or (time.time() - now) < timeout):
            iddata, address, filename, filesize, downloadedsize, status, \
            timestamp, curtime, direction = receiver_api.bt_opp_check_service()
            if (len(iddata) < 1) or (iddata[0] == ''):
                self._logger.debug("No data received from BT OPP Service")
                continue

            # Calculation of Throughput
            if self._tp_enable:
                self._throughput_calc(address, filename, direction, status,
                                      downloadedsize, timestamp, curtime, self._file_list)

            for current in range(len(iddata)):
                # Find verbose direction to print
                dir_v = BtOppDir.d[int(direction[current])]
                self._logger.debug("BT OPP Service: id:%s - remote addr:%s - dir:%s - status:%s - file name:%s - " \
                                   "(dwn:%s / size:%s)" % (iddata[current], address[current], dir_v, status[current],
                                                           filename[current], downloadedsize[current], filesize[current]))
                # Filter only Rx files (dir=DL)
                if dir_v == BtOppDir.UL or address[current] != receiver_addr:
                    self._logger.debug("Do not care of UL file: %s from %s status: %s" \
                                      % (filename[current], address[current], status[current]))
                    continue

                self._logger.debug("Update file status in the table. file: %s status: %s" \
                                  % (filename[current], status[current]))

                if not filename[current] in files.keys():
                    msg = "Unexpected file: %s" % filename[current]
                    self._logger.error(msg)
                    raise DeviceException(DeviceException.OPERATION_FAILED, msg)

                files[filename[current]][self.FILE_TABLE_STATUS] = status[current]
                files[filename[current]][self.FILE_TABLE_SIZE] = \
                    int(downloadedsize[current])
                status_reached = self._check_files_status(files, waiting_status,
                                                          any_file)
                if status_reached:
                    break

            if not status_reached:
                time.sleep(1)

        if status_reached:
            self._logger.info("BT OPP File transfer is %s" % waiting_status)
        else:
            msg = "%s has not been received before timeout." % waiting_status
            self._logger.warning(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    def _process_files(self, base_path, file_list, reverse):
        """
        for each file name in file_list creates the full name (prepending base_path),
        checks its existence and return a tuple with a str containing the list of
        the full name of the files, and the total size of the files

        :type base_path: str
        :param base_path: the folder where the files will be located
        :type file_list: str array
        :param file_list: the list of file names
        :type reverse: bool
        :param reverse: if True files in the returning str are written in reverse order
                        This is done to make it easier understand which file is being
                        transfered on which side, when bidirectional.
                        Mainly used for debug purpose.
        :return: a str containing the full path of the files
                (FILE_NAME_SEPARATOR separated) and the total size of the files
        """

        full_files = []
        total_size = 0

        for file_name in file_list:
            if file_name:
                file_name = posixpath.join(base_path, file_name)

                # If unable to get the file size something is wrong with the file,
                # hence fail
                size = self._phonesystem_api.get_file_size(file_name)
                if size <= 0:
                    msg = "Filesize is not accessible. " \
                        + "File is probably missing on the device"
                    self._logger.error(msg)
                    raise DeviceException(DeviceException.FEATURE_NOT_AVAILABLE, msg)

                total_size += size
                full_files.append(file_name)

        final_list = ""
        for file_name in full_files:
            if not reverse:
                final_list = final_list + file_name + self.FILE_NAME_SEPARATOR
            else:
                final_list = file_name + self.FILE_NAME_SEPARATOR + final_list

        if final_list[-1:] != self.FILE_NAME_SEPARATOR:
            # there must always be a "separator" at the end of the str
            msg = "The calculated list is supposed to end with a separator"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # remove the last "separator"
        final_list = final_list[:-1]

        return final_list, total_size

    def _check_files_checksum(self, sender_api, sender_path, receiver_api, msg):
        """
        Compare checksum for each file in self._file_list between the sender and receiver
        :type sender_api: bluetooth api
        :param sender_api: api of the sender
        :type sender_path: str
        :param sender_path: folder where the files are in the sender device
        :type receiver_api: bluetooth api
        :param receiver_api: api of the receiver
        :type msg: str
        :param msg: message to show if check fails
        """
        # Retrieve checksum for all the sent files in the sender device
        source_files = sender_api.bt_opp_get_files_checksum(sender_path,
                                                            self._file_list)

        # Retrieve checksum for all the received files in the received device
        dest_files = receiver_api.bt_opp_get_files_checksum(None, self._file_list)

        # All the checksum must match, otherwise something went wrong
        if not source_files == dest_files:
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    def _throughput_calc(self, address, filename, direction, status, downloaded_size, timestamp, cur_time, file_list):
        """
        Throughput calculation based on size vs duration.
        Store Value in self._tp_meas

        Throughput = size / duration

        Average value:
        Throughput = downloaded_size / (cur_time - starttime) = B/ms
                   = downloaded_size / (cur_time - starttime) * 8 * 1000 / 1024 = kbit/s

        Note: starttime is different of timestamp due to latency to accept incoming file
            starttime = 1st iteration with status downloading

        Instant value:
        delta_size = downloaded_size - previous_dnsize
        delta_time = cur_time - previous_time
        Throughput = delta_size / delta_time * 8 * 1000 / 1024 = kbit/s

        :type address: list of str
        :param address:  The list of address of remote device involved in that transfer
        :type filename: List of str
        :param filename:  The list of filenames being transfered as key of the table
        :type direction: list of str
        :param direction:  The list of direction of the transfer
            value can be BtOppDir.DL or BtOppDir.UL
        :type status: list of str
        :param status:  The list of status of transfer
            value can be : "waiting_accept", "downloading" or "downloaded"
        :type downloaded_size: list of long
        :param downloaded_size: The list of size already downloaded, in byte
        :type timestamp: list of long
        :param timestamp: The list of timestamp (start time) of the transfer, in millisecond
        :type cur_time: list of long
        :param cur_time: The list of time of the transfer, in millisecond
        :type file_list: list of str
        :param file_list:  The list of file to be transfered

        :return: nothing
        """

        for i in range(len(address)):

            dwnsize = long(downloaded_size[i])
            tstamp = long(timestamp[i])
            ctime = long(cur_time[i])

            # Look for direction from DUT stand point
            dir_v = BtOppDir.d[int(direction[i])]
            if address[i] == self._phone1_addr:
                # revert direction as DUT is remote
                curr_dir = self.UPLOAD if  dir_v == BtOppDir.DL else self.DOWNLOAD
            else:
                # DUT is the receiver
                curr_dir = self.DOWNLOAD if  dir_v == BtOppDir.DL else self.UPLOAD

            # Take care only of file to be transfered
            if filename[i] not in file_list:
                # do not care of that file
                self._logger.debug("Do not care that file for tp: %s. dir:%s" \
                                % (filename, curr_dir))
                continue

            msg = "filename <%s> not in file list to transfer" % filename[i]
            if filename[i] not in self._tp_meas:
                self._logger.error(msg)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

            if self._is_bidirectional and filename[i] not in self._tp_meas_bidir:
                self._logger.error(msg)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

            # 1. Instant measure
            meas = 0.0
            if self._tp_meas[filename[i]][self.TP_DIR] == curr_dir:
                dtime = ctime - self._tp_meas[filename[i]][self.CURTIME]
                if dtime != 0:
                    dsize = dwnsize - self._tp_meas[filename[i]][self.FILE_TABLE_SIZE]
                    # add 1 digit precision: 123.4kbps => float(tp*10) /10
                    meas = long(80000 * dsize / dtime / 1024)
                    meas = float(meas) / 10
                # Store instant throughput
                self._tp_meas[filename[i]][self.INSTANT] = meas
                self._tp_meas[filename[i]][self.CURTIME] = ctime

            # 2. Instant measure bi-direction
            if self._is_bidirectional and \
               self._tp_meas_bidir[filename[i]][self.TP_DIR] == curr_dir:
                dtime = ctime - self._tp_meas_bidir[filename[i]][self.CURTIME]
                if dtime != 0:
                    dsize = dwnsize - self._tp_meas_bidir[filename[i]][self.FILE_TABLE_SIZE]
                    # add 1 digit precision: 123.4kbps => float(tp*10) /10
                    meas = long(80000 * dsize / dtime / 1024)
                    meas = float(meas) / 10
                # Store instant throughput
                self._tp_meas_bidir[filename[i]][self.INSTANT] = meas

            msg = "instant throughput file:%s dir:%s TP=%01.1f Kbit/s" \
                           % (filename[i], curr_dir, meas)
            self._logger.debug(msg)

            # 3. Average measure
            # Initialize starttime
            if self._tp_meas[filename[i]][self.FILE_TABLE_STATUS] != status[i] and \
                status[i] == self.ST_DOWNLOADING:
                # Store starttime and timestamp
                self._tp_meas[filename[i]][self.STARTTIME] = ctime
                self._tp_meas[filename[i]][self.TIMESTAMP] = tstamp
                msg = "file:%s dir:%s timestamp=%d vs starttime=%d" \
                                % (filename[i], curr_dir, tstamp, ctime)
                self._logger.debug(msg)

            # start measurement when transfer is started
            if status[i] == self.ST_WAITING_ACCEPT:
                continue

            dtime = ctime - self._tp_meas[filename[i]][self.STARTTIME]
            meas = 0.0
            if dtime != 0:
                # add 1 digit precision: 123.4kbps => float(tp*10) /10
                meas = long(80000 * dwnsize / dtime / 1024)
                meas = float(meas) / 10

            msg = None
            # Update average till status changed to Downloaded
            if self._tp_meas[filename[i]][self.TP_DIR] == curr_dir and \
               self._tp_meas[filename[i]][self.FILE_TABLE_STATUS] != self.ST_DOWNLOADED:
                self._tp_meas[filename[i]][self.AVERAGE] = meas
                self._tp_meas[filename[i]][self.FILE_TABLE_STATUS] = status[i]
                self._tp_meas[filename[i]][self.FILE_TABLE_SIZE] = dwnsize
                msg = "average throughput file:%s dir:%s TP=%01.1f Kbit/s" \
                                       % (filename[i], curr_dir, meas)

            # Average measure bi-direction
            if self._is_bidirectional and \
               self._tp_meas_bidir[filename[i]][self.TP_DIR] == curr_dir and \
               self._tp_meas[filename[i]][self.FILE_TABLE_STATUS] != self.ST_DOWNLOADED:
                self._tp_meas_bidir[filename[i]][self.AVERAGE] = long(meas)
                self._tp_meas_bidir[filename[i]][self.FILE_TABLE_STATUS] = status[i]
                self._tp_meas_bidir[filename[i]][self.FILE_TABLE_SIZE] = dwnsize
                msg = "average throughput file:%s dir:%s TP=%01.1f Kbit/s" \
                                        % (filename[i], curr_dir, meas)
            if msg is not None:
                self._logger.debug(msg)

    def _compare_tp_threshold(self):
        """
        Compare average throughput with threshold given in parameters of the UC
        Raise error if average is lower than threshold

        :return: nothing
        """

        self._logger.info("Check Throughput measure versus threshold.")

        # Select Threshold
        if self._direction == self.UPLOAD:
            tp_thd = self._throughput_targets.ul_target.value
        else:
            tp_thd = self._throughput_targets.dl_target.value
        # remove margin
        tp_thd = tp_thd * (100 - self._throughtput_margin) / 100
        if tp_thd < 0:
            tp_thd = 0.0

        # variable to check that all measure are good
        isvalidated = 0
        # For each file, TP shall be greater than the threshold
        for rcv_file in self._tp_meas:
            msg = "file:%s Average TP_%s = %01.1f KBps status:%s size=%d" \
                            % (rcv_file,
                               self._tp_meas[rcv_file][self.TP_DIR],
                               self._tp_meas[rcv_file][self.AVERAGE],
                               self._tp_meas[rcv_file][self.FILE_TABLE_STATUS],
                               self._tp_meas[rcv_file][self.FILE_TABLE_SIZE])
            self._logger.debug(msg)

            # Verify that ALL measures are greater than Threshold
            if self._tp_meas[rcv_file][self.AVERAGE] < tp_thd:
                msg = "TP too low: TP= %01.1f KBps vs threshold = %d KBps" \
                            % (self._tp_meas[rcv_file][self.AVERAGE], tp_thd)
                self._logger.error(msg)
                continue

            isvalidated += 1
            msg = "TP OK: %01.1f KBps vs threshold = %d KBps" \
                        % (self._tp_meas[rcv_file][self.AVERAGE], tp_thd)
            self._logger.info(msg)

        if self._is_bidirectional:
            # Select Threshold
            if self._direction == self.UPLOAD:
                tp_thd_bi = self._throughput_targets.dl_target.value
            else:
                tp_thd_bi = self._throughput_targets.ul_target.value
            # remove margin
            tp_thd_bi = tp_thd_bi * (100 - self._throughtput_margin) / 100
            if tp_thd_bi < 0:
                tp_thd_bi = 0.0

            for rcv_file in self._tp_meas_bidir:
                msg = "file:%s Average TP_%s = %01.1f KBps status:%s size=%d" \
                                % (rcv_file,
                                   self._tp_meas_bidir[rcv_file][self.TP_DIR],
                                   self._tp_meas_bidir[rcv_file][self.AVERAGE],
                                   self._tp_meas_bidir[rcv_file][self.FILE_TABLE_STATUS],
                                   self._tp_meas_bidir[rcv_file][self.FILE_TABLE_SIZE])
                self._logger.debug(msg)

                # Verify that ALL measures are greater than Threshold
                if self._tp_meas_bidir[rcv_file][self.AVERAGE] < tp_thd_bi:
                    msg = "TP too low: TP= %01.1f KBps vs threshold = %d KBps" \
                                % (self._tp_meas_bidir[rcv_file][self.AVERAGE], tp_thd_bi)
                    self._logger.error(msg)
                    continue

                isvalidated += 1
                msg = "TP OK: %01.1f KBps vs threshold = %d KBps" \
                            % (self._tp_meas_bidir[rcv_file][self.AVERAGE], tp_thd_bi)
                self._logger.info(msg)

        # Verify number of file_validated equal number of file to transfer
        if not self._is_bidirectional and (isvalidated == len(self._file_list)):
            final_verdict = True
        elif self._is_bidirectional and (isvalidated == (2 * len(self._file_list))):
            final_verdict = True
        else:
            final_verdict = False

        if not final_verdict:
            msg = "Throughput under threshold !!"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        msg = "ALL Throughput measures are OK !!"
        self._logger.info(msg)

    def _reset_tp_meas(self, file_list, direction, isbidir):
        """
        Reset self._tp_meas table used to calculate throughput

        :type file_list: list of str
        :param file_list:  The list of file(s) to be transfered
        :type direction: str
        :param direction:  The direction of the transfer. from xml file
            value can be self.UPLOAD or self.DOWNLOAD
        :type isbidir: boolean
        :param isbidir:  True if bi-directional transfer.

        :return: nothing
        """
        for file_name in file_list:
            self._tp_meas[file_name] = {self.AVERAGE: 0,
                                        self.INSTANT: 0,
                                        self.CURTIME: 0,
                                        self.TP_DIR: direction,
                                        self.FILE_TABLE_SIZE: 0,
                                        self.FILE_TABLE_STATUS: ''}
            if isbidir:
                curr_dir = self.DOWNLOAD if (direction == self.UPLOAD) else self.UPLOAD
                self._tp_meas_bidir[file_name] = {self.AVERAGE: 0,
                                                  self.INSTANT: 0,
                                                  self.CURTIME: 0,
                                                  self.TIMESTAMP: 0,
                                                  self.STARTTIME: 0,
                                                  self.TP_DIR: curr_dir,
                                                  self.FILE_TABLE_SIZE: 0,
                                                  self.FILE_TABLE_STATUS: ''}
