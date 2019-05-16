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
:author: fbongiax

"""

import time
from acs_test_scripts.Device.UECmd.UECmdTypes import BtOppDir
from LIVE_DUAL_PHONE_BT_BASE import LiveDualPhoneBTBase
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global
from Device.DeviceManager import DeviceManager
import acs_test_scripts.Utilities.NetworkingUtilities as NetworkingUtil
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class LiveDualPhoneBTOppCancel(LiveDualPhoneBTBase):
    """
    Live Dual Phone BT OPP cancel test.
    """

    # Time to wait for the transfer to occur in bytes/second
    BT_OPP_SPEED_FOR_TIMEOUT = (1024.0 * 1024.0 / (1.5 * 9.0))
    # # Wait for state downloading and cancelled time out in seconds
    TIME_OUT = 10

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LiveDualPhoneBTBase.__init__(self, tc_name, global_config)

        # Get the phone system apis
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")
        self._phonesystem2_api = self._phone2.get_uecmd("PhoneSystem")

        # Get TC Parameters
        self._direction = str(self._tc_parameters.get_param_value("DIRECTION"))
        self._filename = str(self._tc_parameters.get_param_value("FILENAME"))

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
        self._fullpath_filename = ""

        self._timeout = 0
        self._filesize = 0

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

        # Set sender/receiver APIs and varialbes
        if self._direction.upper() == "UL":
            self._sender_api = self._bt_api
            self._sender_add = self._phone1_addr
            self._sender_device = self._device
            self._sender_phonesys_api = self._phonesystem_api
            self._receiver_api = self._bt_api2
            self._receiver_add = self._phone2_addr
            self._receiver_device = self._phone2
            self._receiver_phonesys_api = self._phonesystem2_api
            self._fullpath_filename = self._multimedia_path1 + "/" \
                + self._filename
            self._filesize = self._phonesystem_api.\
                get_file_size(self._fullpath_filename)
        elif self._direction.upper() == "DL":
            self._sender_api = self._bt_api2
            self._sender_add = self._phone2_addr
            self._sender_device = self._phone2
            self._sender_phonesys_api = self._phonesystem2_api
            self._receiver_api = self._bt_api
            self._receiver_add = self._phone1_addr
            self._receiver_device = self._device
            self._receiver_phonesys_api = self._phonesystem_api
            self._fullpath_filename = self._multimedia_path2 + "/" \
                + self._filename
            self._filesize = self._phonesystem2_api.\
                get_file_size(self._fullpath_filename)
        else:
            msg = "Invalid DIRECTION parameter [%s]" % self._direction
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # Control the existence of the file to send and get the file size \
        # in order to determine the timeout duration
        if self._filesize < 0:
            msg = "Filesize is not accessible. " \
                + "File is probably missing on TX device"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.FEATURE_NOT_AVAILABLE, msg)

        self._timeout = int(self._filesize / self.BT_OPP_SPEED_FOR_TIMEOUT) + 20

        # Set receiver discoverable by sender
        self._receiver_api.set_bt_discoverable("connectable", 0)
        # First Scan devices around to speed-up run_test
        self._sender_api.bt_scan_devices()

        # unlock screen and set display ON
        # Mandatory prior to use bt_opp_send_file UECmd
        self._sender_phonesys_api.display_on()
        self._sender_phonesys_api.set_phone_lock(0)

        return Global.SUCCESS, "No errors"

    def run_test(self):
        """
        Execute the test
        """
        LiveDualPhoneBTBase.run_test(self)

        # If necessary, remove the file on the destination device
        self._receiver_api.bt_opp_init(self._filename)

        # Clean the notification list
        self._receiver_api.bt_opp_clean_notification_list()
        self._sender_api.bt_opp_clean_notification_list()

        # Request for the First transfer to be registered
        self._logger.info("Start sending file %s" % self._fullpath_filename)
        self._sender_api.bt_opp_send_file(self._fullpath_filename,
                                          self._receiver_add)

        # Wait for the transfer First to started
        self._wait_for_transfer("downloading", self._receiver_api,
                                self._sender_add, self.TIME_OUT)

        # Cancel the transfer
        self._logger.info("Cancel file transfer")
        self._sender_api.bt_opp_cancel_send()

        # Wait for the transfer state to be cancelled
        self._wait_for_transfer("cancelled", self._receiver_api,
                                self._sender_add, self.TIME_OUT)

        # Send the file again
        self._logger.info("Start sending file %s again" % self._fullpath_filename)
        self._sender_api.bt_opp_send_file(self._fullpath_filename,
                                          self._receiver_add)

        # Wait for the transfer to be complete
        self._wait_for_transfer("downloaded", self._receiver_api,
                                self._sender_add, self._timeout)

        # bt_opp_get_files_checksum wants a list, but there's only one file
        # so put it into a list
        file_list = [self._filename]

        source_path = self._multimedia_path1 if self._direction.upper() == "UL" \
        else self._multimedia_path2

        source_files = self._sender_api.bt_opp_get_files_checksum(source_path,
                                                                  file_list)
        dest_files = self._receiver_api.bt_opp_get_files_checksum(None, file_list)

        # Check the file matches on the RX phone
        self._logger.info("Compare file checksum on sender and receiver")
        if not source_files == dest_files:
            msg = "The file hasn't been received or it doesn't match the sent one"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        return Global.SUCCESS, "No errors"

    def tear_down(self):
        """
        End and dispose the test
        """

        # to have a log "We enter into the tear_down"
        UseCaseBase.tear_down(self)

        # Clean the notification list
        self._receiver_api.bt_opp_clean_notification_list()
        self._sender_api.bt_opp_clean_notification_list()

        self._receiver_api.bt_opp_init(self._filename)

        # lock screen and set display OFF
        self._sender_phonesys_api.set_phone_lock(1)
        self._sender_phonesys_api.display_off()

        # Switch OFF BT at the end
        LiveDualPhoneBTBase.tear_down(self)

        return Global.SUCCESS, "No errors"

    def _wait_for_transfer(self, waiting_status, receiver_api, remote_addr, timeout=None):
        """
        Wait for the transfer to complete. Trigger the message in the DUT logs.

        Return a BLOCK exception in case status 'downloading' not catch in time

        :type waiting_status: The status to wait before continue
        :param waiting_status: value can be : "waiting_accept", "downloading" or "downloaded"
        :type receiver_api: The BT Local Connectivity API
        :param receiver_api: the api (receiver or sender) to check the notification list
        :type remote_addr: The BT remote device address
        :param remote_addr: the remote BT address (receiver or sender)
        :type timeout: int
        :param timeout: timeout to exit the function if the message
                        has not been caught
        :return: the downloaded size (at this stage) of the file or 0 if timed out
        """
        self._logger.info("Waiting for the file to reach '%s' status (%s sec max)"
                          % (waiting_status, str(timeout)))

        now = time.time()

        # waiting file download completed
        file_received = False
        file_size = 0
        while (file_received == False) and (timeout is None or (time.time() - now) < timeout):
            iddata, address, filename, filesize, downloadedsize, status, timestamp, curtime, \
                direction = receiver_api.bt_opp_check_service()
            if (len(iddata) < 1) or (iddata[0] == ''):
                self._logger.debug("No data received from BT OPP Service")
                continue

            for current in range(len(iddata)):
                # Find verbose direction to print
                dir_v = BtOppDir.d[int(direction[current])]
                self._logger.debug("BT OPP Service: id:%s - remote addr:%s - dir:%s - status:%s - file name:%s - " \
                                   "(dwn:%s / size:%s)" % (iddata[current], address[current], dir_v, status[current],
                                                           filename[current], downloadedsize[current], filesize[current]))
                # Filter only Rx files (dir=DL)
                if dir_v == BtOppDir.UL or address[current] != remote_addr:
                    self._logger.debug("Do not care of UL file: %s from %s status: %s" \
                                      % (filename[current], address[current], status[current]))
                    continue

                self._logger.debug("Update file status in the table. file: %s status: %s" \
                                  % (filename[current], status[current]))

                if (address[current] == remote_addr) and \
                   (status[current] == waiting_status) and \
                   (filename[current] == self._filename):
                    file_received = True
                    file_size = int(filesize[current])
                    break

            if not file_received:
                time.sleep(1)

        if not file_received:
            msg = "%s has not been reached before timeout." % waiting_status
            self._logger.error(msg)
            raise DeviceException(DeviceException.TIMEOUT_REACHED, msg)

        self._logger.info("BT OPP File transfer is %s" % waiting_status)

        return file_size
