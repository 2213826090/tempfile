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
:summary: Implements FTP methods used over the whole ACS solution
:since: 14/11/2014
:author: mbrisbax
"""
import acs_test_scripts.Device.UECmd.UECmdTypes as UECmdTypes
import os
import time
from UtilitiesFWK.Utilities import Global


class MultipleFtpTransfer(object):
    """
    Class that handles multiple FTP transfers
    """

    def __init__(self,
                 nb_transfer,
                 server_ip_address,
                 direction,
                 username,
                 password,
                 dut_ip_address,
                 ftp_api,
                 remote_file,
                 dl_remote_file,
                 logger,
                 timeout,
                 target_tput=None,
                 target_dl_tput=None,
                 local_path=None,
                 local_dl_path=None):
        """
        :type nb_transfer: int
        :param nb_transfer: number of simultaneous transfer to start

        :type server_ip_address: str
        :param server_ip_address: FTP server IP address

        :type direction: str
        :param direction: FTP transfer direction UL/DL/BOTH

        :type username: str
        :param username: FTP login

        :type password: str
        :param password: FTP password

        :type remote_file: str
        :param remote_file: file path on FTP server

        :type dl_remote_file: str
        :param dl_remote_file: file path on FTP server

        :type local_path: str
        :param local_path: file path on DUT

        :type dut_ip_address: str
        :param dut_ip_address: DUT IP address

        :type ftp_api: object
        :param ftp_api: ftp object for FTP UeCmd

        :type local_dl_path: str
        :param local_dl_path: DL file path on DUT

        :type target_tput: float
        :param target_tput: target throughput to reach for upload

        :type target_dl_tput: float
        :param target_dl_tput: target throughput to reach for download
        """
        # List of files to upload name on DUT
        self._ul_file = []
        # List of files to download name on DUT
        self._dl_file = []
        # List of FTP transfer ids
        self._ftp_transfer_ids = []
        self._direction = direction
        self._nb_transfer = nb_transfer
        # If we want to perform simultaneous UL/DL transfer
        # we need to divide by 2 as launch_ftp_transfer will start automatically a DL and a UL transfer
        if self._direction == "BOTH":
            self._nb_transfer /= 2
        self._server_ip_address = server_ip_address
        self._username = username
        self._password = password
        self._dut_ip_address = dut_ip_address
        self._ftp_api = ftp_api
        self._timeout = timeout
        self._logger = logger
        self._remote_file = remote_file
        self._dl_remote_file = dl_remote_file
        self._local_path = local_path
        self._local_dl_path = local_dl_path
        self._target_tput = target_tput / self._nb_transfer
        self._target_dl_tput = target_dl_tput / self._nb_transfer
        self._local_file = os.path.join(self._local_path, os.path.basename(self._remote_file))
        for i in range(self._nb_transfer):
            # for each transfer the file on DUT have a different name
            # For instance if we want to download 3 times get100M, we will get 3 files: get100M0, get100M1, get100M2
            # If we want to upload put50M, there will be 3 file on server put50M0, put50M1, put50M2
            if self._local_path is not None:
                self._ul_file.insert(i, self._remote_file + str(i))
            if self._local_dl_path is not None:
                self._dl_file.insert(i, os.path.join(self._local_dl_path, os.path.basename(self._dl_remote_file)) + str(i))

    def launch(self):
        """
        Launch multiple FTP transfers

        :rtype: (int, str)
        :return: (status, msg)
        """
        msg = "%s %s FTP transfers successfully started" % (self._nb_transfer, self._direction)
        # Launch FTP transfers
        for i in range(self._nb_transfer):
            try:
                self._ftp_transfer_ids.insert(i, launch_ftp_transfer(self._direction,
                                                                     self._server_ip_address,
                                                                     self._username,
                                                                     self._password,
                                                                     self._ul_file[i],
                                                                     self._local_path,
                                                                     self._dut_ip_address,
                                                                     self._ftp_api,
                                                                     self._dl_remote_file,
                                                                     self._local_dl_path,
                                                                     os.path.basename(self._local_file),
                                                                     os.path.basename(self._dl_file[i])))
            except Exception as e:
                msg = "Failed to start transfer %d of %d: %s" % (i + 1, self._nb_transfer, str(e))
                return Global.FAILURE, msg
        return Global.SUCCESS, msg

    def _wait_transfer(self, transfer_nb, ftp_id, msg, status, remote_file, local_file, direction, target_tput):
        if ftp_id is not None:
            try:
                result, log = wait_end_ftp_transfer(ftp_id, self._timeout, self._ftp_api, remote_file, local_file, self._server_ip_address, self._username, self._password)
                if result == Global.SUCCESS:
                    result, log = check_throughput(self._ftp_api, ftp_id, self._logger, os.path.basename(remote_file), direction, target_tput)
                else:
                    self._logger.error(log)
                msg += " %s transfer %s of %s: " % (direction, transfer_nb + 1, self._nb_transfer)
                if result == Global.FAILURE:
                    msg += "- FAILED -: "
                    status = Global.FAILURE
                else:
                    self._logger.error(log)
                msg += log
            except Exception as e:
                self._logger.error("Error with %s FTP transfer %s: %s" % (direction, transfer_nb, str(e)))
                status = Global.FAILURE
                msg += "Error with %s FTP transfer %s: %s" % (direction, transfer_nb, str(e))
        return msg, status

    def wait_end_of_transfers(self):
        """
        Wait end of several FTP transfers
        """
        # Wait end of data transfers and checks their throughput
        status = Global.SUCCESS
        msg = ""
        for i in range(len(self._ftp_transfer_ids)):
            dl_ftp_id = self._ftp_transfer_ids[i][0]
            ul_ftp_id = self._ftp_transfer_ids[i][1]
            # Wait end of DL transfer
            msg, status = self._wait_transfer(i, dl_ftp_id, msg, status, self._dl_remote_file, self._dl_file[i], "DL", self._target_dl_tput)
            # Wait end of UL transfer
            msg, status = self._wait_transfer(i, ul_ftp_id, msg, status, self._ul_file[i], self._local_file, "UL", self._target_tput)

        self.stop()

        return status, msg

    def stop(self):
        """
        Stops several FTP transfers
        """
        # Stops all on going FTP transfers
        for i in range(len(self._ftp_transfer_ids)):
            # Flush self._ftp_transfer_ids list for future use
            try:
                dl_ftp_id, ul_ftp_id = self._ftp_transfer_ids.pop()
            except:
                self._logger.error("FTP transfer %s does not exist" % i)
                dl_ftp_id = None
                ul_ftp_id = None
            if dl_ftp_id is not None:
                try:
                    (tput, status, log) = self._ftp_api.stop_ftp(dl_ftp_id)
                    self._logger.debug("DL FTP transfer %s of %s was forcibly stopped at the end of test: " % (i + 1, self._nb_transfer))
                    self._logger.debug("Overrall data rate: %s" % tput)
                    self._logger.debug("Final status: %s" % status)
                    self._logger.debug("Execution log: %s" % log)
                except:
                    # if an exception is raised, it means that ftp transfer was already stopped. And it is not a problem
                    pass
            if ul_ftp_id is not None:
                try:
                    (tput, status, log) = self._ftp_api.stop_ftp(ul_ftp_id)
                    self._logger.debug("DL FTP transfer %s of %s was forcibly stopped at the end of test: " % (i, self._nb_transfer))
                    self._logger.debug("Overrall data rate: %s" % tput)
                    self._logger.debug("Final status: %s" % status)
                    self._logger.debug("Execution log: %s" % log)
                except:
                    # if an exception is raised, it means that ftp transfer was already stopped. And it is not a problem
                    pass


def launch_ftp_transfer(direction, server_ip_address, username, password, remote_file, local_path, dut_ip_address, ftp_api, dl_remote_file, local_dl_path, local_file=None, local_dl_file=None):
    """
    Launch a FTP transfer

    :type direction: str
    :param direction: FTP transfer direction UL/DL/BOTH

    :type server_ip_address: str
    :param server_ip_address: FTP server IP address

    :type username: str
    :param username: FTP login

    :type password: str
    :param password: FTP password

    :type remote_file: str
    :param remote_file: file path on FTP server

    :type dl_remote_file: str
    :param dl_remote_file: file path on FTP server

    :type local_path: str
    :param local_path: file path on DUT

    :type dut_ip_address: str
    :param dut_ip_address: DUT IP address

    :type ftp_api: object
    :param ftp_api: ftp object for FTP UeCmd

    :type local_dl_path: str
    :param local_dl_path: DL file path on DUT

    :type local_file: str
    :param local_file: file to upload name on DUT

    :type local_dl_file: str
    :param local_dl_file: file to download name on DUT

    :rtype: (long, long)
    :return: (dl ftp transfer id, ul ftp transfer id)
    """
    ul_ftp_id = None
    dl_ftp_id = None
    if direction == "UL":
        ul_ftp_id = ftp_api.start_ftp(UECmdTypes.XFER_DIRECTIONS.UL,  # pylint: disable=E1101
                                      server_ip_address,
                                      username,
                                      password,
                                      remote_file,
                                      local_path,
                                      dut_ip_address,
                                      local_file)
    elif direction == "DL":
        # the FTP download file will write to /data/ to get max throughput
        # because of emmc sequential write limitation
        dl_ftp_id = ftp_api.start_ftp(UECmdTypes.XFER_DIRECTIONS.DL,  # pylint: disable=E1101
                                      server_ip_address,
                                      username,
                                      password,
                                      dl_remote_file,
                                      local_dl_path,
                                      dut_ip_address,
                                      local_dl_file)
    elif direction == "BOTH":
        # Launch FTP upload
        ul_ftp_id = ftp_api.start_ftp(UECmdTypes.XFER_DIRECTIONS.UL,  # pylint: disable=E1101
                                      server_ip_address,
                                      username,
                                      password,
                                      remote_file,
                                      local_path,
                                      dut_ip_address,
                                      local_file)
        # Launch FTP download
        # the FTP download file will write to /data/ to get max throughput
        # because of emmc sequential write limitation
        dl_ftp_id = ftp_api.start_ftp(UECmdTypes.XFER_DIRECTIONS.DL,  # pylint: disable=E1101
                                      server_ip_address,
                                      username,
                                      password,
                                      dl_remote_file,
                                      local_dl_path,
                                      dut_ip_address,
                                      local_dl_file)
    return dl_ftp_id, ul_ftp_id


def perform_ftp_transfer(direction,
                         server_ip_address,
                         username,
                         password,
                         remote_file,
                         xfer_timeout,
                         local_path,
                         dut_ip_address,
                         ftp_api,
                         target_tput,
                         logger,
                         dl_remote_file=None,
                         target_dl_tput=None,
                         local_dl_path=None):
    """
    Launch a FTP transfer and wait its completion

    :type direction: str
    :param direction: FTP transfer direction UL/DL/BOTH

    :type server_ip_address: str
    :param server_ip_address: FTP server IP address

    :type username: str
    :param username: FTP login

    :type password: str
    :param password: FTP password

    :type remote_file: str
    :param remote_file: file path on FTP server

    :type dl_remote_file: str
    :param dl_remote_file: file path on FTP server

    :type xfer_timeout: int
    :param xfer_timeout: FTP transfer timeout

    :type local_path: str
    :param local_path: file path on DUT

    :type dut_ip_address: str
    :param dut_ip_address: DUT IP address

    :type ftp_api: object
    :param ftp_api: ftp object for FTP UeCmd

    :type target_tput: str
    :param target_tput: minimum throughput to reach for FTP transfer

    :type logger: object
    :param logger: logger object

    :type target_dl_tput: str
    :param target_dl_tput: minimum throughput to reach for DL transfer

    :type local_dl_path: str
    :param local_dl_path: DL file path on DUT

    :rtype: (bool, str)
    :return: (FTP transfer status, message)
    """

    """
    Need to kill existing ftp before performing FTP transfer
    """

    try:
        ftp_api.kill_ftp()
    except Exception as e:
        logger.error(str(e))

    if direction == "DL":
        if local_dl_path is None:
            local_dl_path = local_path
        if dl_remote_file is None:
            dl_remote_file = remote_file
        if target_dl_tput is None:
            target_dl_tput = target_tput
    elif direction == "BOTH":
        if local_dl_path is None:
            local_dl_path = local_path

    dl_ftp_id, ul_ftp_id = launch_ftp_transfer(direction, server_ip_address, username, password, remote_file, local_path, dut_ip_address, ftp_api, dl_remote_file, local_dl_path)
    status = Global.SUCCESS
    msg = ""
    if dl_ftp_id is not None:
        (status, msg) = wait_end_ftp_transfer(dl_ftp_id, xfer_timeout, ftp_api, dl_remote_file, os.path.join(local_dl_path, os.path.basename(dl_remote_file)), server_ip_address, username, password)
        if status == Global.SUCCESS:
            (status, msg) = check_throughput(ftp_api, dl_ftp_id, logger, os.path.basename(dl_remote_file), "DL", target_dl_tput)
        else:
            logger.error(msg)
    if ul_ftp_id is not None:
        (ul_status, ul_msg) = wait_end_ftp_transfer(ul_ftp_id, xfer_timeout, ftp_api, remote_file, os.path.join(local_path, os.path.basename(remote_file)), server_ip_address, username, password)
        if ul_status == Global.SUCCESS:
            (ul_status, ul_msg) = check_throughput(ftp_api, ul_ftp_id, logger, os.path.basename(remote_file), "UL", target_tput)
        else:
            logger.error(ul_msg)
        msg += ul_msg
        if ul_status == Global.FAILURE:
            status = Global.FAILURE

    return status, msg


def check_throughput(ftp_api, ftp_id, logger, filename, direction, target_throughput):
    """
    Check if measured throughput is above target

    :type ftp_api: object
    :param ftp_api: ftp object for FTP UeCmd

    :type ftp_id: long
    :param ftp_id: FTP transfer id

    :type logger: object
    :param logger: logger object

    :type direction: string
    :param direction: target throughput to reach

    :type target_throughput: float
    :param target_throughput: target throughput to reach

    :rtype: (bool, str)
    :return: (Target throughput reached, message)
    """

    (measured_throughput, status, log) = ftp_api.stop_ftp(ftp_id)

    logger.debug("FTP transfer: " + str(direction) + " file: " + str(filename) + " successful")
    # If a target throughput has been given, check if measured throughput is greater or equal than the target value
    if target_throughput is not None:
        # Measured throughput is returned in kBytes/sec
        tput_list = measured_throughput.split("kBytes/sec")
        tput = tput_list[0]
        output = " FTP transfer: " + str(direction) + " file: " + str(filename) + " successful"
        output += " - target throughput: " + str(target_throughput) + "kbits/s"
        # Convert measured throughput to kbits/s
        try:
            tput_val = float(tput) * 8
            output += " - throughput: " + str(tput_val) + "kbits/s"
        except ValueError:
            output += "!!! WRONG UNIT OR VALUE in measured throughput, it shall be x.ykBytes/sec!!!! - throughput: " + str(measured_throughput)
            return Global.FAILURE, output
        logger.info(output)
        # Compare measured and target throughput
        if tput_val < float(target_throughput):
            return Global.FAILURE, output
    else:
        output = str(direction) + " file: " + str(filename) + " successfully"
        output += " - throughput: " + str(measured_throughput)
        logger.info(output)

    return Global.SUCCESS, output


def wait_end_ftp_transfer(ftp_id,
                          timeout,
                          ftp_api,
                          remote_file,
                          local_file,
                          server_ip_address,
                          username,
                          password):
    """
    Wait end of a FTP transfer

    :type ftp_id: long
    :param ftp_id: FTP transfer id

    :type timeout: int
    :param timeout: FTP transfer timeout

    :type ftp_api: object
    :param ftp_api: ftp object for FTP UeCmd

    :type remote_file: str
    :param remote_file: file path on server

    :type local_file: str
    :param local_file: file path on DUT

    :type server_ip_address: str
    :param server_ip_address: FTP server IP address

    :type username: str
    :param username: FTP login

    :type password: str
    :param password: FTP password

    :rtype: (bool, str)
    :return: (FTP transfer status, message)
    """
    ftp_finished = False
    end = time.time() + timeout
    while time.time() < end:
        ftp_status = ftp_api.get_ftp_status(ftp_id)
        if ftp_status == "transfer successful":
            try:
                # Check if file has the same size on FTP server and DUT
                isSizeCorrect = ftp_api.check_file_size(server_ip_address, username, password, remote_file, local_file)
            except:
                isSizeCorrect = False
            if isSizeCorrect:
                status = Global.SUCCESS
                msg = "FTP finished"
                ftp_finished = True
            else:
                status = Global.FAILURE
                msg = "FTP transfer failed: File size is not the same on FTP server and DUT"
            break
        elif ftp_status in {"notrunning", "transfer failed"}:
            ftp_finished = True
            result = ftp_api.stop_ftp(ftp_id)
            msg = "FTP transfer failed - current FTP status: " + result[1]
            status = Global.FAILURE
            break
        time.sleep(1)
    if not ftp_finished:
        result = ftp_api.stop_ftp(ftp_id)
        msg = "FTP transfer timed out - current FTP status: " + result[1]
        status = Global.FAILURE
    return status, msg
