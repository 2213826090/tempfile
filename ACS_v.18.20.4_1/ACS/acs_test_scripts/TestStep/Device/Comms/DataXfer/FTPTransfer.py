"""
@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

:organization: INTEL NDG SW
:summary: This file implements a Test Step for a full FTP transfer
:since 08/09/2014
:author: jfranchx
"""
import re
from acs_test_scripts.TestStep.Device.Comms.DataXfer.DataXferBase import DataXferBase
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import Global


class FTPTransfer(DataXferBase):
    """
    Implements the full FTP transfer
    """
    _FTP_THROUGHPUT_PARSING_PATTERN = r'throughput: *([0-9\.]+) *(\w*)[bB]ytes/sec'

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DataXferBase.run(self, context)

        # Mandatory parameters
        server_ip_address = str(self._pars.server_ip)
        direction = str(self._pars.direction)
        username = str(self._pars.username)
        password = str(self._pars.password)
        filename = str(self._pars.filename)
        timeout = int(self._pars.timeout)

        # Optional parameters
        source_ip = self._pars.client_ip
        if "none" in source_ip.lower():
            source_ip = None

        # Where the information will be stored into the context
        save_as = self._pars.save_throughput_as

        self._logger.info("Perform FTP transfer with %s ..." % server_ip_address)

        ftp_result_status, ftp_result_log = self._networking_api.ftp_xfer(direction,
                                                                          server_ip_address,
                                                                          username, password,
                                                                          filename, timeout,
                                                                          self._device.multimedia_path,
                                                                          target_throughput=None,
                                                                          client_ip_address=source_ip)

        if ftp_result_status != Global.SUCCESS:
            msg = "Error with FTP transfer result : %s - %s" % (ftp_result_status, ftp_result_log)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # Retrieve the FTP throughput
        throughput = re.search(self._FTP_THROUGHPUT_PARSING_PATTERN, ftp_result_log)
        if throughput:
            # throughput is in ?bytes/sec, convert it to ?bits/sec
            throughput_value = float(throughput.group(1)) * 8
            # unit is k, m or nothing + bits/sec
            throughput_unit = throughput.group(2).lower() + "bits/sec"
        else:
            msg = "Error invalid throughput during FTP transfer : %s" % ftp_result_log
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # We have the value, let's save it into the context
        context.set_info(save_as+":VALUE", throughput_value)
        context.set_info(save_as+":UNIT", throughput_unit)
        self.ts_verdict_msg = "VERDICT: %s stored as {0} - unit {1}".format(str(throughput_value), throughput_unit) \
                              % save_as
        self._logger.debug(self.ts_verdict_msg)
