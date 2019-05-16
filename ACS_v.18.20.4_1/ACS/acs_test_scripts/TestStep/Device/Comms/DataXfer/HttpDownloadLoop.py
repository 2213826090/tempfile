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
otherwise. Any license under such intellectual property rights must be expressed
and approved by Intel in writing.

@summary: This file implements a Test Step to repeatedly stress HTTP
@since 8 July 2014
@author: Jongyoon Choi
@organization: INTEL PEG-SVE-DSV
"""

import socket
import hashlib
import os
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import Global
from Device.DeviceManager import DeviceManager

class HttpDownloadLoop(DeviceTestStepBase):
    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        self._logger.info("HttpDownloadStress: Run")

        self._device_manager = DeviceManager()

        runtime_minutes = self._pars.duration
        sleep_time_seconds = self._pars.sleep_time
        serve_from_host = self._pars.serve_from_host
        file_download_url = self._pars.file_download_url
        expected_file_checksum = self._pars.expected_file_checksum
        allowed_retries = self._pars.allowed_retries
        host_file_name = self._pars.host_file_name

        device_directory = self._pars.scripts_path

        self._device.remove_device_files(device_directory, "*.log")
        self._device.remove_device_files(device_directory, "*.bin")

        download_file_extension = os.path.splitext(os.path.basename(file_download_url))[1]
        self._device.remove_device_files(device_directory, '*.{0}'.format(download_file_extension))

        # If we aren't serving from the host and the file download URL is not provided,
        #  we don't have anything for the device to download
        if serve_from_host==False and file_download_url==None:
            msg = "No file to download. SERVE_FROM_HOST==False and FILE_DOWNLOAD_URL was not specified."
            self._ts_verdict_msg = msg
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        if serve_from_host:
            # Rewrite our URL to point to this host
            # This is a really ugly URL. But to serve from the non-current-working-directory
            #  is a lot more work so we'll save that for another time.
            # Get our port number
            report_path =  self._device_manager.get_global_config().campaignConfig.get("campaignReportTree").get_report_path()

            tc_name = self._tc_parameters.get_name()

            http_running_file_path = os.path.join(report_path,tc_name,'http_download_loop','.http_running')
            run_file = open(http_running_file_path)
            port = run_file.read()
            run_file.close()
            file_download_url = "http://%s:%s/%s"%(socket.gethostbyname(socket.getfqdn()), port, host_file_name)
            # If we are serving from the host, lets calculate the checksum from the file on the host
            http_serve_file = open(os.path.join(report_path,tc_name, 'http_download_loop', 'http_serve', host_file_name), 'rb')
            md5_hash = hashlib.md5()
            read_data = http_serve_file.read(1024*1024)
            while read_data:
                md5_hash.update(read_data)
                read_data = http_serve_file.read(1024*1024)
            expected_file_checksum = md5_hash.hexdigest()
            http_serve_file.close()

        # Run test with all parameters
        # Give it a 10 minute (600 second) extra timeout because the script will finish the main
        #  loop on time, but then it removes a bunch of files. On a 30 minute run,
        #  it took 2 minutes and 20 seconds to delete the files.
        cmd = 'adb shell "cd %s; ./file_download.sh --duration_minutes=%d --sleep_seconds=%d --url=%s --file_checksum=%s --allowed_retries=%d"'%(device_directory, runtime_minutes, sleep_time_seconds, file_download_url, expected_file_checksum, allowed_retries)
        verdict, output = self._device.run_cmd(cmd, (runtime_minutes + 1)*60)

        if verdict == Global.FAILURE:
            msg = "Http_Download_Stress: run_cmd failed\n"
            self._ts_verdict_msg = msg
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        else:
            self._logger.info("Http_Download_Stress: PASSED")

