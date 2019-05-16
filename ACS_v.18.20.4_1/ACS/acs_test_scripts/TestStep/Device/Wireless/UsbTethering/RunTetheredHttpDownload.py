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

@summary: This file implements a Test Step to download files from remote server using USB tethering. Host will download files using HTTP protocol
@since 25 Aug 2014
@author: Jongyoon Choi
@organization: INTEL PEG-SVE-DSV
"""
import time
import os
import socket
import urllib
import re
import tempfile
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.Equipment.EquipmentManager import EquipmentManager

class RunTetheredHttpDownload(DeviceTestStepBase):
    """
    Implements a Test Step to download files from remote server using USB tethering. Host will download files using HTTP protocol
    """

    def run(self, context):
        """
        Download files from remote server using USB tethering

        :type context: TestStepContext
        :param context: test case context
        """
        DeviceTestStepBase.run(self, context)

        # Get the HTTP file server root path as well as the local download path and runtime
        self._http_src_url = self._pars.http_source_url
        self._runtime_seconds = self._pars.duration * 60
        self._xfer_rate = self._pars.avg_xfer_rate_sec

        self._logger.info('Tethered HTTP download will run for {0} minutes'.format(self._pars.duration))

        self._netAPI = self._device.get_uecmd("Networking")

        # Create the target download directory path under temporary directory
        temp_path = tempfile.gettempdir()
        self._targetPath = os.path.join(temp_path, "RunTetheredHttpDownload_{0}".format(time.strftime('%Y%m%d_%H%M', time.localtime())))

        self._logger.debug("make {0} folder".format(self._targetPath))
        try:
            os.makedirs(self._targetPath)
        except Exception, e:
            msg = "Failed to create local download directory, \"{0}\": {1}".format(self._targetPath, e)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # Set the default UsbTethering timeout to the run UsbTethering
        socket.setdefaulttimeout( self._runtime_seconds + 30 )

        # Download as many files as possible within duration
        test_start_time = time.time()
        test_end_time = test_start_time + float( self._runtime_seconds )
        self._logger.info('[Tethered HTTP download] Download from {0}'.format(self._http_src_url))
        self._logger.info('[Tethered HTTP download] Test started at {0}'.format(time.strftime('%m\\%d\\%Y %H:%M:%S', time.localtime())))

        iteration_count = 0
        while (time.time() < test_end_time):
            iteration_count += 1
            iteration_start_time = time.localtime()
            self._logger.info('[Tethered HTTP download - Iteration {0}] Started at {1}'.format(str(iteration_count), time.strftime('%H:%M:%S', iteration_start_time)))

            test_remaining_time = test_end_time - time.time()
            max_capable_xfer_size = self._xfer_rate * test_remaining_time
            self._logger.debug('[Tethered HTTP download - Iteration {0}] {1} seconds remaining, and {2} MB can be downloaded'.format(iteration_count, test_remaining_time, round(max_capable_xfer_size/1024/1024)))

            if max_capable_xfer_size < 512*2**20:
                path = self._http_src_url+'/2GBFiles/1.5GBFiles/1GBFiles/768MBFiles/512MBFiles/256MBFiles/'
                self._logger.debug('[Tethered HTTP download - Iteration {0}] Download 256 MB files'.format(iteration_count))
            elif max_capable_xfer_size >= 512*2**20 and max_capable_xfer_size < 768*2**20:
                path = self._http_src_url+'/2GBFiles/1.5GBFiles/1GBFiles/768MBFiles/512MBFiles/'
                self._logger.debug('[Tethered HTTP download - Iteration {0}] Download 512 MB files'.format(iteration_count))
            elif max_capable_xfer_size >= 768*2**20 and max_capable_xfer_size < 1024*2**20:
                path = self._http_src_url+'/2GBFiles/1.5GBFiles/1GBFiles/768MBFiles/'
                self._logger.debug('[Tethered HTTP download - Iteration {0}] Download 768 MB files'.format(iteration_count))
            elif max_capable_xfer_size >= 1024*2**20 and max_capable_xfer_size < 1536*2**20:
                path = self._http_src_url+'/2GBFiles/1.5GBFiles/1GBFiles/'
                self._logger.debug('[Tethered HTTP download - Iteration {0}] Download 1 GB files'.format(iteration_count))
            elif max_capable_xfer_size >= 1536*2**20 and max_capable_xfer_size < 2048*2**20:
                path = self._http_src_url+'/2GBFiles/1.5GBFiles/'
                self._logger.debug('[Tethered HTTP download - Iteration {0}] Download 1.5 GB files'.format(iteration_count))
            else:
                path = self._http_src_url+'/2GBFiles/'
                self._logger.debug('[Tethered HTTP download - Iteration {0}] Download 2 GB files'.format(iteration_count))

            # Download all files from selected path
            self.download_files(path, self._targetPath, iteration_count)

        # Write _targetPath to the context so that it can be used by other TestSteps
        context.set_info(self._pars.target_path, self._targetPath)
        self._logger.info('[Tethered HTTP download] Test finished at {0}'.format(time.strftime('%m\\%d\\%Y %H:%M:%S', time.localtime())))

    def download_files(self, path, targetPath, iteration):
        """
        Download all files which have the given extension from HTTP path to targetPath

        :type path: str
        :param path: Remote path of the files to be downloaded

        :type targetPath: str
        :param targetPath: Target path to store the downloaded files

        :type iteration: int
        :param iteration: current iteration

        :return: None
        """
        self.check_and_recover_usb_tether(path)
        pattern = '<a href=".*?">(.*?txt)</a>'
        htmlIndex = urllib.urlopen(path).read()
        for txtFile in re.findall(pattern, htmlIndex):
            try:
                if os.path.exists(os.path.join(targetPath, txtFile)):
                    destPath = os.path.join(targetPath, str(iteration)+txtFile)
                else:
                    destPath = os.path.join(targetPath, txtFile)

                self._logger.info("Downloading %s..."%txtFile)
                self.check_and_recover_usb_tether(path)
                urllib.urlretrieve(path+txtFile, destPath)
                md5File = txtFile.rsplit('.',1)[0] + '.md5'
                md5Path = destPath.rsplit('.',1)[0] + '.md5'
                self._logger.info("Finished downloading {0}.\nDownloading {1}...".format(txtFile,md5File))

                self.check_and_recover_usb_tether(path)
                urllib.urlretrieve(path+md5File, md5Path)
                self._logger.info("Finished downloading {0}...".format(md5File))
            except Exception, e:
                msg = "Failed to download {0} (and/or corresponding MD5 file) from {1}: {2}".format(txtFile, path, e)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    def check_and_recover_usb_tether(self, path='www.google.com', numRetries=5):
        """
        If the USB tether has gone down, attempt to recover it before continuing

        :type path: str
        :param path: Remote path to be used for connection test

        :type numRetries: int
        :param numRetries: the number of times to retry

        :return: None
        """
        hostpc = EquipmentManager().get_computer(eqt_name="COMPUTER1")
        try:
            time.sleep(1)
            urllib.urlopen(path).read()
        except Exception:
            for i in range(1,numRetries+1):
                try:
                    self._logger.debug("USB tether connection went down. Attempting to recover...")
                    time.sleep(1)
                    self._netAPI.start_usb_tethering(unplug=True)
                    if 'linux' in os.sys.platform:
                        usbInterface = 'usb0'
                        hostpc.dhclient(usbInterface)
                except Exception, e:
                    if i == numRetries:
                        msg = "Failed to recover the USB tether: {0}".format(e)
                        raise DeviceException(DeviceException.OPERATION_FAILED, msg)
                    else:
                        self._logger.debug('Attempt %d to recover USB tether failed. Trying again...'.format(i))

        self._logger.debug("RunTetheredHttpDownload: Recovered USB tether connection")
