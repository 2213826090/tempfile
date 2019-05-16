"""

:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

:organization: INTEL PEG SVE DSV

:summary: Install an artifact from ACS file cache to one of the following directory
            1. If "DEST_DIR" is start with "[REPORT]/", [campaign_report_dir]/[test_case_name]/[dest_dir] will be used as a destination
            2. If "DEST_DIR" is set as full path, absolute path will be used as a destination
            3. If "TYPE" is zipped_tarball, tarball, or zipped and "DEST_DIR" is same as "SRC_FILE_PATH", it will install in same directory that source file currently resides in.
:since: 07/01/2014
:author: jongyoon
"""

import os
import shutil
import tarfile
from Core.TestStep.TestStepBase import TestStepBase
from ErrorHandling.AcsConfigException import AcsConfigException
from Device.DeviceManager import DeviceManager

class InstallFileHost(TestStepBase):

    """
    Install a file on a host
    """
    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        TestStepBase.run(self, context)

        # Check whether source file exist or not
        if not os.path.isfile(self._pars.src_file_path):
            error_msg = "{0}: File {1} does not exist!".format(self._pars.id, self._pars.src_file_path)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        basename_file = os.path.basename(self._pars.src_file_path) # File name of the source file
        zipped = self._pars.type == "zipped_tarball" or self._pars.type == "tarball" or self._pars.type == "zipped"

        (dest_parentdir, sep, dest_subdir) = self._pars.dest_dir.partition('/')
        if dest_parentdir == '[REPORT]':
            #Destination directory will be located under _Report directory
            self._device_manager = DeviceManager()
            report_path =  self._device_manager.get_global_config().campaignConfig.get("campaignReportTree").get_report_path()
            host_destination = os.path.join(report_path, self._tc_parameters.get_name(), dest_subdir) # Destination path

            #Create the destination directory
            if not os.path.exists(host_destination):
                self._logger.debug("{0}: make {1} folder".format(self._pars.id, host_destination))
                os.makedirs(host_destination)
        elif self._pars.src_file_path == self._pars.dest_dir and zipped:
            # We will unzip in the same directory as the src_file path.
            host_destination = os.path.dirname(self._pars.dest_dir)
            basename_file = ""
        else:
            #Destination directory is given as an absolute path
            host_destination = self._pars.dest_dir

            #Create the destination directory
            if not os.path.exists(host_destination):
                self._logger.debug("{0}: make {1} folder".format(self._pars.id, host_destination))
                os.mkdir(host_destination)

        if self._pars.type == 'media' or self._pars.type == "bin":
            shutil.copy (self._pars.src_file_path, host_destination)
        elif zipped:
            tfile = tarfile.open(self._pars.src_file_path)
            try:
                tfile.extractall(host_destination)
            except IOError as e:
                try:
                    # Have seen it fail the first time before due to "Text file busy" error of one member.
                    tfile.extractall(host_destination)
                except IOError as e:
                    # Two times should have been enough tries.
                    self._logger.error("{0}: IOError happened two times trying to extract {1} to {2}!  Raising exception!\nI/o error({3}):  {4}".format(self._pars.id, basename_file, host_destination, e.errno, e.strerror))
                    raise
                except:
                    self._logger.error("{0}: Unexpected error occured while trying to extract {1} to {2}\n".format(self._pars.id, basename_file, host_destination) + traceback.format_exc())
                    raise
            except:
                self._logger.error("Unexpected error occured while trying to extract {0} to {1}\n".format(basename_file, host_destination) + traceback.formate_exc())
                raise
            tfile.close()

        if os.path.exists(host_destination):
            msg = "{0}: Installed {1} to {2}".format(self._pars.id, basename_file, host_destination)
            self._logger.info(msg)
            context.set_info(self._pars.destination_stored_path, os.path.join(host_destination, basename_file))
        else:
            msg = "{0}: Failed to install {1} to {2}".format(self._pars.id, basename_file, host_destination)
            self._logger.info(msg)
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, msg)
