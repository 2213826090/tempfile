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

@summary: This file implements a Test Step to compare given MD5 values to calculated MD5 values from files in a given directory
@since 25 Aug 2014
@author: Jongyoon Choi
@organization: INTEL PEG-SVE-DSV
"""
import hashlib
import os
import shutil
from Core.TestStep.DeviceTestStepBase import TestStepBase
from ErrorHandling.DeviceException import DeviceException
from Device.DeviceManager import DeviceManager

class CompareMD5Values(TestStepBase):
    """
    Implements a Test Step to compare given MD5 values to calculated MD5 values from files in a given directory
    """

    def run(self, context):
        """
        Compare MD5 values

        :type context: TestStepContext
        :param context: test case context
        """
        TestStepBase.run(self, context)

        self._extension  = self._pars.extension

        # Get the target directory path
        self._targetPath  = self._pars.target_path

        # Calculate the MD5 checksum for each file downloaded and compare it with the
        # checksum value stored in the corresponding *.md5 file
        totalDataDl_Bytes = 0
        for dlFile in os.listdir(self._targetPath):
            if dlFile.endswith('.{0}'.format(self._extension)):
                md5_file = dlFile.rsplit('.',1)[0]+'.md5'

                localChecksum = hashlib.md5(open(os.path.join(self._targetPath,dlFile)).read()).hexdigest()
                try:
                    checksum = open(os.path.join(self._targetPath,md5_file)).read()
                    if localChecksum != checksum:
                        #Get a report path to failed data file and md5 file
                        report_path =  self._device_manager.get_global_config().campaignConfig.get("campaignReportTree").get_report_path()
                        tc_name = self._tc_parameters.get_name()
                        report_root = os.path.join(report_path, tc_name, 'compare_MD5_values')

                        #Copy failed files
                        shutil.copy(os.path.join(self._targetPath,dlFile), os.path.join(report_root,dlFile))
                        shutil.copy(os.path.join(self._targetPath,md5_file), os.path.join(report_root,md5_file))

                        msg = "Checksum for {0} FAILED!.Expected MD5 value is {1}, but current MD5 value is {2}".format(dlFile, localChecksum, checksum)
                        raise DeviceException(DeviceException.OPERATION_FAILED, msg)
                    else:
                        self._logger.debug("Checksum for {0} PASSED!".format(dlFile))
                        totalDataDl_Bytes += os.path.getsize(os.path.join(self._targetPath,dlFile))
                except Exception, e:
                    msg = "Failed to open md5 sum file for {0}. The file download must have failed: {1}".format(dlFile,e)
                    raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # Remove all files downloaded for this test
        for dlFile in os.listdir(self._targetPath):
            try:
                os.remove(os.path.join(self._targetPath,dlFile))
            except Exception, e:
                msg = "Failed to remove downloaded files: {0}".format(e)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)