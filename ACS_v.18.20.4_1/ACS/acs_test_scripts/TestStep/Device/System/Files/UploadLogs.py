"""
@summary: This file implements a Test Step to use during TearDown, to upload
    logs created by scripts or non-standard applications.
@since 21 March 2014
@author: Val Peterson
@organization: INTEL PEG-SVE-DSV

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
"""


import os
import datetime
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase

class UploadLogs(DeviceTestStepBase):
    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self.file_api = self._device.get_uecmd("File")

    def run(self, context):
        DeviceTestStepBase.run(self, context)
        src_files = self._pars.file_path
        if src_files[-1] != '/':
            src_files = src_files + '/'
        src_files = src_files + self._pars.files
        #Conveniently, File.exist returns the output of the listing
        (result, listing) = self.file_api.find_files(self._pars.file_path, self._pars.files)
        if result and len(listing) > 0:
            report_root = self._device.get_report_tree().get_report_path()

            #When running multiple test cases in a campaign, we want to put these logs in a directory
            # under the test case report directory, in case the same test step is used in multiple
            # test cases within that campaign.
            testcase = self._testcase_name.split("\\")[-1]
            dest_dir = os.path.join(report_root, testcase, self._pars.host_subdir)
            if not os.path.exists(dest_dir):
                os.makedirs(dest_dir)

            #Build list of files to pull, since 'adb pull' doesn't like wildcards
            log_file_list = list()
            for filename in listing.split('\n'):
                #Depending on what was passed in to the "files" parameter, the directory listing may have
                #the full paths given, or it may just list the names of the files.  We will prefix it with
                #the directory name if that's not already there.
                if self._pars.file_path not in filename:
                    log_file_list.append(self._pars.file_path + '/' + filename)
                else:
                    log_file_list.append(filename)

            #Now pull the file(s) off of the DUT, and into our reports directory
            for filename in log_file_list:
                self._device.pull(filename, dest_dir, timeout=30)
            #Pass the destination directory up to a context variable in case a subsequent test step needs to know it
            context.set_info(self._pars.destination_stored_path, dest_dir)
        else:
            #Communicate to the caller that no files were uploaded, by setting DESTINATION_STORED_PATH to "None" string.
            context.set_info(self._pars.destination_stored_path, "None")
