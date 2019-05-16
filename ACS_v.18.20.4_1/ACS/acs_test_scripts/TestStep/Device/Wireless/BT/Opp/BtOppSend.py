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
:summary: This file implements a Test Step that Send files via OPP
:since:18/12/2013
:author: fbongiax
"""

import posixpath
from UtilitiesFWK.Utilities import split_and_strip
from acs_test_scripts.TestStep.Device.Wireless.BT.Constants import Constants
from acs_test_scripts.TestStep.Device.Wireless.BT.Base import BtBase


class BtOppSend(BtBase):
    """
    Implements the test step to send files via OPP
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Constructor
        """

        BtBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """

        BtBase.run(self, context)

        multimedia_path = self._device.multimedia_path
        address = self._pars.bdaddr

        # Split the file names and strip each of them
        file_list = split_and_strip(self._pars.files, Constants.FILE_NAME_SEPARATOR)

        fullpath_filenames, file_size = self._process_files(multimedia_path, file_list, False)

        self._api.bt_opp_clean_notification_list()

        self._save_file_size_if_needed(file_size, context)
        self._save_file_chksum_if_needed(multimedia_path, file_list, context)

        # Request for the First transfer to be registered
        self._api.bt_opp_send_file(fullpath_filenames, address)


    def _save_file_size_if_needed(self, file_size, context):
        """
        Save files size in the context if needed
        """
        if self._pars.save_info_as:
            context.set_nested_info([self._pars.save_info_as, Constants.OPP_INFO_FILE_SIZE], file_size)

    def _save_file_chksum_if_needed(self, path, filenames, context):
        """
        Save files checksum in the context if needed
        """
        if self._pars.save_info_as:
            items = self._api.bt_opp_get_files_checksum(path, filenames)
            context.set_nested_info([self._pars.save_info_as, Constants.OPP_INFO_FILE_CHKSUM], str(items))

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
                    self._raise_device_exception("Filesize is not accessible. File is probably missing on the device")

                total_size += size
                full_files.append(file_name)

        final_list = ""
        for file_name in full_files:
            if not reverse:
                final_list = final_list + file_name + Constants.FILE_NAME_SEPARATOR
            else:
                final_list = file_name + Constants.FILE_NAME_SEPARATOR + final_list

        assert final_list[-1:] == Constants.FILE_NAME_SEPARATOR, "The calculated list must always end with a separator"

        # remove the last "separator"
        final_list = final_list[:-1]

        return final_list, total_size
