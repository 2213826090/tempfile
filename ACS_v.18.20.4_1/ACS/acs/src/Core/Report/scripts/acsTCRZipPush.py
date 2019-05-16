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
:summary: Zip the source folder, get md5sum and push it to remote server
:author: jreynaud

"""
# pylint: disable=W0621

import hashlib
import json
import os
import re
import shutil
import socket
import time

from Core.Report.ACSLogging import LOGGER_FWK as LOGGER

CACHE_PUSH_BASE_FOLDER = "push_TCR"
LOCK = ".lock"


def get_subdirectories(dir_name):
    return [item for item in os.listdir(dir_name)
            if os.path.isdir(os.path.join(dir_name, item))]


class ZipFolderUtilities:

    @staticmethod
    def add_folder_to_zip(zip_file, folder, path_inside_zip):
        for root, _, files in os.walk(folder):
            for f in files:
                full_path = os.path.abspath(os.path.join(root, f))
                f_loc = full_path.rsplit(path_inside_zip, 1)[1].strip(os.path.sep)
                file_path = os.path.join(path_inside_zip, f_loc)
                LOGGER.info('File added: {0} as {1}'.format(full_path, file_path))
                zip_file.write(full_path, file_path)

    @staticmethod
    def add_file_to_zip(zip_file, file_name, file_path):
        full_path = os.path.abspath(os.path.join(file_path, file_name))
        path_inside_zip = os.path.relpath(full_path, os.path.abspath(file_path))
        LOGGER.info('File added: {0} as {1}'.format(full_path, path_inside_zip))
        zip_file.write(full_path, path_inside_zip)

    @staticmethod
    def zip_tcr_campaign_data(original_folder, dest_folder, folders_and_files):
        """
            This archive contains:
            dut log, acs logs
        """
        import zipfile
        try:
            acs_logfile_name = folders_and_files["acs_logfile_name"]
            tcr_live_reporting_logfile = folders_and_files["TCR_LIVE_REPORTING"]
            aplog_folder = folders_and_files["AP_LOGS"]
            bplog_folder = folders_and_files["LOGCAT_LOGS"]
            dbglog_folder = folders_and_files["DEBUG_LOGS"]
            logcat_folder = folders_and_files["BP_LOGS"]
            pti_folder = folders_and_files["PTI_LOGS"]
            serial_folder = folders_and_files["SERIAL_LOGS"]
            root_dut_name = folders_and_files["ROOT_DUT_NAME"]
            report_style_filename = folders_and_files["REPORT_STYLE_FILENAME"]
            filename = "{0}.zip".format(dest_folder)
            zip_file = zipfile.ZipFile(filename, 'w',
                                       compression=zipfile.ZIP_DEFLATED,
                                       allowZip64=True)
            LOGGER.info('Create TCR campaign zip file: {0}'.format(filename))

            ZipFolderUtilities.add_file_to_zip(zip_file, os.path.basename("{0}.log".format(acs_logfile_name)),
                                               original_folder)
            ZipFolderUtilities.add_file_to_zip(zip_file, os.path.basename("{0}.xml".format(acs_logfile_name)),
                                               original_folder)
            if os.path.exists(tcr_live_reporting_logfile):
                ZipFolderUtilities.add_file_to_zip(zip_file, tcr_live_reporting_logfile, original_folder)
            ZipFolderUtilities.add_file_to_zip(zip_file, report_style_filename, original_folder)

            for f in [f for f in get_subdirectories(original_folder) if f.startswith(root_dut_name)]:
                for local_path in ["{0}{2}{1}".format(f, folder, os.path.sep)
                                   for folder in (aplog_folder,
                                                  bplog_folder,
                                                  logcat_folder,
                                                  dbglog_folder,
                                                  pti_folder,
                                                  serial_folder)]:
                    ZipFolderUtilities.add_folder_to_zip(
                        zip_file, os.path.join(original_folder, local_path), local_path)
            zip_file.close()
            status = Global.SUCCESS
            out_file = os.path.abspath(filename)
        except IOError as error:
            LOGGER.error('Cannot create zip file: {0} - {1}'.format(filename, error))
            status = Global.FAILURE
            out_file = ""
        return status, out_file


def safe_remove_file(file_path, max_retry=5):
    """
    Removes the file.

    .. note:: Due to windows limitation, sometimes, the file cannot be removed immediately
        so implements retry loop.

    :type file_path: str
    :param file_path: The path to the folder/file to remove.

    :type max_retry: int
    :param max_retry: Max remove retry.

    :rtype: tuple
    :return: Status and output log
    """
    remove_ok = False
    retry = 1
    status = Global.FAILURE
    output = ''
    file_name = os.path.basename(file_path)

    while not remove_ok and retry <= max_retry:
        try:
            if os.path.isfile(file_path):
                os.remove(file_path)
            elif os.path.isdir(file_path):
                shutil.rmtree(file_path)
            remove_ok = True
            status = Global.SUCCESS
            output = 'Remove {0} OK after {1}/{2} tries'.format(file_name, str(retry), str(max_retry))
            LOGGER.info(output)
        except Exception as ex:  # pylint: disable=W0703
            time.sleep(1)
            retry += 1
            output = 'Fail to remove {0} after {1}/{2} tries ({ex})'.format(file_name,
                                                                            str(retry),
                                                                            str(max_retry), ex=ex)
            LOGGER.error(output)
    return status, output


class Global(object):

    """Global return value"""
    SUCCESS = 0
    FAILURE = -1


class TCRpush(object):

    """
    This class will do the push of a report to an TCR server
    """

    def __init__(self, original_acs_report, cache_reports,
                 metacampaign_uuid="", user_mail="", dev_campaign=False,
                 log_folders_and_files=""):
        self.original_report_path = original_acs_report
        self.user_mail = user_mail
        self.dev_campaign = dev_campaign
        self.cache_reports = cache_reports
        # metacampaign_uuid of the campaign execution
        self.metacampaign_uuid = metacampaign_uuid
        # computed full path of new report (without extension)
        self.new_report_path = ""
        # path reports as zip
        self.zip_acs_report_tcr = ""
        # path checksum file associated to the zip
        self.md5_acs_report_tcr = ""
        self._lock_file = None
        self.report_name = ""
        # timeout for pushing to server
        self._log_folders_and_files = log_folders_and_files

    def __zip_analysis(self, zip_status, zip_output):
        if zip_status != Global.SUCCESS:
            LOGGER.error("Archiving file FAIL, please check log file for more details")
            status = Global.FAILURE
        else:
            LOGGER.info("Archiving file OK, zip file size is {}".format(self._get_file_size(zip_output)))
            # Create md5sum file and retrieve md5sum file path
            LOGGER.info("Create md5sum file from previously created archive ...")

            (status, output) = self.create_md5sum_file(zip_output)

            if status == Global.SUCCESS:
                LOGGER.info("Md5sum file creation OK")
                self.md5_acs_report_tcr = output
            else:
                LOGGER.error("Md5sum file creation FAIL, please check log file for more details")
                status = Global.FAILURE

        msg = "File are not ready to upload!"
        if status == Global.SUCCESS:
            msg = "File are ready to upload!"

        return status, msg

    def _get_file_size(self, file_path):
        """
        Get size of file given as argument

        :type file_path: str
        :param file_path: path to the file to get size

        :rtype: string
        :return: file size
        """
        try:
            file_size = os.path.getsize(file_path)
        except os.error:
            return "unknown"
        if not isinstance(file_size, (int, long)) or file_size < 0:
            return "unknown"
        for unit in ['', 'K', 'M']:
            if file_size < 1024:
                return "%.1f %sB" % (file_size, unit)
            file_size /= 1024.0
        return "%.1f %sB" % (file_size, 'G')

    def prepare_files(self):
        """
        Create report zip file and md5 files

        :rtype: tuple
        :return: Status and output log
        """
        # Zip folder and retrieve zip file path
        LOGGER.info("Create TCR archive")
        status, self.zip_acs_report_tcr = ZipFolderUtilities.zip_tcr_campaign_data(
            self.original_report_path, self.new_report_path, self._log_folders_and_files)
        return self.__zip_analysis(status, self.zip_acs_report_tcr)

    def dump_json_infos(self):
        """
        Dump all class attributes in a json file.
        This json file will store all necessaries information to reproduce the push if current one fails.

        :rtype: None
        :return: None
        """
        json_path = self.new_report_path + ".json"
        if os.path.exists(json_path):
            os.remove(json_path)
        with open(json_path, 'w') as json_file:
            json.dump(self.__dict__, json_file, indent=4)

    def load_json_info(self, json_path):
        """
        Load all public class attributes from a json file.

        :type json_path: str
        :param json_path: path to a json file

        :rtype: None
        :return: None
        """

        def remove_protected_and_private_attrib(obj):
            filtered_dict = dict((key, value) for key, value in obj.iteritems() if not key.startswith("_"))
            return filtered_dict

        if os.path.exists(json_path):
            with open(json_path, 'r') as json_file:
                try:
                    self.__dict__.update(json.load(json_file, object_hook=remove_protected_and_private_attrib))
                except Exception as ex:
                    LOGGER.error("Cannot load the json file %s (%s)" % (json_file, str(ex)))
                    return None
                return True

    def build_file_name(self, src_folder_path):
        """
        Creates a file name as:
        <Hostname>_<user>_[file_name]

        :type src_folder_path: str
        :param src_folder_path: The original folder path

        :rtype: tuple
        :return: status and output log (final file name)
        """
        # Compute the file name
        LOGGER.info("Build standardized file name ...")
        # If folder ending character is present, should remove it before treatment
        if src_folder_path[-1:] in ('\\\\', '\\', '/'):
            LOGGER.info("Removing tailing character from folder name parameter")
            file_name = str(os.path.basename(src_folder_path[:-1]))
        else:
            file_name = str(os.path.basename(src_folder_path))

        if file_name in (None, ''):
            return Global.FAILURE, "Unable to build file name from \"" + str(src_folder_path) + "\""

        expr = "(?P<user>([\.-_0-9a-zA-Z]).*)@.*"  # pylint: disable=W1401
        # Test matching correct email
        matches_str = re.compile(expr).search(str(self.user_mail))
        # Return True or False according matching result
        if matches_str is not None:
            user = matches_str.group("user")
        else:
            return Global.FAILURE, "Unable to compute user from email " + str(self.user_mail)

        output_file_name = '{0}_{1}_{2}'.format(str(socket.gethostname()), user, file_name)
        LOGGER.info("FILE NAME: %s" % output_file_name)
        return Global.SUCCESS, output_file_name

    @classmethod
    def _get_md5sum(cls, file_path, block_size=256 * 128):
        """
        Gets md5sum of input file

        :param file_path: The full file path
        :type file_path: str

        :param block_size: define block size for reading file by chunk by chunk (mandatory for huge files)
        :type block_size: int

        :rtype: tuple
        :return: status and output log (The md5 hash)
        """
        try:
            hex_value = None
            md5hash = hashlib.md5()  # pylint: disable=E1101
            with open(file_path, 'rb') as f:
                for chunk in iter(lambda: f.read(block_size), b''):
                    md5hash.update(chunk)
                hex_value = md5hash.hexdigest()
        except Exception as ex:  # pylint: disable=W0703
            error_msg = "GET_MD5: Fail, %s" % str(ex)
            LOGGER.error(error_msg)
            return Global.FAILURE, error_msg

        LOGGER.info("MD5 HASH: " + hex_value)
        return Global.SUCCESS, hex_value

    def create_md5sum_file(self, file_path):
        """
        Create md5sum file of input file

        :param file_path: The full file path
        :type file_path: str

        :rtype: tuple
        :return: status and output log (The md5sum file path and hash value)
        """
        try:
            md5hash = self._get_md5sum(file_path)[1]

            md5sum_file_name = file_path + ".md5sum"
            file_name = os.path.basename(file_path)

            md5sum_file = open(md5sum_file_name, 'w')
            md5sum_file.write(md5hash)
            md5sum_file.write(' ')
            md5sum_file.write(file_name)
            md5sum_file.write('\r\n')
            md5sum_file.close()
        except Exception as ex:  # pylint: disable=W0703
            error_msg = "CREATE_MD5_FILE: Fail, " + str(ex)
            LOGGER.error(error_msg)
            return Global.FAILURE, error_msg

        LOGGER.info("MD5SUM FILE: %s" % md5sum_file_name)
        return Global.SUCCESS, md5sum_file_name

    def setup(self):
        """
        Setup files which will be push to TCR server

        :rtype: tuple
        :return: Status and output log
        """
        # Check original acs report path
        status = Global.FAILURE
        msg = ""
        if not os.path.exists(self.original_report_path):
            msg = "Cannot retrieve original ACS results: %s" % self.original_report_path
            LOGGER.error(msg)
            status = Global.FAILURE
        else:
            # compute the report folder name (used to generate html file)
            if not self.report_name:
                # If folder ending character is present, should remove it before treatment
                if self.original_report_path[-1:] in ('\\\\', '\\', '/'):
                    self.report_name = str(os.path.basename(self.original_report_path[:-1]))
                else:
                    self.report_name = str(os.path.basename(self.original_report_path))

            if not self.new_report_path:
                (status, output) = self.build_file_name(self.original_report_path)
                if status == Global.SUCCESS:
                    LOGGER.info("Building file name OK")
                    # each push must have its own dedicated directory
                    # it will be easier to manage reports cache like this : each folder
                    # contains zip + md5 file + json info
                    sub_folder_push = os.path.join(
                        self.cache_reports, "%s_%s" % (CACHE_PUSH_BASE_FOLDER,
                                                       time.strftime("%Y-%m-%d_%Hh%M.%S")))
                    self._lock_file = os.path.join(sub_folder_push, LOCK)
                    if not os.path.isdir(sub_folder_push):
                        os.makedirs(sub_folder_push)
                    with open(self._lock_file, 'w') as lock:
                        lock.write("locked")
                    # output contains computed report dirname
                    self.new_report_path = os.path.join(sub_folder_push, output)
                    # create report zip file & md5 file associated
                    status, msg = self.prepare_files()
                else:
                    msg = "Building file name FAIL: %s" % output
                    status = Global.FAILURE
        if status == Global.SUCCESS:
            msg = "TCR push SETUP : OK"
        return status, msg

    def start(self):
        """
        Upload md5 & report zip files to specified url
        Generate in report dir a html file which redirects to the TCR campaign result

        :rtype: tuple
        :return: Status and output log
        """
        msg = ""
        (status, _) = self.upload_all_files()
        # Create the shortcut
        if status != Global.SUCCESS:
            status = Global.FAILURE
            msg = "Could not upload files to TCR website"
            LOGGER.info(msg)
        return status, msg
