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
:summary: Utilities class for Mtp Sync implementation
:since: 07/01/2014
:author: Jongyoon Choi
"""
import time
import sys
import hashlib
import errno
import os
from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT
from ErrorHandling.DeviceException import DeviceException

class MtpUtility:
    """
    This class contains all the options related to resetting the phone
    """
    def __init__(self, testRunnerConfig):
        """
        Constructor
        """
        # Instantiate the logger
        self._logger = LOGGER_TEST_SCRIPT

        self.configuration = testRunnerConfig

        self.target_device = None
        self.files_system_cache = None
        self.root_filesystem_id = None
        self.current_os = None
        self.current_search = None

    def check_folder_match(self, folder_name, item_to_check, target_folder_path):
        """
        checks to see if the given item matches our current folder and returns a descriptor of that folder

        :folder_name: the name of the folder that we are trying to match
        :item_to_check: the folder or file descriptor that we're checking against (we technically don't know what it is yet)
        :target_folder_path: the target path that is being matched. This is only used when a conflict occurs and a we want to throw an informative error.

        returns (True, folder descriptor) when a match
        """
        matched_at_level = False
        folder_descriptor = None

        folder_name_match = (item_to_check['name'] == folder_name)  # do we have a match
        self._logger.debug("Checking for match. Result: " + str(folder_name_match) + ", Target folder: " + folder_name + ", Current_item['name']: " + item_to_check['name'])
        if folder_name_match:
            matched_at_level = True
            folder_descriptor = item_to_check

            if not item_to_check['is_folder']:
                # This is bad. We're not supposed to be matching with a file
                error_message = "ERROR! Cannot find or create the requested folder. A file with the same name already exists in the path.  File was: '" + item_to_check['name'] + "', Full path was: '" + target_folder_path + "'"
                self._logger.error(error_message)
                raise DeviceException(DeviceException.OPERATION_FAILED, error_message)

        return matched_at_level, folder_descriptor

    def acquire_target_device(self):
        """
        Uses WPD on Windows to acquire the target device that will be tested using MTP operations.
        """
        #DELETE ME - self.files_system_cache = None  # clear the cache whenever we attache to a new device

        self.current_os = sys.platform

        sys.path.append(self.configuration.test_files_path)

        if self.current_os == "win32":
            import windows_mtp #@UnresolvedImport

            self.target_device = windows_mtp.MtpDevice(self._logger.info, self._logger.debug, self._logger.error)
        elif self.current_os == "linux2":
            import linux_mtp #@UnresolvedImport

            self.target_device = linux_mtp.MtpDevice(self._logger.info, self._logger.debug, self._logger.error)
        else:
            raise DeviceException(DeviceException.OPERATION_FAILED,"ERROR! The current OS is not supported. Legal values are win32 or linux2. Current OS is: " + str(self.current_os))

        if self.target_device is None:
            raise DeviceException(DeviceException.OPERATION_FAILED,"ERROR! The required target device is None. Please check that your target is connected and powered on.")

        self._logger.debug(os.linesep + "Fetching device data")
        self._logger.debug(self.target_device.data)
        self._logger.debug(os.linesep)

    def check_for_local_media_files(self, files_to_sync):
        """
        Verifies that the files that will be synced exist on the local host (and copies from remote if needed).

        :type files_to_sync: list
        :param files_to_sync: the list of MediaFile objects whose signatures will be generated
        """
        for target_media_file in files_to_sync:
            self._logger.info("Checking for media file: " + target_media_file.local_path)
            if os.path.exists(target_media_file.local_path):
                self._logger.info("Local copy found.")
            else:
                self._logger.info("Local copy NOT found... Aborting test.")
                raise DeviceException(DeviceException.OPERATION_FAILED,"ERROR! Could not find media file: " + target_media_file.local_path)

    def generate_media_file_signatures(self, files_to_sync):
        """
        Generates the signatures that will be used to verify the correctness of the files after the sync operation.

        :type files_to_sync: list
        :param files_to_sync: the list of MediaFile objects whose signatures will be generated
        """
        for file_to_hash in files_to_sync:
            self._logger.debug("getting digest for " + file_to_hash.local_path)
            file_to_hash.generate_hex_digest(hashlib.md5())
            self._logger.info("digest for " + file_to_hash.local_path + " is " + file_to_hash.digest)

    def sync_using_mtp_over_usb(self):
        """
        Performs a single iteration of the sync operation using MTP over USB.

        :return: (pass_count, fail_count) - a tuple of the current iteration's pass count and fail count
        """
        self._logger.info("**** Cleaning up before sync (just in case a previous run crashed)")
        self._logger.info("**** Cleaning up old target files...")
        self.cleanup_before_sync(self.configuration.target_media_files)  # we can't push files to the target if the old file are still there
        self._logger.info("**** Cleaning up old synced files...")
        self.cleanup_synced_files(self.configuration.target_media_files)
        self._logger.info("**** Pre-sync clean up complete")

        self._logger.info(os.linesep)

        self._logger.info("**** Pushing files to the target")
        self.push_files_to_target(self.configuration.target_media_files)
        self._logger.info("**** Push complete")

        self._logger.info(os.linesep)

        self._logger.info("**** Pulling files from the target...")
        self.pull_files_from_target(self.configuration.target_media_files)
        self._logger.info("**** Pull complete")

        self._logger.info(os.linesep)

        self._logger.info("**** Cleaning up the target...")
        self.delete_files_on_target(self.configuration.target_media_files, True)
        self._logger.info("**** Clean up of target files is complete")

        self._logger.info(os.linesep)

        self._logger.info("**** Checking integrity of synced files...")
        sync_pass_count, sync_fail_count = self.check_sync_file_integrity(self.configuration.target_media_files)
        self._logger.info("**** Integrity check complete")

        self._logger.info(os.linesep)

        self._logger.info("**** Cleaning up synced files...")
        self.cleanup_synced_files(self.configuration.target_media_files)
        self._logger.info("**** Clean up of synced files complete")

        return sync_pass_count, sync_fail_count

    def cleanup_before_sync(self, files_to_cleanup):
        """
        Cleans up the target by checking for the requested media files and deleting them if they exist.

        :type files_to_check: list
        :param files_to_check: list of MediaFile objects that will be cleaned up
        """
        for file_to_cleanup in files_to_cleanup:
            cleanup_path = file_to_cleanup.target_file_path
            if MtpUtility.is_none_or_empty(cleanup_path):
                raise Exception(file_to_cleanup.file_name + " doesn't have a target_file_path")

            file_exists, file_descriptor = self.target_file_exists(cleanup_path)  #@UnusedVariable

            if file_exists:
                self.delete_file_on_target(file_to_cleanup)
            else:
                self._logger.info(file_to_cleanup.file_name + " - skipping cleanup. File doesn't exist on target")

    def check_sync_file_integrity(self, files_to_check):
        """
        Checks the integrity of the files that were pulled from the target by comparing their signatures against stored signatures.

        :type files_to_check: list
        :param files_to_check: list of MediaFile objects that will be cleaned up
        """
        fail_count = 0
        pass_count = 0
        for file_to_check in files_to_check:
            pulled_file_path = file_to_check.pull_file_path()
            self._logger.info("Checking " + file_to_check.file_name + "(original: " + file_to_check.local_path + ", sync'ed: " + pulled_file_path + ")")
            expected_digest = file_to_check.digest  # this is the  original digest before the test was run
            actual_digest = MediaFile.get_hex_digest(pulled_file_path, hashlib.md5())
            result_message = "- expected digest: '" + expected_digest + "', actual digest: '" + actual_digest + "'"
            if actual_digest != expected_digest:
                fail_count += 1
                self._logger.info("Mismatch! " + result_message)
            else:
                pass_count += 1
                self._logger.info("Match " + result_message)

        return pass_count, fail_count

    def cleanup_synced_files(self, target_media_files):
        """
        Clean up the synced files on the host PC.

        :type target_media_files: list
        :param target_media_files: list of MediaFile objects that will be cleaned up
        """
        for target_file in target_media_files:
            sync_file_path = target_file.pull_file_path()
            if os.path.exists(sync_file_path):
                self._logger.info("Deleting sync file:  " + sync_file_path)
                os.remove(sync_file_path)
            else:
                self._logger.info("Skipping sync file delete (file doesn't exist): " + sync_file_path)

    def filesystem_cache_callback(self, obj, level):
        """
        The callback function that will be used for building the filesystem cache.

        :obj: the descriptor of the item at the current level
        :level: a number that indicates the level of the current object in the file system.
        """
        level_count = 0
        prefix = ''
        while level_count < level:
            prefix += "\t"
            level_count += 1
        self._logger.info(prefix + '- ' + obj.get('name', '') + ", id: " + obj.get('id', '') + ", level: " + str(level))

        return True  # return True to iterate through sub-folders when fetching the filesystem cache

    def fetch_root_filesystem_id(self):
        """
        Fetches the root file system id that will be used to transfer files to and from the target.
        """
        self._logger.info("Fetching the target's root filesystem id...")
        storage_keys = self.target_device.data.get('storage', [])

        if len(storage_keys) <= 0:
            raise Exception("ERROR! Failed to fetch storage data from the target device. Please check whether the target has an SDCARD installed.")

        self._logger.info("storage_keys[0]: " + str(storage_keys[0]))
        # we'll stick with the first storage device that was found for now
        self.root_filesystem_id = storage_keys[0].get('id')
        self._logger.info("Root filesystem id is: " + str(self.root_filesystem_id))
        if MtpUtility.is_none_or_empty(self.root_filesystem_id):
            raise Exception("ERROR! Failed to fetch the name of the main hd from the target device")

    def push_files_to_target(self, files_to_sync):
        """
        Push the specified files to the target

        :files_to_sync: list of MediaFile objects that will be pushed to the target
        """
        for target_file in files_to_sync:
            self._logger.debug("target_file: " + str(target_file))
            self._logger.debug("target_file: " + str(target_file.target_file_path))
            parent_path = self.parent_path(target_file.target_file_path)
            self._logger.debug("parent_path: " + parent_path)
            parent_folder = self.find_folder(parent_path, create_path_if_not_found=True)

            if parent_folder is None:
                # we can't keep going if the path we need doesn't exist on the target
                error = OSError()
                error.filename = parent_path
                error.errno = errno.ENOENT
                error_message = "The requested parent path was not found. Path was '" + parent_path + "'"
                error.strerror = error_message
                error.message = error_message
                raise error

            with open(target_file.local_path, 'rb') as push_file_stream:
                # open a file stream in read binary mode
                self._logger.info("Pushing " + target_file.file_name + " to " + parent_folder['name'])
                self.clear_progress_indicators()
                target_file.target_file_descriptor = self.target_device.put_file(self.root_filesystem_id,
                                                                                 parent_folder['id'],
                                                                                 target_file.file_name,
                                                                                 push_file_stream,
                                                                                 os.path.getsize(target_file.local_path),
                                                                                 self.file_transfer_progress_callback)
                self._logger.debug("Done")
                if target_file.target_file_descriptor is None:
                    raise Exception("ERROR! Failed to receive a valid file descriptor when pushing " + target_file.file_name + " to " + parent_folder)

    def parent_path(self, file_path):
        """
        generates the path of the parent folder for the given file

        :file_path: full path to the file whose parent will be returned
        """
        if MtpUtility.is_none_or_empty(file_path):
            path_value = ""
            if file_path is None:
                path_value = "None"

            error = OSError()
            error.filename = file_path
            error.errno = errno.ENOENT
            error.strerror = "ERROR! The required file_path is '" + path_value + "'"
            error.message = "ERROR! The required file_path is '" + path_value + "'"
            raise error

        self._logger.debug("getting parent path from: '" + file_path + "'")

        return os.path.split(file_path)[0]

    def file_name(self, file_path):
        """
        extracts the name of the file from the given path

        :file_path: the full path to the target file
        """
        if MtpUtility.is_none_or_empty(file_path):
            path_value = ""
            if file_path is None:
                path_value = "None"

            error = OSError()
            error.filename = file_path
            error.errno = errno.ENOENT
            error.strerror = "ERROR! The required file_path is '" + path_value + "'"
            error.message = "ERROR! The required file_path is '" + path_value + "'"
            raise error

        self._logger.debug("getting file name from: '" + file_path + "'")

        return os.path.split(file_path)[1]

    @staticmethod
    def split_folder_levels(folder_path):
        """
        splits the folder levels so that a function like 'find_or_create_folder' can walk the directory tree

        :folder_path: the path that will be split into a list of path_levels
        """
        path_levels = folder_path.split(r"/")
        path_levels = filter(None, path_levels)  # remove invalid or empty path levels
        return path_levels

    def find_folder(self, target_folder_path, create_path_if_not_found=True):
        """
        Finds or creates the specified target_folder_path on the target.
        Returns the descriptor of the folder or None if the folder doesn't exist

        :target_folder_path: the path on the target that we are trying to find
        :create_path_if_not_found: when True (default), this function will create the path if it is not found
        """

        if MtpUtility.is_none_or_empty(target_folder_path):
            path_value = ""
            if target_folder_path is None:
                path_value = "None"

            error = OSError()
            error.filename = target_folder_path
            error.errno = errno.ENOENT
            error.strerror = "ERROR! The required file_path is '" + path_value + "'"
            error.message = "ERROR! The required file_path is '" + path_value + "'"
            raise error

        self._logger.debug("target_folder_path: " + target_folder_path)
        self.current_search = MtpSearch(target_folder_path)
        self.target_device.get_filesystem(self.root_filesystem_id, self.find_file_callback)

        if self.current_search.resulting_entry is None:
            self._logger.debug("We did not find a file or a part of the path required to walk to the file")

            if not create_path_if_not_found:
                self._logger.debug("We did not find a match and the user doesn't want us to create a folder... The folder doesn't exist.")
            else:
                parent_id = self.root_filesystem_id
                self._logger.debug("self.current_search.level: " + str(self.current_search.levels))
                for create_level in self.current_search.levels:
                    if create_level['descriptor'] is None:
                        # no entry was found at this level... create it
                        self._logger.debug("Creating the level (parent_id: " + str(parent_id) + ", current_folder_name: " + create_level['name'] + ")")
                        # create the folder and populate the level in the current_search... at the end the resulting_entry will point at our target folder
                        create_level['descriptor'] = self.target_device.create_folder(self.root_filesystem_id, parent_id, create_level['name'])
                    else:
                        # an entry was found at this level... capture the parent_id and move onto the next level
                        parent_id = create_level['descriptor']['id']

        self._logger.debug("Resulting entry: " + str(self.current_search.resulting_entry))
        return self.current_search.resulting_entry

    @staticmethod
    def list_or_dictionary_iterator(obj):
        if isinstance(obj, dict):
            for key in obj:
                yield key
        else:
            for index, value in enumerate(obj):  #@UnusedVariable
                yield index

    def create_folder_on_target(self, parent_id, folder_name):
        """
        create a given folder on the target

        :parent_id: the id of the parent folder
        :folder_name: the name of the folder that will be created
        """
        self._logger.debug("executing commmand: self.target_device.create_folder(" + str(parent_id) + ", " + str(folder_name) + ")")
        return self.target_device.create_folder(self.root_filesystem_id, parent_id, folder_name)

    def clear_progress_indicators(self):
        """
        clears the variables that are used to show file operation progress (ie. file upload) to a user
        """
        self.last_transfer_bytes = 0
        self.next_status_level = 0
        self.percent_points_per_print = 10  # the script will print progress at 10 percent intervals

    def file_transfer_progress_callback(self, bytes_written, total_size):
        """
        Use this to report progress when doing push or pull file transactions

        :bytes_written: indicates the total number of bytes written (it is possible for multiple callbacks to occur where the bytes_written doesn't change)
        :total_size: indicates the total number of bytes in the transaction
        """
        if self.last_transfer_bytes == bytes_written:
            # no progress has been made
            return

        self.last_transfer_bytes = bytes_written

        # calculate and print progress
        percent_complete = round(bytes_written/total_size * 100, 2)
        if percent_complete > self.next_status_level:
            self._logger.info(str(percent_complete) + "% complete (" + str(bytes_written) + "/" + str(total_size) + " bytes)")
            self.next_status_level += self.percent_points_per_print

    def pull_files_from_target(self, files_to_sync):
        """
        Pull the specified files from the target

        :files_to_sync: list of MediaFile objects that will be pulled from the target
        """
        for target_file in files_to_sync:
            parent_folder = self.parent_path(target_file.target_file_path)
            pulled_file_path = target_file.pull_file_path()
            with open(pulled_file_path, 'wb') as pull_file_stream:
                # open a file stream in read binary mode
                self._logger.info("Pulling " + target_file.file_name + " (target path: " + parent_folder + ", local path: " + pulled_file_path + ")")
                self.clear_progress_indicators()
                self.target_device.get_file(target_file.target_file_descriptor['id'], pull_file_stream, self.file_transfer_progress_callback)
                self._logger.debug("Done")

    def delete_files_on_target(self, files_to_sync, errors_are_fatal=False):
        """
        Delete the specified files from the target

        if errors_are_fatal=True, any error will cause a fatal exception. This should be the case where the files have to exist on the target.
        if errors_are_fatal=False (Default), any error will be treated as a warning. This should be the case when trying to clean the system at the beginning of a test
        """
        failures = []

        for target_file in files_to_sync:
            try:
                self.delete_file_on_target(target_file)
            except Exception as e:
                # this is normally fatal but, we want to collect all of the errors into one single error message so that a user can fix them in one shot
                failures.append(str(e))

        num_failures = len(failures)
        if num_failures > 0:
            error_message = "Failed to delete " + str(num_failures) + " files with the following messages: " + os.linesep
            for failure in failures:
                error_message += "\t" + failure

            if errors_are_fatal:
                raise Exception("ERROR! " + error_message)

            self._logger.info("Warning... " + error_message)

    def delete_file_on_target(self, file_to_delete):
        """
        delete the specified file on the target
        :file_to_delete: MediaFiLe object that will be deleted from the target. It must include a valid target_file_path
        """

        timeout_sec = 60

        target_file_path = file_to_delete.target_file_path
        if MtpUtility.is_none_or_empty(target_file_path):
            raise Exception(file_to_delete.file_name + " doesn't have a target_file_path")

        # Give the file up to <timeout_sec> seconds to appear
        startTime = time.time()
        file_exists, file_descriptor = self.target_file_exists(target_file_path)
        while not file_exists:
            time.sleep(1)
            file_exists, file_descriptor = self.target_file_exists(target_file_path)
            if (time.time()-startTime) > timeout_sec:
                break
        if file_exists:
            self._logger.info("Deleting " + file_descriptor['name'] + " (" + target_file_path + ")")
            self.delete_target_file(file_descriptor['id'])
        else:
            raise Exception(file_to_delete.file_name + " - File not found on target. (" + target_file_path + ")")

    def target_file_exists(self, path_to_file_on_target):
        """
        returns True and the file descriptor if the file exists
        return False and None if the file doesn't exist
        """
        file_descriptor = self.find_file(path_to_file_on_target)
        return file_descriptor is not None, file_descriptor

    def find_file(self, path_to_file_on_target):
        """
        searches for the given file

        returns the descriptor if the file was found
        return None if the file doesn't exist

        raise a MtpSync.PathConflictException if a folder with the same name is found at the level where the file is supposed to exist
        """

        self._logger.debug("path_to_file_on_target: " + path_to_file_on_target)
        self.current_search = MtpSearch(path_to_file_on_target)
        self.target_device.get_filesystem(self.root_filesystem_id, self.find_file_callback)

        if self.current_search.resulting_entry is None:
            self._logger.debug("We did not find the file or a part of the path required to walk to the file")
        else:
            self._logger.debug("Resulting entry: " + str(self.current_search.resulting_entry))

        return self.current_search.resulting_entry

    def find_file_callback(self, entry, level):
        """
        In this callback, we are trying to find a file so we don't want to iterate recursively. We do this by returning False
        """
        #return False - temporarily changed - put this back!

        recurse_through_current_folder = False  # assume that we haven't matched our target entry and therefore don't want to walk the tree for the current entry

        if self.current_search.levels[level]['name'] == entry['name']:
            self._logger.debug("MATCH! - entry: " + entry['name'] + ", parent_id: " + str(entry['parent_id']))
            self.current_search.levels[level]['descriptor'] = entry  # populate the entry for actions that may occur after the search (like creating folders)
            if level < (len(self.current_search.levels) - 1):
                recurse_through_current_folder = True  # we haven't reached our target level yet.. continue down the rabbit hole

        return recurse_through_current_folder

    def delete_target_file(self, id_of_file_to_delete):
        self._logger.debug("Deleting file with id: " + str(id_of_file_to_delete))
        self.target_device.delete_object(id_of_file_to_delete)
        self._logger.debug("Done")

    def calculate_total_run_time(self, start_time):
        """ calculates the difference between the start and end of the test """
        run_stop = float(time.time())
        sr = round((run_stop - start_time), 0)
        s = int("%d"%sr)
        hours = s // 3600
        s = s - (hours * 3600)
        minutes = s//60
        seconds = s - (minutes * 60)
        run_duration = '%sh:%sm:%ss' % (hours, minutes, seconds)
        self.Logger.info("Test duration for Loop " + str(self.loop_count) + ": " + str(run_duration))

    @staticmethod
    def is_none_or_empty(string_to_check):
        """
        determines whether a given string is None or empty
        """
        return string_to_check is None or string_to_check == ''


class MtpSearch(object):
    """
    Contains all of the items needed to search for a file or folder along with the result
    """
    def __init__(self, path_to_search):
        self.path = path_to_search

        self.__levels = []
        level_names = MtpSearch.split_folder_levels(self.path)
        for level_name in level_names:
            self.__levels.append({'name': level_name, 'descriptor': None})

    @staticmethod
    def split_folder_levels(folder_path):
        """
        splits the folder levels so that a function like 'find_or_create_folder' can walk the directory tree

        :folder_path: the path that will be split into a list of path_levels
        """
        path_levels = folder_path.split(r"/")
        path_levels = filter(None, path_levels)  # remove invalid or empty path levels
        return path_levels

    @property
    def levels(self):
        """
        returns an array of {name, descriptor} dictionaries that represents all of the levels in this search.
        """
        return self.__levels

    @property
    def resulting_entry(self):
        """
        if there was a match, this returns the matching entry
        if there was no match, this returns None (the default value of all of the levels)
        """
        target_level = len(self.__levels) - 1
        return self.__levels[target_level]['descriptor']


class MtpSyncConfiguration(object):
    """
    Configuration for the MtpSync class.
     - establishes defaults and allows for overrides
    """
    def __init__(self):
        self.test_files_path = None
        self.test_content_path = None
        self.test_log_dir = None
        self.test_duration_minutes = 1
        self.max_test_iterations = -1
        self.target_media_files = []
        self.print_to_console = False

    def __str__(self):
        if (self.max_test_iterations <= 0) and (self.test_duration_minutes > 0):
            string = """
               test_duration_minutes: %(tdm)s
               test_log_path: %(tlp)s
               print_to_console: %(ptc)s
               """ % {"tdm": str(self.test_duration_minutes),
                      "tlp": str(self.test_log_dir),
                      "ptc": str(self.print_to_console)}
        elif (self.max_test_iterations > 0) and (self.test_duration_minutes <= 0):
            string = """
                   max_test_iterations: %(mti)s
                   test_log_path: %(tlp)s
                   print_to_console: %(ptc)s
                   """ % {"mti": str(self.max_test_iterations),
                          "tlp": str(self.test_log_dir),
                          "ptc": str(self.print_to_console)}
        else:
            string = """
                   test_duration_minutes: %(tdm)s
                   max_test_iterations: %(mti)s
                   test_log_path: %(tlp)s
                   print_to_console: %(ptc)s
                   """ % {"tdm": str(self.test_duration_minutes),
                          "mti": str(self.max_test_iterations),
                          "tlp": str(self.test_log_dir),
                          "ptc": str(self.print_to_console)}
        return string

class MediaFile(object):
    """
    Contains all of the information pertaining to a media file that will be synced to a target
    """
    def __init__(self, fileName = "", localPath="", digest="", targetFilePath=None):
        self.file_name = fileName
        self.local_path = localPath
        self.digest = digest
        self.target_file_path = targetFilePath  # path to where this file should exist on the target
        self.target_file_descriptor = None

    def generate_hex_digest(self, hasher, block_size=65536):
        """
        calculate a hex digest for the media file pointed at by local_path
        """
        self.digest = MediaFile.get_hex_digest(self.local_path, hasher, block_size)

    def __str__(self):
        """
        print the media file details
        """
        return """
               - file_name: %(fn)s
               - local_path: %(lp)s
               - digest: %(dig)s
               - target_file_path: %(tfp)s
               - target_file_descriptor: %(tfd)s
               """ % {"fn": self.file_name,
                      "lp": self.local_path,
                      "dig": self.digest,
                      "tfp": self.target_file_path,
                      "tfd": self.target_file_descriptor}


    @staticmethod
    def get_hex_digest(file_to_hash_path, hasher, block_size=65536):
        """
        calculate a hex digest for a given media file
        """
        data_pending = True

        with open(file_to_hash_path, 'rb') as fileToHash:
            while data_pending:
                # read a block from the file
                hash_data = fileToHash.read(block_size)
                if len(hash_data) > 0:
                    # update the hash
                    hasher.update(hash_data)
                else:
                    # we are done
                    data_pending = False

        return hasher.hexdigest()

    def pull_file_path(self):
        """
        generates the full path that will be used when pulling a file back from the host. The intent is to keep separate copies in order to compare the pulled one to the original.
        """
        base_dir, file_name = os.path.split(self.local_path)
        file_base, file_ext = os.path.splitext(file_name)
        pulled_file_path = os.path.join(base_dir, file_base + "_sync" + file_ext)
        return pulled_file_path

class ResultCode:
    """ all possible result codes """
    PASS = 0
    ERROR_FAILED_TO_MEET_MINIMUM_REQUIREMENTS = 1
    FAIL = 2
    UNKNOWN_ERROR = 3
    FABRIC_ERROR = 4
    TIMEOUT = 5
    ERROR = 6
    UNEXPECTED_ARGUMENTS = 7
    SKIPPED = 8

    def __init__(self, initial_value):
        """ default initializer """
        self.value = initial_value

    def __str__(self):
        if self.value == self.PASS:
            return "Pass"
        elif self.value == self.ERROR_FAILED_TO_MEET_MINIMUM_REQUIREMENTS:
            return "ERROR! Failed to meet minimum requirements"
        elif self.value == self.FAIL:
            return "Fail"
        elif self.value == self.FABRIC_ERROR:
            return "Fabric error"
        elif self.value == self.TIMEOUT:
            return "Timeout"
        elif self.value == self.ERROR:
            return "Error"
        elif self.value == self.UNEXPECTED_ARGUMENTS:
            return "Unexpected arguments"
        return "Unknown result code - value was " + str(self.value)