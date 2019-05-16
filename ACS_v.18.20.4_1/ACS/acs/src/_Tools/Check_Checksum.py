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
:summary: External tool for check checksum of zip file
:since: 05/10/2015
:author: nprecigx
"""
import sys
import os
import tempfile
import tarfile
import shutil

root_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
if os.path.exists(root_path) and root_path not in sys.path:
    sys.path.append(root_path)

from UtilitiesFWK.Utilities import compute_checksum

CHECKSUM_TMP_FOLDER = ""
CHECKSUM_FILE = "reports_compute"


def check_checksum(path):
    """
    Return the list of file modified, added or deleted in the archive given in parameter
    :type path: str
    :param path: zip path or folder path
    """
    if os.path.isfile(path):
        unzip_report(path)
    elif os.path.isdir(path):
        global CHECKSUM_TMP_FOLDER
        CHECKSUM_TMP_FOLDER = os.path.join(tempfile.mkdtemp() + "report")
        shutil.copytree(path, CHECKSUM_TMP_FOLDER)
    else:
        print "Path must be a tar.gz archive or a folder"

    checksum_file_path = os.path.join(CHECKSUM_TMP_FOLDER, CHECKSUM_FILE)
    # Check if the archive contains the file with checksum computed
    if os.path.isfile(checksum_file_path):
        try:
            shutil.move(checksum_file_path, tempfile.gettempdir())
            checksum_file_path = os.path.join(tempfile.gettempdir(), CHECKSUM_FILE)
        except shutil.Error as error:
            print ("Error during move checksum file: {}".format(error))
    else:
        print ("Error: No checksum file present in the archive!!!!!!!!")
        exit()

    # Compute checksum of the archive given in parameter and save it
    try:
        new_checksum = compute_checksum(CHECKSUM_TMP_FOLDER)
    except Exception as error:
        print("Error during compute checksum: {}".format(error))

    try:
        # Read reference checksum list
        with open(checksum_file_path, 'r') as ref_checksum_file:
            reference_checksum = ref_checksum_file.read()
            reference_checksum = eval(reference_checksum)
    except Exception as error:
        print ("Error during read reference checksum file: {}".format(error))

    file_modified = []
    files_checked = 0
    # Check the presence of modified files
    for archive_file, archive_checksum in new_checksum:
        for ref_file, ref_checksum in reference_checksum:
            if (archive_file == ref_file):
                files_checked = 1
                if ref_checksum != archive_checksum:
                    file_modified += [(os.path.basename(archive_file), ref_checksum, archive_checksum)]
                break
        # Check if the files is added in the new archive
        if files_checked == 0:
            file_modified += [(os.path.basename(archive_file), "New File", archive_checksum)]
        else:
            files_checked = 0

    files_checked = 0
    # Check the presence of files deleted
    for ref_file, ref_checksum in reference_checksum:
        for archive_file, archive_checksum in new_checksum:
            if os.path.basename(archive_file) == os.path.basename(ref_file):
                files_checked = 1
                break
        # Check if the files is deleted in the new archive
        if files_checked == 0:
            file_modified += [(os.path.basename(ref_file), ref_checksum, "File Deleted")]
        files_checked = 0

    if file_modified:
        print "\n************************************************************************************"
        print("List of file modified:")
        for file_modified_ in file_modified:
            print("************************************************************************************\n"
                  "{}\n"
                  "Original Checksum: {}\n"
                  "Modified Checksum: {}\n".format(*file_modified_))
    else:
        print "*****************"
        print "No modified files"
        print "*****************"

    clean()

    return True


def unzip_report(zip_path):
    """
    Unzip tar.gz archive
    :type zip_path: str
    :param zip_path: Zip path
    """
    if os.path.isfile(zip_path):
        print "Unzip file"
        global CHECKSUM_TMP_FOLDER
        CHECKSUM_TMP_FOLDER = tempfile.mkdtemp()
        try:
            with tarfile.open(zip_path, 'r:gz') as tgz:
                tgz.extractall(CHECKSUM_TMP_FOLDER)
        except Exception as error:
            print ("Error duing unzip file: {}".format(error))
    else:
        print "ZIP report file does not exist"
        exit()


def clean():
    """
    Delete temporary files
    """
    print "Clean temporary files"
    try:
        if os.path.isfile(os.path.join(tempfile.gettempdir(), CHECKSUM_FILE)):
            os.remove(os.path.join(tempfile.gettempdir(), CHECKSUM_FILE))
        if os.path.isdir(CHECKSUM_TMP_FOLDER):
            shutil.rmtree(CHECKSUM_TMP_FOLDER)
    except Exception as error:
        print ("Error duing clean temporary files: {}".format(error))


def main():
    return check_checksum(path=sys.argv[1])


if __name__ == '__main__':
    sys.exit(main())
