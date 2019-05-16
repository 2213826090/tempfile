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
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

:organization: INTEL MCG PSI
:summary: Implements methods for text file handling
:since: 03/07/2014
:author: vdechefd
"""

import shutil
import os
from tempfile import mkstemp
from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT


def modify_text_file(file_path, action, pattern, substitute):
    """
    Modify a text file, applying a customizable action on each line

    :type  file_path: str
    :param file_path: path of the file to modify
    :type  action: function
    :param action: action to apply on each line of the file.
                   It will receive 4 params:
                       - the line, as a string
                       - the pattern to detect in the line, as a string
                       - the file where to write the modified line, as a file
                       - the substitute that will replace the pattern, as a string
    :type  pattern: str
    :param pattern: the pattern to detect in each line
    :type  substitute: str
    :param substitute: the substitute for pattern
    :rtype: boolean
    :return: True if file was modified successfully, False otherwise
    """

    result = False
    try:
        # Create temp file
        fh, abs_path = mkstemp()
        # open files
        new_file = open(abs_path, 'wb')
        old_file = open(file_path, 'r')

        # apply action for each line
        for line in old_file:
            action(line, pattern, new_file, substitute)

        # close temp file
        new_file.close()
        os.close(fh)
        old_file.close()
        # Remove original file
        os.remove(file_path)
        # Move new file
        shutil.move(abs_path, file_path)
        result = True
    except Exception as e:
        LOGGER_TEST_SCRIPT.warning("Cannot modify text file %s\n%s" % (abs_path, e.message))

    return result


def add_line_before_pattern(line, pattern, dest_file, substitute):
    """
    Text line action, that will insert 'substitute' before any line matching exactly 'pattern'

    :type  line: str
    :param line: the line to write
    :type  pattern: str
    :param pattern: the pattern to detect in each line
    :type  dest_file: file
    :param dest_file: the file where to write action result
    :type  substitute: str
    :param substitute: the substitute for pattern
    :rtype: NoneType
    :return: None
    """

    if line.strip() == pattern:
        dest_file.write(substitute + "\n")
    dest_file.write(line)


def replace_pattern_in_line(line, pattern, dest_file, substitute):
    """
    Text line action, that will replace 'pattern' by 'substitute' in line

    :type  line: str
    :param line: the line to write
    :type  pattern: str
    :param pattern: the pattern to detect in each line
    :type  dest_file: file
    :param dest_file: the file where to write action result
    :type  substitute: str
    :param substitute: the substitute for pattern
    :rtype: NoneType
    :return: None
    """
    dest_file.write(line.replace(pattern, substitute))


def rm_line_matching_pattern(line, pattern, dest_file, substitute):
    """
    Text line action, that will remove line if it does not match 'pattern'

    :type  line: str
    :param line: the line to write
    :type  pattern: str
    :param pattern: the pattern to detect in each line
    :type  dest_file: file
    :param dest_file: the file where to write action result
    :type  substitute: str
    :param substitute: unused
    :rtype: NoneType
    :return: None
    """
    if not pattern in line:
        dest_file.write(line)
