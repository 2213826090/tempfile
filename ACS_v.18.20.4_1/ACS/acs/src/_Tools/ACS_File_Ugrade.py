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
:summary: This script updates ACS campaigns & TCs with latest version of templates
:author: cbresoli

"""

from optparse import OptionParser, OptionGroup

import xml.etree.ElementTree as ET
import os
import re
import sys
import shutil
import subprocess

EXECUTION_CONFIG_FOLDER = "_ExecutionConfig"
TEMPLATES_FOLDER = "_Templates"
SCRIPT_FOLDER = "_Tools"

DIFF_TOOL_WIN = "C:/Program Files (x86)/Beyond Compare 3/BCompare.exe"
DIFF_TOOL_LINUX = "/bin/BCompare"

# ------------------------------------------------------------------------------


def setup_arg_parser():
    usage = "usage: %prog -f TARGET_FILE -p TARGET_FOLDER -t TOOL -b BACKUP"
    parser = OptionParser(usage=usage)

    optional_group = OptionGroup(parser, "Optional Parameters")

    optional_group.add_option("-f", "--file",
                              help="TARGET_FILE to update: CP/cp for campaigns; TC/tc for Testcases",
                              default="TC",
                              type="string",
                              dest="target_file")

    optional_group.add_option("-p", "--path",
                              help=("TARGET_FOLDER to update (relative or absolute path); default is %s" %
                                    EXECUTION_CONFIG_FOLDER),
                              default=EXECUTION_CONFIG_FOLDER,
                              type="string",
                              dest="target_folder")

    optional_group.add_option("-t", "--tool",
                              help=("TOOL used for comparison; default for Windows is %s, for Linux is %s" %
                                    (DIFF_TOOL_WIN, DIFF_TOOL_LINUX)),
                              default="None",
                              type="string",
                              dest="tool")

    optional_group.add_option("-b", "--backup",
                              help="Backup target folder before update",
                              action="store_true",
                              default=False,
                              dest="is_backup_required")

    parser.add_option_group(optional_group)

    return parser

# ------------------------------------------------------------------------------


def get_ucase_from_file(file_path):
    """ get_ucase_from_file
    """

    try:
        tree = ET.parse(file_path)
        root = tree.getroot()
        uc_id = root.find("UseCase").text

        if re.match("[A-Z_]", uc_id):
            return uc_id
        else:
            return None

    except (KeyboardInterrupt, SystemExit):
        raise
    except:
        return None

# ------------------------------------------------------------------------------


def get_version_from_file(file_path):
    """ get_version_from_file
    """

    try:
        tree = ET.parse(file_path)
        root = tree.getroot()
        if len(root.attrib) != 0:
            version = root.attrib["version"]

            if version is not None and re.match("[0-9.]", version):
                year = version.split(".")[0]
                week = version.split(".")[1]
                return year, week
            else:
                return None, None
        else:
            return None, None

    except (KeyboardInterrupt, SystemExit):
        raise
    except:
        return None

# ------------------------------------------------------------------------------


def get_type_from_file(file_path):
    """ get_type_from_file
    """

    try:
        tree = ET.parse(file_path)
        root = tree.getroot()

        if root.tag == "Campaign":
            file_type = "CP"
        elif root.tag == "TestCase":
            file_type = "TC"
        elif root.tag == "BenchConfig":
            file_type = "BC"
        else:
            file_type = None

        return file_type

    except (KeyboardInterrupt, SystemExit):
        raise
    except:
        return None

# ------------------------------------------------------------------------------


def upgrade_tc(options, template_folder_path, target_folder_path):
    """ upgrade_tc
    """
    try:
        print ("Checking all TC in %s ..." % target_folder_path)

        # Walk along target folder and search all .xml files
        os.chdir(target_folder_path)
        for usr_root, _, usr_files in os.walk(target_folder_path):
            for usr_file in usr_files:

                if ".xml" in usr_file:
                    tc_abs_path = str(os.path.join(usr_root, usr_file))

                    # Check the type of the file
                    if get_type_from_file(tc_abs_path) != "TC":
                        continue

                    # Get the UC id from TC if any
                    tc_uc_id = get_ucase_from_file(tc_abs_path)

                    if (tc_uc_id is not None) and \
                       (tc_uc_id not in ["EXEC", "EXEC_SCRIPT", "ANDROID_UI_TEST"]):

                        found = False

                        # Search matching template file
                        os.chdir(template_folder_path)
                        for template_root, _, template_files in os.walk(template_folder_path):
                            for template_file in template_files:

                                if ".xml" in template_file:
                                    template_abs_path = str(os.path.join(template_root, template_file))

                                    # Check the type of the file
                                    if get_type_from_file(template_abs_path) != "TC":
                                        continue

                                    # Get the UC id if any
                                    template_uc_id = get_ucase_from_file(template_abs_path)

                                    if template_uc_id == tc_uc_id:

                                        # Read version of the TC if any
                                        tc_version = get_version_from_file(tc_abs_path)

                                        # Read version of the matching template if any
                                        template_version = get_version_from_file(template_abs_path)

                                        # If version of the file is below the one of the template, upgrade is needed
                                        # If template version is empty, upgrade!
                                        if tc_version[0] < template_version[0] or \
                                           tc_version[1] < template_version[1] or \
                                           template_version[0] is None or \
                                           template_version[1] is None:

                                            is_upgrading = False
                                            print ("Upgrading %s with %s ..." % (tc_abs_path, template_abs_path))

                                            if sys.platform == "win32":

                                                is_upgrading = True
                                                if options.tool != "None":
                                                    tool = options.tool
                                                else:
                                                    tool = DIFF_TOOL_WIN

                                            elif sys.platform in ["linux2", "cygwin"]:

                                                is_upgrading = True
                                                if options.tool != "None":
                                                    tool = options.tool
                                                else:
                                                    tool = DIFF_TOOL_LINUX

                                            if is_upgrading:
                                                cmd = [tool, template_abs_path, tc_abs_path]
                                                subprocess.call(cmd)
                                                print ("Upgrading done...")
                                            else:
                                                print ("No upgrade; OS not supported!")
                                        else:
                                            print ("No upgrade required for %s!" % tc_abs_path)

                                        found = True
                                        break
                            if found:
                                break

                        os.chdir(target_folder_path)

        print ("TC Check complete!")

        # Restore initial Working Directory
        os.chdir(cwd)

    except (KeyboardInterrupt, SystemExit):
        raise
    except:
        raise

# ------------------------------------------------------------------------------


def upgrade_campaign(options, template_folder_path, target_folder_path):
    """ upgrade_tc
    """

    try:
        print ("Checking all Campaigns in %s ..." % target_folder_path)

        # Walk along target folder and search all .xml files
        os.chdir(target_folder_path)
        for usr_root, _, usr_files in os.walk(target_folder_path):
            for usr_file in usr_files:

                if ".xml" in usr_file:
                    cp_abs_path = str(os.path.join(usr_root, usr_file))

                    # Check the type of the file
                    if get_type_from_file(cp_abs_path) != "CP":
                        continue

                    found = False

                    # Search matching template file
                    os.chdir(template_folder_path)
                    for template_root, _, template_files in os.walk(template_folder_path):
                        for template_file in template_files:

                            if ".xml" in template_file:
                                template_abs_path = str(os.path.join(template_root, template_file))

                                # Check the type of the file
                                if get_type_from_file(template_abs_path) != "CP":
                                    continue

                                # Read version of the TC if any
                                tc_version = get_version_from_file(cp_abs_path)

                                # Read version of the matching template if any
                                template_version = get_version_from_file(template_abs_path)

                                # If version of the file is below the one of the template, upgrade is needed
                                # If template version is empty, upgrade!
                                if tc_version[0] < template_version[0] or \
                                   tc_version[1] < template_version[1] or \
                                   template_version[0] is None or \
                                   template_version[1] is None:

                                    is_upgrading = False
                                    print ("Upgrading %s with %s ..." % (cp_abs_path, template_abs_path))

                                    if sys.platform == "win32":

                                        is_upgrading = True
                                        if options.tool != "None":
                                            tool = options.tool
                                        else:
                                            tool = DIFF_TOOL_WIN

                                    elif sys.platform in ["linux2", "cygwin"]:

                                        is_upgrading = True
                                        if options.tool != "None":
                                            tool = options.tool
                                        else:
                                            tool = DIFF_TOOL_LINUX

                                    if is_upgrading:
                                        cmd = [tool, template_abs_path, cp_abs_path]
                                        subprocess.call(cmd)
                                        print ("Upgrading done...")
                                    else:
                                        print ("No upgrade; OS not supported!")
                                else:
                                    print ("No upgrade required for %s!" % cp_abs_path)

                                found = True
                                break
                        if found:
                            break

                    os.chdir(target_folder_path)

        print ("Campaign Check complete!")

        # Restore initial Working Directory
        os.chdir(cwd)

    except (KeyboardInterrupt, SystemExit):
        raise
    except:
        raise

# ------------------------------------------------------------------------------


if __name__ == "__main__":
    try:
        # Init arg parser
        arg_parser = setup_arg_parser()

        # Parse cmd line
        (options, args) = arg_parser.parse_args()

        # Recall Current Working Directory
        cwd = os.getcwd()
        new_cwd = cwd.split(SCRIPT_FOLDER)[0]
        os.chdir(new_cwd)

        # Get Template folders
        template_folder_path = os.path.abspath(os.path.normpath(os.path.join(".", TEMPLATES_FOLDER)))

        # Check if it is a relative path
        if not os.path.isabs(options.target_folder):
            # Enable right cwd
            if options.target_folder != EXECUTION_CONFIG_FOLDER:
                os.chdir(cwd)
            target_folder_path = os.path.abspath(os.path.normpath(os.path.join(".", options.target_folder)))
        else:
            target_folder_path = os.path.normpath(options.target_folder)

        # Check target path exists!
        if not os.path.exists(target_folder_path):
            print("\'%s\' target folder does not exist!" % target_folder_path)
            sys.exit(-1)

        print ("File type to upgrade = %s" % str(options.target_file))
        print ("Target path = %s" % target_folder_path)
        if options.tool != "None":
            print ("Comparison tool = %s" % options.tool)
        print ("Backup required = %s" % str(options.is_backup_required))

        if options.is_backup_required:

            target_folder_path_backup = target_folder_path + "_Backup"

            # If backup folder already exixts, delete is to get a fresh version of it
            if os.path.exists(target_folder_path_backup):
                print ("Deleting previous backup (%s) ..." % target_folder_path_backup)
                shutil.rmtree(target_folder_path_backup)

            print ("Backup in progress (%s) ..." % target_folder_path)
            shutil.copytree(target_folder_path, target_folder_path_backup)
            print ("Backup complete ...")

        # Take action required following type of file to upgrade
        if options.target_file.lower() == "tc":
            upgrade_tc(options, template_folder_path, target_folder_path)

        elif options.target_file.lower() == "cp":
            upgrade_campaign(options, template_folder_path, target_folder_path)

        elif options.target_file.lower() == "all":
            upgrade_campaign(options, template_folder_path, target_folder_path)
            upgrade_tc(options, template_folder_path, target_folder_path)

        else:
            print("Unknown file type %s to upgrade!" % options.target_file)

    except KeyboardInterrupt:
        exit("Killed by user")
    except Exception as exception:
        print (exception)
