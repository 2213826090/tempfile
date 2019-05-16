#!/usr/bin/env python
# -*- coding: utf-8 -*-

# -------------------------------------------------------------------------------
# @copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
# The source code contained or described here in and all documents related
# to the source code ("Material") are owned by Intel Corporation or its
# suppliers or licensors. Title to the Material remains with Intel Corporation
# or its suppliers and licensors. The Material contains trade secrets and
# proprietary and confidential information of Intel or its suppliers and
# licensors.

# The Material is protected by worldwide copyright and trade secret laws and
# treaty provisions. No part of the Material may be used, copied, reproduced,
# modified, published, uploaded, posted, transmitted, distributed, or disclosed
# in any way without Intel's prior express written permission.

# No license under any patent, copyright, trade secret or other intellectual
# property right is granted to or conferred upon you by disclosure or delivery
# of the Materials, either expressly, by implication, inducement, estoppel or
# otherwise. Any license under such intellectual property rights must be express
# and approved by Intel in writing.

# @organization: INTEL MCG PSI
# @summary: The Setup Class itself
# @since: 4/18/14
# @author: nbrissox
# -------------------------------------------------------------------------------
# flake8: noqa: E402
import os
import sys
import platform

from os import path, environ

# In order to use this Class as a standalone script
TOP_DIR = path.abspath(path.join(path.dirname(__file__), '..'))
if TOP_DIR not in sys.path:
    sys.path.append(TOP_DIR)

from setups import logger, CONFIG_PROFILES_DIR
from setups.context import format_configuration_path, OsSupport, SETUP_SESSION, DEFAULT_CACHE_FOLDER

from setups.vendors import argparse
from setups.helpers import Loader, types, setup_exception

from setups.managers import ManagerBase
from setups.managers.binary import BinaryManager, EquipmentManager
from setups.managers.pypi import PyPiManager
from setups.managers.filesystem import FsManager


# Python supported version
PYTHON_MIN_VERSION_STR = "2.6.6"
PYTHON_MIN_VERSION_HEX = int(0x20606f0)
PYTHON_MAX_VERSION_STR = "2.7.8"
PYTHON_MAX_VERSION_HEX = int(0x20708f0)

# OS supported
SUPPORTED_OS = {
    "Windows": ["7 32bit", "7 64bit", "8 32bit", "8 64bit", "2008ServerR2 64bit (not officially tested !)"],
    "Linux": ["Ubuntu 12.04 32bit (not officially tested !)",
              "Ubuntu 12.04 64bit",
              "Ubuntu 14.04 32bit (not officially tested !)",
              "Ubuntu 14.04 64bit (not officially tested !)",
              "Ubuntu 16.04 32bit (not officially tested !)",
              "Ubuntu 16.04 64bit (not officially tested !)"]
}


class ACSSetup(types.StringBase):

    """
    Setups according Supported Environment (OS) all ACS requirements.

    """
    DEBUG = 1

    @staticmethod
    def build_parser():
        """
        Builds options from :mod:`argparse` module, :class:`argparse.ArgumentParser`

        :return: The parser
        :rtype: argparse.ArgumentParser

        """
        parser = argparse.ArgumentParser(description='Helper to setup automatically your Environment', prog='Setup')
        parser.add_argument('execute',
                            choices=['install', 'uninstall'],
                            help="Un/Install binaries, pypi packages, files, ... based on Configuration (JSON)")

        parser.add_argument('profile',
                            choices=[x.replace(".json", "") for x in os.listdir(CONFIG_PROFILES_DIR)],
                            nargs='*',
                            help="Specify which profile to run.")

        return parser

    @property
    def is_root(self):
        """
        Checks if the script is run with SUDO privileges.

        :return: True if run with SUDO privileges else False
        :rtype: bool

        """
        return_code = False
        if SETUP_SESSION.os == OsSupport.windows:
            output_cmd = ManagerBase.run_command("net session", silent_mode=True)
            if "Access is denied" not in output_cmd.stderr:
                return_code = True
        else:
            if 'SUDO_UID' in os.environ.iterkeys():
                return_code = True
        return return_code

    @property
    def is_os_supported(self):
        """
        Check if current OS version is supported

        :return: True if supported else False
        :rtype: bool

        """
        logger.info("Checking OS version ...")
        logger.info("Supported OS are:")
        for os_name in SUPPORTED_OS:
            for os_version in SUPPORTED_OS[os_name]:
                logger.info("\t{0} {1}".format(os_name, os_version))
        logger.info("")

        current_os_name = platform.system()
        if 'Windows' in current_os_name:
            # On Windows, users can install python 32 bits on a 64 bits machine
            # In this case platform.architecture will return 32 bits as we expect 64 bits.
            # To avoid that, we check environment variables. For further details refer to:
            # http://stackoverflow.com/questions/2764356/python-get-windows-os-version-and-architecture
            current_os_arch = '32bit' if ('x86' in environ.get("PROCESSOR_ARCHITECTURE") and
                                          'AMD64' not in environ.get("PROCESSOR_ARCHITEW6432", "")) else '64bit'
            current_os_version = platform.release()
        else:
            current_os_arch = platform.architecture()[0]
            current_os_version = " ".join(platform.dist()[:-1])

        logger.info("Current OS version: {0} {1} {2}".format(current_os_name, current_os_version, current_os_arch))

        if current_os_name in SUPPORTED_OS:
            current_os_info = "{0} {1}".format(current_os_version, current_os_arch)

            return_code = any([x for x in SUPPORTED_OS[current_os_name] if current_os_info in x])
        else:
            return_code = False

        logger.info("")
        return return_code

    @property
    def is_python_supported(self):
        """
        Check if current python version is supported

        :return: True if supported else False
        :rtype: bool

        """
        logger.info("Checking python version ...")
        logger.info("Python minimal version: {0} - maximum version: {1}".format(PYTHON_MIN_VERSION_STR,
                                                                                PYTHON_MAX_VERSION_STR))
        logger.info("\nCurrent python version: {0}\n".format(platform.python_version()))

        if sys.hexversion < PYTHON_MIN_VERSION_HEX or sys.hexversion > PYTHON_MAX_VERSION_STR:
            return_code = False
        else:
            return_code = True
        return return_code

    def __init__(self, options):
        """
        Constructor

        :param options: An instance of settings options
        :type options: argparse.Namespace

        """
        # if not self.is_root:
        #    raise setup_exception(ManagerBase.InstallationException,
        #                          'Please execute this script with super user privileges (sudo).')

        # Instantiation
        self.profiles = {}
        self.setup = {}

        self.setup_names = tuple(options.profile)
        self.execute = str(options.execute).strip()

        # Loading all known profiles (from configs/profiles)
        self.load_profiles()
        # Gathering all Profile(s) data according command-line arguments
        self.gather_profiles()

        # looking for cache folder path if provided in Environment or default one
        self.cache = format_configuration_path(os.environ.get('ACS_CACHE_FOLDER')) or DEFAULT_CACHE_FOLDER
        if self.cache:
            logger.info('Your Cache root directory location is `{0}`'.format(self.cache))

        artifacts_location = self.setup.get('artifacts_location', ManagerBase.DEFAULT_REPO_BASE)

        # Assigning common Manager keyword arguments
        mgr_options = dict(execute=self.execute, repo=artifacts_location, cache=self.cache)

        # Casting all sets into list (mutable)
        self.binaries_data = list(self.setup.get('binaries', []))
        self.equipments_data = list(self.setup.get('equipments', []))
        self.pypi_data = list(self.setup.get('pypi', {}))
        self.files_data = list(self.setup.get('files', []))

        # Create all required Managers
        self.bin_manager = BinaryManager(data=self.binaries_data, **mgr_options)
        self.equipment_manager = EquipmentManager(data=self.equipments_data, **mgr_options)
        self.pypi_manager = PyPiManager(data=self.pypi_data, **mgr_options)
        self.fs_manager = FsManager(data=self.files_data, **mgr_options)

    def load_profiles(self):
        """
        Loads all Profiles from JSON to Structure

        """
        for p in os.listdir(CONFIG_PROFILES_DIR):
            profile = path.abspath(path.join(CONFIG_PROFILES_DIR, p))
            if p.endswith('.json'):
                self.profiles[p.replace('.json', '')] = Loader.load(profile)
            else:
                logger.info('Profile ``{0}`` seems not to be JSON format, '
                            'as it MUST ends with `.json` extension'.format(p))

    def gather_profiles(self):
        """
        Merge all called profile(s) if so

        """
        # Use of set() as it keeps no duplicates at all!
        binaries, equipments, pypi, files = set(), set(), set(), list()

        # Referencing them to the internal `setup` dict
        self.setup = {
            'cache': "",
            'binaries': binaries,
            'equipments': equipments,
            'pypi': pypi
        }
        # Gathering all required profiles
        for p in self.setup_names:
            # Getting each profile data
            profile = self.profiles[p]
            # Extracting main data info &
            # Merging all together (without duplicates => use of `set()`)
            [binaries.add(binary) for binary in profile.get('binaries', [])]
            [equipments.add(equipment) for equipment in profile.get('equipments', [])]
            [pypi.add(package) for package in profile.get('pypi', [])]
            [files.append(f) for f in profile.get('files', [])]

        self.setup['files'] = types.DictSet(files, key='name')

    def run(self):
        """
        Runs the Setup process.

        """
        logger.info('{0}ing {1} profile(s) ...'.format(self.execute.capitalize(), self.setup_names))

        # The order is important !
        # As some binaries might be dependencies for Python package(s) such as dll, ...
        self.bin_manager.run()
        self.pypi_manager.run()
        self.equipment_manager.run()
        self.fs_manager.run()


def main(args):
    """
    Main Entry point

    :param args: Command line arguments
    :type args: tuple

    :return: Execution status
    :rtype: int

    """
    try:
        parser = ACSSetup.build_parser()
        setup = ACSSetup(parser.parse_args(args))

        if not setup.is_os_supported:
            raise setup_exception(ManagerBase.ValidationException,
                                  'This OS is not supported.')

        if not setup.is_python_supported:
            raise setup_exception(ManagerBase.ValidationException,
                                  'Please install a supported python version.')

        setup.run()
        return_code = 0
    except Exception as installation_exception:
        logger.error(installation_exception)
        return_code = 1

    return return_code


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
