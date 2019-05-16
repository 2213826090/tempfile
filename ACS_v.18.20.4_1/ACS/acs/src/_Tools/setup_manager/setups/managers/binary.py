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
# @summary: Binaries & Equipment Managers.
# @since: 4/23/14
# @author: nbrissox
# -------------------------------------------------------------------------------
import os
import traceback

from os import path
from urlparse import urljoin

from setups import logger, CONFIG_BINARIES_DIR, CONFIG_EQUIPMENTS_DIR
from setups.context import OsSupport
from setups.helpers import Loader
from setups.managers import ManagerBase


class BinaryManager(ManagerBase):

    """
    Manages binary un/install

    .. uml::

        class ManagerBase

        class BinaryManager {
            CACHE_NAME = 'binaries'
            DEFAULT_REPO_BASE = ManagerBase.DEFAULT_REPO_BASE, 'acs/dependencies/third_parties/tools/'
            DEFAULT_CACHE = ManagerBase.DEFAULT_CACHE, CACHE_NAME

            CONFIG_DIR = CONFIG_BINARIES_DIR

            run(*args, **options)
        }

        ManagerBase <|- BinaryManager

    """

    CACHE_NAME = 'binaries'
    EXTRA_REPO = 'acs/dependencies/third_parties/tools/'
    DEFAULT_CACHE = path.join(ManagerBase.DEFAULT_CACHE, CACHE_NAME)

    CONFIG_DIR = CONFIG_BINARIES_DIR

    @classmethod
    def is_executable_present(cls, binary):
        """
        Gets the executable path, if installed.

        :param binary: The executable dict
        :type binary: dict

        :return: The executable exists or not
        :rtype: boolean
        """

        ret = False
        output = ""
        ops = "windows" if cls.session.os == "win32" else "linux"
        method, possible_values = binary.get(ops, {}).get('check_exists', ["", []])
        for program in possible_values:
            if cls.session.os == OsSupport.windows:
                output = cls.win_which(method, program)
                if output:
                    ret = True
                    break
            else:
                output = cls.run_command('{0} {1}'.format(method, program), cwd="", env=os.environ.copy())
                if output.succeeded and path.exists(output):
                    ret = True
                    break
        return ret, output

    @classmethod
    def win_which(cls, method, program):
        """
        Emulates Unix like Which Command on Windows.

        :param method: Which method
        :type method: str

        :param program: Which program possible path
        :type program: str

        :return: The executable full path or None
        :rtype: str | None

        """
        def is_exe(filepath):
            """
            Tells if a given path is executable or not.

            :param filepath:
            :type filepath: str

            :return: Whether or not the given file si an executable or not.
            :rtype: bool

            """
            return os.path.isfile(filepath) and os.access(filepath, os.X_OK)

        ret = None
        if method == 'winreg':
            import _winreg as winreg
            reg_key = r"{0}".format(program.replace('/', '\\\\'))
            try:
                with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, reg_key, 0, winreg.KEY_READ) as key:
                    ret = key.Detach()
                    logger.debug('WINREG key info (handle: {1}): {0}'.format(winreg.QueryInfoKey(ret), ret))
            except WindowsError:
                if cls.DEBUG:
                    print(traceback.format_exc())
        elif method == 'env':
            ret = os.environ.get(program)
        else:
            if not program.endswith('.exe'):
                program = '{0}.exe'.format(program)

            fpath, fname = os.path.split(program)
            if fpath:
                if is_exe(program):
                    ret = program
            else:
                for p in os.environ["PATH"].split(os.pathsep):
                    p = p.strip('"')
                    exe_file = os.path.join(p, program)
                    if is_exe(exe_file):
                        ret = exe_file
        return ret

    def run(self):
        """
        Runs the manager

        """
        if self.data:
            super(BinaryManager, self).run()

            win32 = self.session.os == OsSupport.windows
            is32 = self.session.system.is32bit

            for b in self.data:
                binary_dirname = b.split('@')[0].strip()
                binary_desc = path.abspath(path.join(self.CONFIG_DIR, b.strip() + '.json'))
                binary = Loader.load(binary_desc)

                is_binary_installed, binary_installed_path = self.is_executable_present(binary)
                artifact_location = binary.get('artifacts_location')

                bin_data = binary.get('windows') if win32 else binary.get('linux')
                if not bin_data:
                    logger.warning('Missing configuration file (.json) for binary '
                                   '{0} on {1}'.format(binary_dirname, self.session.os))
                    continue

                # Execute pre installation scripts
                pre_processing_instructions = bin_data.get("pre_{0}".format(self.execute))
                post_processing_instructions = bin_data.get("post_{0}".format(self.execute))

                # handling Ubuntu direct command form system packages manager. (Apt-get)
                formatted_cmd = bin_data.get(self.execute)
                if formatted_cmd is None:
                    logger.info('No {0} command provided for binary {1}'.format(self.execute, binary_dirname))
                    continue

                if not win32 and formatted_cmd and 'apt-get' in formatted_cmd:

                    if is_binary_installed:
                        info_msg = 'Binary {0} is already {1}ed !'.format(binary_dirname, self.execute)
                        logger.info(info_msg)
                        print(info_msg)
                        continue

                    self.pre_processing(pre_processing_instructions)
                    output_cmd = self.run_command(formatted_cmd)
                    if output_cmd.return_code != 0:
                        error_msg = 'Error while executing {0}'.format(formatted_cmd)
                        self.error('SetupManagerException', error_msg)
                    else:
                        self.post_processing(post_processing_instructions)
                        continue

                artifact_location = bin_data.get('artifacts_location', artifact_location)
                if not artifact_location:
                    artifact_location = self.DEFAULT_REPO_BASE

                # Handling OS arch
                if is32:
                    bin_arch = bin_data.get('32')
                else:
                    bin_arch = bin_data.get('64')
                    if not bin_arch:
                        logger.info('No 64bits version found for binary {0}'.format(binary_dirname))
                        logger.info('Looking for 32bits ...')
                        bin_arch = bin_data.get('32')

                if not bin_arch and win32:
                    logger.warning('None artifact found for binary {0} at all!! '
                                   'Check your Configuration!'.format(binary_dirname))
                    continue

                # Handling Python binaries, str.format(), will NOT replace anything if not expected
                # see Configuration file (.json)
                bin_arch = bin_arch.format(pyversion=self.session.python.version)

                binary_name = bin_arch.split('/')[-1:][0]
                binary['name'] = binary_dirname

                print("{0}ing binary {1}".format(self.execute.capitalize(), binary_dirname))
                if ((self.execute == "install" and is_binary_installed) or
                        (self.execute == "uninstall" and not is_binary_installed)):
                    info_msg = 'Binary {0} is already {1}ed !'.format(binary_dirname, self.execute)
                    logger.info(info_msg)
                    print(info_msg)
                    continue

                if (self.execute == "uninstall" and
                        is_binary_installed and
                        path.exists(ur'{0}'.format(binary_installed_path))):
                    local_artifact = binary_installed_path
                else:
                    local_artifact = os.path.join(self.cache, binary_dirname, binary_name)

                if not os.path.exists(local_artifact):
                    artifact_uri = urljoin(artifact_location, bin_arch)
                    opts = {}
                    # Internal Artifacts location (ACS Artifactory repository) else external (PFT, ...)
                    if not artifact_location.strip() == self.DEFAULT_REPO_BASE:
                        opts['local_location'] = '{0}/{1}'.format(binary_dirname, artifact_uri[len(artifact_location):])
                    local_artifact = self.download_file(artifact_uri, **opts)

                if local_artifact:
                    if local_artifact.endswith('.zip'):
                        rootdir, name = path.split(local_artifact)
                        self.unzip(local_artifact, rootdir)
                        zip_info = bin_data.get('zip')
                        if zip_info:
                            local_artifact = path.join(rootdir, zip_info.get('bin'))
                        else:
                            local_artifact = local_artifact.replace('.zip', '.exe')

                    # Executing pre-processing actions
                    self.pre_processing(pre_processing_instructions)

                    execute_cmd = ''
                    if win32:
                        execute_cmd += 'start /w '
                    execute_cmd += formatted_cmd.format(bin=r'%s' % local_artifact)
                    logger.info('Executing {1} {0} ...'.format(execute_cmd, self.execute.upper()))
                    output_cmd = self.run_command(execute_cmd)
                    if output_cmd.return_code != 0:
                        error_msg = 'Error while executing {0}'.format(execute_cmd)
                        self.error('SetupManagerException', error_msg)
                    else:
                        # Executing post-processing actions
                        self.post_processing(post_processing_instructions)

                else:
                    error_msg = "Cannot install binary {0} !".format(binary_name)
                    logger.error(error_msg)
                    self.error('SetupManagerException', error_msg)


class EquipmentManager(BinaryManager):

    """
    Equipments Manager

    .. uml::

        class ManagerBase

        class BinaryManager

        class EquipmentManager {
            CACHE_NAME = 'equipments'
            DEFAULT_REPO_BASE = ManagerBase.DEFAULT_REPO_BASE, 'acs/dependencies/equipment/'
            DEFAULT_CACHE = BinaryManager.DEFAULT_CACHE, CACHE_NAME

            CONFIG_DIR = CONFIG_EQUIPMENTS_DIR

            run(*args, **options)
        }

        ManagerBase <|- BinaryManager

        BinaryManager <|- EquipmentManager

    """

    CACHE_NAME = 'equipments'
    EXTRA_REPO = 'acs/dependencies/equipment/'
    DEFAULT_CACHE = path.join(BinaryManager.DEFAULT_CACHE, CACHE_NAME)

    CONFIG_DIR = CONFIG_EQUIPMENTS_DIR
