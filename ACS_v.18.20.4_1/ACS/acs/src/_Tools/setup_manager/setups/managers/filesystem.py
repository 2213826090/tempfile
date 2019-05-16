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
# @summary: File system Manager
# @since: 4/22/14
# @author: nbrissox
# -------------------------------------------------------------------------------
from os import environ, makedirs, path, unlink
from shutil import rmtree
from urlparse import urljoin

from setups import logger
from setups.context import OsSupport, format_configuration_path, CURRENT_USER
from setups.managers import ManagerBase


class FsManager(ManagerBase):

    """
    File System Manager

    .. uml::

        class ManagerBase

        class FsManager {
            CACHE_NAME = 'files'
            DEFAULT_REPO_BASE = ManagerBase.DEFAULT_REPO_BASE, 'acs/pypi/doctool/'
            DEFAULT_CACHE = ManagerBase.DEFAULT_CACHE, CACHE_NAME

            run(*args, **options)
        }

        ManagerBase <|- FsManager

    """

    CACHE_NAME = 'files'
    EXTRA_REPO = 'acs/dependencies/third_parties/files/'
    DEFAULT_CACHE = path.join(ManagerBase.DEFAULT_CACHE, CACHE_NAME)

    def __uninstall_operation(self, data_to_remove):
        """
        Uninstall Files given in data_to_remove dictionary.

        :param data_to_remove: Data to remove
        :type data_to_remove: dict
        """
        file_name = data_to_remove.get('name')
        destination = format_configuration_path(data_to_remove.get('destination'))

        try:
            # Remove installed file
            if path.isfile(destination):
                logger.info("Removing file '{0}'".format(file_name))
                unlink(destination)
            elif path.isdir(destination):
                logger.info("Removing folder '{0}'".format(destination))
                rmtree(destination)

            # Remove shortcut if any
            shortcut_list = data_to_remove.get('shortcut')
            if shortcut_list:
                for shortcut_args in shortcut_list:
                    sc_name = shortcut_args.get('name')
                    self.remove_shortcut(sc_name)
        except Exception as sc_exception:
            error_msg = "Uninstall error of {0} ! ({1})".format(file_name, sc_exception)
            logger.error(error_msg)
            self.error('SetupManagerException', error_msg)

    def run(self):
        """
        Runs the manager
        """

        if self.data:
            super(FsManager, self).run()

            install = self.execute == 'install'
            for f in self.data:
                name = f.get('name')
                destination = format_configuration_path(f.get('destination'))
                if not destination:
                    print("'destination' for {0} is not set !".format(name))
                    continue
                if not install:
                    self.__uninstall_operation(f)
                    continue

                artifact_location = f.get('artifacts_location')
                if not artifact_location:
                    artifact_location = ManagerBase.DEFAULT_REPO_BASE

                source = urljoin(artifact_location, f.get('source'))
                if not path.isdir(destination):
                    # Create destination folder if not exists
                    makedirs(destination)
                destination = path.join(destination, name)
                local_artifact = self.download_file(source, local_destination=destination)

                if local_artifact:
                    if self.file_extensions(local_artifact, ('.zip', '.tar', '.tar.gz')):
                        self.unzip(local_artifact, path.dirname(destination))
                        unlink(local_artifact)
                    else:
                        self.move(local_artifact, destination)

                    if self.session.os == OsSupport.linux:
                        self.run_command('chown -R {1}:{1} {0}'.format(path.dirname(destination), CURRENT_USER),
                                         env=environ, cwd="", shell="/bin/bash")

                    shortcut_list = f.get('shortcut')
                    if shortcut_list:
                        for shortcut_args in shortcut_list:
                            (sc_name,
                             sc_type,
                             sc_path,
                             description,
                             icon_path,
                             extra_args) = (shortcut_args.get('name'),
                                            shortcut_args.get('type'),
                                            self.format_configuration_path(shortcut_args.get('path')),
                                            shortcut_args.get('description'),
                                            self.format_configuration_path(shortcut_args.get('icon_path')),
                                            self.format_configuration_path(shortcut_args.get('extra_args')))

                            try:
                                self.create_desktop_shortcut(sc_name,
                                                             sc_type,
                                                             sc_path,
                                                             description,
                                                             icon_path,
                                                             extra_args)
                            except Exception as sc_exception:
                                logger.warning('Fail to create shortcut {0} ! ({1})'.format(sc_name, sc_exception))
                else:
                    error_msg = "Cannot download artifact {0} to {1}".format(source, destination)
                    logger.error(error_msg)
                    self.error('SetupManagerException', error_msg)


if __name__ == '__main__':
    FsManager.create_desktop_shortcut('ACS Launcher',
                                      r'/usr/bin/python /opt/_Dev/acs/src/AcsLauncher.py',
                                      r'/home/nbrissox/Desktop/ACS Launcher.desktop',
                                      sc_description='Automation Control System (ACS) GUI Launcher',
                                      sc_icon_path=r'/opt/_Dev/acs/src/_Tools/launcher/static/images/logo.png')
