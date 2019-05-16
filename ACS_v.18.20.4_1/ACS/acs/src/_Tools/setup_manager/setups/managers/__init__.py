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
# @summary: A manager Base Class
# @since: 4/18/14
# @author: nbrissox
# -------------------------------------------------------------------------------
import os
import shutil
import sys
import urllib2
import traceback
import zipfile

from urlparse import urljoin

try:
    import subprocess32 as subproc
except (ImportError, NameError):
    import subprocess as subproc

from os import path

from setups import logger, SysErrors, DEBUG as GLOBAL_DEBUG

from setups.context import (OsSupport,
                            format_configuration_path,
                            SETUP_SESSION,
                            USERPROFILE,
                            CURRENT_USER)

from setups.helpers import types
from setups.helpers import system


class ManagerBase(object):
    """
    Partial Base Manager

    .. uml::

        class ManagerBase {

            DEBUG = 0

            CACHE_NAME = '.acs'
            DEFAULT_REPO_BASE = "https://mcg-depot.intel.com/artifactory/"
            DEFAULT_CACHE = USERPROFILE, CACHE_NAME

            InstallationException = 'InstallationException'
            ValidationException = 'ValidationException'

            session = SETUP_SESSION

            setup_proxy(**options)
            run_command(command, shell=None, cwd="", env=None)
            chunk_report(bytes_so_far, chunk_size, total_size)
            chunk_read(response, file_path, chunk_size=8192, report_hook=None)
            download_file(self, uri)
            move(self, artifact, dest=None)
            run(*args, **options)
        }

    """

    # Exceptions
    InstallationException = 'InstallationException'
    ValidationException = 'ValidationException'

    DEBUG = GLOBAL_DEBUG

    # Defaults
    CACHE_NAME = path.join('.acs', 'cache')
    DEFAULT_REPO_BASE = "https://mcg-depot.intel.com/artifactory/"
    EXTRA_REPO = ""

    DEFAULT_CACHE = path.join(USERPROFILE, CACHE_NAME)

    session = SETUP_SESSION
    proxy_set = 0

    @classmethod
    def create_desktop_shortcut(cls, sc_name, sc_type, sc_path, sc_description="", sc_icon_path="", sc_extra_args=None):
        """
        Creates a Shortcut on desktop (Windows && Linux)

        :param sc_name: The Shortcut's name
        :type sc_name: str

        :param sc_type: The Shortcut's type (linux only)
        :type sc_type: str

        .. note:: Possible values: application, folder, link

        :param sc_path: The shortcut's path
        :type sc_path: str

        :param sc_description: (optional) The shortcut's description
        :type sc_description: str

        :param sc_icon_path: (optional) The shortcut's icon path
        :type sc_icon_path: str

        :param sc_extra_args: (optional) If `sc_path` points to a binary, extra command line arguments
        :type sc_extra_args: tuple, list

        """

        def create_windows_shortcut():
            """
            Inner method for Windows Only.

            """
            try:
                import pythoncom
                from win32com.shell import shell, shellcon
            except (ImportError, NameError) as import_exception:
                logger.info("Error on libraries to create shortcut ! ({0})".format(import_exception))
                shell, shellcon, pythoncom = None, None, None

            if pythoncom and shell and shellcon:
                shortcut = pythoncom.CoCreateInstance(
                    shell.CLSID_ShellLink,
                    None,
                    pythoncom.CLSCTX_INPROC_SERVER,
                    shell.IID_IShellLink
                )

                shortcut.SetPath(sc_path)
                shortcut.SetArguments(sc_extra_args) if sc_extra_args else None
                shortcut.SetDescription(sc_description)
                if sc_icon_path and path.exists(sc_icon_path):
                    shortcut.SetIconLocation(sc_icon_path, 0)

                desktop_path = shell.SHGetFolderPath(0, shellcon.CSIDL_DESKTOP, 0, 0)
                persist_file = shortcut.QueryInterface(pythoncom.IID_IPersistFile)
                persist_file.Save(path.join(desktop_path, "{0}.lnk".format(sc_name)), 0)

        def create_linux_shortcut():
            """
            Inner method for Linux Only.

            .. seealso:: http://standards.freedesktop.org/desktop-entry-spec/latest/

            """
            exec_value = "{0} {1}".format(sc_path, sc_extra_args) if sc_extra_args else sc_path
            link = ("[Desktop Entry]{3}"
                    "Encoding=UTF-8{3}"
                    "Version=1.0{3}"
                    "Type={0}{3}"
                    "Name={1}{3}"
                    "Exec={2}{3}").format(sc_type.capitalize(), sc_name, exec_value, os.linesep)
            if sc_icon_path and path.exists(sc_icon_path):
                link += "Icon={0}{1}".format(sc_icon_path, os.linesep)

            desktop = path.join(USERPROFILE, 'Desktop')
            if not path.exists(desktop):
                desktop = path.join(USERPROFILE, 'Bureau')

            if path.exists(desktop):
                shortcut = path.join(desktop, sc_name + '.desktop')
                with open(shortcut, 'w+') as shortcut_file:
                    shortcut_file.write(link)

                if path.exists(shortcut):
                    out = cls.run_command('chown -R {1}:{1} "{0}" && chmod +x "{0}"'.format(shortcut, CURRENT_USER))
                    if out.failed:
                        cls.error(cls.InstallationException,
                                  "Error while setting executable bit on Shortcut file: {0}".format(shortcut))
                    if GLOBAL_DEBUG:
                        print(out)

        if cls.session.os == OsSupport.windows:
            create_windows_shortcut()
        elif cls.session.os == OsSupport.linux:
            create_linux_shortcut()
        else:
            cls.error(cls.InstallationException, "Unhandled Operating System")

    @classmethod
    def remove_shortcut(cls, sc_file_path):
        """
        Remove shortcut file
        :param sc_file_path: Shortcut file path
        :type sc_file_path: str
        """
        shortcut_extension = ".lnk" if cls.session.os == OsSupport.windows else ".desktop"

        # Try to get find the shortcut in desktop path in case the file is not found
        if not path.exists(sc_file_path):
            desktop = path.join(USERPROFILE, 'Desktop')
            if not path.exists(desktop):
                desktop = path.join(USERPROFILE, 'Bureau')
            if path.exists(desktop):
                sc_file_path = path.normpath(path.join(desktop, sc_file_path + shortcut_extension))

        logger.info("Removing shortcut '{0}' ...".format(sc_file_path))
        if path.exists(sc_file_path):
            os.unlink(sc_file_path)
        else:
            logger.warning("Fail to remove shortcut {0} ! File not found.".format(sc_file_path))

    @classmethod
    def build_repo_uri(cls, repo_base=DEFAULT_REPO_BASE, extra_repo=EXTRA_REPO):
        """
        Build url to download
        :param repo_base:
        :param extra_repo:
        """
        if repo_base:
            cls.DEFAULT_REPO_BASE = repo_base

        if extra_repo:
            cls.EXTRA_REPO = extra_repo

        return urljoin(repo_base or cls.DEFAULT_REPO_BASE, extra_repo or cls.EXTRA_REPO)

    def __init__(self, **opts):
        """
        Constructor

        :param repo: An URI
        :type repo: str

        :param cache: A local URI
        :type cache: str

        """
        full_repo = opts.get('full_repo')
        if full_repo:
            self.repo = full_repo
        else:
            self.repo = self.build_repo_uri(repo_base=opts.get('repo'),
                                            extra_repo=opts.get('extra_repo'))

        self.cache = opts.get('cache') or self.DEFAULT_CACHE
        self.execute = opts.get('execute', 'install')
        self.data = opts.get('data') or {}

    @classmethod
    def format_configuration_path(cls, conf_path, value=None):
        """
        Formats a configuration path of the form:

            :home:/relative/path/to/artifact
            :program:/relative/path/to/bin
            :default:/relative/path

            ...

        :param conf_path:
        :type conf_path: str

        :param value: The value to replace to
        :type value: str

        :return: The formatted path
        :rtype: str

        """
        return format_configuration_path(conf_path, value)

    @classmethod
    def file_extensions(cls, destination, extensions):
        """
        Tells if the provided string endswith on of passed extensions.

        :param destination:
        :type destination: str

        :param extensions:
        :type extensions: tuple, list

        :return: True or False
        :rtype: int, bool

        """
        for ext in extensions:
            if str(destination).endswith(ext):
                return 1
        return 0

    @classmethod
    def error(cls, flag, message):
        """
        Inner method to create on the fly an Exception according session.

        :param flag: One of the {InstallationException, ValidationException, ...}
        :type flag: str

        :param message: The Exception message
        :type message: str

        :raises: InstallationException, ValidationException

        .. seealso:: :func:`system.setup_exception`

        """
        print(message)
        raise system.setup_exception(flag, message)

    @classmethod
    def unzip(cls, zip_archive, where):
        """
        Runs given binary.

        :param zip_archive: The full path of zipped archive.
        :type zip_archive: str

        :param where: The full path where to extract archive
        :type where: str

        """
        archive = None
        try:
            print("Extracting {0} to {1} ...".format(zip_archive, where))
            archive = zipfile.ZipFile(zip_archive, 'r')
            archive.extractall(where)
        except zipfile.BadZipfile:
            logger.exception('Error while extracting ZIP archive `{0}`'.format(zip_archive))
        finally:
            if archive:
                archive.close()

    @classmethod
    def setup_proxy(cls, **options):
        """
        Wraps :func:`system.setup_proxy` for easiness

        :param options: Options for the proxy settings
        :type options: dict

        """
        system.setup_proxy(**options)

    @classmethod
    def run_command(cls, command, shell=None, cwd="", env=None, ignore_errors=False, silent_mode=False):
        """
        Run a command on the local system.

        ``shell`` is passed directly to `subprocess.Popen
        <http://docs.python.org/library/subprocess.html#subprocess.Popen>`_'s
        ``execute`` argument (which determines the local shell to use.)  As per the
        linked documentation, on Unix the default behavior is to use ``/bin/sh``,
        so this option is useful for setting that value to e.g ``/bin/bash``.

        :param command: The command-line to execute
        :type command: str

        :param shell: See :mod:`subprocess.Popen`
        :type shell: object

        :param cwd: The current directory to execute the command
        :type cwd: str

        :param env: The OS Environment dictionary
        :type env: dict

        :param ignore_errors: Ignore errors
        :type ignore_errors: bool

        :param silent_mode: Log message or not
        :type silent_mode: bool

        :return: A Attribute Dict like object containing all data about just run command
        :rtype: _collections.AttributeStr

        """
        br = os.linesep
        if not silent_mode:
            print("Executing command: {0}".format(command))
            logger.info("[subprocess]: " + command + br)

        out_stream = subproc.PIPE
        err_stream = subproc.PIPE

        try:
            if cwd:
                command = "cd {0} && {1}".format(cwd, command)

            cmd_arg = command if SETUP_SESSION.os == OsSupport.windows else [command]
            options = dict(shell=True,
                           stdout=out_stream,
                           stderr=err_stream,
                           env=env or os.environ.copy())
            if shell is not None:
                options['executable'] = shell

            pop = subproc.Popen(cmd_arg, **options)
            stdout, stderr = pop.communicate()
        finally:
            pass

        # Handle error condition (deal with stdout being None, too)
        out = types.AttributeStr(stdout.strip() if stdout else "")
        err = stderr.strip() if stderr else ""
        rcode = pop.returncode

        out.failed = False
        out.return_code = rcode
        out.stdout = stdout.strip()
        out.stderr = err
        out.cmd = command

        if rcode != 0:
            out.failed = True
            msg = "run_command() encountered an error (return code %s) while executing '%s'" % (rcode, command)
            if ignore_errors:
                msg += " but ignored on user demand !"
            if not silent_mode:
                logger.error(msg + br)

        out.succeeded = not out.failed

        if err and not silent_mode:
            sys.stderr.write(err + br)

        return out

    @classmethod
    def chunk_report(cls, bytes_so_far, chunk_size, total_size, previous_percent=-1.0):
        """

        :param bytes_so_far:
        :type bytes_so_far: int

        :param chunk_size:
        :type chunk_size: int

        :param total_size:
        :type total_size: int

        """

        percent = float(bytes_so_far) / total_size
        percent = round(percent * 100, 0)

        if percent % 5 == 0:
            if previous_percent != percent:
                info_msg = "Downloaded %d of %d bytes (%0.2f%%)" % (bytes_so_far, total_size, percent)
                logger.info(info_msg)
                print(info_msg)
                previous_percent = percent

        return previous_percent

    @classmethod
    def chunk_read(cls, response, file_path, chunk_size=8192):
        """

        :param response:
        :type response:

        :param file_path:
        :type file_path: str

        :param chunk_size:
        :type chunk_size: int

        """
        total_size = response.info().getheader('Content-Length').strip()
        total_size = int(total_size)
        bytes_so_far = 0

        file_rootdir = path.dirname(file_path)
        if not path.isdir(file_rootdir):
            os.makedirs(file_rootdir)

        previous_percent = -1.0
        with open(file_path, 'wb') as file_stream:
            while 1:
                chunk = response.read(chunk_size)
                bytes_so_far += len(chunk)

                if not chunk:
                    logger.info('\n')
                    break

                file_stream.write(chunk)
                previous_percent = cls.chunk_report(bytes_so_far,
                                                    chunk_size,
                                                    total_size,
                                                    previous_percent)

    def download_file(self, uri, local_location="", local_destination=""):
        """
        Downloads a file from a Remote Host (https://mcg-depot.intel.com/artifactory/)

        :param uri: The given Artifact's Uri
        :type uri: str

        :param local_location: (optional) Logical local location (relative path to be concatenated to ACS cache).
        :type local_location: str

        :param local_destination: (optional) Logical local location (full path).
        :type local_destination: str

        .. note:: `local_location` if provided, overrides default auto-build logical file path from given URI.

        :return: Local file path
        :rtype: str

        """
        file_path = local_destination or path.join(self.cache, local_location or uri[len(self.repo):])
        file_path = path.abspath(file_path)

        info_msg = "Downloading {0} ...".format(uri)
        print(info_msg)
        logger.info(info_msg)

        # Disable the proxy
        proxy_handler = urllib2.ProxyHandler({})
        opener = urllib2.build_opener(proxy_handler)
        try:
            # Download the file
            self.chunk_read(opener.open(uri), file_path)
        except urllib2.HTTPError as error:
            try:
                os.unlink(file_path)
            except SysErrors:
                pass

            file_path = None
            if error.code == 404:
                exc = ('HTTP404: The file appears to be missing! '
                       'Please, Contact ACS Support asking for file `{0}` '
                       'to be added to Artifactory repository').format(uri)
            elif error.code == 401:
                exc = ('HTTP401: The file appears to be protected from external download! '
                       'Please, Contact ACS Support asking for file `{0}` '
                       'to be added to Artifactory repository').format(uri)
            elif error.code == 407:
                exc = 'HTTP407: The file appears to be blocked by a Proxy, check your Proxy configuration! '.format(uri)
            else:
                exc = 'HTTP{0}: {1}'.format(error.code, error)

            if self.DEBUG:
                exc += '\n\n{0}'.format(traceback.format_exc())

            logger.error(exc)

        return file_path

    def move(self, artifact, dest=None):
        """
        Moves a file from src to dest.
        Ensures root directory existence.
        Auto build destination based on self.cache property if not provided

        :param artifact: Source path
        :type artifact: str

        :param dest: (optional) Destination path
        :type dest: str

        :return: Destination path
        :rtype: str

        """
        _, artifact_name = path.split(artifact)
        destination = dest or path.join(self.cache, artifact_name)

        print("Moving {0} to {1} ...".format(artifact, dest))

        destination_rootdir = path.abspath(path.dirname(destination))
        if not path.isdir(destination_rootdir):
            os.makedirs(destination_rootdir)

        if path.exists(artifact):
            shutil.move(artifact, destination)

        return destination

    def pre_processing(self, instructions):
        """
        Any Pre processing actions before package installation

        :param instructions: A list of pre-processing instructions
        :type instructions: list

        :raise: SetupManagerException

        """
        if instructions:
            print("Executing pre-processing actions...")
            for instruction in instructions:
                pre_install_script = format_configuration_path(instruction.get("cmd"))
                ignore_errors = types.str2bool(instruction.get("ignore_errors", "false").lower())
                output_cmd = self.run_command(pre_install_script, ignore_errors=ignore_errors)
                if output_cmd.return_code != 0:
                    error_msg = "Error while executing '{0}'".format(pre_install_script)
                    if ignore_errors:
                        error_msg += ", but ignored on user demand"
                    error_msg += " ! ({0})".format(output_cmd)

                    print(error_msg)
                    if not ignore_errors:
                        self.error('SetupManagerException', error_msg)

    def post_processing(self, instructions):
        """
        Any Post processing actions after package installation

        :param instructions: A list of post-processing instructions
        :type instructions: list

        :raise: SetupManagerException

        """
        if instructions:
            print("Executing post-processing actions...")
            for instruction in instructions:
                post_install_script = format_configuration_path(instruction.get("cmd"))
                ignore_errors = types.str2bool(instruction.get("ignore_errors", "false").lower())
                output_cmd = self.run_command(post_install_script, ignore_errors=ignore_errors)
                if output_cmd.return_code != 0:
                    error_msg = "Error while executing '{0}'".format(post_install_script)
                    if ignore_errors:
                        error_msg += ", but ignored on user demand"
                    error_msg += " ! ({0})".format(output_cmd)

                    print(error_msg)
                    if not ignore_errors:
                        self.error('SetupManagerException', error_msg)

    def run(self):
        """
        Runs the manager, to be implemented in sub classes!

        """
        if not ManagerBase.proxy_set:
            self.setup_proxy(reset=True)
            ManagerBase.proxy_set = 1
