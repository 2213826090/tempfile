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
# @summary: Groups all OS Environment Context data.
# @since: 4/22/14
# @author: nbrissox
# -------------------------------------------------------------------------------
import os
import platform
import sys
import getpass

from os import path
from os import environ

from setups.helpers import types
from setups.helpers.patterns import Singleton
from setups.helpers.system import setup_exception


class OsSupport(types.AttributeStr):

    """
    A simple :class:`AttributeStr` subclass acting as an Enum

    """
    linux = 'linux2'
    windows = 'win32'
    cygwin = 'cygwin'

    @classmethod
    def current(cls):
        """
        Auto-detects Platform Operating System.

        :return: The Platform Operating System.
        :rtype: str

        """
        _os = sys.platform.lower()
        if _os.startswith('linux'):
            current_os = cls.linux
        elif _os.startswith('win32'):
            current_os = cls.windows
        elif _os.startswith('cygwin'):
            current_os = cls.cygwin
        else:
            raise setup_exception('UnhandledOS',
                                  '\n\t**Your Operating System ({0}) is not supported!**'.format(_os))
        return current_os


class SetupSession(types.StringBase):

    """
    Acts as a global session class.

    .. note:: Attributes can be accessed as a dict.

    """

    __metaclass__ = Singleton

    os = OsSupport.current()
    python_version = str(platform.python_version())
    python_architecture = platform.architecture()[0]
    if 'win32' in os:
        # On Windows, users can install python 32 bits on a 64 bits machine
        # In this case platform.architecture will return 32 bits as we expect 64 bits.
        # To avoid that, we check environment variables. For further details refer to:
        # http://stackoverflow.com/questions/2764356/python-get-windows-os-version-and-architecture
        platform_architecture = '32bit' if ('x86' in environ.get("PROCESSOR_ARCHITECTURE") and
                                            'AMD64' not in environ.get("PROCESSOR_ARCHITEW6432", "")) else '64bit'
    else:
        platform_architecture = platform.architecture()[0]

    system = types.AttributeStr("Platform",
                                architecture=platform_architecture,
                                is64bit='64bit' in platform_architecture,
                                is32bit='32bit' in platform_architecture,
                                release=platform.release())

    python = types.AttributeStr("Python",
                                version='.'.join(platform.python_version_tuple()[:-1]),
                                full_version=python_version,
                                architecture=python_architecture,
                                is64bit='64bit' in python_architecture,
                                is32bit='32bit' in python_architecture,
                                executable_path=sys.executable)

SETUP_SESSION = SetupSession()
format_windows_path = lambda p: str(p).replace('\\', '/')
if SETUP_SESSION.os == OsSupport.windows:
    ICON_EXTENSION = "ico"
    CURRENT_USER = getpass.getuser()
    USERPROFILE = format_windows_path(os.getenv('USERPROFILE'))
    PYTHONW_EXECUTABLE = path.join(sys.prefix, "pythonw.exe")

    if SETUP_SESSION.system.is32bit:
        PROGRAM_FILES = format_windows_path(os.getenv('ProgramFiles(x86)'))
    else:
        PROGRAM_FILES = format_windows_path(os.getenv('ProgramFiles'))
else:
    ICON_EXTENSION = "png"
    CURRENT_USER = os.environ.get('SUDO_USER')
    USERPROFILE = '/home/{0}/'.format(CURRENT_USER)
    if not path.isdir(USERPROFILE):
        USERPROFILE = os.environ.get('HOME') or path.abspath(path.expanduser('~'))
        # Checking if the CURRENT_USER is known ?
        if not CURRENT_USER:
            # Getting it from USERPROFILE
            CURRENT_USER = USERPROFILE.split(os.sep)[-1]

    PROGRAM_FILES = '/usr/bin'
    PYTHONW_EXECUTABLE = path.abspath(sys.executable)

DEFAULT_CACHE_FOLDER = path.join(USERPROFILE, '.cache')

# Tags handled
HOME_TAG = ":home:"
PROGRAM_TAG = ":program:"
DEFAULT_TAG = ":default:"
PYTHON_PATH_TAG = ":pythonpath:"
PYTHON_EXECUTABLE_TAG = ":pythonexecutable:"
PYTHONW_EXECUTABLE_TAG = ":pythonwexecutable:"
ICON_EXTENSION_TAG = ":iconextension:"


def format_configuration_path(conf_path, value=""):
    """
    Formats a configuration path of the form ::

        :home:/relative/path/to/artifact
        :program:/relative/path/to/bin
        :default:/relative/path
        :python_path:/full/path/to/python/folder
        :python_executable:/full/path/to/python/executable

        ...

    :param conf_path: The configuration path
    :type conf_path: str

    :param value: The value to replace to
    :type value: str

    :return: The formatted path
    :rtype: str

    """
    if conf_path:
        tags = {
            DEFAULT_TAG: value,
            HOME_TAG: USERPROFILE,
            PROGRAM_TAG: PROGRAM_FILES,
            PYTHON_PATH_TAG: path.abspath(sys.prefix),
            PYTHON_EXECUTABLE_TAG: path.abspath(sys.executable),
            PYTHONW_EXECUTABLE_TAG: PYTHONW_EXECUTABLE,
            ICON_EXTENSION_TAG: ICON_EXTENSION
        }
        for tag, replacement in tags.iteritems():
            if tag in conf_path:
                conf_path = conf_path.replace(tag, replacement)
    return conf_path
