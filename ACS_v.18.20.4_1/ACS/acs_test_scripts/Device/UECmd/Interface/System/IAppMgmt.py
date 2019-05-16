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
:summary: This script implements the interface for application management uecmd.
:since: 10/12/2014
:author: vdechefd
"""
from ErrorHandling.DeviceException import DeviceException


class IAppMgmt():

    """
    Abstract class that defines the interface to be implemented
    by device system operations handling sub classes.

    All method that shall be redefined in sub-classes raise a
    I{DeviceException} error.
    """

    def __init__(self, device):
        """
        Initializes this instance.

        Nothing to be done in abstract class.
        """
        pass

    def install_device_app(self, app_path, timeout=0, allow_downgrade=False):
        """
        Install a device application

        :type app_path: str
        :param app_path: file to be installed

        :type  timeout: int
        :param timeout: operation timeout

        :type  allow_downgrade: bool
        :param allow_downgrade: allow the downgrade of application

        :rtype: list
        :return: Output status and output log
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def backup_app(self, backup_file, app_location, timeout=0):
        """
         Backup application to host file from app location on device

        :type  backup_file: str
        :param backup_file: file path of backup file

        :type  app_location: str
        :param app_location: file path of application on the device

        :type  timeout: int
        :param timeout: time before issuing command

        :rtype: str, str
        :return: status and associated error message
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_path_of_device_app(self, package_name):
        """
        Retrieve the path on device of device application

        :type  package_name: str
        :param package_name: package name of the device application

        :rtype: str
        :return: device path to the application
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_package_name_of_device_app(self, app_name):
        """
        Retrieve package name from an android application

        :type  app_name: str
        :param app_name: the apk file name

        :rtype: tuple (str, str)
        :return: package name and output log
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def clear_cache_data_of_device_app(self, package_name):
        """
        Clear package cache and data of an android application

        :type  package_name: str
        :param package_name: package name of the device application

        :rtype: tuple (bool, str)
        :return: Output status and output log
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def sign_device_app(self, app_path, sign_path, sign_name, out_path, timeout):
        """
        Sign a device application

        :type app_path: str
        :param app_path: file to sign

        :type sign_path: str
        :param sign_path: folder containing signing keys

        :type sign_name: str
        :param sign_name: name of the signature (eg. filename, without extension)

        :type out_path: str
        :param out_path: signed file

        :type  timeout: int
        :param timeout: operation timeout

        :rtype: list
        :return: Output status and output log
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def uninstall_device_app(self, app_name, timeout=0, backup_file=None, forced=False):
        """
        Uninstall a device application

        :type app_name: str
        :param app_name: reference to the application to remove

        :type  timeout: int
        :param timeout: operation timeout

        :type  backup_file: str
        :param backup_file: if set, previous app will be backup to the specified file

        :type  forced: bool
        :param forced: if true, app uninstall will be forced, even if it is a protected or system app

        :rtype: list
        :return: Output status and output log
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def launch_app(self, intent=None, action=None, flags=None, extras=None, data_uri=None, data_type=None):
        """
        Tell ActivityManager to start an application.

        :type intent: str
        :param intent: Component/.intent to send; value goes with the -n option

        :type action: str
        :param action: Intent action; value goes with the -a option

        :type flags: str
        :param flags: Flags; value goes with the -f option.

        :type extras: str
        :param extras: Extras.  May contain one or more -e* options along with data.

        :type data_uri: str
        :param data_uri: Intent data URI; value goes with the -d option.

        :type data_type: str
        :param data_type: Intent data MIME type; value goes with the -t option.

        :raise AcsConfigException: If no intent or action is specified
        :raise DeviceException: If 'am start' command fails.
        :rtype: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def stop_device_app(self, app_name, timeout=0):
        """
        Stop a device application

        :type app_name: str
        :param app_name: reference to the application to remove (package name)

        :type  timeout: int
        :param timeout: operation timeout

        :rtype: tuple (bool, str)
        :return: Output status and output log
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def app_enable_disable(self, package_name, enable):
        """
        Enable/disable package

        :type package_name: str
        :param package_name: name of the package to enable/disable

        :type enable: boolean
        :param enable: True = enable package, False = disable package
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_app_status(self, package_name):
        """
        Get package status

        :type package_name: str
        :param package_name: name of the package

        :rtype: str
        :return: "enable", "disable", "not available"
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)
