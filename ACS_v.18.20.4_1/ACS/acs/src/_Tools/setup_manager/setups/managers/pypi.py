#!/usr/bin/env python
# -*- coding: utf-8 -*-

# -------------------------------------------------------------------------------
# @copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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
# @summary: <enter_summary_details_here>
# @since: 2/17/14
# @author: nbrissox
# -------------------------------------------------------------------------------
import sys

from urlparse import urljoin
from os import path

try:
    import importlib as importer
except ImportError:
    from setups.vendors import importlib as importer

from setups import (logger, absjoin,
                    CONFIG_PIP_REQUIREMENTS_DIR, CONFIG_PIP_CERT_FILE, CONFIG_PIP_PACKAGES_DIR)

from setups.context import OsSupport
from setups.managers import ManagerBase

from setups.vendors import get_pip

from setups.helpers import Loader


class PyPiManager(ManagerBase):

    """
    Class variable so we don't create a PyPiManager log handler every single time.
    Prevent duplicate logs

    .. uml::

        class ManagerBase

        class PyPiManager {

            CACHE_NAME = 'pypi'

            DEFAULT_REPO_BASE = ManagerBase.DEFAULT_REPO_BASE, 'acs/dependencies/third_parties/pypi/'
            DEFAULT_CACHE = ManagerBase.DEFAULT_CACHE, CACHE_NAME

            PIP_MINIMAL_VERSION = '1.5.0'
            PIP_REQUIRED_VERSION = '1.5.6'

            run(*args, **options)
        }

        ManagerBase <|- PyPiManager

    """

    SetupManagerException = 'SetupManagerException'

    CACHE_NAME = 'pypi'
    EXTRA_REPO = 'acs/dependencies/third_parties/pypi/'
    DEFAULT_CACHE = path.join(ManagerBase.DEFAULT_CACHE, CACHE_NAME)

    # Pip minimal version to ensure (wheel support)
    PIP_CERTS_VERSION = '1.3.0'
    PIP_MINIMAL_VERSION = '1.5.0'
    PIP_REQUIRED_VERSION = '1.5.6'

    def __init__(self, **opts):
        """
        Constructor

        :param repo: An Uri
        :type repo: str

        """
        super(PyPiManager, self).__init__(**opts)

        if 'artifacts_location' in self.data:
            pypi_location = self.data['artifacts_location']
            formatted_loc = self.format_configuration_path(pypi_location, self.repo)
            if formatted_loc == pypi_location:
                self.repo = urljoin(self.repo, pypi_location)
            else:
                self.repo = formatted_loc

        self.packages = []
        self.installed_packages = {}
        self.requirements = self.data or []
        if not isinstance(self.requirements, (tuple, list)):
            self.requirements = [self.requirements]

    def validate(self):
        """
        All validations are done here

        """
        self.__validate_repo()
        self.__validate_cache()

    def __validate_repo(self):
        """
        Validates the repository, ensuring 2 main points:

            * If the given URI is included in HTTP[S] patterns, ensures the URL is pointing to a valid repository.
            * If the given URI is a local path, ensures its validity.

        :raise: ValidationException

        """
        if not self.repo:
            self.error(self.ValidationException, 'You MUST provide a Repository URI!')

    def __validate_cache(self):
        """
        Validates the Cache folder path, ensuring 1 main point:

            * Ensures validity of the given local path.

        :raise: ValidationException

        """
        if not self.cache:
            self.error(self.ValidationException, 'You MUST provide a Cache folder path!')

    @classmethod
    def import_package(cls, package_name):
        """
        Try to load the module

        :param package_name: The Python Package name
        :type package_name: str

        :return: The Package instance or None and its existing flag
        :rtype: tuple

        """
        package, exists = None, False

        handled_exceptions = ImportError,
        if cls.session.os == OsSupport.windows:
            handled_exceptions += WindowsError,
        try:
            package = importer.import_module(package_name)
            exists = True
        except handled_exceptions:
            pass
        return package, exists

    @classmethod
    def extract_package_version(cls, pkg, canonical_name=""):
        """
        Look up in each known package we got from `pkg_resources.working_set` with the provided name in order to
        extract its version.

        .. note:: #. We try to find the version from `pkg_resources.working_set` if properly registered.
            The provided name might *NOT* be exactly what's Pip expects to get ! Therefore, we look up in our internal
            dictionary Map (packages_names_map) to see if we got a mapping for this package name.

            This step should be the most of time successful, but as a fallback :

            #. We try to get a version from known module attributes:

                * __version__
                * VERSION
                * version
                * api_version

            #. If None version could be found, 'unknown' is returned !

        :param pkg: The package module instance
        :type: object

        :param canonical_name: (optional) if provided will be checked too
        :type canonical_name: str

        :return: The package version if found else 'unknown'
        :rtype: str, unicode

        """
        try:
            import pkg_resources as resources
        except ImportError:
            resources = None

        version, names_hints = None, set()
        if hasattr(pkg, '__name__'):
            pkg_name = pkg.__name__
            if resources:
                # names_hints = pkg_name.lower(), pkg_name
                names_hints.add(pkg_name)
                names_hints.add(pkg_name.lower())
                if canonical_name:
                    names_hints.add(canonical_name)

                for package in resources.working_set:
                    for hint in names_hints:
                        if package.key == hint:
                            version = package.version
                            break
                        elif package.project_name == hint:
                            version = package.version
                            break

        if not version:
            if hasattr(pkg, '__version__'):
                version = u'{0}'.format(pkg.__version__)
            elif hasattr(pkg, 'VERSION'):
                version = u'{0}'.format(pkg.VERSION)
            elif hasattr(pkg, 'version'):
                version = u'{0}'.format(pkg.version)
            elif hasattr(pkg, 'api_version'):
                version = u'{0}'.format(pkg.api_version)
            else:
                version = u'unknown'

        return version

    @classmethod
    def extract_package_info(cls, pkg, formatted_msg=u'', canonical_name=""):
        """
        Extracts `__version__` and `__name__` from python module.

        .. note:: More data from the module could be extracted if needed ...

        :param pkg: The package returned by the :mod:`importlib` module
            (:meth:`importlib.import_module` method)
        :type: object (module)

        :param formatted_msg: The formatted message to be filled with package data
        :rtype: str, unicode

        :return: A formatted string with package info
        :rtype: str, unicode

        """
        formatted_msg = formatted_msg or (u'The Package : `{pkg.__name__}`:({version})'
                                          u' is already present on your platform ! ')
        return formatted_msg.format(pkg=pkg, version=cls.extract_package_version(pkg, canonical_name=canonical_name))

    def extract_packages_from_requirement(self, requirement_file):
        """
        Extracts python packages from requirement file.

        :param requirement_file: The requirement file path
        :type requirement_file: str, unicode

        """
        packages_from_requirement = []
        if requirement_file and path.isfile(requirement_file):
            with open(requirement_file, 'r') as handle:
                packages_from_requirement += [req.strip() for req in handle.readlines()]

        for req in packages_from_requirement:
            self.append_package_metadata(req)

    def extract_packages(self):
        """
        Extracts all requirements based on Configuration

        """
        if self.requirements:
            for requirement in self.requirements:
                requirement_file = requirement
                if requirement_file and not path.isfile(requirement_file):
                    requirement_file = path.join(CONFIG_PIP_REQUIREMENTS_DIR, requirement)

                if path.isfile(requirement_file):
                    self.extract_packages_from_requirement(requirement_file)
                else:
                    self.append_package_metadata(str(requirement))

    def get_installed_packages(self):
        """
        Get all installed packages using "pip freeze" command

        """
        logger.info("Getting list of installed python packages...")
        import pip
        for package_info in pip.get_installed_distributions():
            self.installed_packages.update({package_info.project_name: package_info.version})

    def append_package_metadata(self, requirement):
        """
        Loads & Appends Package metadata.

        :param requirement: The Package requirement name
        :type requirement: str

        """
        meta_filename = path.join(CONFIG_PIP_PACKAGES_DIR, '{0}.json'.format(requirement.lower()))
        package = Loader.load(meta_filename)
        if 'windows' if self.session.os == OsSupport.windows else 'linux' in package:
            self.packages.append(package)

    def handle_package_by_name(self, pkg_name, **options):
        """
        Installs a package from its name

        .. important:: The package name might contains version specs

        :param pkg_name: The package name
        :type pkg_name: str

        :param options:
        :type options: dict

        :return: The commandline output as an Types.AttributeString instance
        :rtype: Types.AttributeString

        :raise: SetupManagerException

        """
        command_args = self.format_pip_command(pkg_name, **options)
        output_cmd = self.run_command(' '.join(command_args))
        if output_cmd.return_code != 0:
            error_msg = 'Error while executing pip {0} {1}'.format(' '.join(command_args), output_cmd)
            logger.error(error_msg)
            self.error(self.SetupManagerException, error_msg)
        else:
            print(output_cmd)
        return output_cmd

    def ensure_pip_wheel(self):
        """
        Configures all requirements for allowing use of :mod:`pip` module.
        Ensures Pip version is minimum 1.5.6+

        :raise: SetupManagerException

        @Todo: Sometimes, especially on Unix platform, when pip was installed from System packages Manager
            such as `apt-get install python-pip`, then trying to install it from `get_pip.py` script will cause
            troubles in the system path, as the package might not be found from the Command line Terminal!
            Therefore, strange errors like: "/bin/sh: pip not found!" could occur

            This is due (but more reasons might produce this strange behavior) to remaining "pip" files in
            /usr/local/lib/python2.7/dist-packages

            2 directories:

            * pip
            * pip-1.5.6.dist-info

            @solution: look for those patterns in appropriated folders before ensuring pip installation.
                If match(es) is/are found, remove it/them!
        """
        def error_handler(this, callback, args):
            code = callback(args)
            if code != 0:
                msg = 'Error while executing PIP {0}'.format(' '.join(args))
                logger.error(msg)
                this.error(this.SetupManagerException, msg)

        command_args = ['--index-url', self.repo,
                        '--find-links', self.repo]
        try:
            import pip
            import pkg_resources
        except (ImportError, NameError):
            logger.info("'pip' module not found !")
            opts = dict(upgrade=True)
            if self.PIP_REQUIRED_VERSION:
                opts['specific_version'] = ">=" + self.PIP_REQUIRED_VERSION
            error_handler(self, get_pip.main, command_args)

            import pip
            try:
                import pkg_resources
            except ImportError:
                import pip._vendor.pkg_resources as pkg_resources

        # Ensuring All required modules are known from the system
        sys.modules['pip'] = pip
        sys.modules['pkg_resources'] = pkg_resources

        parse_v = pkg_resources.parse_version
        current_v = self.extract_package_version(pip)
        if current_v != u'unknown':
            need2upgrade = parse_v(current_v) < parse_v(self.PIP_MINIMAL_VERSION)
        else:
            # Unknown version, therefore we need to upgrade
            need2upgrade = True

        if need2upgrade:
            error_handler(self, get_pip.main, command_args)

    def format_pip_command(self,
                           requirement,
                           use_wheel=True,
                           download_cache=False,
                           find_links=True,
                           index_url=True,
                           upgrade=True):
        """
        Formats a Pip command arguments as a list to be directly consumes by our internal :meth:`run_command`

        :param requirement: The package name or a requirement file path
        requirement guess: str, unicode

        :param use_wheel: If set to False, pip won't lookup for wheel packages (.whl)
        :type use_wheel: bool

        :param download_cache: If set to True, all downloaded packages are stored in a cache folder
        :type download_cache: bool

        :param find_links: If set to True,
            the specified "pypi" link or the default one is set as "--find-links" Pip option
        :type find_links: bool

        :param index_url: If set to True,
            the specified "pypi" link or the default one is set as "--index-url" Pip option
        :type index_url: bool

        :param upgrade: If set to True, an upgrade to the newest available version is attempted
            even if the package is already installed
        :type upgrade: bool

        :return: The Pip command arguments as a list
        :rtype: list

        """
        command_args = ['pip']
        if self.session.os == OsSupport.windows:
            py_exe = path.dirname(self.session.python.executable_path)
            # Virtualenv case
            if py_exe.lower().endswith('scripts'):
                pip_exe = absjoin(py_exe, "pip")
            else:
                pip_exe = absjoin(py_exe, "Scripts", "pip")
            command_args = [pip_exe]

        command_args.append(self.execute)
        cap_action = self.execute.capitalize()

        # If the `requirement` exists and is a file, then we've got a requirement file
        if requirement and path.isfile(requirement):
            command_args.append('--requirement')
            info_msg = ("{0}ing python packages based on pip"
                        "requirement file : {1} ...").format(cap_action, requirement)
        # Else we've got a package name only
        else:
            info_msg = "{0}ing python package {1} using pip ...".format(cap_action, requirement)

        command_args.append(requirement)

        if self.execute == "install":
            command_args.append('--{0}use-wheel'.format('' if use_wheel else 'no-'))
            if upgrade:
                command_args.append('--upgrade')

            if download_cache:
                command_args.append('--download-cache')
                command_args.append(self.cache)

            if find_links or index_url:
                command_args.append('--find-links')
                command_args.append(self.repo)

            if index_url:
                command_args.append('--index-url')
                command_args.append(self.repo)
            else:
                command_args.append('--no-index')

            command_args.append('--cert')
            command_args.append('"{0}"'.format(CONFIG_PIP_CERT_FILE))

        else:
            command_args.append('--yes')

        print(info_msg)
        logger.info(info_msg)

        return command_args

    def install_packages(self):
        """
        Install the list of packages
        """

        import pkg_resources
        installed_package_list = self.installed_packages.keys()

        for pkg in self.packages:
            package_name = pkg.get('name')
            package_version = pkg.get('version')

            is_already_installed = package_name in installed_package_list

            if is_already_installed:
                versions_parser = pkg_resources.parse_version
                installed_package_version = self.installed_packages[package_name]
                package_version = installed_package_version if package_version == '' else package_version
                if versions_parser(package_version) > versions_parser(installed_package_version):
                    # Reset the flag to force update/installation of the package
                    is_already_installed = False

            if is_already_installed:
                print("The package '{0} ({1})' is already installed !".format(package_name, package_version))

            else:
                # extracting meta_data specific to the OS
                meta_data = pkg.get('windows' if self.session.os == OsSupport.windows else 'linux', {})

                # Execute pre installation scripts
                pre_processing_instructions = meta_data.get("pre_install")
                post_processing_instructions = meta_data.get("post_install")

                self.pre_processing(pre_processing_instructions)
                # Install python package
                out = self.handle_package_by_name(package_name, upgrade=True)
                if out.failed:
                    logger.error(u'Error while installing package : {0} ({1})'.format(package_name, out))
                else:
                    is_already_up2date = bool([hint for hint in ('requirement already up-to-date',
                                                                 'requirement already satisfied')
                                               if hint in out.lower()])
                    if not is_already_up2date:
                        self.post_processing(post_processing_instructions)

    def uninstall_packages(self):
        """
        Uninstall the list of packages
        """

        installed_package_list = self.installed_packages.keys()

        for pkg in self.packages:
            package_name = pkg.get('name')

            if package_name in installed_package_list:
                # extracting meta_data specific to the OS
                meta_data = pkg.get('windows' if self.session.os == OsSupport.windows else 'linux', {})

                # Execute pre uninstalling scripts
                pre_processing_instructions = meta_data.get("pre_uninstall")
                post_processing_instructions = meta_data.get("post_uninstall")

                self.pre_processing(pre_processing_instructions)
                # Uninstall python package
                command_args = self.format_pip_command(package_name)
                out = self.run_command(' '.join(command_args))
                print(out)
                is_ignored_errors = bool([hint for hint in ['not installed', 'access is denied']
                                          if hint in out.lower()])
                if out.failed and not is_ignored_errors:
                    logger.error(u'Error while uninstalling package : {0} ({1})'.format(package_name, out))
                else:
                    if 'not installed' not in out.lower():
                        self.post_processing(post_processing_instructions)
            else:
                print("The package '{0}' is already uninstalled !".format(package_name))

    def run(self):
        """
        Runs the Pypi Manager.

        #. We try to ensure our environment is properly configured in order to setup correctly all dependencies
            * Setuptools >= 4.0.1
            * Pip >= 1.5.6

        #. When our environment is configured, we setup all
            needed python packages according our configuration profile

        .. note:: Some Python packages have compiled C library dependency which might need some
            custom behavior according Operating System on which it is installed (especially Windows)

            For those special package, more metadata are provided from the Configuration file like :

                * Pre-Installation action(s)
                * Post-Installation action(s)

        """
        if self.data:
            super(PyPiManager, self).run()

            self.validate()
            self.ensure_pip_wheel()
            self.extract_packages()
            self.get_installed_packages()

            if self.execute == "install":
                self.install_packages()
            else:
                self.uninstall_packages()


def main():
    """
    Entry point

    """
    PyPiManager().run()


if __name__ == '__main__':
    sys.exit(main())
