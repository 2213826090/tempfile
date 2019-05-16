#!/usr/bin/env python

"""
The main goal of that script is to ensure User and Developer setup for ACS to run properly.

It handles:

    1. Python packages management
    2. Binaries management
    3. Files management

Supported platforms:

    * Linux

        * Ubuntu 10.04
        * Ubuntu 12.04

    * Windows

        * 7 professional
        * 8 professional

.. todo::

        On most Unix platforms, some python packages can be installed straight from OS package manager (apt, ...) AND/or
        via Python common processes (pip, easy_install, setup.py, ...)

        Let's assumes a python package, for example PyYaml, is installed once with package manager
        and once with pip manager.

        As this is completely possible, what could be side effects about having same python package installed twice?

        Moreover, package version might be disconnected !?
        (apt-get install -> python-yaml (3.10) and pip install pyyaml (3.11)).
"""

import os
import shutil
import subprocess
import time
import Queue
from threading import Thread
import platform
import urllib2
import sys
import logging

from os import path

# In order to use ACS Core framework Classes (Re-usability is the key :)
SELF_DIRNAME = path.abspath(path.join(path.dirname(__file__), '..'))
if SELF_DIRNAME not in sys.path:
    sys.path.append(SELF_DIRNAME)
# Here, goes all ACS Core Framework imports
from UtilitiesFWK.Caching import ArtifactoryCacheManager


logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger('ACS UPDATE PACKAGE(s) SCRIPT')


PY_VERSION = platform.python_version()
PLATFORM = platform.system().lower()

ON_POSIX = 'linux' in PLATFORM
ON_WINDOWS = 'windows' in PLATFORM
ON_CYGWIN = 'cygwin' in PLATFORM


def test_pip_path(possible_paths):
    """
    Ensures given paths are valid.

    .. important:: The first Valid path encountered is returned

    :param possible_paths: List of pip possible paths
    :type possible_paths: list

    :return: Pip full path or ""
    :rtype: str

    """
    pip_path = ""
    for test_path in possible_paths:
        if os.path.isfile(test_path):
            pip_path = test_path
            break
    return pip_path

if "2.6" in PY_VERSION:
    PY_VERSION = "26"
elif "2.7" in PY_VERSION:
    PY_VERSION = "27"
else:
    logger.error("Unsupported python version, please install python 2.6 or 2.7")
    sys.exit(-1)

if ON_POSIX or ON_CYGWIN:
    PIP = test_pip_path([r"/usr/local/bin/pip", r"/usr/bin/pip"])
else:
    PIP = test_pip_path([r"C:\Python{0}\Scripts\pip.exe".format(PY_VERSION)])

TEMP_DIR = os.path.expanduser('~/.acs/cache')
# 10 GB of Cache should be enough :)
CACHE_MAX_SIZE = 1024 ** 3 * 10
CacheManager = ArtifactoryCacheManager(TEMP_DIR, None, max_size_in_bytes=CACHE_MAX_SIZE)

ARTIFACTORY_PYPI_URL = "https://mcg-depot.intel.com/artifactory/"
SSL_CERTIFICATE = path.abspath(path.join(SELF_DIRNAME, '_Tools', 'ca-certificates.crt'))

LINUX_DEB_PACKAGES = ["python-pip",
                      "libxml2-dev",
                      "libxslt1-dev",
                      "python-dev",
                      "python-numpy",
                      "python-scipy",
                      "python-imaging",
                      "python-yaml",
                      "rst2pdf",
                      "graphviz",
                      "python-reportlab",
                      "gphoto2",
                      "mediainfo"]

# Miscellaneous files
MISC_FILES = {
    '~/.doc/pypi/doctool/plantuml.jar': (
        "{0}pypi/doctool/plantuml.7995.jar".format(ARTIFACTORY_PYPI_URL), 'DEVEL'
    )
}
# All Windows binaries required (Python-free)
WINDOWS_BINARIES = {
    "dot": ("{0}pypi/windows/graphviz-2.28.0.msi".format(ARTIFACTORY_PYPI_URL))
}

PYTHON_DEV_MODULES = ["pylint",
                      "flake8",
                      "autopep8",
                      "mock",
                      "unittest2",
                      "sphinx",
                      "sphinxcontrib-plantuml",
                      "sphinx-bootstrap-theme",
                      "pytest",
                      "pytest-xdist"]

PYTHON_MODULES = ["importlib",
                  "psutil",
                  "argparse",
                  "pyserial",
                  "requests",
                  "py-dom-xpath",
                  "lxml",
                  "beautifulsoup",
                  "pytz",
                  "tzlocal"]

build_artifact_uri = lambda package, os_platform="windows": "{0}pypi/{2}/{1}".format(ARTIFACTORY_PYPI_URL,
                                                                                     package,
                                                                                     os_platform)

# Dictionary keys represents the real Python package name as it would be imported from a Python script!
# Let's take an example: PyYaml
# Although, the package name is PyYaml, you'd import it as yaml (import yaml)
# In order not to re-install each package every time the update_package script is run,
# we've got kind of python which command, which is "importlib" package.
# For it to detect the package is or not installed the imported name MUST be used.

# Example:
# { ...
#   yaml=PyYaml_binary_name.exe
# ... }

PYTHON_WIN_MODULES = {
    "26": dict(easy_install=build_artifact_uri("setuptools-5.8.win32-py2.6.exe"),
               pip=build_artifact_uri("pip-1.5.6.win32-py2.6.exe"),
               psutil=build_artifact_uri("psutil-1.2.1.win32-py2.6.exe"),
               lxml=build_artifact_uri("lxml-3.2.4.win32-py2.6.exe"),
               reportlab=build_artifact_uri("reportlab-2.7.win32-py2.6.exe"),
               win32api=build_artifact_uri("pywin32-218.5.win32-py2.6.exe"),
               Crypto=build_artifact_uri("pycrypto-2.6.win32-py2.6.exe"),
               numpy=build_artifact_uri("numpy-MKL-1.8.1.win32-py2.6.exe"),
               pyaudio=build_artifact_uri("PyAudio-0.2.8.win32-py2.6.exe"),
               yaml=build_artifact_uri("PyYAML-3.11.win32-py2.6.exe"),

               # If a list is provided, it means there's some dependencies for this package
               # As a list keeps its insertion order, place dependencies in the right order.
               scipy=(build_artifact_uri("ScientificPython-2.9.3.win32-py2.6.exe"),
                      build_artifact_uri("scipy-0.13.3.win32-py2.6.exe")),

               wxPython=(build_artifact_uri("wxPython-common-2.8.12.1.win32-py2.6.exe"),
                         build_artifact_uri("wxPython-2.8.12.1.win32-py2.6.exe")),

               pygtk=(build_artifact_uri("py2cairo-1.10.0.win32-py2.6.exe"),
                      build_artifact_uri("pygobject-2.28.6.win32-py2.6.exe"),
                      build_artifact_uri("pygtk-2.22.0.win32-py2.6.exe")),

               PIL=(build_artifact_uri("Pillow-2.4.0.win32-py2.6.exe"),
                    build_artifact_uri("PIL-1.1.7.win32-py2.6.exe"))),

    "27": dict(easy_install=build_artifact_uri("setuptools-5.8.win32-py2.7.exe"),
               pip=build_artifact_uri("pip-1.5.6.win32-py2.7.exe"),
               psutil=build_artifact_uri("psutil-1.2.1.win32-py2.7.exe"),
               lxml=build_artifact_uri("lxml-3.2.4.win32-py2.7.exe"),
               reportlab=build_artifact_uri("reportlab-2.7.win32-py2.7.exe"),
               win32api=build_artifact_uri("pywin32-218.5.win32-py2.7.exe"),
               Crypto=build_artifact_uri("pycrypto-2.6.win32-py2.7.exe"),
               numpy=build_artifact_uri("numpy-MKL-1.8.1.win32-py2.7.exe"),
               pyaudio=build_artifact_uri("PyAudio-0.2.8.win32-py2.7.exe"),
               yaml=build_artifact_uri("PyYAML-3.11.win32-py2.7.exe"),

               # If a list is provided, it means there's some dependencies for this package
               # As a list keeps its insertion order, place dependencies in the right order.
               scipy=(build_artifact_uri("ScientificPython-2.9.3.win32-py2.7.exe"),
                      build_artifact_uri("scipy-0.13.3.win32-py2.7.exe")),

               wxPython=(build_artifact_uri("wxPython-common-2.8.12.1.win32-py2.7.exe"),
                         build_artifact_uri("wxPython-2.8.12.1.win32-py2.7.exe")),

               pygtk=(build_artifact_uri("py2cairo-1.10.0.win32-py2.7.exe"),
                      build_artifact_uri("pygobject-2.28.6.win32-py2.7.exe"),
                      build_artifact_uri("pygtk-2.22.0.win32-py2.7.exe")),

               PIL=(build_artifact_uri("Pillow-2.4.0.win32-py2.7.exe"),
                    build_artifact_uri("PIL-1.1.7.win32-py2.7.exe")))
}


class _CommandResultStr(str):

    """
    String subclass which allows arbitrary attribute access.

    """


def is_root():
    """
    Checks if the script is run with SUDO privileges.

    :rtype: bool

    """
    if 'SUDO_UID' not in os.environ.keys():
        logger.error("Please execute that script with super user privileges.")
        sys.exit(1)
    else:
        return True


# noinspection PyUnresolvedReferences
def is_admin():
    """

    :return:
    :rtype: bool

    """
    try:
        # only windows users with admin privileges can read the C:\windows\temp
        os.listdir(os.sep.join([os.environ.get('SystemRoot', r'C:\windows'), 'temp']))
    except WindowsError:
        logger.error("Please execute that script in admin DOS prompt.")
        sys.exit(1)
    else:
        return True


def is_os_supported():
    """
    Ensure Windows 32bit architecture.

    """
    if ON_WINDOWS and "32" not in platform.architecture()[0]:
        logger.error("Please install python 32bit version on windows.")
        sys.exit(1)


def run_command(command, shell=None, cwd=".", env=None):
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

    :return: A Attribute Dict like object containing all data about just run command
    :rtype: _collections._CommandResultStr

    """
    br = os.linesep
    sys.stdout.write("[subprocess]: " + command + br)

    out_stream = subprocess.PIPE
    err_stream = subprocess.PIPE

    try:
        if cwd:
            command = "cd {0} && {1}".format(cwd, command)

        cmd_arg = command if ON_WINDOWS else [command]
        options = dict(shell=True, stdout=out_stream, stderr=err_stream, env=env)
        if shell is not None:
            options['executable'] = shell

        pop = subprocess.Popen(cmd_arg, **options)
        stdout, stderr = pop.communicate()
    finally:
        pass

    # Handle error condition (deal with stdout being None, too)
    out = _CommandResultStr(stdout.strip() if stdout else "")
    err = stderr.strip() if stderr else ""
    rcode = pop.returncode

    out.failed = False
    out.return_code = rcode
    out.stderr = err

    if rcode != 0:
        out.failed = True
        msg = "run_command() encountered an error (return code {0}) while executing '{1}'".format(rcode, command)
        sys.stderr.write(msg + br)

    out.succeeded = not out.failed

    if err:
        sys.stderr.write(err + br)

    return out


def run_binary(binary_file):
    """
    Runs given binary.

    :param binary_file: The full path of binary to be run.
    :type binary_file: str

    """
    logger.info("Execute {0}...".format(binary_file))

    if binary_file.endswith('.msi'):
        binary_file = 'msiexec /i {0}'.format(os.path.normpath(binary_file))

    out = subprocess.call(binary_file)

    if out != 0:
        logger.error("Cannot install  {0}".format(binary_file))
        sys.exit(-1)


def which(program):
    """
    Emulates Unix like Which Command on Windows.

    :param program: The program to be found
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
    if ON_WINDOWS and not program.endswith('.exe'):
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


def clean_cache_name(name, sub_len=0):
    """
    Makes a Cache Artifact name with given name.

    :param name: The given name or url.
    :type name: str

    :param sub_len: The len to substring from name
    :type sub_len: int

    :return: A clean Cache name if possible.
    :rtype: str

    """
    uri = name[sub_len:]

    uri = uri.replace('~', '')
    uri = uri.replace('\\', '/')
    uri_parts = uri.split('/')

    return '/'.join([u.replace('/', '') for u in uri_parts if u])


def install_windows_binary(binary_name, url):
    """
    Install Windows binary on the platform.

    :param binary_name: the Version-free name.
    :type binary_name: str

    :param url: Remote artifact url.
    :type url: str

    """
    if not which(binary_name):
        artifact = get_artifact_from_cache(url)
        logger.debug('WINDOWS ARTIFACT: {0}'.format(artifact))
        run_binary(artifact.value)


def install_windows_binaries():
    """
    Installs all Windows Binaries from WINDOWS_BINARIES dictionary

    """
    for binary_name, url in WINDOWS_BINARIES.iteritems():
        install_windows_binary(binary_name, url)


def install_misc_file(logical_location, url):
    """
    Install Windows binary on the platform.

    :param url: Remote artifact url.
    :type url: str

    :param logical_location: the Version-free name.
    :type logical_location: str

    """
    artifact = get_artifact_from_cache(clean_cache_name(logical_location, sub_len=len(TEMP_DIR)),
                                       alias=clean_cache_name(url, sub_len=len(ARTIFACTORY_PYPI_URL)))

    if not os.path.exists(logical_location):
        rootdir = os.path.dirname(logical_location)
        if not os.path.isdir(rootdir):
            os.makedirs(rootdir)

        shutil.move(artifact.value, logical_location)

        environ = os.environ.copy()
        user = environ.get('HOME')

        # As the script must be run as "root" on Linux OS,
        # file permissions can be problematic for future use by ACS.
        # Therefore, a recursive change owner command line (chown -R current_user:current_user_group <directory>)
        # is applied on the parent directory.
        if (ON_POSIX or ON_CYGWIN) and user:
            user = user.split(os.sep)[-1].strip()
            run_command('chown -R {1}:{1} {0}'.format(os.path.dirname(logical_location), user),
                        env=environ, cwd="", shell="/bin/bash")


def install_misc_files(options):
    """
    Installs miscellaneous files on disk.
    Optionally, set en Environment Variable.

    :param options: The Namespace arguments got from Command-Line
    :type options: argparse.Namespace

    """
    current_target = "DEVEL" if options.developer else "USER"
    for misc_name, data in MISC_FILES.iteritems():
        url, target = data
        if target == current_target:
            if misc_name.startswith('~'):
                name = os.path.expanduser(misc_name)
            else:
                name = os.path.abspath(misc_name)
            install_misc_file(name, url)


def ensure_pip_on_windows():
    """
    Ensures PIP is installed properly on WINDOWS (Only)
    As it is an executable on Windows.

    If PIP has been installed manually (easy_install for instance)

    .. code-block:: python

        importlib.import_module(module_name)

    will find the module, but could be just a remaining "idle" egg file.

    As PIP does not need to be triggered twice, see :func:`install_windows_packages`

    The ``pip`` key is deleted from PYTHON_WIN_MODULES[PY_VERSION] dictionary.

    """
    if ON_WINDOWS and not which('pip'):
        pip_artifact_uri = str(PYTHON_WIN_MODULES[PY_VERSION]['pip'])
        del PYTHON_WIN_MODULES[PY_VERSION]['pip']
        run_binary(download_file(pip_artifact_uri))


def install_windows_packages():
    """
    Installs Windows Packages only.

    .. note:: A simple dependencies mechanism has been put in place.

    .. todo:: Brain-storm about this mechanism implementation.

    """
    for package_name, package_uri in PYTHON_WIN_MODULES[PY_VERSION].iteritems():
        # Handle simple dependencies mechanism.
        if isinstance(package_uri, (tuple, set, frozenset, list)):
            for package_url in package_uri:
                install_windows_package(package_name, package_url)
        else:
            # noinspection PyTypeChecker
            install_windows_package(package_name, package_uri)


def install_windows_package(package_name, package_uri):
    """
    Installs a windows Python package_name on Windows Only.

    Mostly compiled packages (.exe)

    :param package_name: the package_name (name) to be installed
    :type package_name: str

    :param package_uri: The Remote or local uri
    :type package_uri: str

    """
    try:
        # noinspection PyUnresolvedReferences
        import importlib
        importlib.import_module(package_name)
    except ImportError:
        cleaned_cache_name = clean_cache_name(package_uri, sub_len=len(ARTIFACTORY_PYPI_URL))
        artifact = get_artifact_from_cache(cleaned_cache_name)
        logger.info("Execute {0}...".format(artifact.value))
        ret = subprocess.call(artifact.value)
        if ret != 0:
            logger.error("Cannot install  {0}".format(artifact.value))
            sys.exit(-1)


def enqueue_output(out, queue):
    """
    Enqueue Outputs.

    :param out: The read output
    :type out: fileIO

    :param queue: The Queue
    :type queue: Queue.Queue()

    """
    for line in iter(out.readline, ''):
        queue.put(line)
    out.close()


def run_local_command(args):
    """
    Executes the formatted command.

    :param args: Command-Line arguments.
    :type args: list|tuple

    :returns: The Process created by Popen instance and The Output Queue
    :rtype: subprocess.Popen, Queue.Queue()

    """
    env = os.environ.copy()
    process = subprocess.Popen(args, shell=False, stdout=subprocess.PIPE, stdin=subprocess.PIPE,
                               stderr=subprocess.STDOUT, bufsize=1, close_fds=ON_POSIX,
                               env=env)
    queue = Queue.Queue()
    t = Thread(target=enqueue_output, args=(process.stdout, queue))
    t.name = "Thread exec: {0}".format(args)
    t.daemon = True  # thread dies with the program
    t.start()
    return process, queue


def read_output(output):
    """
    Reads output from Queue.

    :param output: The Queue
    :type output: Queue.Queue()

    :return: Concatenated line from Output Queue.
    :rtype: str

    """
    data = ""
    if output:
        try:
            while not output.empty():
                out = output.get_nowait()
                if out:
                    data += out
        except Queue.Empty:
            pass
    return data


def get_artifact_from_cache(artifact_name, alias=""):
    """
    Gets an Artifact from Local Cache or Downloads it if not there.

    :param artifact_name: The Artifact name
    :type artifact_name: str

    :param alias: An alias to be concatenated to remote root URI.
    :type alias: str

    .. note:: Typically, plantuml.jar is a good example.

        The artifact is named on remote host **plantuml.7995.jar**

        As we do not want to handle version in code, we need to be able to give
        an alias to get the same code and allow to follow versioning product.

    :return: The Artifact instance
    :rtype: Caching.CachedArtifact

    """
    if artifact_name.startswith('http'):
        download_uri = artifact_name
        artifact_name = artifact_name[len(ARTIFACTORY_PYPI_URL):]
    else:
        download_uri = '{0}{1}'.format(ARTIFACTORY_PYPI_URL, alias if alias else artifact_name)

    artifact = CacheManager.get(artifact_name)
    if not artifact:
        artifact = CacheManager.add(artifact_name, download_file(download_uri))
    return artifact


def download_file(uri):
    """
    Downloads a file from a Remote Host (https://mcg-depot.intel.com/artifactory/pypi/)

    :param uri: The given Artifact's Uri
    :type uri: str

    :return: Local file path
    :rtype: str

    """
    file_path = os.path.join(TEMP_DIR, uri[len(ARTIFACTORY_PYPI_URL):])
    rootdir = os.path.dirname(file_path)
    if not os.path.isdir(rootdir):
        os.makedirs(rootdir)

    logger.info("Downloading {0} ...".format(uri))

    # Disable the proxy
    proxy_handler = urllib2.ProxyHandler({})
    http_handler = urllib2.HTTPHandler()
    https_handler = urllib2.HTTPSHandler()

    opener = urllib2.build_opener(proxy_handler, http_handler, https_handler)

    try:
        # Download the file
        url_stream = opener.open(uri)
    except urllib2.HTTPError:
        logger.error('ERROR: Please, check your Internet & Proxy settings!')
        raise

    with open(file_path, 'wb') as file_stream:
        while True:
            data = url_stream.read(8192)
            if data:
                file_stream.write(data)
            else:
                break
    return file_path


def install_linux_deb_packages():
    """
    Installs Linux (.deb) packages.

    """
    try:
        for module in LINUX_DEB_PACKAGES:
            logger.info("Install {0}".format(module))
            p, q = run_local_command(["apt-get", "install", module])

            while p.poll() is None:
                output = read_output(q).rstrip('\r\n')
                if output:
                    logger.info(output)
                    if ("After this operation" in output or
                            "Do you want to continue" in output or
                            "Install these packages without verification" in output):
                        time.sleep(1)
                        p.stdin.write("Y\n")
                time.sleep(0.5)

            output = read_output(q).rstrip('\r\n')
            if output:
                logger.info(output)

            if 0 != p.poll():
                logger.error("Failed to install {0}".format(module))
                sys.exit(-1)

    except OSError as exception:
        logger.error("Cannot execute apt-get: {0}".format(exception))
        sys.exit(-1)


def update_pip():
    """
    Updates pip

    """
    pypi_repo_opt = "{0}pypi/".format(ARTIFACTORY_PYPI_URL)
    cert_pip_opt = "--cert={0}".format(SSL_CERTIFICATE)
    env = os.environ.copy()
    reset_proxy(env)

    if not PIP:
        # Pip is not installed, do it with easy_install
        logger.warning("pip not installed, install it with easy_install")

        # HTTPS logic, easy_install takes not parameter for SSL verification, We therefore need to ensure
        # All Bundle certificate(s) are present on the system (LINUX Only)
        certs_dir = "/etc/pki/tls/certs/"
        out = run_command('mkdir -p {0}'.format(certs_dir))

        if out.failed:
            logger.error("Could NOT make directory : "
                         "{0} Ensure you've got right permissions for it.".format(certs_dir))
            sys.exit(1)

        out = run_command('cp {0} /etc/pki/tls/certs/ca-bundle.crt'.format(SSL_CERTIFICATE))
        if out.failed:
            logger.error("Could NOT copy file : "
                         "{0} Ensure you've got right permissions for it.".format(SSL_CERTIFICATE))
            sys.exit(1)

        cmd = ["easy_install", "{0}pip-1.5.6-intelsslcerts.tar.gz".format(pypi_repo_opt)]
        logger.info("Install pip from artifactory...")
        logger.debug(cmd)
        result = subprocess.call(cmd, env=env)
        if 0 != result:
            logger.error("Failed, install pip from official repo...")
            cmd = ["easy_install", "pip"]
            result = subprocess.call(cmd, env=os.environ.copy())
    else:
        logger.info("Update pip...")
        cmd = [PIP,
               "install",
               "--upgrade",
               "--no-index",
               "--find-links={0}".format(pypi_repo_opt),
               cert_pip_opt,
               "pip"]
        result = subprocess.call(cmd, env=env)

    if 0 != result:
        cmd = [PIP,
               "install",
               "--upgrade",
               "--no-index",
               "--find-links={0}".format(pypi_repo_opt),
               cert_pip_opt,
               "pip"]

        result = subprocess.call(cmd)

        if 0 != result:
            msg = ("Cannot install/upgrade pip, check the proxy conf to enable the connection to:"
                   "\n{0}\nor to internet".format(ARTIFACTORY_PYPI_URL))
            logger.error(msg)
            sys.exit(-1)


def reset_proxy(env=None):
    """
    Resets PROXY Setting if there.

    :param env: os.environ.copy()
    :type env: dict (proxy)

    """
    env = env or os.environ.copy()
    env["NO_PROXY"] = ".intel.com"
    if "HTTP_PROXY" in env:
        del env["HTTP_PROXY"]
    if "HTTPS_PROXY" in env:
        del env["HTTPS_PROXY"]


def get_install_cmd(module, artifactory_enable, upgrade=True):
    """
    Gets PIP install command for the given Python module.

    :param module: Python module's name
    :type module: str

    :param artifactory_enable:
    :type artifactory_enable: bool

    :param upgrade: add --upgrade option to pip command-line
    :type upgrade: bool

    :returns: The command as list and Environment Dictionary
    :rtype: list, dict

    .. seealso:: `Pip Documentation<http://pip.readthedocs.org/en/latest/index.html>`_

    """
    env = os.environ.copy()
    cmd = [PIP, "install"]

    if upgrade:
        cmd.append('--upgrade')

    if artifactory_enable:
        cmd.append('--no-index')
        cmd.append('--find-links={0}pypi/'.format(ARTIFACTORY_PYPI_URL))
        cmd.append('--cert={0}'.format(SSL_CERTIFICATE))
        reset_proxy(env)

    cmd.append(module)

    return cmd, env


def install_python_modules(options=None):
    """
    Installs Python Modules.

    :param options: Options passed via sys.argv
    :type options: argparse.Namespace

    """
    options = options or get_options()

    artifactory_access = True
    python_modules = PYTHON_MODULES

    if options.developer:
        python_modules += PYTHON_DEV_MODULES

    if ON_WINDOWS:
        # Do not install modules that has been installed with the setup
        win_python_module = PYTHON_WIN_MODULES[PY_VERSION]
        python_modules = [x for x in python_modules if x not in win_python_module]

    for module in python_modules:
        # Install the python module
        logger.info("Install {0}...".format(module))
        up = not len(str(module).split('==')) > 1
        cmd, env = get_install_cmd(module, artifactory_access, upgrade=up)
        logger.debug('Command-Line: "{0}"'.format(' '.join(cmd)))
        result = subprocess.call(cmd, env=env)
        if 0 != result and not artifactory_access:
            # Failed, we may not have access to artifactory, try using official pip repo
            artifactory_access = False
            # Retry the failed one
            cmd, env = get_install_cmd(module, artifactory_access)
            logger.debug('Command-Line: "{0}"'.format(' '.join(cmd)))
            result = subprocess.call(cmd, env=env)

        if 0 != result:
            logger.error("Cannot install {0}".format(module))
            sys.exit(1)


def get_options(arguments=None):
    """
    Gets options from :mod:`argparse` module, :class:`argparse.ArgumentParser`

    :param arguments:
    :type arguments:

    :return:
    :rtype:

    """
    # optparse is to be abandoned for argparse starting from python 2.7+

    # import argparse
    # parser = argparse.ArgumentParser(description="Helper to setup automatically your Environment")
    # parser.add_argument("--dev",
    #                     action="store_true",
    #                     dest="developer",
    #                     help="Install User & Developer packages (actually all packages)",
    #                     default=False)
    #
    # parser.add_argument("--user",
    #                     action="store_true",
    #                     dest="user",
    #                     help="Install acs and development packages",
    #                     default=True)
    #
    # return parser.parse_args(arguments or sys.argv[1:])

    import optparse

    parser = optparse.OptionParser()
    parser.add_option("--dev",
                      help="Install User & Developer packages (actually all packages)",
                      dest="developer",
                      default=False,
                      action='store_true')

    parser.add_option("--user",
                      help="Install Acs and development packages",
                      dest="user",
                      default=True,
                      action='store_true')

    return parser.parse_args()[0]


def main():
    """
    Entry point

    """
    is_os_supported()
    options = get_options()

    if ON_POSIX or ON_CYGWIN:

        if ON_POSIX and is_root():
            install_linux_deb_packages()

        update_pip()

    elif ON_WINDOWS:
        ensure_pip_on_windows()
        install_windows_packages()
        install_windows_binaries()

    install_python_modules(options)
    install_misc_files(options)

    return 0


if __name__ == '__main__':
    print("""
This script is obsolete !
Please use setup_mgr.py script from _Tools/setup_manager/setups folder ...

- Go to setup_manager folder:
    cd setup_manager/setups

- To install ACS third parties:
    python setup_mgr.py install acs_third_parties

- To install developer third parties:
    python setup_mgr.py install devel
""")

    sys.exit(1)
