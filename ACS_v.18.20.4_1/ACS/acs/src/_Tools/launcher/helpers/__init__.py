#-------------------------------------------------------------------------------
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
# @since: 5/15/14
# @author: nbrissox
#-------------------------------------------------------------------------------
"""
ACS Launcher Helper module

"""
import os
import subprocess
import qt_compat

DEBUG = qt_compat.DEBUG
WIN32 = qt_compat.WIN32
ACS_ENVIRONMENT_NAME = 'ACS_EXECUTION_CONFIG_PATH'

#  Aliases
path = os.path

if WIN32:
    try:
        # noinspection PyUnresolvedReferences
        import win32api

        # noinspection PyUnresolvedReferences
        import _winreg as wr

    except (ImportError, NameError):
        raise ImportError("the pywin32 module is not installed: "
                          "see http://sourceforge.net/projects/pywin32")
else:
    import pwd
    import signal

    BASHRC_FILE = path.abspath(path.join(path.expanduser('~'), '.bashrc'))

__xml_library_used = "None"
try:
    from lxml import etree
    __xml_library_used = "lxml.etree"
except ImportError:
    try:
        # normal cElementTree install
        # noinspection PyPep8Naming
        import cElementTree as etree
        __xml_library_used = "cElementTree"
    except ImportError:
        try:
            # normal ElementTree install
            # noinspection PyUnresolvedReferences
            # noinspection PyPep8Naming
            import elementtree.ElementTree as etree
            __xml_library_used = "ElementTree"
        except ImportError:
            __xml_library_used = "Failed to import ElementTree!"
            raise

if DEBUG:
    print("running with {0}".format(__xml_library_used))


class XMLWrapper(object):

    """
    Simple XML Helper, which wraps lxml module

    """

    parse = staticmethod(etree.parse)

    def __init__(self, source):
        self.root = None
        tree = self.parse(source)
        if tree:
            self.root = tree.getroot()

    def __contains__(self, item):
        return self.root and item in self.root.tag


class QCommand(qt_compat.QtCore.QObject):

    """
    QCommand Class.

    """

    # noinspection PyUnresolvedReferences
    polling = qt_compat.QtCore.Signal(object)

    class QCommandResult(str):

        """
        QCommandResult Class.

        """

    def __init__(self, command, parent=None):
        super(QCommand, self).__init__(parent)
        self._command = command

        self.out = subprocess.PIPE
        self.err = subprocess.PIPE

        self._result = None
        self._process = None

    def interrupt(self):
        """
        Interrupts the Current Command Execution.

        :return: None

        .. warning:: Here, we have no choice but to kill the process brutally
            Only starting from python version 2.7.7 a 'signal.CTRL_C_EVENT' becomes available

        .. todo:: Change both methods with something like 'self._process.send_signal(signal.CTRL_C_EVENT)'
            when ACS will change its python version to 2.7.7+

        """
        pid = self._process.pid
        if WIN32:
            # import ctypes
            # ctypes.windll.kernel32.GenerateConsoleCtrlEvent(0, self._process.pid)
            command = QCommand('taskkill /T /F /PID {0}'.format(pid))
            command.execute()
        else:
            self._process.send_signal(signal.SIGTERM)
        self.polling.emit("ABORT ACS process ({0}) Killed".format(pid))

    def execute(self, cwd="", env=None, shell=None, polling=False, **kwargs):
        """
        Launches 'command' windowless and waits until finished.

        :param cwd: Current Working Directory
        :type cwd: str

        :param env: The Environment Context (Commonly, a copy of :mod:`os.environ`)
        :type env: dict

        :param shell: Which shell executable is to be used (/bin/bash, /bin/sh, ...)
        :type shell: str

        :param polling: if True, The Command Execution outputs std{out,err}
            are propagated via Signal in (almost) real time
        :type polling: bool

        :param kwargs: Any extra :class:`subprocess.Popen` options
        :type kwargs: dict

        :return: Returns A QCommandResult instance
        :rtype: QCommand.QCommandResult

        """
        self._result = ""
        try:
            if cwd:
                self._command = "cd {0} && {1}".format(cwd, self._command)

            cmd_arg = self._command if WIN32 else [self._command]
            options = dict(shell=True, stdout=self.out, stderr=self.err, env=env)

            if kwargs:
                options.update(kwargs)

            if shell is not None:
                options['executable'] = shell

            if DEBUG:
                print('Executed Command: {0}'.format(self._command))

            self._process = subprocess.Popen(cmd_arg, **options)

            # Shall we emit in (almost) real time QCommand Execution outputs?
            if polling:
                # Emitting `stdout` lines
                for line in iter(self._process.stdout.readline, ''):
                    self.polling.emit(line)

                # Emitting `stderr` lines
                for line in iter(self._process.stderr.readline, ''):
                    self.polling.emit(line)

                self._process.stdout.flush()
                self._process.stderr.flush()

                self._process.stdout.close()
                self._process.stderr.close()
            # Or just wait for the execution to end
            else:
                stdout, stderr = self._process.communicate()
                # Handle error condition (deal with stdout being None, too)
                out = stdout.strip() if stdout else ""
                err = stderr.strip() if stderr else ""

                self._result = self.QCommandResult(out or err)
                self._result.failed = False

                self._result.stdout = out
                self._result.stderr = err
                self._result.return_code = rcode = self._process.returncode

                if rcode != 0:
                    self._result.failed = True

                self._result.succeeded = not self._result.failed
        finally:
            pass
        return self._result


def get_username():
    """
    Returns the username of the current logged on user.

    .. important:: Portable on Windows and Unix.
    """
    if WIN32:
        return win32api.GetUserName()
    else:
        # on Unix the info is extracted from /etc/passwd:
        return pwd.getpwuid(os.getuid()).pw_name


def get_environ(name, throw=False):
    """
    Gets an Environment Variable if exists else None.
    Optionally, raises an EnvironmentError exception if specified.

    :param name:
    :param throw:

    :raise: EnvironmentError

    """
    value = None

    if WIN32:
        key = None
        try:
            key = wr.OpenKey(wr.HKEY_CURRENT_USER, 'Environment')
            value, _dataType = wr.QueryValueEx(key, name)
        except Exception as error:
            print(error)
        finally:
            if key is not None:
                wr.CloseKey(key)
    else:
        with open(BASHRC_FILE, 'r') as bashrc:
            lines = bashrc.readlines()
            for l in lines:
                if l.__contains__(name):
                    try:
                        value = str(l.split('=')[1]).strip()
                    except IndexError:
                        value = None

    if value is None:
        value = os.environ.get(name, None)

    if throw and value is None:
        raise EnvironmentError("Unable to find '%s'" % name)

    return value


def set_environ(**k_values):
    """
    Sets Environment Variables

    :param k_values: The dictionary representing association of {key, value}
        to be set as environment variable(s)
    :type k_values: dict

    """
    def clean_value(val):
        """
        Cleans the passed Value replacing all '/' with os specific separator

        :param val: The Value
        :type val: str

        :return: Cleaned and replaced string
        :rtype: str

        """
        return r'{0}'.format(val).replace('/', os.sep)

    template = r'setx {0} "{1}"' if WIN32 else 'export {0}={1}\n'
    if not WIN32:

        with open(BASHRC_FILE, 'r') as bashrc:
            lines = bashrc.readlines()
            to_remove, new_lines = [], lines[:]
            index = 0
            for l in lines:
                if [k for k in k_values.iterkeys() if l.__contains__(k)]:
                    new_lines.pop(index)
                else:
                    index += 1

        for k, v in k_values.iteritems():
            value = clean_value(v)
            export = template.format(k, value)
            new_lines.append(export)

            command = QCommand(export)
            command.execute(env=os.environ)

            os.environ[k] = value

        with open(BASHRC_FILE, 'w+') as bashrc:
            bashrc.writelines(new_lines)

        command = QCommand('source {0}'.format(BASHRC_FILE))
        command.execute(env=os.environ)
    else:
        for k, v in k_values.iteritems():
            value = clean_value(v)
            command = QCommand(template.format(k, value))
            command.execute(env=os.environ)
            os.environ[k] = value
