""" @copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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

@organization: INTEL MCG PSI
@summary: <enter_summary_details_here>
@since: 5/27/14
@author: nbrissox
"""

import os
import threading
import helpers

# noinspection PyUnresolvedReferences
from Device.DeviceConfig.DeviceConfigLoader import DeviceConfigLoader

from lxml import etree
from qt_compat import (QtCore, FWK_DIR, DEBUG)


GLOBAL_MUTEX = threading.RLock()
DEFAULT_TIMEOUT = 10 * 1000  # n * 1000 ms (1 s)


class QtMediator(QtCore.QObject):

    """
    Acts as a Qt Object,
    thus bearing all Event-driven State Machine mechanism

    """
    # noinspection PyUnresolvedReferences
    exit = QtCore.Signal()


class WorkerList(list):

    """
    List Sub-Class, which connects self to the loader 'finished' signal,
    to flush it from its list

    """
    timeout = DEFAULT_TIMEOUT
    cleaning = False

    def __init__(self, seq=None):
        super(WorkerList, self).__init__(seq or ())
        self.qt_mediator = QtMediator()

    def clean(self):
        """
        Cleans all Tasks, asking them to terminate if not already so.

        """
        while len(self):
            loader = self.pop(0)
            if DEBUG:
                print('WorkerThread {0} has timed out! Terminating it!'.format(loader.objectName()))
            if loader and loader.isRunning():
                try:
                    # Attempt to `brutally` terminate the job thread
                    loader.terminate()
                finally:
                    if loader:
                        del loader

    def append(self, loader):
        """
        Overrides :meth:`append` to connect each loader on its finished signal if so;
        in order to delete itself properly after it has done its work (:meth:`run` returned 0)

        :param loader: A thread worker
        :type loader: QtCore.QThread

        """
        if hasattr(loader, 'finished'):

            def delete_loader(l):
                """
                Inner Handler

                :param l: Loader

                """
                name = 'Unknown'
                if hasattr(l, 'objectName'):
                    name = l.objectName()
                try:
                    self.remove(l)
                except ValueError:
                    pass
                finally:
                    del l
                    if DEBUG:
                        print('Loader: {0} Removed from Active list!'.format(name))

                if self.cleaning and not len(self):
                    self.qt_mediator.exit.emit()

            loader.finished.connect(lambda: delete_loader(loader))
        super(WorkerList, self).append(loader)


class WorkerThread(QtCore.QThread):

    """
    QtCore.QThread Worker

    """

    # noinspection PyUnresolvedReferences
    starting = QtCore.Signal()

    # noinspection PyUnresolvedReferences
    ending = QtCore.Signal(object)

    def __init__(self, parent, **options):
        super(WorkerThread, self).__init__(parent)
        self.setObjectName(self.__class__.__name__)
        # Pass here any type of parameters
        self._options = options
        self._with_mutex = options.get('with_mutex', True)

    def do_work(self):
        """
        Abstract Method.
        Must be override in sub-classes

        :raise: NotImplementedError

        .. note:: Here, we can NOT use :mod:`abc` to implement abstract methods
            as PySide is only a Wrapper over C++ Classes and thus does not allow it ( different layouts )

        """
        raise NotImplementedError('You must override the `do_work` method!')

    def do_run(self):
        """ Inner Method """
        self.starting.emit()
        try:
            result = self.do_work()
        except Exception as errors:
            result = errors
        self.ending.emit(result)

    def run(self):
        """
        Running method

        :return:

        """
        if self._with_mutex:
            with GLOBAL_MUTEX:
                self.do_run()
        else:
            self.do_run()
        return 0


class AcsWorker(WorkerThread):

    """
    Loads Devices models in a separated thread

    """

    # noinspection PyUnresolvedReferences
    polling = QtCore.Signal(object)

    # noinspection PyUnresolvedReferences
    interrupt = QtCore.Signal()

    def poll_it(self, line):
        """

        :param line:

        """
        self.polling.emit(line)

    def interrupt_it(self):
        """
        Interrupts The Current QCommand execution

        """
        self.command.interrupt()

    def do_work(self):
        """
        Override

        """
        command = helpers.QCommand(self._options.get('command'))
        # Connecting Event's handlers Mechanism
        self.interrupt.connect(self.interrupt_it)
        command.polling.connect(self.poll_it)
        # Setting attribute `command` to self, holding the ref to the QCommand instance
        setattr(self, 'command', command)
        return command.execute(polling=True,
                               cwd=FWK_DIR,
                               env=os.environ.copy())


class DevicesWorker(WorkerThread):

    """
    Loads Devices models in a separated thread

    """

    def do_work(self):
        """
        Override

        """
        return DeviceConfigLoader.retrieve_device_model_list()


class CampaignsWorker(WorkerThread):

    """
    Loads Campaigns found from parsing in a separated thread

    """

    def do_work(self):
        """
        Override


        """
        initial = self._options.get('initial', False)
        user_data_folder = self._options.get('user_data_folder')

        if not initial:
            helpers.set_environ(**{helpers.ACS_ENVIRONMENT_NAME: user_data_folder})

        campaigns, benches = {}, {}
        data = campaigns, benches

        for subdir, dirs, files in os.walk(user_data_folder, followlinks=True):
            for f in files:
                if f.endswith('.xml'):
                    file_path = os.path.join(subdir, f)
                    try:
                        _, basename = os.path.split(file_path)
                        cleaned_name = file_path[len(user_data_folder):]
                        section = cleaned_name[0:-len(basename)]

                        # elem = helpers.XMLWrapper(file_path)

                        tree = etree.parse(file_path)
                        root = tree.getroot()
                        if not len(root):
                            continue

                        if 'Campaign' in root.tag:
                            if section not in campaigns:
                                campaigns[section] = []
                            campaigns[section].append(basename)

                        elif 'BenchConfig' in root.tag:
                            if section not in benches:
                                benches[section] = []
                            benches[section].append(basename)

                    except Exception as exc:
                        if DEBUG:
                            print(exc)
                        continue

        return data
