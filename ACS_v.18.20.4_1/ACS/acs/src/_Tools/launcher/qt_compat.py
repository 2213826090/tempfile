#!/usr/bin/env python
# -*- coding: utf-8 -*-

#-------------------------------------------------------------------------------
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
# @summary: "This module makes code compatible with either PyQt or PySide"
# @since: 5/13/14
# @author: nbrissox
#-------------------------------------------------------------------------------
"""
ACS Launcher QT Library compatibility module

 -- pyside = PySide
 -- pyqt = PyQt4

"""

import sys
import os


# Aliases
path = os.path


# Constants

DEBUG = False
DEVEL = False

NAMESPACE = sys.argv[1:]

__DEBUG_TAGS = ('--debug', '--dbg')
__DEVEL_TAGS = ('--devel', '--dev')

if NAMESPACE:
    for option in NAMESPACE:
        tag = option.lower()
        if tag in __DEBUG_TAGS:
            DEBUG = True
        if tag in __DEVEL_TAGS:
            DEVEL = True

# Os part
WIN32 = sys.platform == 'win32'

# Qt part
DEFAULT_QT_LIB = 'PySide'
ENV_QT_API = os.environ.get('QT_API', DEFAULT_QT_LIB)

__DIR__ = path.abspath(path.dirname(__file__))
__LOGICAL_REPO_DIR__ = path.abspath(path.join(__DIR__, '..', '..', '..', '..'))

__ACS_PREFIX__ = 'acs'
if not DEVEL:
    __ACS_PREFIX__ += '_fwk'
__LOGICAL_FWK_DIR__ = FWK_DIR = path.abspath(path.join(__LOGICAL_REPO_DIR__, __ACS_PREFIX__, 'src'))

HELPERS_DIR = path.join(__DIR__, 'helpers')

# Not used, keep it for future needs, just in case
DATABASE = {
    'sqlite': 'sqlite:///{0}'.format(path.join(__DIR__, 'static/acs.db'))
}

if not __DIR__ in sys.path:
    sys.path.append(__DIR__)

if not __LOGICAL_FWK_DIR__ in sys.path:
    sys.path.append(__LOGICAL_FWK_DIR__)

if '--pyside' in sys.argv:
    SELECTED_QT_LIB = 'PySide'
elif '--pyqt4' in sys.argv:
    SELECTED_QT_LIB = 'PyQt4'
elif ENV_QT_API == 'pyside':
    SELECTED_QT_LIB = 'PySide'
elif ENV_QT_API == 'pyqt':
    SELECTED_QT_LIB = 'PyQt4'
else:
    SELECTED_QT_LIB = DEFAULT_QT_LIB

if SELECTED_QT_LIB == 'PySide':

    # This will be passed on to new versions of matplotlib
    os.environ['QT_API'] = 'pyside'

    from PySide import QtGui, QtCore
    from PySide import QtUiTools

    class _UiLoader(QtUiTools.QUiLoader):

        """
        Subclass :class:`~PySide.QtUiTools.QUiLoader` to create the user interface
        in a base instance.

        Unlike :class:`~PySide.QtUiTools.QUiLoader` itself this class does not
        create a new instance of the top-level widget, but creates the user
        interface in an existing instance of the top-level class.

        This mimics the behaviour of :func:`PyQt4.uic.loadUi`.
        """

        def __init__(self, baseinstance):
            """
            Create a loader for the given ``baseinstance``.

            The user interface is created in ``baseinstance``, which must be an
            instance of the top-level class in the user interface to load, or a
            subclass thereof.

            ``parent`` is the parent object of this loader.
            """
            QtUiTools.QUiLoader.__init__(self, baseinstance)
            self.baseinstance = baseinstance

        def createWidget(self, class_name, parent=None, name=''):
            """
            Creates a Qt Widget properly

            .. note:: workaround for PySide.

            :param class_name: The Qt Classname widget
            :type class_name: str

            :param parent: The Parent (cf: Qt Documentation)
            :type parent: QWidget

            :param name: Matching :meth:`QtCore.QObject.objectName`() method
            :type name: str

            :return: QWidget

            """
            widget = self.baseinstance
            if parent is None and self.baseinstance:
                # supposed to create the top-level widget,
                # return the base instance instead
                pass
            else:
                # create a new widget for child widgets
                widget = QtUiTools.QUiLoader.createWidget(self, class_name, parent, name)
                if self.baseinstance:
                    # set an attribute for the new child widget on the base
                    # instance, just like PyQt4.uic.loadUi does.
                    setattr(self.baseinstance, name, widget)

            return widget

    def uic_loader(ui_file, instance=None):
        """
        UIC Loader function.

        :param ui_file: The UI file path
        :type ui_file: str

        :param instance: A QtGui instance

        :return: The initialized QtGui instance according ui file description

        """
        loader = _UiLoader(instance)
        widget = loader.load(ui_file)
        # noinspection PyArgumentList
        QtCore.QMetaObject.connectSlotsByName(widget)
        return widget

elif SELECTED_QT_LIB == 'PyQt4':
    import sip
    API2_CLASSES = 'QData', 'QDateTime', 'QString', 'QTextStream', 'QTime', 'QUrl', 'QVariant',

    for cl in API2_CLASSES:
        sip.setapi(cl, 2)

    from PyQt4 import QtGui, QtCore
    QtCore.Signal = QtCore.pyqtSignal
    QtCore.QString = str
    os.environ['QT_API'] = 'pyqt'

    def uic_loader(ui_file, instance=None):
        """
        UIC Loader function.

        :param ui_file: The UI file path
        :type ui_file: str

        :param instance: A QtGui instance

        :return: The initialized QtGui instance according ui file description

        """
        from PyQt4 import uic
        return uic.loadUi(ui_file, instance)
else:
    raise ImportError("Python Variant not specified")


def _error(class_name, message):
    """
    Provides an easy way to raise Exception

    :param class_name: The Class Name's Exception to be created
    :type class_name: str

    :param message: The Exception message
    :type message: str

    :raise: A sub class instance of Exception with given parameters

    """
    fullname = '{0}'.format(class_name)
    raise type(fullname, (Exception,), {})(message)


class UiLoader(object):

    """
    UI Helper

    Loads ui files into Qt instances.

    """

    UI_FOLDER = path.abspath(path.join(__DIR__, 'static/ui'))

    @classmethod
    def load(cls, ui_filename, instance=None):
        """
        Wrapped UIC Loader function.

        :param ui_filename: The UI file path
        :type ui_filename: str

        :param instance: A QtGui instance

        :return: The initialized QtGui instance according ui file description

        """
        filename = str(ui_filename)

        if not filename.endswith('.ui'):
            filename += '.ui'

        if not path.isfile(filename):
            filename = path.abspath(path.join(cls.UI_FOLDER, filename))

        if not path.isfile(filename):
            _error(class_name='BadFilename', message='the filename `{0}` is not a file!'.format(filename))

        return uic_loader(filename, instance)


class Css(object):

    """
    Handles Css for all Windows

    """

    STATIC_FOLDER = path.abspath(path.join(__DIR__, 'static'))
    CSS_FOLDER = path.join(STATIC_FOLDER, 'css')

    @classmethod
    def content(cls, css_name):
        """
        Loads the Css file and return it as a String

        :param css_name:
        :type css_name: str

        :return: The Css content as a String
        :rtype: str

        """
        css_filename = str(css_name)
        if not css_filename.endswith('.css'):
            css_filename += '.css'

        if not path.isfile(css_filename):
            css_filename = path.abspath(path.join(cls.CSS_FOLDER, css_filename))

        if not path.isfile(css_filename):
            _error(class_name='BadFilename', message='the filename `{0}` is not a file!'.format(css_filename))

        with open(css_filename, 'rb') as css:
            content = css.read()
            if not WIN32:
                # Some Unix UI rendering effects bugs workarounds
                content += '''
                #ACSLauncher QGroupBox {
                    border: 1px solid white;
                    border-radius: .5em;
                    padding: 1em;
                }
                '''

            return content

    @classmethod
    def image(cls, name, ext='.png'):
        """
        Builds a Css file path and return it as a String, taking as basedir .static/images/

        :param name: Image name
        :type name: str

        :param ext: The file extension (default is '.png')
        :type ext: str

        :return: Absolute logical image path or whatever is contained into basedir
        :rtype: str

        """
        image = str(name)
        if not image.endswith(ext):
            image += ext
        return path.abspath(path.join(cls.CSS_FOLDER, '../images', image))


# Aliases
Qt = QtCore.Qt

__all__ = ['QtGui', 'QtCore', 'SELECTED_QT_LIB', 'UiLoader', 'Css', 'Qt']
