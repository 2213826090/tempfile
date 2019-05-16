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
# @summary: ""
# @since: 5/13/14
# @author: nbrissox
#-------------------------------------------------------------------------------
import os
import traceback

from qt_compat import QtCore, QtGui, UiLoader

# Globals #
Qt = QtCore.Qt


class QCommon(object):

    """
    All common behaviors are centralized in a simple Python Class

    Implements UiLoader instance as Class attribute

    """
    uic = UiLoader()

    _open_uri = QtGui.QDesktopServices.openUrl

    def __init__(self, ui_filename):
        self._load_ui(ui_filename)

        # Store Widget initial QSize()
        self.initial_size = None

        # Store Widget initial Maximum Size (maximumSize)
        self.initial_max_size = self.maximumSize()

        # Common Error Dialog Handler
        self.messages_dlg = None

    @classmethod
    def center_widget(cls, widget, move=False):
        """
        Get the Center rectangle, optionally move the widget according the centered rectangle

        :param widget: The widget
        :type widget: QtGui.QWidget

        :param move: If True the widget is moved according new centered rectangle
        :type move: bool

        :return: The Centered rectangle instance
        :rtype: QtCore.QRect

        """
        # noinspection PyArgumentList
        screen_rect = QtGui.QApplication.desktop().screen().rect().center()
        widget_rect = widget.rect().center()
        centered_rect = screen_rect - widget_rect
        if move:
            widget.move(centered_rect)

        return centered_rect

    def center_self(self):
        """
        Center the window according its own rectangle center to the screen rectangle

        """
        # noinspection PyTypeChecker
        self.center_widget(self, move=True)

    # noinspection PyUnresolvedReferences
    def show(self):
        """
        Override.
        Center the Window on the Desktop

        """
        super(QCommon, self).show()

        # self.center_self()

        if not self.initial_size:
            self.initial_size = self.size()

    def open_uri(self, link, scheme='url'):
        """
        Wraps QtGui.QDesktopServices.openUrl + some custom logic

        :param link: The link to open
        :type link: str

        :param scheme: The URI scheme (url, file, mail, etc.)
        :type scheme: str

        :return: Whether or not the process was successful
        :rtype: bool

        """
        status, errors, lf = False, "", "<br />"
        if scheme == 'file':
            if os.path.exists(link):
                link = r'file:///{0}'.format(link)
            else:
                errors = 'Specified directory (or file) does not exists!'

        if not errors:
            status = self._open_uri(QtCore.QUrl(link))

        if not status:
            message = "Exception occurred while trying opening link: {1}[{0}]!".format(link, lf)
            traces = str(traceback.format_exc()).strip()

            if errors:
                message += lf * 2 + errors

            if traces not in (str(None), None):
                message += lf * 2 + traces

            if not self.messages_dlg:
                self.messages_dlg = QDialogClose(parent=self,
                                                 ui_filename='dialog',
                                                 flags=Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint)

            self.messages_dlg.dialog_title.setText('Error Message')
            self.messages_dlg.message.setHtml(message)
            self.messages_dlg.show()

        return status

    def create_movie(self, animation, start=True, speed=None):
        """
        Basically, allow to create a QtGui.QMovie with a given animation file
        like GIF file, ...

        :param animation: The animation file path
        :param animation: str

        :param start: If True, the QtGui.QMovie is started (:meth:`QtGui.QMovie.start` method)
        :param start: bool

        :param speed: Rotation Speed
        :param speed: int

        :return: The movie animation
        :rtype: QtGui.QMovie

        """
        movie = QtGui.QMovie(animation, QtCore.QByteArray(), self)
        movie.setCacheMode(QtGui.QMovie.CacheAll)
        if speed is not None:
            movie.setSpeed(speed)
        if start:
            movie.start()
        return movie

    def trUtf8(self, p_str, str_disambiguation=None, int_n=-1):
        """
        Override.

        :param p_str:
        :param str_disambiguation:
        :param int_n:


        .. note::

            A prefix of 'b' or 'B' is ignored in Python 2
            it indicates that the literal should become a bytes literal in Python 3 (e.g. when code is automatically
            converted with 2to3).
            A 'u' or 'b' prefix may be followed by an 'r' prefix.

            ..seealso:: https://docs.python.org/2/reference/lexical_analysis.html#string-literals,
                http://stackoverflow.com/questions/6269765/what-does-the-b-character-do-in-front-of-a-string-literal

        """
        # noinspection PyUnresolvedReferences
        super(QCommon, self).trUtf8(b'{0}'.format(p_str), str_disambiguation=str_disambiguation, int_n=int_n)

    def _load_ui(self, ui_filename):
        """
        A common method to allow to load its Graphical (UI) elements dynamically from .ui file (xml)

        :param ui_filename: The UI file path (QtDesigner)
        :type ui_filename: str

        """
        if ui_filename:
            self.uic.load(ui_filename, self)


## Windows ##


class QDialog(QCommon, QtGui.QDialog):

    """
    Acs derived QtGui.QDialog Class.

    """

    def __init__(self, parent=None, flags=None, ui_filename=""):
        QtGui.QDialog.__init__(self, parent, flags if flags is not None else Qt.Dialog)
        QCommon.__init__(self, ui_filename)


class QDialogClose(QDialog):

    """
    Error Dialog Class
    """

    def __init__(self, parent=None, flags=None, ui_filename="", button_name="btn_close"):
        QDialog.__init__(self, parent=parent, flags=flags, ui_filename=ui_filename)
        btn = getattr(self, button_name, None)
        if btn:
            btn.clicked.connect(self.close)


class QWidget(QCommon, QtGui.QWidget):

    """
    Acs derived QtGui.QDialog Class.

    """

    def __init__(self, parent=None, flags=None, ui_filename=""):
        QtGui.QWidget.__init__(self, parent, flags if flags is not None else Qt.Widget)
        QCommon.__init__(self, ui_filename)


class QMainWindow(QCommon, QtGui.QMainWindow):

    """
    Acs derived QtGui.QMainWindow Class.

    """

    def __init__(self, parent=None, flags=None, ui_filename=""):
        QtGui.QMainWindow.__init__(self, parent, flags if flags is not None else Qt.Window)
        QCommon.__init__(self, ui_filename)
