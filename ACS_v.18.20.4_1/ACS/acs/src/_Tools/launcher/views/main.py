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
# @summary: ""
# @since: 5/14/14
# @author: nbrissox
# -------------------------------------------------------------------------------
import os
import sys
import re
import helpers

from helpers import workers
from qt_compat import (QtCore, QtGui, Qt,
                       Css, FWK_DIR, DEBUG, WIN32)
from views import common


# Aliases
path = os.path


class ACSCompleter(QtGui.QCompleter):

    """
    Custom Campaigns & Benches paths completion rules

    """

    def __init__(self, parent=None,
                 mode=QtGui.QCompleter.PopupCompletion,
                 case=Qt.CaseInsensitive,
                 role=Qt.DisplayRole,
                 sorting=QtGui.QCompleter.CaseInsensitivelySortedModel):

        super(ACSCompleter, self).__init__(parent)

        self.source_model = None
        self.setCaseSensitivity(case)
        self.setCompletionMode(mode)
        self.setCompletionRole(role)
        self.setModelSorting(sorting)

        self.popup().setStyleSheet("background: #017dc5; "
                                   "color: white; "
                                   "padding: 10px; "
                                   "font: bold 12pt; "
                                   "min-height: 50px;")

    def setModel(self, model):
        """

        :param model: The QModel instance.
        :type model: QtGui.QModel

        """
        self.source_model = model
        super(ACSCompleter, self).setModel(self.source_model)

    def update(self, completion_prefix):
        """
        Updates the QCompleter Model instance.

        :param completion_prefix:
        :type completion_prefix:

        """
        class InnerProxyModel(QtGui.QSortFilterProxyModel):

            """
            Inner Proxy Model
            """

            def filterAcceptsRow(self, src_row, src_parent):
                """
                Override

                :param src_row:
                :param src_parent:

                :return: Whether or not the item match search algorithm
                :rtype: bool

                """
                index0 = self.sourceModel().index(src_row, 0, src_parent)
                data = self.sourceModel().data(index0).lower()
                completion = completion_prefix.lower()

                is_valid = not data.startswith(os.sep)
                if is_valid:
                    multiple_hints = completion.split()
                    if multiple_hints:
                        is_valid = len([h for h in multiple_hints if h in data]) == len(multiple_hints)
                    else:
                        is_valid = completion in data
                return is_valid

        proxy_model = InnerProxyModel(self)
        proxy_model.setSourceModel(self.source_model)
        super(ACSCompleter, self).setModel(proxy_model)

    def splitPath(self, completion_prefix):
        """

        :param completion_prefix: The Completer path
        :type completion_prefix: str

        :return: Empty string
        :rtype: str

        """
        self.update(completion_prefix)
        return ""


class ACSLauncher(common.QWidget):

    """
    ACS Main Launcher Window

    """
    EXEC_DEFAULT_LOCATION = path.abspath(path.join(path.expanduser("~"),
                                                   "Intel",
                                                   "ACS",
                                                   "acs_fwk",
                                                   "src",
                                                   "_ExecutionConfig"))
    # Errors
    ValidationError = type('ValidationError', (Exception,), dict())
    ValidationError.types = []

    SEARCHBOXES_METAINFO = {
        'devices': {'button': 'btn_devices_search', 'combo': 'cb_quickstart_devices'},
        'campaigns': {'button': 'btn_campaigns_search', 'combo': 'cb_quickstart_campaign'},
        'benches': {'button': 'btn_benches_search', 'combo': 'cb_quickstart_benches'}
    }

    @classmethod
    def validate(cls, params):
        """
        Validates extra parameters syntax according known ACS options

        :param params: The extra parameters to be validated
        :raise: ValidationError

        """

        known_params = ("-f", "--flash_file", "--ff",
                        "-n", "--run_nb", "--nb",
                        "-r", "--random_mode",
                        "-s", "--device_id", "--sr",
                        "--rf", "--report_folder",
                        "-o", "--override_device_parameter", "--op",
                        "-u", "--user",
                        "--metacampaign_uuid", "--uuid",
                        "--no_report", "--nr",)
        for par in params:
            param = str(par).strip()
            if param.startswith('-'):
                if param not in known_params:
                    return 0, ('Extra Parameter: ``{0}`` is unknown!\n'
                               'Allowed parameters are:\n'
                               '* {1}'.format(param, '\n* '.join(known_params)))
        return 1, ""

    def keyPressEvent(self, event):
        """
        Override

        Add some KeyCombos functionality:

         + search logs on CTRL + F
         + Then while in a search mode, F3 goes to the next found occurrence is any.

        :param event: The Event to handle
        :type event: QtCore.QKeyEvent

        """
        super(ACSLauncher, self).keyPressEvent(event)
        key = event.key()
        if key == Qt.Key_F:
            if event.modifiers() == Qt.ControlModifier:
                # CTRL + F pressed
                self.show_search_logs()
        if key == Qt.Key_F3 and self._searching_logs:
            # F3 pressed while searching logs
            self.highlight_logs_search(self.search_logs.text(), QtGui.QTextDocument.FindWholeWords)

    def closeEvent(self, event):
        """
        Overrides the CloseEvent handler to end properly threads, ...

        :param event: The Event associated with this Signal
        :type event: QtCore.QCloseEvent

        .. todo:: Could Change method hide() to something more sexy, like a small panel with a task bar
            showing the user the ending's progression of all background tasks.

        """
        self.hide()
        # Have we background task(s) still running?
        if self._loaders:
            # Ignoring Current Event
            event.ignore()

            # Wrapping loaders jobs into Timeout pattern thanks to QTimer Class
            timer = QtCore.QTimer(self)

            # Calling the clean_exit receiver on QTimer's timeout signal.

            # noinspection PyUnresolvedReferences
            timer.timeout.connect(lambda: self._loaders.clean())

            # Setting WorkerList cleaning flag to True
            self._loaders.cleaning = True
            # Starting the timer
            timer.start(self._loaders.timeout)
        else:
            # Calling the parent :meth:`closeEvent` method (super)
            super(ACSLauncher, self).closeEvent(event)

    def __init__(self, parent=None, flags=None, ui_filename=None):
        super(ACSLauncher, self).__init__(parent, flags, ui_filename)

        self.file_dialog = None
        self.dw_logging_size_memo = None
        # self.bottom_spacer = None

        self._user_data_folder = None
        self._update_movie = None

        self._update_movie_started = False
        self._is_acs_running = False
        self._initial = True
        self._searching_logs = False
        self._report_info = []

        self._loaders = workers.WorkerList()

        self.setup_ui_components()
        self.connect_slots()

    @property
    def user_data_folder(self):
        """
        Property{Getter}

        Ensures the first time to evaluate whether or not the Environment Variable
        ``ACS_EXECUTION_CONFIG_PATH`` is set with a valid path, if not the default location
        is to be used (~/Intel/ACS/executable/_ExecutionConfig) and set as Environment Variable.

        :return: The User Data Folder absolute path.
        :rtype: str

        """
        if not self._user_data_folder:
            env = self._user_data_folder = helpers.get_environ(helpers.ACS_ENVIRONMENT_NAME)
            if not env:
                helpers.set_environ(**{helpers.ACS_ENVIRONMENT_NAME: self.EXEC_DEFAULT_LOCATION})
                self._user_data_folder = helpers.get_environ(helpers.ACS_ENVIRONMENT_NAME)
        return self._user_data_folder

    @property
    def short_user_data_folder(self):
        """

        :return:
        """
        mx_chars = 100
        chars = len(self._user_data_folder)
        return ('... {0}'.format(self._user_data_folder[chars - mx_chars:])
                if chars > mx_chars
                else self._user_data_folder)

    @property
    def formatted_report_info(self):
        """
        Formats the report info into nice HTML output

        :return: Html output
        :rtype: str

        """
        if not self._report_info:
            return "<strong>No Report</strong>"

        file_path_regex = r'^[\w?\s]+?(C:\\[\w?\\[_\.-]+)$' if WIN32 else r'^[\w?\s?:?\t]+?((?:/[^/\n]+)*?)$'

        report = '<strong>Your report folder(s) location (Just click to open it)</strong>'
        report += '<ul>'
        for info in self._report_info:
            report_line = info.split('FWK')[-1].split('\t')[-1].strip()
            matches = re.match(file_path_regex, report_line, re.IGNORECASE)
            if matches:
                report_path = matches.group(1)
                report += '<li>'
                report += r'<a href="file:///{0}">{0}</a>'.format(path.abspath(report_path))
                report += '</li>'
        report += '</ul>'

        return report

    @property
    def arguments(self):
        """
        Property{Getter}

        :return: A formatted command line arguments string
        :rtype: str

        """
        root = self.user_data_folder
        campaign_text = self.cb_quickstart_campaign.currentText()
        section = self.cb_quickstart_campaign.itemData(self.cb_quickstart_campaign.currentIndex())

        errors = ""
        if not section:
            # noinspection PyUnresolvedReferences
            self.ValidationError.types += ['campaign', ]
            errors += 'You must specify a file not a folder path!'

        campaign = os.path.abspath('{0}/{1}/{2}'.format(root, section, campaign_text))
        device = self.cb_quickstart_devices.currentText()

        arguments = '-c "{0}" -d {1}'.format(campaign, device)
        if self.advanced_panel_trigger.isChecked():
            bench_text = self.cb_quickstart_benches.currentText()
            section = self.cb_quickstart_benches.itemData(self.cb_quickstart_benches.currentIndex())

            if section:
                bench = os.path.abspath('{0}/{1}/{2}'.format(root, section, bench_text))
                arguments += ' -b "{0}"'.format(bench)

            extra = str(self.le_quickstart_extra_params.text())
            if extra:
                valid, e = self.validate(extra.split(' '))
                if not valid:
                    # noinspection PyUnresolvedReferences
                    self.ValidationError.types += ['extra', ]
                    errors += e
                arguments += ' {0}'.format(extra)
        if errors:
            raise self.ValidationError(errors)
        return '{0}'.format(arguments)

    def setup_ui_components(self):
        """
        Setups/Initialize all UI components, not handle with QtDesigner

        """
        self.file_dialog = QtGui.QFileDialog(self)
        self.file_dialog.setFileMode(QtGui.QFileDialog.Directory)
        self.file_dialog.setOption(QtGui.QFileDialog.ShowDirsOnly, 1)

        # User data folder selection QLabel
        self.exec_config_link.setToolTip(self.user_data_folder)
        self.exec_config_link.setText(self.short_user_data_folder)

        # Hiding Logs Panel by default.
        self.dw_logging.hide()
        for lb in (self.devices_loader,
                   self.campaigns_loader,
                   self.benches_loader):
            lb.setMovie(self.create_movie('static/images/loader.gif'))

        # Creating an Updater loader
        self._update_movie = self.create_movie('static/images/links/loader.gif', start=False)

        # Hidden for the moment
        self.btn_update.hide()

        # Hiding advanced panel by default
        self.advanced_panel.hide()

        # Hiding search logs bar
        self.hide_search_logs()

        self.exec_config_link.setFlat(True)
        self.exec_config_link.setIcon(QtGui.QIcon(Css.image('icons/transparent20x20')))

        links_data = ((self.btn_pb, 'Help'),)

        icon_size = QtCore.QSize(64, 64)

        for link_data in links_data:
            link, text = link_data
            ico_name = str(link.objectName()).replace('btn_', '').lower()
            ico_path = Css.image('links/{0}'.format(ico_name))
            if not path.exists(ico_path):
                ico_path = Css.image('links/default')

            link.setIcon(QtGui.QIcon(ico_path))
            link.setIconSize(icon_size)
            link.setSizePolicy(QtGui.QSizePolicy.MinimumExpanding,
                               QtGui.QSizePolicy.MinimumExpanding)

            link.setStyleSheet('max-width: 150px; padding: 0; margin: 0;')
            link.setText(text)

        self.layout().setSpacing(0)
        self.setMaximumSize(self.layout().sizeHint())

        self.create_devices_worker()
        self.create_campaigns_and_benches_worker()

        self.adjustSize()
        self.setWindowFlags(self.windowFlags() ^ Qt.WindowMaximizeButtonHint)

    def connect_slots(self):
        """
        Centralise Signals/Slots Logic

        """

        def anchor_clicked(link):
            html = self.te_logs.toHtml()
            self.open_uri(link)
            self.te_logs.setHtml(html)

        self.te_logs.anchorClicked.connect(anchor_clicked)

        self.btn_campaigns_search.clicked.connect(lambda: self.open_search_mode('campaigns'))
        self.btn_benches_search.clicked.connect(lambda: self.open_search_mode('benches'))
        self.btn_devices_search.clicked.connect(lambda: self.open_search_mode('devices'))

        self.cb_quickstart_campaign.currentIndexChanged.connect(lambda index: self.close_search_mode(index,
                                                                                                     'campaigns'))
        self.cb_quickstart_benches.currentIndexChanged.connect(lambda index: self.close_search_mode(index, 'benches'))
        self.cb_quickstart_devices.currentIndexChanged.connect(lambda index: self.close_search_mode(index, 'devices'))
        self.btn_pb.clicked.connect(lambda: self.open_uri('https://soco.intel.com/groups/automation-control-system'))

        self.exec_config_link.clicked.connect(lambda: self.open_uri(self.user_data_folder, scheme='file'))
        self.btn_settings.clicked.connect(self.select_user_data_folder)

        self.btn_update.clicked.connect(self.search_for_update)
        # Connect onto the QLabel QMovie frameChanged signal to animate the Update QPushButton icon
        self._update_movie.frameChanged.connect(self.update_movie_loader)

        self.advanced_panel_trigger.clicked.connect(self.enable_advanced)
        self.btn_toggle_logs.clicked.connect(self.toggle_logs)

        self.btn_quickstart_launch.clicked.connect(self.run_acs)
        self._loaders.qt_mediator.exit.connect(self.clean_exit)

        self.btn_valid.clicked.connect(self.hide_search_logs)
        self.search_logs.textChanged.connect(self.highlight_logs_search)

    def show_search_logs(self):
        """
        Displays the Search Logs QLineEdit

        """
        if self.dw_logging.isVisible():
            self.search_logs.show()
            self.btn_valid.show()
            self._searching_logs = True
            self.search_logs.setFocus()

    def hide_search_logs(self):
        """
        Hides the Search Logs QLineEdit

        """
        self.search_logs.setText("")
        self.search_logs.hide()
        self.btn_valid.hide()
        self._searching_logs = False

    def highlight_logs_search(self, text, direction=None):
        """
        Highlights Searched words in the Document.

        :param text: The searched text
        :type text: str

        """
        if direction:
            self.te_logs.find(text, direction)
        else:
            self.te_logs.moveCursor(QtGui.QTextCursor.Start)
            self.te_logs.find(text)

    def open_search_mode(self, sender):
        """
        Opens the Search mode for ComboBoxes

        :param sender: The Sender widget

        """
        current_searchbox = getattr(self, self.SEARCHBOXES_METAINFO[sender].get('combo', ""), None)
        if not current_searchbox:
            return

        button = getattr(self, self.SEARCHBOXES_METAINFO[sender].get('button', ""), None)
        if isinstance(button, QtGui.QPushButton):
            button.setEnabled(False)

        current_searchbox.setEditable(True)
        line_edit = current_searchbox.lineEdit()
        line_edit.returnPressed.connect(lambda: self.close_search_mode(current_searchbox.currentText(), sender))
        line_edit.setText('')
        line_edit.setPlaceholderText('Type your search here ...')

        current_searchbox.setInsertPolicy(QtGui.QComboBox.NoInsert)

        completer = ACSCompleter(parent=current_searchbox)
        completer.setModel(current_searchbox.model())

        current_searchbox.setCompleter(completer)
        # noinspection PyUnresolvedReferences
        completer.activated.connect(lambda text: self.close_search_mode(text, sender))

        line_edit.setFocus()

    def close_search_mode(self, index_or_text, sender=""):
        """
        Closes the Search mode for ComboBoxes

        :param index_or_text: The text or index according the caller
        :param sender: The sender widget

        """
        current_searchbox = getattr(self, self.SEARCHBOXES_METAINFO[sender].get('combo', ""), None)
        if not current_searchbox:
            return

        is_int = isinstance(index_or_text, int)
        if DEBUG:
            dbg = '{0} Selection changed to index: {1}' if is_int else '{0} Selection changed to: {1}'
            print(dbg.format(sender.upper(), index_or_text))
        current_searchbox.setEditable(False)

        btn_name = self.SEARCHBOXES_METAINFO[sender].get('button', "")
        button = getattr(self, btn_name, None)
        if isinstance(button, QtGui.QPushButton):
            enabled = True
            if btn_name == 'btn_benches_search':
                enabled = self.advanced_panel_trigger.isChecked()
            button.setEnabled(enabled)

        if not is_int:
            index = current_searchbox.findText(index_or_text)
            if index != -1:  # -1 for not found
                current_searchbox.setCurrentIndex(index)

    def create_acs_worker(self, command, start=True):
        """
        Wraps all Threading steps

        :param command: The Command Line to execute
        :type command: str

        :param start: If set to True the Worker is started
        :type start: bool

        """
        worker = workers.AcsWorker(self, command=command)

        worker.starting.connect(self.acs_worker_starting, Qt.QueuedConnection)
        worker.polling.connect(self.acs_worker_polling, Qt.QueuedConnection)
        worker.ending.connect(self.acs_worker_ending, Qt.QueuedConnection)

        if start:
            worker.start()

        self._loaders.append(worker)

        return worker

    def create_campaigns_and_benches_worker(self, start=True):
        """
        Wraps all Threading steps

        :param start: If set to True the Worker is started
        :type start: bool

        """
        worker = workers.CampaignsWorker(self, user_data_folder=self.user_data_folder, initial=self._initial)
        worker.starting.connect(self.campaigns_loader_starting, Qt.QueuedConnection)
        worker.ending.connect(self.campaigns_loader_ending, Qt.QueuedConnection)
        if start:
            worker.start()

        self._loaders.append(worker)

        if self._initial:
            self._initial = False

        return worker

    def create_devices_worker(self, start=True):
        """
        Wraps all Threading steps

        :param start: If set to True the Worker is started
        :type start: bool

        """
        worker = workers.DevicesWorker(self)
        worker.starting.connect(self.devices_loader_starting, Qt.QueuedConnection)
        worker.ending.connect(self.devices_loader_ending, Qt.QueuedConnection)
        if start:
            worker.start()

        self._loaders.append(worker)

        return worker

    def devices_loader_starting(self):
        """
        Threaded Data Loading Start to UI Callback

        """
        self.cb_quickstart_devices.setEnabled(False)

        if self.btn_devices_search.isVisible():
            self.btn_devices_search.hide()

        self.devices_loader.show()
        self.devices_loader.movie().start()

    def devices_loader_ending(self, devices):
        """
        Threaded Data Loading Start to UI Callback

        :param devices: A list of ACS Supported devices
        :type devices: list

        """
        self.cb_quickstart_devices.clear()
        self.cb_quickstart_devices.addItems(devices)
        self.cb_quickstart_devices.setEnabled(True)

        self.devices_loader.hide()
        self.devices_loader.movie().stop()

        if not self.btn_devices_search.isVisible():
            self.btn_devices_search.show()

    def campaigns_loader_starting(self):
        """
        Threaded Data Loading Start to UI Callback

        """
        for cb in (self.cb_quickstart_benches,
                   self.cb_quickstart_campaign):
            cb.setEnabled(False)

        if self.btn_campaigns_search.isVisible():
            self.btn_campaigns_search.hide()

        self.campaigns_loader.show()
        self.campaigns_loader.movie().start()

        if self.btn_benches_search.isVisible():
            self.btn_benches_search.hide()

        self.benches_loader.show()
        self.benches_loader.movie().start()

        self.btn_quickstart_launch.setEnabled(False)

    def campaigns_loader_ending(self, data):
        """
        Threaded Data Loading Start to UI Callback

        :param data: Campaigns & Benches available from User Data Folder (_ExecutionConfig folder)
        :type data: tuple

        """
        campaigns, benches = data

        for cb in (self.cb_quickstart_benches,
                   self.cb_quickstart_campaign,):
            cb.clear()

        folder_ico = QtGui.QIcon('static/images/icons/folder.png')
        file_xml_ico = QtGui.QIcon('static/images/icons/file-xml.png')

        if campaigns:
            for section in sorted(campaigns.iterkeys()):
                self.cb_quickstart_campaign.addItem(folder_ico, section, 0)
                for f in campaigns[section]:
                    self.cb_quickstart_campaign.addItem(file_xml_ico, f, section)
            self.cb_quickstart_campaign.setEnabled(True)

        if benches:
            for section in sorted(benches.iterkeys()):
                self.cb_quickstart_benches.addItem(folder_ico, section, 0)
                for f in benches[section]:
                    self.cb_quickstart_benches.addItem(file_xml_ico, f, section)
            self.cb_quickstart_benches.setEnabled(self.advanced_panel_trigger.isChecked())

        self.campaigns_loader.hide()
        self.campaigns_loader.movie().stop()

        if not self.btn_campaigns_search.isVisible():
            self.btn_campaigns_search.show()

        self.benches_loader.hide()
        self.benches_loader.movie().stop()

        if not self.btn_benches_search.isVisible():
            self.btn_benches_search.show()

        self.btn_quickstart_launch.setEnabled(True)

    def clean_exit(self):
        """
        Calls the Super :meth:`close`

        """
        super(ACSLauncher, self).close()

    # noinspection PyUnusedLocal
    def update_movie_loader(self, frame):
        """

        :param frame: QMovie frame

        """
        self.btn_update.setIcon(QtGui.QIcon(self._update_movie.currentPixmap()))
        self.btn_update.setIconSize(QtCore.QSize(60, 60))

    def search_for_update(self):
        """
        Search for ACS new update, if so trigger installer with --update option.

        """
        if self._update_movie_started:
            self._update_movie.stop()
            self.btn_update.setStyleSheet('image: url("static/images/links/update.png");')
            self.btn_update.setIcon(QtGui.QIcon(""))
            self._update_movie_started = False
        else:
            self._update_movie.start()
            self.btn_update.setStyleSheet('image: none;')
            self._update_movie_started = True

    def run_acs(self):
        """
        Runs ACS.py python script
        """

        if self._is_acs_running:
            acs_runner = [l for l in self._loaders if l.objectName() == "AcsWorker"]
            if acs_runner:
                current = acs_runner.pop(0)
                current.interrupt.emit()
                return

        program = os.path.abspath(os.path.join(FWK_DIR, 'ACS.py'))
        if not os.path.exists(program):
            program = os.path.abspath(os.path.join(FWK_DIR, 'ACS.pyc'))
        try:
            python_exe = sys.executable
            python_exe = python_exe.replace("pythonw", "python") if "pythonw" in python_exe else python_exe
            command = r'{0} "{1}" {2}'.format(python_exe, program, self.arguments)
            self.lb_campaign.setStyleSheet('color: white;')
            if self.advanced_panel_trigger.isChecked():
                self.lb_cmdline.setStyleSheet('color: white;')
        except self.ValidationError as e:
            if 'campaign' in e.types:
                self.lb_campaign.setStyleSheet('color: red;')
                e.types.remove('campaign')
            else:
                self.lb_campaign.setStyleSheet('color: white;')

            if 'extra' in e.types:
                self.lb_cmdline.setStyleSheet('color: red;')
                e.types.remove('extra')
            else:
                self.lb_cmdline.setStyleSheet('color: white;')
            return

        self.te_logs.setHtml('<strong><u>Executing Command</u></strong>:<br />')
        self.te_logs.append('<p><strong style="color: #8AA0CC;">{0}'
                            '</strong></p>'.format(self.format_command_line(command)))

        self.btn_toggle_logs.setStyleSheet('background-image: url("static/images/icons/arrow_left.png");')
        self.btn_toggle_logs.setEnabled(True)

        self.create_acs_worker(command)

    def acs_worker_starting(self):
        """
        This callback is called each time ACS Worker starts its work

        """
        self._report_info = []
        self._is_acs_running = True
        self.te_logs.append('<br /><strong><u>Output</u></strong>:<br />')
        self.btn_quickstart_launch.setStyleSheet('image: url("static/images/stop_run.png");')
        self.dw_logging.show()
        self.set_size_constraint(fixed=False)
        # self.center_self()

    def acs_worker_polling(self, line):
        """


        :param line: Each QCommand std{out,err} output line
        :type line: str

        """
        tpl_line = '<br /><strong style="color: {1};"><u>{0}</u></strong>'
        highlights = (
            ('ACS OUTCOME: SUCCESS', 'green'),
            ('ACS OUTCOME: FAILURE', 'red'),
            ('WARNING', 'orange'),
            ('ERROR', 'red'),
            ('EXCEPTION', 'lightred'),
            ('DEBUG', '#555555'),
            ('ABORT', '#923E3E'),
            ('_Reports', 'green')
        )

        for highlight in highlights:
            pattern, color = highlight
            if line.__contains__(pattern):
                if pattern == '_Reports':
                    self._report_info.append(line)
                self.te_logs.append(tpl_line.format(line, color))
                break
        else:
            self.te_logs.append('<strong>{0}</strong>'.format(line))

    def acs_worker_ending(self, out):
        """

        :param out:

        """
        self._is_acs_running = False
        self.btn_quickstart_launch.setStyleSheet('image: url("static/images/go_run.png");')
        if self._report_info:
            self.te_logs.append('<br /><hr />{0}<br /><hr />'.format(self.formatted_report_info))

    def toggle_logs(self):
        """
        Ensures Show/Hide Logs button functionality
        """
        visible = self.dw_logging.isVisible()
        if visible:
            self.dw_logging.hide()
            self.btn_toggle_logs.setStyleSheet('background-image: url("static/images/icons/arrow.png");')
            self.set_size_constraint()
            if self.isMaximized():
                self.showNormal()
        else:
            self.dw_logging.show()
            self.btn_toggle_logs.setStyleSheet('background-image: url("static/images/icons/arrow_left.png");')
            self.set_size_constraint(fixed=False)
        self.adjustSize()

    def enable_advanced(self):
        """
        Enables the Advanced Run Parameters Section

        """
        checked = self.advanced_panel_trigger.isChecked()

        self.cb_quickstart_benches.setEnabled(checked)
        self.btn_benches_search.setEnabled(checked)
        self.advanced_panel.show() if checked else self.advanced_panel.hide()

    def select_user_data_folder(self):
        """
        Callback on btn_user_data_selection click event.

        """
        if self.user_data_folder:
            self.file_dialog.setDirectory(self.user_data_folder)

        dlg_code = self.file_dialog.exec_()
        if dlg_code == QtGui.QDialog.Accepted:
            data_folder = '{0}'.format(self.file_dialog.directory().path())
            if self._user_data_folder == data_folder:
                return
            self._user_data_folder = data_folder
            self.exec_config_link.setText(self.short_user_data_folder)
            self.exec_config_link.setToolTip(self._user_data_folder)

            self.create_campaigns_and_benches_worker()

    # Instance Methods

    def format_command_line(self, cmd):
        """

        :param cmd: The final ACS computed Command Line

        :return: A formatted Command Line String for GUI representation
        :rtype: str

        """
        root = str(self.exec_config_link.text()).replace('/', '\\')
        short_cmd = str(cmd).replace(root, '.')
        return short_cmd

    def set_size_constraint(self, fixed=True):
        """
        Set the main windows size fixed or not

        :param fixed:

        """
        self.setMaximumSize(self.initial_size if fixed else self.initial_max_size)
        self.adjustSize()
