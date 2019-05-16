"""
:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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

:organization: INTEL QCTV
:summary: database accessor GUI.
:since: 18/08/2014
:author: jduran4x
"""

import sys
from PySide.QtCore import *
from PySide.QtGui import *
import os

# this part is for running the GUI in standalone
import re
r = re.search("(.*)acs_test_scripts", __file__)
sys.path.append(os.path.join(r.group(1), "acs_test_scripts"))
sys.path.append(os.path.join(r.group(1), "acs", "src"))
# end standalone part

import CommandsDatabaseUtilities as Factory
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Database.CommandsDatabase import *


class MainWindow(QDialog):
    """
    this is the main window of the DB ACCESSOR
    """
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setWindowTitle("DB ACCESSOR")

        # Create widgets
        self.editbox = QLineEdit(":memory:")
        self.filter = QLineEdit("")
        self.btn_browse = QPushButton("Browse")
        self.btn_browse.clicked.connect(self.browse)
        self.btn_load = QPushButton("Load")
        self.btn_load.clicked.connect(self.load)
        self.tables = QListWidget()
        self.tables.clicked.connect(self.display_content)
        self.content = QRightClickListWidget(self)
        # Create layout and add widgets
        layout = QVBoxLayout()
        self.setLayout(layout)

        db_layout = QHBoxLayout()
        db_layout.addWidget(self.editbox)
        db_layout.addWidget(self.btn_browse)
        db_layout.addWidget(self.btn_load)
        layout.addLayout(db_layout)

        db_layout = QHBoxLayout()
        db_layout.addWidget(self.tables)
        db_layout.addWidget(self.content)
        layout.addLayout(db_layout)

        self._table_name = ""

        self.err_logs = LogWindow(self)
        # database object
        self.db = None

    def get_database(self):
        """
        returns the database
        :return: database connection object
        :rtype: CommandsDatabase
        """
        return self.db

    def load(self):
        """
        actions performed when button "Load" is clicked
        """
        if "Un" in self.btn_load.text():
            self.tables.clear()
            self.content.clear()
            self.db.close()
            self.btn_load.setText("Load")
            self.btn_browse.setText("Browse")
            self.btn_browse.clicked.disconnect(self.import_csv)
            self.btn_browse.clicked.connect(self.browse)

        else:
            print "loading database %s" % self.editbox.text()
            self.db = CommandsDatabase(self.editbox.text())
            if self.editbox.text() == ":memory:" or not os.path.exists(self.editbox.text()):
                self.db.create_database()
            self.db.connect()
            print "Database Loaded"
            self.btn_load.setText("UnLoad")
            self.btn_browse.setText("Import csv")
            self.btn_browse.clicked.disconnect(self.browse)
            self.btn_browse.clicked.connect(self.import_csv)

            # call functions to display tables
            self.tables.clear()
            for t in Factory.DISPLAYED_TABLES:
                item = QListWidgetItem(self.tables)
                item.setText(t)

    def display_content(self):
        """
        actions performed when a table name is selected in the
        tables list widget
        """
        self.content.clear()
        self._table_name = self.tables.currentItem().text()
        self.content.set_table(self._table_name)
        self.content.filter_enabled(Factory.get_filter_status(self._table_name))
        col = self.db.get_name_column(self._table_name)
        content = self.db.get_content(self._table_name)
        names = [c[col] for c in content]
        self.content.fill(names)

    def browse(self):
        """
        actions performed when the button 'Browse' is clicked
        """
        f = QFileDialog(self).getOpenFileName()
        self.editbox.setText(f[0])

    def import_csv(self):
        """
        actions performed when the button 'Import csv' is clicked
        """
        f = QFileDialog(self).getOpenFileName(filter="*.csv")
        import csv

        with open(f[0], "r") as csv_file:
            csv_reader = csv.reader(csv_file)
            titles = csv_reader.next()
            # translate the columns to be database columns
            titles[titles.index("type")] = "cmdType"
            titles[titles.index("vendor")] = "vendorId"
            titles[titles.index("feature")] = "featId"
            titles[titles.index("description")] = "desc"
            for row in csv_reader:
                row_dict = dict(zip(titles, row))
                # get? and set? are mixed in the askable db column
                get_val = 1 if row_dict["get?"] == "Yes" else 0
                set_val = 2 if row_dict["set?"] == "Yes" else 0
                del row_dict["get?"]
                del row_dict["set?"]
                row_dict["askable"] = get_val + set_val
                row_dict["vendorId"] = self._translate_vendor_to_id(row_dict["vendorId"])
                row_dict["featId"] = self._translate_feature_to_id(row_dict["featId"])
                # options are part of a separate table. remove from dict dedicated to commands
                # and treat it separately
                options = row_dict["options"]
                del row_dict["options"]
                try:
                    self.db.insert("commands", row_dict)
                except Exception as e:
                    self.err_logs.error("'%s': %s" % (row[titles.index("name")], e))
                item = self.db.get_item("commands", row_dict["name"])
                self._insert_options(item["cmdId"], options)

        msg = QMessageBox()
        if self.err_logs.has_log():
            msg.setText("CSV import ended with errors")
            msg.exec_()
            self.err_logs.show()
        else:
            msg.setText("CSV import ended successfully")
            msg.exec_()

    def _translate_vendor_to_id(self, name):
        """
        interface function to translate the vendor name into its Id
        in vendors table
        :type name: str
        :param name: the vendor name
        :rtype: int
        :return: vendor Id
        """
        return self.__translate_to_id(name, "vendors")

    def _translate_feature_to_id(self, name):
        """
        interface function to translate the feature name into its Id
        in features table
        :type name: str
        :param name: the feature name
        :rtype: int
        :return: feature Id
        """
        return self.__translate_to_id(name, "features")

    def _insert_options(self, parent_id, opts):
        result = []
        options = opts.split("|")
        for o in options:
            item = self.db.get_item("options", o)
            if item is None:
                item = {"name": o}
            self.db.insert("options", item, [parent_id])
        return result

    def __translate_to_id(self, name, table):
        """
        translates 'name' into its Id
        in 'table'
        :type name: str
        :param name: the feature name
        :type table: str
        :param table: table name
        :rtype: int
        :return: Id
        """
        item = self.db.get_item(table, name)
        if item is not None:
            table_id = self.db.TABLE_ID_NAME_MAPPING[table][0]
            return item[table_id]
        else:
            self.err_logs.warning("'%s' doesn't exist in table '%s'" % (name, table))
            return None


class ItemEditor(QDialog):
    """
    this is the window used for editing a database entry
    """
    def __init__(self, title, parent):
        super(ItemEditor, self).__init__(parent)
        self.setWindowTitle(title)
        self._parameters = {}
        self._table = ""
        self._db = parent.get_database()
        if type(parent).__name__ == "ItemEditor":
            self.item_ids = parent.item_ids
        else:
            self.item_ids = []

    def get_database(self):
        """
        returns the database
        :return: database connection object
        :rtype: CommandsDatabase
        """
        return self._db

    def dynamic_build(self, table, item=None):
        """
        dynamically builds the editing window according to
        selected table and selected item.
        :type table: str
        :param table: table name
        :type item: str
        :param item: the item name
        """
        self._table = table
        layout = QVBoxLayout()
        self.setLayout(layout)
        self._add_item(layout, table, item)
        self._add_buttons(layout)

    def _add_item(self, layout, table, item):
        """
        adds all widgets for displaying the database entry
        corresponding to table and item
        :type layout: QLayout
        :param layout: the parent layout
        :type table: str
        :param table: table name
        :type item: str
        :param item: the item name
        """
        glayout = QGridLayout()
        layout.addLayout(glayout)
        item_id, attributes = Factory.get_displayed_attributes(self._db, table, item)
        self.item_ids.append(item_id)
        for ind, attr in enumerate(attributes):
            self._add_label_on_grid(glayout, ind, 0, attr["name"])
            self._add_value_on_grid(glayout, ind, 1, attr)

    def _add_buttons(self, layout):
        """
        adds Submit and Cancel buttons
        :type layout: QLayout
        :param layout: the parent layout
        """
        hlayout = QHBoxLayout()
        layout.addLayout(hlayout)
        self._commit = QPushButton("Submit")
        self._commit.clicked.connect(self.commit)
        self._abort = QPushButton("Cancel")
        self._abort.clicked.connect(self.cancel)
        hlayout.addWidget(self._commit)
        hlayout.addWidget(self._abort)

    @staticmethod
    def _add_label_on_grid(layout, row, column, label):
        """
        adds the label part of the item element display on a grid
        :type layout: QGridLayout
        :param layout: the parent layout
        :type row: int
        :param row: the row index
        :type column: int
        :param column: the column index
        :type label: str
        :param label: the label text to display
        """
        label = QLabel("<b>%s</b>" % label)
        layout.addWidget(label, row, column)

    def _add_value_on_grid(self, layout, row, column, value):
        """
        adds the value part of the item element display on a grid
        :type layout: QGridLayout
        :param layout: the parent layout
        :type row: int
        :param row: the row index
        :type column: int
        :param column: the column index
        :type value: dict
        :param value: dict containing all values to display
        """
        obj = None
        if value["type"] == "shorttext":
            obj = self.create_line_edit(value["text"])
        elif value["type"] == "list":
            obj = self.create_list(value["table"], value["values"])
        elif value["type"] == "combo":
            obj = self.create_combo_box(value["selected"], value["values"])
        elif value["type"] == "longtext":
            obj = self.create_text_edit(value["text"])
        elif value["type"] == "choice":
            hlayout = QHBoxLayout()
            for n in value["values"].keys():
                obj = self.create_check_box(n, value["values"][n])
                self._parameters[value["name"] + n] = obj
                hlayout.addWidget(obj)
            obj = hlayout
        self._parameters[value["name"]] = obj
        if "ayout" not in type(obj).__name__:
            layout.addWidget(obj, row, column)
        else:
            layout.addLayout(obj, row, column)

    @staticmethod
    def create_line_edit(line):
        """
        creates a QLineEdit object
        :type line: str
        :param line: the text to display
        :rtype: QLineEdit
        :return: the QLineEdit object
        """
        return QLineEdit(line)

    @staticmethod
    def create_text_edit(text):
        """
        creates a QTextEdit object
        :type text: str
        :param text: the text to display
        :rtype: QTextEdit
        :return: the QTextEdit object
        """
        text = QTextEdit(text)
        text.setFixedHeight(text.sizeHint().height() / 3)
        return text

    def create_list(self, table, values):
        """
        creates a list object
        :param table: table name
        :type table: str
        :param values: list of values to display
        :type values: list
        :return: the QListWidget object (or sub object)
        :rtype: QListWidget
        """
        my_list = QRightClickListWidget(self, table)
        for opt in values:
            item = QListWidgetItem(my_list)
            item.setText(str(opt))
        # create at least an empty item at the end of the list
        QListWidgetItem(my_list)
        my_list.filter_enabled(False)
        # my_list.set_add_action(self.parent().edit)
        # my_list.set_edit_action(self.parent().edit)
        # my_list.set_delete_action(self.parent().delete)
        return my_list

    @staticmethod
    def create_combo_box(index, values):
        """
        creates a combo box
        :param index: index of the selected value
        :type index: int
        :param values: list of displayed values
        :type values: list
        :return: the combo box
        :rtype: QComboBox
        """
        box = QComboBox()
        for item in values:
            box.addItem(item)
        box.setCurrentIndex(int(index))
        return box

    @staticmethod
    def create_check_box(text, value):
        """
        creates a check box
        :param text: label of the check box
        :type text: str
        :param value: state of the check box
        :type value: bool
        :return: the check box
        :rtype: QCheckBox
        """
        box = QCheckBox(text)
        box.setChecked(value)
        return box

    def commit(self):
        """
        actions performed when "Submit" button is clicked
        """
        print "commit"
        title = self.windowTitle()
        values = {}
        for p in self._parameters:
            obj = self._parameters[p]
            obj_type = type(obj).__name__
            if obj_type in ("QLineEdit",):
                values[p] = obj.text()
            elif obj_type in ("QTextEdit",):
                values[p] = obj.toPlainText()
            elif obj_type in ("QCheckBox",):
                if obj.checkState() == Qt.CheckState.Checked:
                    values[p] = 1
                else:
                    values[p] = 0
            elif obj_type in ("QComboBox",):
                values[p] = obj.currentIndex()
        values = Factory.translate_values(self._table, values)
        if title.lower() == "edit":
            self._db.update(self._table, self.item_ids[-1], values)
        if title.lower() == "add":
            self._db.insert(self._table, values, self.item_ids[:-1])
        self.done(0)

    def cancel(self):
        """
        actions performed when 'Cancel' button is clicked
        """
        self.done(0)


class QRightClickListWidget(QListWidget):
    """
    QListWidget for which right clicking displays a menu
    """
    def __init__(self, parent, table=""):
        QListWidget.__init__(self)
        self.clicked_mouse_btn = None
        self.itemClicked.connect(self.show_menu)
        self._table_name = ""
        self._filter = False
        self._table = table
        self._parent = parent
        self._menu = self._build_menu()

    def mousePressEvent(self, event):
        """
        mouse event action
        :param event: event
        """
        self.clicked_mouse_btn = event.button()
        QListWidget.mousePressEvent(self, event)

    def _build_menu(self):
        """
        dynamically builds the menu to display
        """
        my_menu = QMenu(self)
        self.a = QAction("add", my_menu)
        self.e = QAction("edit", my_menu)
        self.d = QAction("delete", my_menu)
        self.f = QAction("add to filter", my_menu)
        my_menu.addAction(self.a)
        my_menu.addAction(self.e)
        my_menu.addAction(self.d)
        my_menu.addAction(self.f)
        self.a.triggered.connect(self._edit)
        self.e.triggered.connect(self._edit)
        self.d.triggered.connect(self._delete)
        return my_menu

    def _edit(self):
        """
        actions performed when the 'edit' menu item is selected
        """
        title = self.sender().text()
        p = self.sender().parent().parent()
        table = p.get_table()
        f = ItemEditor(title, self._parent)
        item = p.currentItem()
        if title.lower() == "edit":
            item = item.text()
        if title.lower() == "add":
            item = None
        f.dynamic_build(table, item)
        f.show()

    def _delete(self):
        """
        actions performed when the 'delete' menu item is selected
        """
        self._parent.get_database().delete(self._table, self.currentItem().text())

    def fill(self, content):
        """
        fills a QListWidget, adding an empty line at the end
        :param content: list of items to display
        :type content: list
        """
        for c in content:
            item = QListWidgetItem(self)
            item.setText(str(c))
        # add an extra item
        QListWidgetItem(self)

    def set_table(self, table):
        """
        setter
        :param table: table name
        :type table: str
        """
        self._table = table

    def get_table(self):
        """
        getter
        :return: table name
        :rtype: str
        """
        return self._table

    def filter_enabled(self, value):
        """
        sets if the filter is enabled or disabled
        :param value: True for enabled, else False
        :type value: bool
        """
        self._filter = value

    def show_menu(self):
        """
        actions performed when a QListWidget item is right clicked
        """
        if self.clicked_mouse_btn == Qt.RightButton:
            if self.currentItem().text():
                if self._filter:
                    self.f.setVisible(True)
                else:
                    self.f.setVisible(False)
                self.a.setVisible(False)
                self.e.setVisible(True)
                self.d.setVisible(True)
            else:
                self.a.setVisible(True)
                self.e.setVisible(False)
                self.d.setVisible(False)
                self.f.setVisible(False)
            self._menu.activateWindow()
            self._menu.popup(QCursor.pos())


class LogWindow(QDialog):
    """
    window displaying a log
    Logs are displayed in a table which columns are
    1- severity level (INFO, WARNING, ERROR)
    2- text of the error
    """
    def __init__(self, parent=None):
        super(LogWindow, self).__init__(parent)
        self.setWindowTitle("Log")
        self.resize(980, 200)
        self._list = QTableWidget()

        self._list.setColumnCount(2)
        self._list.setColumnWidth(1, 800)
        self._list.horizontalHeader().setVisible(False)
        self._info_brush = QBrush(QColor(Qt.darkGreen))
        self._warn_brush = QBrush(QColor(Qt.blue))
        self._err_brush = QBrush(QColor(Qt.red))
        self.__row = 0
        self.setLayout(QVBoxLayout())
        self.layout().addWidget(self._list)

    def has_log(self):
        """
        allows to know if errors have been reported
        :return: True if errors, else False
        :rtype: bool
        """
        return self._list.rowCount() > 0

    def info(self, msg):
        """
        interface function allowing to insert a new log with INFO severity level
        :param msg: the log message
        :type msg: str
        """
        self.__add_log("INFO", msg, self._info_brush)

    def warning(self, msg):
        """
        interface function allowing to insert a new log with WARNING severity level
        :param msg: the log message
        :type msg: str
        """
        self.__add_log("WARNING", msg, self._warn_brush)

    def error(self, msg):
        """
        interface function allowing to insert a new log with ERROR severity level
        :param msg: the log message
        :type msg: str
        """
        self.__add_log("ERROR", msg, self._err_brush)

    def __add_log(self, cat, msg, brush):
        """
        allows to insert a new log with custom severity level
        :param cat: severity level
        :type cat: str
        :param msg: the log message
        :type msg: str
        :param brush: the formatting of the message
        :type brush: QBrush
        """
        self._list.insertRow(self.__row)

        item = QTableWidgetItem(cat)
        item.setForeground(brush)
        item.setText(cat)
        item.setTextAlignment(Qt.AlignRight)
        self._list.setItem(self.__row, 0, item)
        item = QTableWidgetItem(msg)
        item.setForeground(brush)
        item.setText(msg)
        item.setTextAlignment(Qt.AlignLeft)
        self._list.setItem(self.__row, 1, item)
        self.__row += 1



if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setApplicationName("DB_ACCESSOR")
    # Create and show the form
    form = MainWindow()
    form.show()
    # Run the main Qt loop
    sys.exit(app.exec_())
