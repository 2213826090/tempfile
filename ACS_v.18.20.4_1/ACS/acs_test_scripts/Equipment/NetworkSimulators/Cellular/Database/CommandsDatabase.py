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
:summary: Class for manipulating the SQLite database specific for
          Network Simulators GPIB commands listing
:since: 18/08/2014
:author: jduran4x
"""
from acs_test_scripts.Utilities.DatabaseInterfaces.sqlite import SQLite
from ErrorHandling.TestEquipmentException import TestEquipmentException

CREATION_STRING = """
-- table for equipment vendors
CREATE TABLE vendors(vendorId INTEGER PRIMARY KEY autoincrement not null,
 name TEXT unique,
 desc TEXT default "");

-- table for features (2G, 3G, and so on)
CREATE TABLE features(featId INTEGER PRIMARY KEY autoincrement not null,
 name TEXT unique,
 desc TEXT default "");

-- table for equipments
CREATE TABLE equipments(eqtId INTEGER PRIMARY KEY autoincrement not null,
 name TEXT unique,
 vendorId INTEGER,
 desc TEXT default "");

CREATE TABLE options(optId INTEGER PRIMARY KEY autoincrement not null,
 name TEXT unique,
 desc TEXT default "");

-- map between equipments and features (1 equipment for several features)
CREATE TABLE equipments_features_map(
 eqtId INTEGER,
 featId INTEGER,
 PRIMARY KEY (eqtId, featId),
 foreign key(eqtId) REFERENCES equipments(eqtId) ON DELETE CASCADE,
 foreign key(featId) REFERENCES features(featId) ON DELETE CASCADE);

-- map between commands and options
CREATE TABLE commands_options_map(
 cmdId INTEGER,
 optId INTEGER,
 PRIMARY KEY (cmdId, optId),
 foreign key(cmdId) REFERENCES commands(cmdId) ON DELETE CASCADE,
 foreign key(optId) REFERENCES options(optId) ON DELETE CASCADE);

create table commands(cmdId INTEGER PRIMARY KEY autoincrement not null,
                       name TEXT unique,
                       command TEXT,
                       cmdType TEXT, -- for now "gpib"
                       vendorId INTEGER REFERENCES vendors(vendorId),
                       featId INTEGER REFERENCES features(featId),
                       askable INTEGER default 0, --0=No, 1=Yes. can the ? be appended
                       desc TEXT default "");
"""


class CommandsDatabase(SQLite):
    TABLE_ID_NAME_MAPPING = {"features": ["featId", "name"],
                            "vendors": ["vendorId", "name"],
                            "equipments": ["eqtId", "name"],
                            "options": ["optId", "name"],
                            "commands": ["cmdId", "name"],
                            "commands_options_map": ["cmdId", "optId"]}

    TABLE_REF_MAPPING = {"commands": {"featId": "features", "eqtId": "equipments", "vendorId": "vendors"},
                         "equipments": {"vendorId": "vendors"}}

    def __init__(self, _file=""):
        SQLite.__init__(self, _file)
        self.__command = None
        self.__command_type = None
        self.__command_name = None

    def connect(self):
        """
        allows to connect to the database
        """
        SQLite.connect(self)
        self.send_request("PRAGMA foreign_keys = ON")

    def create_database(self):
        """
        allow to create the commands database.
        use for first creation
        """
        self.connect()
        self._cursor.executescript(CREATION_STRING)
        self.save()
        if self._db_file != ":memory:":
            self.close()

    def verify_parameters(self, name, option=None, value=None):
        """
        raises an error if one of the input parameter is not valid
        :type name: str
        :param name: the command's name
        :type option: str
        :param option: (optional) an option to the command. defaults to None
        :type value: str
        :param value: (optional) the value to be set by the command (option). defaults to None
        :raises: TestEquipmentException.COMMAND_LINE_ERROR
        """
        is_valid = self.check_command(name)
        if not is_valid:
            raise TestEquipmentException(TestEquipmentException.COMMAND_LINE_ERROR, "Unrecognized command")
        if option is not None:
            is_valid = self.check_option(option)
            if not is_valid:
                raise TestEquipmentException(TestEquipmentException.COMMAND_LINE_ERROR,
                                             "Option not applicable to this command")
        if value is not None:
            is_valid = self.check_value(value)
            if not is_valid:
                raise TestEquipmentException(TestEquipmentException.COMMAND_LINE_ERROR,
                                             "Value not applicable to this command")

    def get_content(self, table_name):
        """
        get all the data recorded in the table_name table

        :type table_name: str
        :param table_name: the table name from which the data is retrieved
        :rtype: list
        :return: the list of all records of the requested table. 1 record is a dict
        """
        values = []
        qry = "select * from %s" % table_name
        content = self.send_request(qry)
        columns = [c[0] for c in self._cursor.description]
        for c in content:
            values.append(dict(zip(columns, c)))
        return values

    def get_id_column(self, table):
        """
        returns the name of the Id column for table in input
        :type table: str
        :param table: the table name
        :rtype: str
        :return: the Id column name or None if table is not a valid one
        """
        if table in self.TABLE_ID_NAME_MAPPING.keys():
            return self.TABLE_ID_NAME_MAPPING[table][0]
        else:
            return None

    def get_name_column(self, table):
        """
        returns the name of the Name column for table in input
        :type table: str
        :param table: the table name
        :rtype: str
        :return: the Name column name or None if table is not a valid one
        """
        if table in self.TABLE_ID_NAME_MAPPING.keys():
            return self.TABLE_ID_NAME_MAPPING[table][1]
        else:
            return None

    def get_ref_table(self, table, item_id):
        """
        returns for a given table the mapping between reference id columns and
        the refered to tables.
        :type table: str
        :param table: the table name
        :rtype: str
        :return: the refered to table name or None if table is not a valid one
        """
        if table in self.TABLE_REF_MAPPING.keys():
            return self.TABLE_REF_MAPPING[table][item_id]
        else:
            return None

    def get_item(self, table, item):
        """
        get an item from the table using the item name

        :type table: str
        :param table: the table name
        :type item: str
        :param item: the name to search in the table. item corresponds to the value associated to
        the column returned by get_name_column
        :rtype: dict
        :return: all attributes associated to the item. Or None if required item has not been found in database
        """
        name = self.TABLE_ID_NAME_MAPPING[table][1]
        return self._get_item_by(table, item, name)

    def get_item_by_id(self, table, item):
        """
        get an item from the table using the item Id

        :type table: str
        :param table: the table name
        :type item: str
        :param item: the Id to search in the table. item corresponds to the value associated to
        the column returned by get_id_column
        :rtype: dict
        :return: all attributes associated to the item. Or None if required item has not been found in database
        """
        name = self.TABLE_ID_NAME_MAPPING[table][0]
        return self._get_item_by(table, item, name)

    def _get_item_by(self, table, item, col):
        """
        get an item from the table. item is a value of column col.

        :type table: str
        :param table: the table name
        :type item: str
        :param item: the Id to search in the table. item corresponds to the value associated to
        the column returned by get_id_column
        :type col: str
        :param col: the table's column to search item value in
        :rtype: dict
        :return: all attributes associated to the item. Or None if required item has not been found in database
        """
        result = self.send_request("select * from %s where %s=?" % (table, col), item)
        columns = [c[0] for c in self._cursor.description]
        if result:
            return dict(zip(columns, result[0]))
        else:
            return None

    def check_command(self, command):
        """
        check the requested command exists.
        :type command: str
        :param command: the command 'human' name
        :rtype: bool
        :return: True if the command has been found in database, else False
        """
        ans = False
        result = self.send_request("select command,cmdType from commands where name=?", command)
        if len(result) == 1:
            # there should be only 1 answer
            # just retain the command and command type for future use
            # this avoids to give the command name again
            self.__command = result[0]["command"]
            self.__command_type = result[0]["cmdType"]
            self.__command_name = command
            if self.__command.endswith("?"):
                self.__command = self.__command[:-1]
            ans = True

        return ans

    def get_command(self):
        """
        allows to retreive the command to be executed.
        the command must be checked first (use check_command)
        :rtype: str
        :return: the command to be executed.
        :raises: TestEquipmentException.COMMAND_LINE_ERROR
        """
        if self.__command is not None:
            return self.__command
        else:
            raise TestEquipmentException(TestEquipmentException.COMMAND_LINE_ERROR,
                                         "Command validity must be checked first")

    def get_command_type(self):
        """
        allows to retreive the command type of the command to be executed.
        the command must be checked first (use check_command)
        :rtype: str
        :return: the command type of the command to be executed.
        :raises: TestEquipmentException.COMMAND_LINE_ERROR
        """
        if self.__command_type is not None:
            return self.__command_type
        else:
            raise TestEquipmentException(TestEquipmentException.COMMAND_LINE_ERROR,
                                         "Command validity must be checked first")

    def check_option(self, opt):
        """
        check the requested option exists for the retained command
        command is retained when it is checked (use check_command)
        :type opt: str
        :param opt: the option
        :rtype: bool
        :return: True if the option exists for retained command, else False
        """
        ans = False
        qry = "select options.name from commands,options,commands_options_map " \
            + "where commands.cmdId=commands_options_map.cmdId " \
            + "and options.optId=commands_options_map.optId " \
            + "and commands.name=?"
        results = self.send_request(qry, self.__command_name)
        for result in results:
            if opt == result["name"]:
                ans = True
                break
        return ans

    def check_value(self, value):
        """
        check the requested value is in the applicable range for the given command/option
        command is retained when it is checked (use check_command)
        TODO: this is a provision.
        :type value: str
        :param value: the value
        :rtype: bool
        :return: True if the value is valid, else False
        """
        return True

    def update(self, table, item_id, item):
        """
        allows to update an entry in the database
        :type table: str
        :param table: the table name in which the entry should be updated
        :type item_id: int
        :param item_id: Id of the item to be updated
        :type item: dict
        :param item: mapping between table's column names and values to be updated
        the table's column names can be retrieved using get_item or get_item_by_id.
        The Id column (refer to get_id_column) must be removed from these results.
        """
        qry = "update %s set " % table
        attr = ["%s=?" % r for r in item.keys()]
        qry += ",".join(attr)
        qry += " where %s=?" % self.TABLE_ID_NAME_MAPPING[table][0]
        self.send_request(qry, item.values() + [item_id])
        self.save()

    def insert(self, table, item, ref_item_ids=0):
        """
        allows to insert a new entry in the database.
        This means inserting a new entry in table 'table' and inserting entries in
        the corresponding mapping table(s).
        For example, when inserting a new option, the mapping between options and
        commands must be updated. Note that the command Id need to be known
        :type table: str
        :param table: the table name in which the entry should be inserted
        :type item: dict
        :param item: mapping between table's column names and values to be inserted
        the table's column names can be retrieved using get_item or get_item_by_id.
        The Id column (refer to get_id_column) must be removed from these results.
        :type ref_item_ids: list
        :param ref_item_ids: optional. List of ids of the referencing items.
        """
        qry = "insert into %s (%s) values (%s)" % \
              (table, ",".join(item.keys()), ",".join(["?"] * len(item.values())))
        try:
            self.send_request(qry, item.values())
        except Exception as e:
            print e
        # check if the tables can be accessed only via other tables
        # thus, mapping have to be updated too
        if table == "options":
            # need to update the mapping
            # 1st get the created item id
            item = self.get_item("options", item[self.TABLE_ID_NAME_MAPPING["options"][1]])
            ref_item_ids.append(item[self.TABLE_ID_NAME_MAPPING["options"][0]])
            qry = "insert into commands_options_map (cmdId,optId) values (?,?)"
            self.send_request(qry, ref_item_ids)
        self.save()

    def delete(self, table, item):
        """
        deletes an entry in the table 'table' using the entry's name
        :type table: str
        :param table: the table name
        :type item: str
        :param item: the name of the item to delete
        """
        qry = "delete from %s where %s=?" % (table, self.TABLE_ID_NAME_MAPPING[table][1])
        self.send_request(qry, item)
        self.save()


