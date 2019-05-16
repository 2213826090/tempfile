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
:summary: set of utility functions used by the database accessor GUI.
These functions are specific to the commands database but have no justification
for beeing in the CommandsDatabase class.
:since: 18/08/2014
:author: jduran4x
"""


DISPLAYED_TABLES = ["vendors", "features", "equipments", "commands"]


def get_displayed_attributes(db, table, item):
    """
    interface function to retrieve the attributes to be displayed
    according to the selected table
    :type db: CommandsDatabase.Singleton
    :param db: the database object
    :type table: str
    :param table: the selected table
    :type item: str
    :param item: the name of the item to display
    :rtype: tuple
    :return: id of the item to display, list of dict.
    dict contains key, value mapping specific to atomic
    display objects. (see below _get_XXX functions)
    :raises: Exception
    """
    fct = "_get_" + table + "_attributes"
    try:
        result = eval(fct)
        return result(db, item)
    except:
        raise Exception("unknown table element to display")


def translate_values(table, item):
    """
    interface function to translate values given by GUI
    into database values according to the selected table
    :type table: str
    :param table: the selected table
    :type item: dict
    :param item: the displayed item
    :return: if tables is not found, returns None
    """
    cmd = "_translate_" + table + "_values"
    try:
        result = eval(cmd)
        return result(item)
    except:
        return None


def _get_equipments_attributes(db, item):
    """
    function to retrieve the attributes to be displayed
    for the equipments table
    :type db: CommandsDatabase.Singleton
    :param db: the database object
    :type item: str
    :param item: the name of the item to display
    :rtype: tuple
    :return: id of the item to display, list of dict.
    dict contains key, value mapping specific to atomic
    display objects. (see below _get_XXX functions)
    """
    answer = []
    if item is not None:
        db_item = db.get_item("equipments", item)
    else:
        db_item = {"eqtId": 0, "name": "", "vendorId": 0, "featId": 0, "desc": ""}
    answer.append(_get_shorttext(db_item["name"]))
    answer[-1]["name"] = "Name"
    answer.append(_get_table_ref(db, db_item["vendorId"], "vendors"))
    answer[-1]["name"] = "Vendor"
    # answer.append(_get_table_ref(db, db_item["featId"], "features"))
    # answer[-1]["name"] = "Features"
    answer.append(_get_longtext(db_item["desc"]))
    answer[-1]["name"] = "Description"
    return db_item["eqtId"], answer


def _get_vendors_attributes(db, item):
    """
    function to retrieve the attributes to be displayed
    for the vendors table
    :type db: CommandsDatabase.Singleton
    :param db: the database object
    :type item: str
    :param item: the name of the item to display
    :rtype: tuple
    :return: id of the item to display, list of dict.
    dict contains key, value mapping specific to atomic
    display objects. (see below _get_XXX functions)
    """
    answer = []
    if item is not None:
        db_item = db.get_item("vendors", item)
    else:
        db_item = {"vendorId": 0, "name": "", "desc": ""}
    answer.append(_get_shorttext(db_item["name"]))
    answer[-1]["name"] = "Name"
    answer.append(_get_longtext(db_item["desc"]))
    answer[-1]["name"] = "Description"
    return db_item["vendorId"], answer


def _get_options_attributes(db, item):
    """
    function to retrieve the attributes to be displayed
    for the options table
    :type db: CommandsDatabase.Singleton
    :param db: the database object
    :type item: str
    :param item: the name of the item to display
    :rtype: tuple
    :return: id of the item to display, list of dict.
    dict contains key, value mapping specific to atomic
    display objects. (see below _get_XXX functions)
    """
    answer = []
    if item is not None:
        db_item = db.get_item("options", item)
    else:
        db_item = {"optId": 0, "name": "", "desc": ""}
    answer.append(_get_shorttext(db_item["name"]))
    answer[-1]["name"] = "Name"
    answer.append(_get_longtext(db_item["desc"]))
    answer[-1]["name"] = "Description"
    # options are related to commands. so give the command Id also
    return db_item["optId"], answer


def _get_features_attributes(db, item):
    """
    function to retrieve the attributes to be displayed
    for the features table
    :type db: CommandsDatabase.Singleton
    :param db: the database object
    :type item: str
    :param item: the name of the item to display
    :rtype: tuple
    :return: id of the item to display, list of dict.
    dict contains key, value mapping specific to atomic
    display objects. (see below _get_XXX functions)
    """
    answer = []
    if item is not None:
        db_item = db.get_item("features", item)
    else:
        db_item = {"featId": 0, "name": "", "desc": ""}
    answer.append(_get_shorttext(db_item["name"]))
    answer[-1]["name"] = "Name"
    answer.append(_get_longtext(db_item["desc"]))
    answer[-1]["name"] = "Description"
    return db_item["featId"], answer


def _get_commands_attributes(db, item):
    """
    function to retrieve the attributes to be displayed
    for the commands table
    :type db: CommandsDatabase.Singleton
    :param db: the database object
    :type item: str
    :param item: the name of the item to display
    :rtype: tuple
    :return: id of the item to display, list of dict.
    dict contains key, value mapping specific to atomic
    display objects. (see below _get_XXX functions)
    """
    answer = []
    if item is not None:
        db_item = db.get_item("commands", item)
    else:
        db_item = {"cmdId": 0, "name": "", "command": "", "cmdType": "", "vendorId": 0,
              "featId": 0, "askable": 0, "options": "", "parameters": "No", "desc": ""}
    answer.append(_get_shorttext(db_item["name"]))
    answer[-1]["name"] = "Name"
    answer.append(_get_shorttext(db_item["command"]))
    answer[-1]["name"] = "Command"
    answer.append(_get_shorttext(db_item["cmdType"]))
    answer[-1]["name"] = "Command Type"
    answer.append(_get_table_ref(db, db_item["vendorId"], "vendors"))
    answer[-1]["name"] = "Vendor"
    answer.append(_get_table_ref(db, db_item["featId"], "features"))
    answer[-1]["name"] = "Feature"
    answer.append(_get_choice("askable", db_item["askable"]))
    answer[-1]["name"] = "Access"

    answer.append(_get_options_and_values(db, db_item["cmdId"]))
    answer[-1]["name"] = "Options/Values"

    answer.append(_get_longtext(db_item["desc"]))
    answer[-1]["name"] = "Description"
    return db_item["cmdId"], answer


def _get_shorttext(name):
    """
    returns the dict associate to shorttext
    :type name: str
    :param name: text to display
    :rtype: dict
    :return: keys and values specific to shorttext
    keys are type, text
    """
    if name is None:
        name = "none"
    return {"type": "shorttext", "text": name}


def _get_boolean(name):
    """
    returns the dict associate to boolean
    :type name: int
    :param name: integer value corresponding to True or False
    :rtype: dict
    :return: keys and values specific to boolean
    keys are type, selected, values
    """
    if name is None:
        # no value means "No"
        name = 0
    ident = 1 if name == 0 else 1
    return {"type": "combo", "selected": ident, "values": ["Yes", "No"]}


def _get_longtext(desc):
    """
    returns the dict associate to longtext
    :type desc: str
    :param desc: text to display
    :rtype: dict
    :return: keys and values specific to longtext
    keys are type, text
    """
    if desc is None:
        desc = ""
    return {"type": "longtext", "text": desc}


def _get_table_ref(db, ident, table):
    """
    returns the dict associate to table reference
    :type ident: int
    :param ident: Id of the selected item in table reference
    :rtype: dict
    :return: keys and values specific to table reference
    keys are type, selected, values
    """
    if ident is None:
        ident = 0
    content = db.get_content(table)
    name_col = db.get_name_column(table)
    content = [c[name_col] for c in content]
    content.insert(0, "")
    return {"type": "combo", "selected": ident, "values": content}


def _get_options_and_values(db, ident):
    """
    returns the dict associate to options and values
    :type ident: int
    :param ident: Id of the selected item in options and values
    :rtype: dict
    :return: keys and values specific to options and values
    keys are type, table, values
    """
    qry = "select options.name from options,commands_options_map " + \
        "where options.optId=commands_options_map.optId and commands_options_map.cmdId=?"
    options = db.send_request(qry, ident)
    if False:
        return {"type": "table", "values": ""}
    else:
        return {"type": "list", "table": "options", "values": [c[0] for c in options]}


def _get_choice(name, value):
    """
    returns the dict associate to choice
    :type value: int
    :param value: the choice value
    :rtype: dict
    :return: keys and values specific to table reference
    keys are type, values
    """
    if name == "askable":
        get_opt = value in (1, 3)
        set_opt = value in (2, 3)
        return {"type": "choice", "values": {"get": get_opt, "set": set_opt}}


def _translate_commands_values(values):
    """
    translate values given by GUI into database values
    for commands table
    :type values: dict
    :param values: the displayed item
    :rtype: dict
    :return: key value mapping where keys are the table columns names
    """
    answer = {"name": values["Name"],
              "command": values["Command"],
              "cmdType": values["Command Type"],
              "vendorId": values["Vendor"],
              "featId": values["Feature"],
              "askable": values["Accessget"] * 1 + values["Accessset"] * 2,
              "desc": values["Description"]}
    if answer["featId"] == 0:
        answer["featId"] = None
    if answer["vendorId"] == 0:
        answer["vendorId"] = None
    return answer


def _translate_options_values(values):
    """
    translate values given by GUI into database values
    for options table
    :type values: dict
    :param values: the displayed item
    :rtype: dict
    :return: key value mapping where keys are the table columns names
    """
    return {"name": values["Name"],
            "desc": values["Description"]}


def _translate_features_values(values):
    """
    translate values given by GUI into database values
    for features table
    :type values: dict
    :param values: the displayed item
    :rtype: dict
    :return: key value mapping where keys are the table columns names
    """
    return {"name": values["Name"],
            "desc": values["Description"]}


def _translate_vendors_values(values):
    """
    translate values given by GUI into database values
    for vendors table
    :type values: dict
    :param values: the displayed item
    :rtype: dict
    :return: key value mapping where keys are the table columns names
    """
    return {"name": values["Name"],
            "desc": values["Description"]}


def _translate_equipments_values(values):
    """
    translate values given by GUI into database values
    for equipments table
    :type values: dict
    :param values: the displayed item
    :rtype: dict
    :return: key value mapping where keys are the table columns names
    """
    answer = {"name": values["Name"],
              "vendorId": values["Vendor"],
              "desc": values["Description"]}
    if answer["vendorId"] == 0:
        answer["vendorId"] = None
    return answer


def get_filter_status(table):
    """
    fonction indicating if the table selected entry
    can be used for filtering
    :type table: str
    :param table: table name
    :rtype: bool
    :return: boolean value
    """
    if table == "commands":
        return False
    else:
        return True
