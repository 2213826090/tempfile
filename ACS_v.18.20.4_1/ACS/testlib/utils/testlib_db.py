from testlib.base import base_utils

import MySQLdb
import time
import re

param_type_ids ={
    "number": "1",
    "string": "2",
    "dict": "3",
    "array": "4",
    "boolean": "5",
    "view": "6",
    "kwargs": "7",
}

def get_step_types(step):
    st_types = []
    step_type = base_utils.parse_string(string = step,
                                        left_separator = "'step'")
    step_type = step_type.strip(":").strip().strip("}").strip("'")
    if step_type not in types:
        step_types = step_type.split(",")
        for st in step_types:
            if st.strip() not in types:
                query = "INSERT INTO `testlib`.`types` (`id`,`name`) VALUES (NULL,'" + st.strip() + "')"
                cursor.execute(query)
                types.append(st.strip())
            st_types.append(st.strip())
    return st_types


def get_category(step):
    category = base_utils.parse_string(string = step,
                                       left_separator = "'category'",
                                       right_separator = "'class'")
    category = category.strip(":").strip().strip(",").strip("'")
    py_path = base_utils.parse_string(string = step,
                                      left_separator = "'python_path'",
                                      right_separator = "'step'")
    py_path = py_path.strip(":").strip().strip(",").strip("'")

    if category not in categories:
        query = "INSERT INTO `testlib`.`categories` (`id`,`name`,`python_path`) VALUES (NULL,'" + category + "','" + py_path + "')"
        cursor.execute(query)
        categories.append(category)
    return category


def get_params(step):
    params = base_utils.parse_string(string = step,
                                     left_separator = "'params'",
                                     right_separator = "'python_path'")
    params = params.strip(":").strip().strip(",").strip("'")
    parameters = []
    for param in params.split(","):
        if "=" in param:
            name, value = param.split("=")
            name = name.strip()
            value = value.strip().strip("\"")
            if value == "None":
                param_type = None
            elif value.isdigit():
                param_type = "number"
            elif value == "True" or value == "False":
                param_type = "boolean"
            elif re.match("^{.*}$", value) != None:
                param_type = "dict"
            elif re.match("^\[.*\]$", value) != None:
                param_type = "array"
            else:
                param_type = "string"
        else:
            name = param.strip()
            value = None
            param_type = None
        if param_type == None:
            if name == "self" or name == "**kwargs":
                continue
            elif "view_to_check" in name or name == "view_to_find" or name == "confirm_view":
                param_type = "view"
            elif "grep" in name:
                param_type = "string"
            elif "command" in name or "cmd" in name:
                param_type = "string"
            elif "serial" in name:
                param_type = "string"
            elif "path" in name:
                param_type = "string"
            elif "host" in name or "pass" in name or "user" in name:
                param_type = "string"
            elif "name" in name:
                param_type = "string"
            elif "url" in name:
                param_type = "string"
            elif "state" in name:
                param_type = "string"
            elif "folder" in name or "local" == name or "remote" == name or "destination" in name or name == "inode":
                param_type = "string"
            elif "language" in name or "text" in name:
                param_type = "string"
            elif name == "platform":
                param_type = "string"
            elif name == "ip" or name == "dev" or name == "dut" or name == "dev_to_find":
                param_type = "string"
            elif name == "db" or name == "table":
                param_type = "string"
            elif name == "out_file":
                param_type = "string"
            elif "timeout" in name or "_time" in name:
                param_type = "number"
            elif name == "x" or name == "y" or name == "sx" or name == "sy" or name == "ex" or name == "ey":
                param_type = "number"
            elif name == "tab_no":
                param_type = "number"
            elif name == "position":
                param_type = "number"
            elif "columns" in name or "value" in name or "developer_options" in name or "_list" in name:
                param_type = "array"
            else:
                param_type = "string"
        parameter = {}
        parameter["name"] = name
        parameter["value"] = value
        parameter["type"] = param_type
        parameters.append(parameter)
    return parameters


def insert_params(params):
    id_params = []
    for param in params:
        if param["value"] == None:
            value = None
        else:
            value = param["value"]
        select_count = single_entry_from_db("parameters", "count(*)",
                                            ["name", "id_parameter_type", "value", "description"],
                                            [param["name"], param_type_ids[param["type"]], value, None])

        if int(select_count) == 0:
            query = "INSERT INTO `testlib`.`parameters` (`id`,`name`,`id_parameter_type`,`value`,`description`) "
            query += "VALUES (NULL,'" + param["name"] + "'," + param_type_ids[param["type"]] + ","
            if value != None:
                if str(value).isdigit():
                    query += value + ",NULL)"
                else:
                    query += "'" + value + "',NULL)"
            elif value == "":
                 query += "'',NULL)"
            else:
                query += "NULL,NULL)"
            cursor.execute(query)
        id_param = single_entry_from_db("parameters", "id",
                                        ["name", "id_parameter_type", "value", "description"],
                                        [param["name"], param_type_ids[param["type"]], value, None])
        id_params.append(int(id_param))
    return id_params


def get_documentation(step):
    if "'params'" in step:
        doc = base_utils.parse_string(string = step,
                                      left_separator = "'doc'",
                                      right_separator = "'params'")
    else:
        doc = base_utils.parse_string(string = step,
                                      left_separator = "'doc'",
                                      right_separator = "'python_path'")
    doc = doc.strip(":").strip().strip(",").strip("'")
    two_spaces = True
    while two_spaces:
        if "  " in doc:
            doc = doc.replace("  ", " ")
        else:
            two_spaces = False
    description = base_utils.parse_string(string = doc,
                                          left_separator = "description:",
                                          right_separator = "usage:")
    description = description.replace("\\n", "").strip()
    if len(description) < 1:
        description = "Unavailable"
    usage = base_utils.parse_string(string = doc,
                                    left_separator = "usage:",
                                    right_separator = "tags:")
    usage = usage.replace("\\n", "").strip()
    if len(usage) < 1:
        usage = "Unavailable"
    tags = base_utils.parse_string(string = doc,
                                   left_separator = "tags:")
    tags = tags.replace("\\n", "").strip()
    if len(tags) < 1:
        tags = "Unavailable"
    return description, usage, tags


def get_name(step):
    name = base_utils.parse_string(string = step,
                                   left_separator = "'class'",
                                   right_separator = "'doc'")
    name = name.strip(":").strip().strip(",").strip("'")
    return name


def get_tags(tag_names):
    global tags
    step_tags = []
    for tag in tag_names.split(","):
        tag = tag.strip().strip("\"").strip()
        if tag not in tags:
            query = "INSERT INTO `testlib`.`tags` (`id`,`name`) VALUES (NULL,'" + tag + "')"
            cursor.execute(query)
            tags.append(tag)
        step_tags.append(tag)
    return step_tags


def get_fields_from_db(table, field):
    query = "SELECT  `" + field + "` from `testlib`.`" + table + "`"
    cursor.execute(query)
    return [x[field] for x in cursor.fetchall()]


def single_entry_from_db(table, select, columns, values, count = 1):
    query = "SELECT " + select + " FROM `testlib`.`" + table + "` WHERE "
    i = 0
    for col in columns:
        if values[i] == None:
            query += "`" + col + "` is NULL and "
        else:
            #default_parameters 	7 	NULL	This paramter set is a default to each step
            if str(values[i]).isdigit():
                query += "`" + col + "` = " + conn.escape_string(str(values[i])) + " and "
            else:
                query += "`" + col + "` = '" + conn.escape_string(str(values[i])) + "' and "
        i+=1
    query = query.strip(" and ")
    cursor.execute(query)
    res = cursor.fetchall()
    
    if count == 1:
        return res[0][select]
    else:
        return [x[select] for x in cursor.fetchall()]


def insert_step(name, description, usage, id_category):
    select_count = single_entry_from_db("steps", "count(*)", ["name", "description", "usage", "id_category"], [name, description, usage, str(id_category)])
    if int(select_count) == 0:
        query = "INSERT INTO `testlib`.`steps` (`id`,`name`,`description`,`usage`,`id_category`) "
        query += "VALUES (NULL,'" + name + "','" + conn.escape_string(description) + "','" + conn.escape_string(usage) + "'," + str(id_category) + ")"
        cursor.execute(query)
    id_step = single_entry_from_db("steps", "id", ["name", "description", "usage", "id_category"], [name, description, usage, str(id_category)])
    return id_step


def get_id_tags(step_tags):
    id_tags = []
    for tag in step_tags:
        id_tag = single_entry_from_db("tags", "id", ["name"], [tag])
        id_tags.append(int(id_tag))
    return id_tags


def get_id_types(step_types):
    id_types = []
    for step_type in step_types:
        id_type = single_entry_from_db("types", "id", ["name"], [step_type])
        id_types.append(int(id_type))
    return id_types


def insert_step_join_ids(table, id_step, id_):
    id_field = "id_" + table.split("_")[1].strip("s")
    select_count = single_entry_from_db(table, "count(*)", ["id_step", id_field], [id_step, id_])
    if int(select_count) == 0:
        query = "INSERT INTO `testlib`.`" + table + "` (`id_step`,`" + id_field + "`) "
        query += "VALUES (" + str(id_step) + "," + str(id_) + ")"
        cursor.execute(query)


conn = MySQLdb.connect(host= "localhost",
                       user="root",
                       passwd="qwe123",
                       db="testlib")
cursor = conn.cursor(MySQLdb.cursors.DictCursor)

in_file = open('testlib_steps.dict', 'r').read()
steps = in_file.split("}\n{")
categories = get_fields_from_db("categories", "name")
types = get_fields_from_db("types", "name")
tags = get_fields_from_db("tags", "name")
param_types = get_fields_from_db("param_types", "name")

for step in steps:

    step = step.lstrip("\n")

    category = get_category(step)
    name = get_name(step)
    id_category = int(single_entry_from_db("categories", "id", ["name"], [category]))
    description, usage, tag_names = get_documentation(step)
    id_step = insert_step(name, description, usage, id_category)
    
    step_tags = get_tags(tag_names)
    id_tags = get_id_tags(step_tags)
    for id_tag in id_tags:
        insert_step_join_ids("step_tags", id_step, id_tag)

    step_types = get_step_types(step)
    id_types = get_id_types(step_types)
    for id_type in id_types:
        insert_step_join_ids("step_types", id_step, id_type)

    id_params = []
    if "'params'" in step:
        params = get_params(step)
        id_params = insert_params(params)
    insert_step_join_ids("step_parameters", id_step, 1)
    for id_param in id_params:
        insert_step_join_ids("step_parameters", id_step, id_param)
conn.commit()
conn.close()

