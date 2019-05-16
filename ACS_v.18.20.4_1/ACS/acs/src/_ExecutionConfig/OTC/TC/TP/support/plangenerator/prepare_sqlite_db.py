# -*- coding:utf-8 -*-

import time
import datetime
import sqlite3
import pymssql
import os
import sys
import argparse
import csv
import httplib
import base64
import logging
import urllib2
import re
import traceback
import xml.etree.ElementTree as ET
from urllib import pathname2url
from ConfigParser import ConfigParser
from xml.etree.ElementTree import *
from xml.dom.minidom import parseString

# reload(sys)
# sys.setdefaultencoding('utf-8')
logging.basicConfig(level=logging.DEBUG)
# logging.basicConfig(level=logging.DEBUG,
#                 format='[line:%(lineno)d]%(message)s',
#                 datefmt='%H:%M:%S',
#                 filename='hpalmrest.log',
#                 filemode='w')

# global var
# Table name
TABLE_NAME = ''
# print sql
SHOW_SQL = False

SQLITE_DB_NAME = 'sqlite3.db'
TABLE_HPALM_TESTS = 'hpalm_tests'
TABLE_JIRA_ISSUES = 'jira_issues'
TABLE_HPALM_REQUIREMENTS = 'hpalm_requirements'
TABLE_HPALM_REQUIREMENTS_COVERAGES = 'hpalm_requirement_coverages'


def get_conn(path):
    conn = sqlite3.connect(path)
    if os.path.exists(path) and os.path.isfile(path):
        print('On Hard Disk:[{}]'.format(path))
        return conn
    else:
        conn = None
        print('In memory:[:memory:]')
        return sqlite3.connect(':memory:')


def get_cursor(conn):
    if conn is not None:
        return conn.cursor()
    else:
        return get_conn('').cursor()


###############################################################
#            create|delete table     START
###############################################################
def drop_table(conn, table):
    #  drop table if it exists
    if table is not None and table != '':
        sql = 'DROP TABLE IF EXISTS ' + table
        if SHOW_SQL:
            print('Execute sql:[{}]'.format(sql))
        cu = get_cursor(conn)
        cu.execute(sql)
        conn.commit()
        print('Drop table [{}] successfully!'.format(table))
        close_all(conn, cu)
    else:
        print('the [{}] is empty or equal None!'.format(table))


def create_table(conn, sql):
    #  create table：student
    if sql is not None and sql != '':
        cu = get_cursor(conn)
        if SHOW_SQL:
            print('Execute sql:[{}]'.format(sql))
        cu.execute(sql)
        conn.commit()
        print('Create table successfully!')
        close_all(conn, cu)
    else:
        print('the [{}] is empty or equal None!'.format(sql))


###############################################################
#            create|delete table     END
###############################################################

def close_all(conn, cu):
    #  cloase connection and cur
    try:
        if cu is not None:
            cu.close()
    finally:
        if cu is not None:
            cu.close()
        conn.close()


###############################################################
#            DB Operation CRUD     START
###############################################################

def save(conn, table, data):
    #  insert records
    column_names = ""

    questions_marks = ""
    if data is not None:
        cu = get_cursor(conn)
        #  get column_names
        for column_name, column_value in data[0].items():
            if column_names != "":
                column_names += ","
            column_names += column_name
            if questions_marks != "":
                questions_marks += ","
            questions_marks += "?"

        for d in data:
            column_values = []
            for column_name, column_value in d.items():
                column_values.append(column_value)

            #  build insert sql
            sql = "INSERT INTO " + table + " (" + column_names + ") VALUES (" + questions_marks + ")"
            #  insert data
            # global DEBUG_FALG
            # if DEBUG_FALG == 1:
            #     print('Execute INSERT sql:[{}]'.format(sql))
            #     DEBUG_FALG = 0

            cu.execute(sql, tuple(column_values))
        conn.commit()
        close_all(conn, cu)
    else:
        print('the data is empty or equal None!')


def fetchall(conn, sql):
    #  query all records
    if sql is not None and sql != '':
        cu = get_cursor(conn)
        if SHOW_SQL:
            print('Execute sql:[{}]'.format(sql))
        cu.execute(sql)
        r = cu.fetchall()
        if len(r) > 0:
            for e in range(len(r)):
                print(r[e])
    else:
        print('the [{}] is empty or equal None!'.format(sql))


def fetchone(conn, sql, data):
    #  query one record
    if sql is not None and sql != '':
        if data is not None:
            # Do this instead
            d = (data,)
            cu = get_cursor(conn)
            if SHOW_SQL:
                print('Execute sql:[{}],params:[{}]'.format(sql, data))
            cu.execute(sql, d)
            r = cu.fetchall()
            if len(r) > 0:
                for e in range(len(r)):
                    print(r[e])
        else:
            print('the [{}] equal None!'.format(data))
    else:
        print('the [{}] is empty or equal None!'.format(sql))


def update(conn, sql, data):
    #  update records
    if sql is not None and sql != '':
        if data is not None:
            cu = get_cursor(conn)
            for d in data:
                if SHOW_SQL:
                    print('Execute sql:[{}],params:[{}]'.format(sql, d))
                cu.execute(sql, d)
                conn.commit()
            close_all(conn, cu)
    else:
        print('the [{}] is empty or equal None!'.format(sql))


def delete(conn, sql, data):
    #  delete records
    if sql is not None and sql != '':
        if data is not None:
            cu = get_cursor(conn)
            for d in data:
                if SHOW_SQL:
                    print('Execute sql:[{}],params:[{}]'.format(sql, d))
                cu.execute(sql, d)
                conn.commit()
            close_all(conn, cu)
    else:
        print('the [{}] is empty or equal None!'.format(sql))


###############################################################
#            db operation CRUD     END
###############################################################


###############################################################
#            test operation     START
###############################################################
def drop_table_test():
    #  delete test table
    print('Drop table...')
    conn = get_conn(SQLITE_DB_NAME)
    drop_table(conn, TABLE_NAME)


def create_table_test():
    #  create test table
    print('Create table...')
    create_table_sql = '''CREATE TABLE `student` (
                          `id` int(11) NOT NULL,
                          `name` varchar(20) NOT NULL,
                          `gender` varchar(4) DEFAULT NULL,
                          `age` int(11) DEFAULT NULL,
                          `address` varchar(200) DEFAULT NULL,
                          `phone` varchar(20) DEFAULT NULL,
                           PRIMARY KEY (`id`)
                        )'''
    conn = get_conn(SQLITE_DB_NAME)
    create_table(conn, create_table_sql)


def save_test():
    #  save test data...
    print('Save data...')
    save_sql = '''INSERT INTO student values (?, ?, ?, ?, ?, ?)'''
    data = [(1, 'Hongten', 'male', 20, 'Guang Zhou', '13423****62'),
            (2, 'Tom', 'male', 22, 'San Francisco', '15423****63'),
            (3, 'Jake', 'female', 18, 'Guang Zhou', '18823****87'),
            (4, 'Cate', 'female', 21, 'Guang Zhou', '14323****32')]
    conn = get_conn(SQLITE_DB_NAME)
    save(conn, save_sql, data)


def fetchall_test():
    #  query all records...
    print('Query all...')
    fetchall_sql = '''SELECT * FROM student'''
    conn = get_conn(SQLITE_DB_NAME)
    fetchall(conn, fetchall_sql)


def fetchone_test():
    #  query one record...
    print('Query one...')
    fetchone_sql = 'SELECT * FROM student WHERE ID = ? '
    data = 1
    conn = get_conn(SQLITE_DB_NAME)
    fetchone(conn, fetchone_sql, data)


def update_test():
    #  update data...
    print('Update data...')
    update_sql = 'UPDATE student SET name = ? WHERE ID = ? '
    data = [('HongtenAA', 1),
            ('HongtenBB', 2),
            ('HongtenCC', 3),
            ('HongtenDD', 4)]
    conn = get_conn(SQLITE_DB_NAME)
    update(conn, update_sql, data)


def delete_test():
    #  delete data...
    print('Delete data...')
    delete_sql = 'DELETE FROM student WHERE NAME = ? AND ID = ? '
    data = [('HongtenAA', 1),
            ('HongtenCC', 3)]
    conn = get_conn(SQLITE_DB_NAME)
    delete(conn, delete_sql, data)


###############################################################
#            test operation     START
###############################################################
###############################################################

# def init():
#     '''初始化方法'''
#     # 数据库文件绝句路径
#     global DB_FILE_PATH
#     DB_FILE_PATH = 'hongten.db'
#     # 数据库表名称
#     global TABLE_NAME
#     TABLE_NAME = 'student'
#     # 是否打印sql
#     global SHOW_SQL
#     SHOW_SQL = True
#     print('show_sql : {}'.format(SHOW_SQL))
#     # 如果存在数据库表，则删除表
#     drop_table_test()
#     # 创建数据库表student
#     create_table_test()
#     # 向数据库表中插入数据
#     save_test()


# def main():
#     init()
#     fetchall_test()
#     print('#' * 50)
#     fetchone_test()
#     print('#' * 50)
#     update_test()
#     fetchall_test()
#     print('#' * 50)
#     delete_test()
#     fetchall_test()
#
#
# if __name__ == '__main__':
#     main()
###############################################################
#            test operation     END
###############################################################
###############################################################


class MsSql:
    def __init__(self, host, user, pwd, db):
        self.host = host
        self.user = user
        self.pwd = pwd
        self.db = db

    def __get_connect(self):
        if not self.db:
            raise(NameError, "db info not set")
        self.conn = pymssql.connect(host=self.host, user=self.user, password=self.pwd, database=self.db, charset="utf8")
        #  cur = self.conn.cursor()
        cur = self.conn.cursor(as_dict=True)
        if not cur:
            raise(NameError, "db conn fail")
        else:
            return cur

    def exec_query(self, sql):
        cur = self.__get_connect()
        cur.execute(sql)
        reslist = cur.fetchall()

        # must close conn
        self.conn.close()
        return reslist

    def exec_non_query(self, sql):
        cur = self.__get_connect()
        cur.execute(sql)
        self.conn.commit()
        self.conn.close()

    def sql_to_csv(self, sql, csvname):
        reslist = self.exec_query(sql)
        print 'get jira record number:'+str(len(reslist))
        #  remove old csv file
        if os.path.exists(csvname):
            os.remove(csvname)

        #  write result to csv file
        with open(csvname, 'wb') as csvfile:  # must add b,else return will become 0d0d0a not 0d0a
            fieldnames = ['domain', 'priority', 'test_id', 'name', 'automation_fw', 'subject']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            for l in reslist:
                writer.writerow({'domain': l[0], 'priority': l[1], 'test_id': str(l[2]),
                                 'name': l[3], 'automation_fw': l[4], 'subject': l[5]})

        return reslist

    #  Query from sql server, then create a new table in sqlite3 db, replace the old one if has.
    def mssql_to_sqlite(self, sqlfile_name, sqlite_db_name, sqlite_table_name):
        with open(os.path.join('sqlfiles', sqlfile_name), 'r') as f:
            sql = f.read()

        reslist = self.exec_query(sql)
        logging.info("Query %s records from %s" % (str(len(reslist)), sqlfile_name))

        # build create table sql
        create_table_sql = r"CREATE TABLE '" + sqlite_table_name + r"' ('id_' INTEGER,"

        for column_name, column_value in reslist[0].items():
            create_table_sql += r"'"
            create_table_sql += column_name
            create_table_sql += r"'"
            create_table_sql += r" varchar(250),"

        create_table_sql += r"PRIMARY KEY(`id_`))"
        logging.info("create_table_sql: %s" % create_table_sql)

        # drop table in sqlite db
        conn = get_conn(os.path.join('sqlfiles', sqlite_db_name))
        drop_table(conn, sqlite_table_name)

        # create table in sqlite db
        logging.info("Ready to create table: %s" % sqlite_table_name)
        conn = get_conn(os.path.join('sqlfiles', sqlite_db_name))
        create_table(conn, create_table_sql)

        # insert all records from sqlserver into sqlite db
        conn = get_conn(os.path.join('sqlfiles', sqlite_db_name))
        save(conn, sqlite_table_name, reslist)

        return reslist


def hanlder_args(args):
    parser = argparse.ArgumentParser(description="ACS Campaign Generator For HP ALM")
    parser.add_argument("--project", "-p",
                        type=str, dest="project",
                        required=False, help="Specify HP ALM project name")
    parser.add_argument("--domain", type=str, required=False,
                        dest="domain", default="SSG",
                        help="Specify HP ALM Domain")
    parser.add_argument("--user", "-u",
                        type=str, dest="user", required=False,
                        help="HP ALM user account name")
    parser.add_argument("--password", "-P",
                        type=str, dest="password",
                        required=False, help="Password")
    parser.add_argument("--filterfile", "-f",
                        type=file, dest="filterfile",
                        required=False, help="Test case filter condition file")
    parser.add_argument("--component", "-c",
                        type=str, dest="component",
                        required=False, help="Generate campaign files limited to component specified")
    parser.add_argument("--tag", "-t",
                        type=str, dest="tag",
                        required=False, help="Add a tag string into the campaign file name")
    parser.add_argument("-d", "--dest",
                        type=str,
                        required=False,
                        default='../../',
                        help="dest top folder. (default is ../../)",)
    parser.add_argument("-l", "--longname",
                        required=False,
                        default=False,
                        action="store_true",
                        dest="longname",
                        help="Generate long test case xml file name",)
    parser.add_argument("-k", "--catalog", required=False, default=False, action="store_true",
                        dest="catalog", help="Generate catalog file")

    parsed_args = parser.parse_args(args)
    return parsed_args


if __name__ == "__main__":
    logging.info("Program Start: %s" % __name__)
    logging.info("Program Start: %s" % os.getcwd())
    start_time = datetime.datetime.now()
    exit_code = 0
    try_times = 10
    handled_args = hanlder_args(sys.argv[1:])
    try:
        if os.path.exists(os.path.join('sqlfiles', SQLITE_DB_NAME)):
            os.remove(os.path.join('sqlfiles', SQLITE_DB_NAME))
        # generate bxt_m_aft.csv which contains data or broxton M P1 feature case list.
        ms = MsSql(host="shwdeotc3f005.ccr.corp.intel.com", user="hpalm_reader", pwd="1234qwer", db="syncenter_new")

        tried_times = 0
        while True:
            try:
                logging.info("Ready to query table from mssql: %s" % TABLE_HPALM_TESTS)
                res_list = ms.mssql_to_sqlite('hpalm_tests.sql', SQLITE_DB_NAME, TABLE_HPALM_TESTS)
                logging.info("Ready to query table from mssql: %s" % TABLE_JIRA_ISSUES)
                res_list = ms.mssql_to_sqlite('jira_issues_feature.sql', SQLITE_DB_NAME, TABLE_JIRA_ISSUES)
                logging.info("Ready to query table from mssql: %s" % TABLE_HPALM_REQUIREMENTS)
                res_list = ms.mssql_to_sqlite('hpalm_requirements.sql', SQLITE_DB_NAME, TABLE_HPALM_REQUIREMENTS)
                logging.info("Ready to query table from mssql: %s" % TABLE_HPALM_REQUIREMENTS_COVERAGES)
                res_list = ms.mssql_to_sqlite('hpalm_requirement_coverages.sql', SQLITE_DB_NAME, TABLE_HPALM_REQUIREMENTS_COVERAGES)
                break
            except Exception, ex:
                logging.error(str(ex))
                traceback.print_exc()
                time.sleep(5)
                tried_times += 1
                if tried_times > 10:
                    raise Exception(ex)
                logging.info("Exception happen,retry times: %s" % str(tried_times))

        # generate bxt_m_aft_hpalm_BROXTON_M_ALL.csv
        # os.chdir("acs_gen")
        # pycommand = "python Data_campaign_read.py "
        # os.system(pycommand)
        # os.chdir("../")
        # print "Finish to pull hpalm data--------------------------------------------------------"
        #
        # print "start to create result csv"
        # namecomp.compcsvfromname("acs_gen/plangenerator/bxt_m_aft_hpalm_BROXTON_M_ALL.csv", "bxt_m_aft_jira.csv",
        #                          "compare_result.csv")

    except Exception, ex:
        logging.error(str(ex))
        traceback.print_exc()
        exit_code = 1

    finally:
        logging.info("Program End: %s" % __name__)

    end_time = datetime.datetime.now()
    logging.info("Program execution seconds: %s" % (end_time - start_time).seconds)

    exit(exit_code)
