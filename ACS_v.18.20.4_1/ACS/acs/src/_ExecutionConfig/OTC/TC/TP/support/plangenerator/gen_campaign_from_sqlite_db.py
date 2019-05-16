# -*- coding:utf-8 -*-

import time
import datetime
import sqlite3
import os
import shutil
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
#SHOW_SQL = False
SHOW_SQL = True
REAL_PATH = os.path.dirname(os.path.realpath(__file__))
CAMPAIGN_FOLDER = os.path.join(REAL_PATH, 'campaign_sql', 'FROM_SQLITE')
SQLITE_DB_FOLDER = 'sqlfiles'
SQLITE_DB_NAME = 'sqlite3.db'
TABLE_HPALM_TESTS = 'hpalm_tests'
TABLE_JIRA_ISSUES = 'jira_issues'
TABLE_HPALM_REQUIREMENTS = 'hpalm_requirements'
TABLE_HPALM_REQUIREMENTS_COVERAGES = 'hpalm_requirement_coverages'

condition_map = {}
condition_map['e_feature_id_broxton_p_ivi_m'] = {'soc_dependency':'Broxton','applicable_for_platforms':'Broxton'}
condition_map['e_feature_id_broxton_p_ivi_o'] = {'soc_dependency':'Broxton','applicable_for_platforms':'Broxton'}

def get_conn(path):
    conn = sqlite3.connect(path)
    if os.path.exists(path) and os.path.isfile(path):
        logging.info('On Hard Disk:%s'% path)
        return conn
    else:
        conn = None
        logging.info('In memory:[:memory:]')
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
            logging.info('Execute sql:%s'% sql)
        cu = get_cursor(conn)
        cu.execute(sql)
        conn.commit()
        logging.info('Drop table %s successfully!'% table)
        close_all(conn, cu)
    else:
        logging.info('the %s is empty or equal None!' % table)


def create_table(conn, sql):
    #  create table：student
    if sql is not None and sql != '':
        cu = get_cursor(conn)
        if SHOW_SQL:
            logging.info('Execute sql:%s'% sql)
        cu.execute(sql)
        conn.commit()
        logging.info('Create table successfully!')
        close_all(conn, cu)
    else:
        logging.info('the %s is empty or equal None!'% sql)


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
        logging.info('the data is empty or equal None!')


def fetchall(conn, sql):
    #  query all records
    if sql is not None and sql != '':
        cu = get_cursor(conn)
        if SHOW_SQL:
            logging.info('Execute sql:%s'% sql)
        cu.execute(sql)
        r = cu.fetchall()
        if len(r) > 0:
            #for e in range(len(r)):
                #print(r[e])
            return r
    else:
        logging.info('the %s is empty or equal None!'% sql)


def fetchone(conn, sql, data):
    #  query one record
    if sql is not None and sql != '':
        if data is not None:
            # Do this instead
            d = (data,)
            cu = get_cursor(conn)
            if SHOW_SQL:
                logging.info('Execute sql:%s,params:%s'% sql % data)
            cu.execute(sql, d)
            r = cu.fetchall()
            if len(r) > 0:
                for e in range(len(r)):
                    logging.info('%s'% r[e])
        else:
            logging.info('the %s equal None!'% data)
    else:
        logging.info('the %s is empty or equal None!'% sql)


def update(conn, sql, data):
    #  update records
    if sql is not None and sql != '':
        if data is not None:
            cu = get_cursor(conn)
            for d in data:
                if SHOW_SQL:
                    logging.info('Execute sql:%s,params:%s'% sql %d)
                cu.execute(sql, d)
                conn.commit()
            close_all(conn, cu)
    else:
        logging.info('the %s is empty or equal None!'% sql)


def delete(conn, sql, data):
    #  delete records
    if sql is not None and sql != '':
        if data is not None:
            cu = get_cursor(conn)
            for d in data:
                if SHOW_SQL:
                    logging.info('Execute sql:%s,params:%s' % sql % d)
                cu.execute(sql, d)
                conn.commit()
            close_all(conn, cu)
    else:
        logging.info('the %s is empty or equal None!'% sql)


###############################################################
#            db operation CRUD     END
###############################################################


###############################################################
#            test operation     START
###############################################################
def drop_table_test():
    #  delete test table
    logging.info('Drop table...')
    conn = get_conn(SQLITE_DB_NAME)
    drop_table(conn, TABLE_NAME)


def create_table_test():
    #  create test table
    logging.info('Create table...')
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
    logging.info('Save data...')
    save_sql = '''INSERT INTO student values (?, ?, ?, ?, ?, ?)'''
    data = [(1, 'Hongten', 'male', 20, 'Guang Zhou', '13423****62'),
            (2, 'Tom', 'male', 22, 'San Francisco', '15423****63'),
            (3, 'Jake', 'female', 18, 'Guang Zhou', '18823****87'),
            (4, 'Cate', 'female', 21, 'Guang Zhou', '14323****32')]
    conn = get_conn(SQLITE_DB_NAME)
    save(conn, save_sql, data)


def fetchall_test():
    #  query all records...
    logging.info("Query all...")
    fetchall_sql = '''SELECT * FROM student'''
    conn = get_conn(SQLITE_DB_NAME)
    fetchall(conn, fetchall_sql)


def fetchone_test():
    #  query one record...
    logging.info("Query one...")
    fetchone_sql = 'SELECT * FROM student WHERE ID = ? '
    data = 1
    conn = get_conn(SQLITE_DB_NAME)
    fetchone(conn, fetchone_sql, data)


def update_test():
    #  update data...
    logging.info("Update data...")
    update_sql = 'UPDATE student SET name = ? WHERE ID = ? '
    data = [('HongtenAA', 1),
            ('HongtenBB', 2),
            ('HongtenCC', 3),
            ('HongtenDD', 4)]
    conn = get_conn(SQLITE_DB_NAME)
    update(conn, update_sql, data)


def delete_test():
    #  delete data...
    logging.info("Delete data...")
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
#     '''init function'''
#     # abs path of db file
#     global DB_FILE_PATH
#     DB_FILE_PATH = 'hongten.db'
#     # table name
#     global TABLE_NAME
#     TABLE_NAME = 'student'
#     #  print sql
#     global SHOW_SQL
#     SHOW_SQL = True
#     print('show_sql : {}'.format(SHOW_SQL))
#     # if table exist, delete it
#     drop_table_test()
#     # create table student
#     create_table_test()
#     # insert data
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
def prettify(elem):
    """return a pretty-printed xml string for the etree"""
    rough_string = tostring(elem, 'utf-8')
    reparsed = parseString(rough_string)
    return reparsed.toprettyxml(indent='    ')

def flush_content(file_path, content):
    """dump contents to specified file"""
    try:
        #if os.path.exists(file_path):
        #    print "File already exists, overwrite!"
        if not os.path.isdir(os.path.dirname(file_path)):
            os.makedirs(os.path.dirname(file_path))
        with open(file_path, 'w') as fd:
            fd.write(content.encode('UTF-8'))
    except Exception, err:
        logging.error("Write %s file error(%s)" % (file_path, str(err)))
        raise


def query_records_from_sqlite(sqlite_db_name, template_sql, android_version, platform, domain, req_priority, lables, tag, sql_raw):
    logging.info("Call function query_records_from_sqlite.")
    logging.info("sqlite_db_name: %s" % sqlite_db_name)
    logging.info("template_sql: %s" % template_sql)
    logging.info("android_version: %s" % android_version)
    logging.info("platform: %s" % platform)
    logging.info("domain: %s" % domain)
    logging.info("req_priority: %s" % req_priority)
    logging.info("lables: %s" % lables)
    logging.info("tag: %s" % tag)
    logging.info("raw_sql: %s" % sql_raw)

    reslist = {}
    with open(os.path.join(REAL_PATH, 'campaign_sql', template_sql), 'r') as f:
        sql = f.read()

    # ***build query records sql START***
    query_records_sql = sql.format(ANDROID_VERSION=android_version, PLATFORM=platform,DOMAIN=domain, REQ_PRIORITY=req_priority,
                                   LABLES=lables, TAG=tag, SOC_DEPENDENCY=condition_map[platform]['soc_dependency'], APPLICABLE_FOR_PLATFORMS=condition_map[platform]['applicable_for_platforms'])  # format sql string

    logging.info("Final query_records_sql is: %s" % query_records_sql)
    # ***build query records sql END***

    # insert all records from sql server into sqlite db
    conn = get_conn(os.path.join(REAL_PATH, SQLITE_DB_FOLDER, sqlite_db_name))
    reslist = fetchall(conn, query_records_sql)

    if len(reslist) < 1:
        logging.info("SQL result res_list is empty!!!")
        raise Exception("SQL result res_list is empty !!!")

    logging.info("Query %d records from db."%len(reslist))
    return reslist

def generate_campaign(res_list, campaign_file_name):
    logging.debug("Saving into campaign file: %s" %campaign_file_name)
    TC_PATH_PREFIX = '../../../../TC/'
    attr_campaign = {"version":"13.49"}
    campaign = Element('Campaign', attrib=attr_campaign)
    parameters = SubElement(campaign, 'Parameters')
    attributes = [
        {"isControlledPSUsed":"False"},
        {"isIoCardUsed":"False"},
        {"skipBootOnPowerCycle":"False"},
        {"bootRetryNumber":"0"},
        {"runHookScripts":"False"},
        {"powerCycleBetweenTC":"False"},
        {"powerCycleOnFailure":"False"},
        {"finalDutState":"NoChange"},
        {"stopCampaignOnCriticalFailure":"False"},
        {"stopCampaignOnFirstFailure":"False"},
        {"loggingLevel":"debug"},
        {"CampaignType":"Other"},
        {"TCRReportViaRESTAPI":"True"},
    ]
    for kv in attributes:
        SubElement(parameters, 'Parameter', attrib=kv)
    testcases = SubElement(campaign, 'TestCases')
    for tc in res_list:
        #tc[2] is test name with . ,  tc[3] is test subject
        tc_name = tc[2].split('.')[-1]
        #if (longname):
        #    tc_name = tc["name"]
        path = tc[3].replace("Subject\\SSG\\System Functional Tests\\","")
        #import pdb
        #pdb.set_trace()
        tc_path = TC_PATH_PREFIX + os.path.join(path.replace("\\","/"), tc_name)
        SubElement(testcases, 'TestCase', Id=tc_path)
    content = prettify(campaign)
    flush_content(campaign_file_name, content)
    logging.info("Finished to write campaign file.")


    return

def generate_merge_campaign(domain, campaign_file_name):
    logging.debug("generate_merge_campaign")
    merge_campaign_file = campaign_file_name
    pretest_path = REAL_PATH[:REAL_PATH.find("OTC")+len("OTC")]+"/CAMPAIGN/SystemFunctional/N/MR_0/TP/"
    pretest_file_name = domain + "_pretest.xml"

    logging.info("pretest_path: %s" % pretest_path)
    logging.info("pretest_file_name: %s" % pretest_file_name)

    if os.path.exists(pretest_path + pretest_file_name):
        #  copy Post_Test.xml and update relative path ,  delete  "../TC/TP/"
        #import pdb
        #pdb.set_trace()
        shutil.copyfile(pretest_path + "Post_Test.xml", CAMPAIGN_FOLDER + os.sep + "Post_Test.xml")
        cmd = r"for i in `ls {0}`;do sed -i 's#../TC/TP/TC#TC#' $i;done;".format(CAMPAIGN_FOLDER + os.sep + "Post_Test.xml")
        os.system(cmd)

        #  copy _pretest.xml and update relative path,  delete  "../TC/TP/"
        shutil.copyfile(pretest_path + pretest_file_name, CAMPAIGN_FOLDER + os.sep +pretest_file_name)
        cmd = r"for i in `ls {0}`;do sed -i 's#../TC/TP/TC#TC#' $i;done;".format(CAMPAIGN_FOLDER + os.sep +pretest_file_name)
        os.system(cmd)
        #  generate merge campaign file
        os.system("./campaign_merge.sh "+CAMPAIGN_FOLDER)


        #  get merge cmapaign name
        merge_campaign_file = campaign_file_name.replace(".xml",".merge"+".xml")

    return merge_campaign_file


def hanlder_args(args):
    parser = argparse.ArgumentParser(description="ACS Campaign Generator From SQLite DB")
    parser.add_argument("--sqlite_db_name", "-D",
                        type=str, dest="sqlite_db_name",
                        required=False, default="sqlite3.db", help="Specify sqlite database name")
    parser.add_argument("--template_sql", "-T",
                        type=str, dest="template_sql",
                        required=False, default="sqlite_template.sql", help="Specify template sql name")
    parser.add_argument("--android_version", "-a",
                        type=str, dest="android_version",
                        required=True, help="Specify android version")
    parser.add_argument("--platform", "-p",
                        type=str, dest="platform",
                        required=True, help="Specify platform")
    parser.add_argument("--domain", "-d",
                        type=str, dest="domain",
                        required=True, help="Specify domain")
    parser.add_argument("--req_priority", "-r",
                        type=str, dest="req_priority",
                        required=False, default="P1-Stopper", help="Specify requirement priority")
    parser.add_argument("--lables", "-l",
                        type=str, dest="lables",
                        required=False, default="", help="Specify lables")
    parser.add_argument("--test_tag","-t",
                        type=str, dest="tag",
                        required=False, default="", help="Specify test tag")
    parser.add_argument("--sql_raw", "-s",
                        type=str, dest="sql_raw",
                        required=False, default="", help="Advanced:Using sql directly instead of parameter.")



    parsed_args = parser.parse_args(args)
    return parsed_args


if __name__ == "__main__":
    logging.info("Program Start: %s" % __name__)
    logging.info("Program Start: %s" % os.getcwd())
    logging.info("REAL_PATH: %s" % REAL_PATH)
    logging.info("Sample: %s" % "cd acs/src/_ExecutionConfig/OTC/TC/TP/support/plangenerator")
    logging.info("%s" % "python gen_campaign_from_sqlite_db.py --android_version O --platform e_feature_id_broxton_p_ivi_o --domain Graphics_Display")
    start_time = datetime.datetime.now()
    exit_code = 0
    try_times = 10
    handled_args = hanlder_args(sys.argv[1:])
    logging.info("handled_args is: %s" % handled_args)
    try:
        if not os.path.exists(os.path.join(REAL_PATH, 'sqlfiles', SQLITE_DB_NAME)):
            logging.info("Ready to query case from sqlite: %s" % os.path.join(REAL_PATH, 'sqlfiles', SQLITE_DB_NAME))
            raise Exception("Sql db file not finded !!!")

        tried_times = 0
        while True:
            try:
                logging.info("Ready to query case from sqlite: %s" % os.path.join('sqlfiles', SQLITE_DB_NAME))
                nowTime = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
                campaign_file_name = '{0}.{1}-{2}_{3}.xml'.format(handled_args.domain,handled_args.platform,handled_args.android_version,nowTime)
                logging.info("campaign_file_name is: %s" % campaign_file_name)

                res_list = query_records_from_sqlite(sqlite_db_name=handled_args.sqlite_db_name, template_sql=handled_args.template_sql,
                                                     android_version=handled_args.android_version,  platform=handled_args.platform, domain=handled_args.domain ,
                                                     req_priority=handled_args.req_priority, lables=handled_args.lables,
                                                     tag=handled_args.tag,sql_raw=handled_args.sql_raw)

                campaign_file = generate_campaign(res_list, os.path.join(CAMPAIGN_FOLDER, campaign_file_name))
                merge_campaign_file=generate_merge_campaign(handled_args.domain , campaign_file_name)

                #return campaign file to shell
                TEST_CAMPAIGN_PARAM = CAMPAIGN_FOLDER[CAMPAIGN_FOLDER.find("_ExecutionConfig")+len("_ExecutionConfig")+1:] + os.sep + merge_campaign_file
                logging.info("The campaign param for ACS is: %s" % TEST_CAMPAIGN_PARAM)
                print(TEST_CAMPAIGN_PARAM)

                break
            except Exception, ex:
                logging.error(str(ex))
                traceback.print_exc()
                time.sleep(5)
                tried_times += 1
                if tried_times >= 1:
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
        logging.info("Program End with code: %d" % exit_code)

    end_time = datetime.datetime.now()
    logging.info("Program execution seconds: %s seconds" % (end_time - start_time).seconds)

    exit(exit_code)
