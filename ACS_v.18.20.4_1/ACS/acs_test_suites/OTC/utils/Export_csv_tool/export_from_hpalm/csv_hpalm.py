import os
import sys
import argparse
import xml.etree.ElementTree as ET
from datetime import datetime
import httplib
import base64
import logging
import csv
import platform
import subprocess
import time
import urllib2
from urllib import pathname2url

from ConfigParser import ConfigParser

import re
from xml.etree.ElementTree import *
from xml.dom.minidom import parseString

reload(sys)
sys.setdefaultencoding('utf-8')

LOGGING_FORMAT = '%(asctime)-15s %(message)s'
logging.basicConfig(format=LOGGING_FORMAT)
logger = logging.getLogger('HPQC_SYNC')
qc = None

class qc_connector:
    DEFAULT_USERNAME = ''
    DEFAULT_PASSWORD = ''
    def __init__(self, host='hpalm.intel.com', port=443, domain='SSG', project='OTC_Android_Testing'):
        self.__host = host
        self.__port = port
        self.__domain = domain
        self.__project = project
        self.__conn = httplib.HTTPSConnection(self.__host, self.__port)
        self.__cookies = {}
        self.cookie_str = None
        self.__logger = logger

    def connect(self):
        print "Create connection"
        self.__conn.connect()

    def __update_cookie(self, cookie_str):
        if (self.cookie_str is not None):
            self.cookie_str = self.cookie_str + cookie_str
        else:
            self.cookie_str = cookie_str
    
    def __get_cookie_str(self):
        return self.cookie_str
        
    def build_entity_url(self, entity_path):
        return '/qcbin/rest/domains/%s/projects/%s/%s' % (self.__domain, self.__project, entity_path)
            
    def is_authenticated(self):
        self.__conn.request('GET', '/qcbin/rest/is-authenticated')
        response = self.__conn.getresponse()
        response.read()
        if response.status == httplib.OK:
            return None
        elif response.status == httplib.UNAUTHORIZED:
            auth_point = 'https://hpalm-pre.intel.com' + '/qcbin/authentication-point/authenticate'
            return auth_point
        else:
            raise 'Network Error. %s %s' %(response.status, response.reason) 
            
    def login(self, username, password):
        auth_point = self.is_authenticated()
        if auth_point is not None:
            auth_str = '%s:%s' %(username,password)
            auth_str_encode = 'Basic %s' % base64.b64encode(auth_str)
            headers = {}
            headers['Authorization'] = auth_str_encode
            self.__conn.request('GET', auth_point, None, headers)
            response = self.__conn.getresponse()
            response.read()
            if response.status == httplib.OK:
                cookie_str = response.getheader('Set-Cookie')
                if cookie_str is not None:
                    self.__update_cookie(cookie_str)
                return True
            else:
                return False
        return False
    
    def logout(self):
        print ("Logout HPQC")
        self.__conn.request('GET', '/qcbin/authentication-point/logout')
        response = self.__conn.getresponse()
        response.read()
        if response.status == httplib.OK:
            return True
        else:
            return False
    
    def open_session(self):
        print ("Open session")
        headers = {}
        cookie_str = self.__get_cookie_str()
        if cookie_str is not None:
            headers['Cookie'] = cookie_str

        self.__conn.request('POST', '/qcbin/rest/site-session', headers=headers)
        response = self.__conn.getresponse()
        response.read()
        if response.status == httplib.OK or response.status == httplib.CREATED:
            cookie_str = response.getheader('Set-Cookie')
            if cookie_str is not None:
                    self.__update_cookie(cookie_str)
            return True
        else:
            return False

    def close_session(self):
        print ("Close session")
        headers = {}
        cookie_str = self.__get_cookie_str()
        if cookie_str is not None:
            headers['Cookie'] = cookie_str

        self.__conn.request('DELETE', '/qcbin/rest/site-session?login-form-required=y',headers=headers)
        response = self.__conn.getresponse()
        response.read()
        if response.status == httplib.OK:
            return True
        else:
            return False


    def do_get(self, url, query, headers={}):
        if query is not None and query.strip() is not '':
            url = '%s?%s' % (url, query)
        self.__logger.debug('GET Request url => ' + url)

        cookie_str = self.__get_cookie_str()
        if cookie_str is not None:
            headers['Cookie'] = cookie_str
        self.__logger.debug('GET Request headers => ')
        self.__logger.debug(headers)
        
        self.__conn.request('GET', url, None, headers)
        response = self.__conn.getresponse()
        rs = response.read()
        if response.status == httplib.OK:
            return rs
        else:
            return None

    def do_put(self, url, entity_type, entity_id, data={}, headers={}):
        url = url + "/" + entity_id
        self.__logger.debug('PUT Request url => ' + url)
        
        cookie_str = self.__get_cookie_str()
        if cookie_str is not None:
            headers['Cookie'] = cookie_str
        self.__logger.debug('PUT Request headers => ')
        self.__logger.debug(headers)
        body = self._get_xml_from_pairs(entity_type,data)
        print body
        self.__conn.request('PUT', url, body, headers)
        response = self.__conn.getresponse()
        
        if response.status == httplib.OK:
            return response.read()
        else:
            self.__logger.debug(str(response.status) + ' ' + response.reason)
            return None


    def close(self):
        if self.__conn is not None:
            print "Close connection"
            self.__conn.close()

    def _get_xml_from_pairs(self, entity_type, pairs):
        entity = ET.Element("Entity")
        entity.attrib["Type"] = entity_type
        fields = ET.SubElement(entity, "Fields")
        for name, value in pairs:
            field = ET.SubElement(fields, "Field")
            field.attrib["Name"] = name
            if value is not None:
                value_elem = ET.SubElement(field, "Value")
                if not isinstance(value, basestring):
                    value = str(value)
                value_elem.text = value
        return ET.tostring(entity)  

def pull_full_folders(qc, folderid, folderpath, scriptpath):
    rs = qc.do_get(qc.build_entity_url('test-folders'), 'query={parent-id[%s]}&fields=id,name' % folderid)
    if rs is None:
        logger.error('No Test Set under this test set folder')
        return None
 
    entities = ET.fromstring(rs).findall("Entity")
    test_set_folders = []
    for entity in entities:
        test_set_folder = {}
        test_set_folder["id"] = entity.find('Fields/Field[@Name="id"]/Value').text
        test_set_folder["name"] = entity.find('Fields/Field[@Name="name"]/Value').text
        test_set_folder["fullpath"] = folderpath + "/" + test_set_folder["name"]
        test_set_folder["scriptpath"] = scriptpath + "/" + test_set_folder["name"]
        test_set_folders.append(test_set_folder)
        subfolders = pull_full_folders (qc, test_set_folder["id"], test_set_folder["fullpath"], test_set_folder["scriptpath"])
        if (subfolders is not None):
            test_set_folders = test_set_folders + subfolders

    return test_set_folders


def prettify(elem):
    """return a pretty-printed xml string for the etree"""
    rough_string = tostring(elem, 'utf-8')
    reparsed = parseString(rough_string)
    return reparsed.toprettyxml(indent='    ')

def pull_full_test_cases (rootfolder, fullfolders, query_str):
    test_sets = []
    counter = 0
    for folder in fullfolders:
        folderid = folder["id"]
        foldername = folder["name"]
        folderfullpath = folder["fullpath"]
        scriptpath = folder["scriptpath"]

        query_str_encoded = pathname2url(query_str)
        query = 'query={parent-id[%s];%s}&page-size=2000' % (folderid, query_str_encoded)
        rs = qc.do_get(qc.build_entity_url('tests'), query)

        if rs is not None:
            entities = ET.fromstring(rs).findall("Entity")
            for entity in entities:
                test_set = {}
                test_set["id"] = entity.find('Fields/Field[@Name="id"]/Value').text
                test_set["name"] = entity.find('Fields/Field[@Name="name"]/Value').text
                description = entity.find('Fields/Field[@Name="description"]/Value').text
                if (description):
                    description = description.replace("\n", " ")
                    description = description.replace("\r", " ")
                    matchobj = re.match(r'\<html\>\<body\>(.*)\<\/body\>\<\/html\>', description, re.M|re.I)
                    if (matchobj):
                        test_set["description"] = matchobj.group(1)
                    else:
                        test_set["description"] = description
                else:
                    test_set["description"] = test_set["name"]
       
                test_set["fullpath"] = folderfullpath
                test_set["scriptpath"] = scriptpath
                test_sets.append(test_set)
                counter = counter + 1
    print "%d test cases found in %s" % (counter, rootfolder["name"])
    return test_sets

def generate_csvs(test_sets, full_csv_name):
    """generate csv files to the current CSV folder"""
    try:
        if os.path.exists(full_csv_name):
            print "File: %s already exists, overwrite!" % full_csv_name
        if not os.path.isdir(os.path.dirname(full_csv_name)):
            os.makedirs(os.path.dirname(full_csv_name))
        with open(full_csv_name, 'w') as fd:
            print "Writing to %s...\n" % full_csv_name
            fd.write("Package,Name\n")
            for tc in test_sets:
                fd.write(tc['fullpath']+","+ tc['name']+"\n")
    except Exception, err:
        raise Exception("Write %s file error(%s)" % (full_csv_name, str(err)))


def get_test_set_folder_by_path(qc_conn, path, include_folders):
    rootpath = path
    path = path + '*' if path[-1] == "/" else path

    ps = path.split("/")
    
    last_test_set_folder_id = 0
    parent_str = ""
    for p in ps:
        rs = qc.do_get(qc.build_entity_url('test-folders'), 'query={'+ parent_str +"name['" +urllib2.quote(p) +"']}&fields=id,name")
        if rs is None:
            logger.error('Not found "{0}" node in Test Lab'.format(p))
            return None

        entities = ET.fromstring(rs)
        if entities.attrib['TotalResults'] != '1':
            logger.error('Found more than 1 {0} name %s in Test Lab, but the first will be used' .format(p))
        value = entities.findall('Entity/Fields/Field[@Name="id"]/Value')[0]
        parent_str = "parent-id[{0}];".format(value.text)
        last_test_set_folder_id = value.text


    rs = qc.do_get(qc.build_entity_url('test-folders'), 'query={parent-id[%s]}&fields=id,name' % last_test_set_folder_id)
    if rs is None:
        logger.error('No Test Set under this test set folder')
        return None
 
    entities = ET.fromstring(rs).findall("Entity")
    test_set_folders = []
    applied_folder_filter = (len(include_folders) > 0)
    for entity in entities:
        test_set_folder = {}
        test_set_folder["id"] = entity.find('Fields/Field[@Name="id"]/Value').text
        test_set_folder["name"] = entity.find('Fields/Field[@Name="name"]/Value').text
        test_set_folder["fullpath"] = rootpath + "/" + test_set_folder["name"]
        test_set_folder["scriptpath"] = test_set_folder["name"]
        if (applied_folder_filter):
            if (test_set_folder["name"] in include_folders):
                test_set_folders.append(test_set_folder)
        else:
            test_set_folders.append(test_set_folder)
    return test_set_folders

def get_test_set_folders(qc_conn, parent_id):
    rs = qc.do_get(qc.build_entity_url('test-set-folders'), 'query={parent-id[%s]}&fields=id,name' % parent_id)
    if rs is None:
        return None
    entities = ET.fromstring(rs).findall("Entity")
    test_set_folders = []
    for entity in entities:
        test_set_folder = {}
        test_set_folder["id"] = entity.find('Fields/Field[@Name="id"]/Value').text
        test_set_folder["name"] = entity.find('Fields/Field[@Name="name"]/Value').text
        test_set_folders.append(test_set_folder)
    return test_set_folders

def get_test_sets(qc_conn, parent_id):
    rs = qc.do_get(qc.build_entity_url('tests'), 'query={parent-id[%s]}' % parent_id)
    if rs is None:
        return None
    entities = ET.fromstring(rs).findall("Entity")
    test_sets = []
    for entity in entities:
        test_set = {}
        test_set["id"] = entity.find('Fields/Field[@Name="id"]/Value').text
        test_set["name"] = entity.find('Fields/Field[@Name="name"]/Value').text
        test_sets.append(test_set)
    return test_sets

def get_tests(qc_conn, test_set_id):
    rs = qc.do_get(qc.build_entity_url('tests'), 'query={test-instance.cycle-id['+ test_set_id +']}&page-size=5000')
    print rs
    
def get_test_instances(qc_conn, test_set_id):
    rs = qc.do_get(qc.build_entity_url('test-instances'), 'query={cycle-id['+ test_set_id +']}&page-size=5000')
    print rs


def get_filter_conditions(filter_fd):
    config = ConfigParser()
    config.optionxform = str
    with filter_fd:
        config.readfp(filter_fd)
    return config.items("query")


class alm_filter:
    
    def __init__ (self, component):
        if (component is not None):
            self.components = component.split(',')
        else:
            self.components = []
        self.fieldlist = []
        self.fieldmap = {}
        self.basefolder = "Subject/SSG/System Functional Tests"
        self.include_folders = []

    def handle_custom_filters(self, key, value):
        if (key == 'BaseFolder'):
            matchobj = re.match(r'\[(.*)\]', value, re.M|re.I)
            if (matchobj):
                self.basefolder = matchobj.group(1)
                self.basefolder.strip()
        elif (key == 'SubFolders'):
            matchobj = re.match(r'\[(.*)\]', value, re.M|re.I)
            if (matchobj):
                self.include_folders = matchobj.group(1).split(' ')

    def get_include_folders(self):
        res_folders = None
        if (len(self.components) > 0):
            res_folders = [folder for folder in self.include_folders if folder in self.components]
        else:
            res_folders = self.include_folders

        if (len(res_folders) == 0 and self.include_folders is not None):
            res_folders = self.include_folders

        return res_folders

    def get_root_folder(self):
        return self.basefolder

    def pull_filedmapping(self, qc):
        rs = qc.do_get(qc.build_entity_url('customization/entities/test/fields'), "query={'name[*]'")
        if (rs is not None):
            fields = ET.fromstring(rs).findall("Field")
            for field in fields:
                fieldattr = {}

                fieldattr["name"] = field.attrib['Name']
                if ('Label' in field.keys()):
                    fieldattr["label"] = field.attrib['Label']
                else:
                    fieldattr["label"] = field.attrib['Name']

                fieldattr["pysicalname"] = field.attrib['PhysicalName']
                self.fieldmap[fieldattr["label"]] = fieldattr["name"]

                self.fieldlist.append(fieldattr)

    def parsing_filter_string(self, filterstr):
        condition_list = filterstr.split(';')
        new_filter = []
        for condition in condition_list:
            ckey = condition.split('[')
            keystr = ckey[0].strip()
            cvalue = '[' + ckey[1]
            if keystr in self.fieldmap.keys():
                newkey = self.fieldmap[keystr]
                newcondition = newkey + cvalue
                new_filter.append(newcondition)
            else:
                self.handle_custom_filters(ckey[0], cvalue)

        query_str = ';'.join(new_filter)
        return query_str


if __name__ == "__main__":
    global gc
    parser = argparse.ArgumentParser(description="Advanced Robot Automation Runner. Automatically pull down the test plan from HPQC. And push the result to HPQC.")
    parser.add_argument("--project", "-p", type=str, dest="project", required=True, help="Specify HPQC Project Name")
    parser.add_argument("--domain", type=str, dest="domain", default="SSG", help="Specify HPQC Domain")
    parser.add_argument("--user", "-u", type=str, dest="user", required=True, help="A valid username for HPQC")
    parser.add_argument("--password", "-P", type=str, dest="password", required=True, help="Password")
    parser.add_argument("--filterfile", "-f", type=file, dest="filterfile", required=True, help="Test case filter condition file")
    parser.add_argument("--component", "-c", type=str, dest="component", required=False, help="Specify generating campaign files for given component list")
    parser.add_argument("--tag", "-t", type=str, dest="tag", required=False, help="Add a tag string into the campaign name")

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


    args = parser.parse_args(sys.argv[1:])
    logger.debug(args)


    filter_conditions = get_filter_conditions (args.filterfile)

    #Connect and login into HPQC for exporting and importing
    if args.domain is None or args.project is None:
        logger.error('Should specify the QC domain, project')
        raise "Should specify the QC domain, project";

    qc = qc_connector(domain=args.domain, project=args.project)

    login_flag = False
    if args.user is None or args.password is None:
        qc.connect()
        login_flag = qc.login(qc_connector.DEFAULT_USERNAME, qc_connector.DEFAULT_PASSWORD)
    else:
        qc.connect()
        login_flag = qc.login(args.user, args.password)
    if login_flag == False:
        logger.error('QC Login Failed.')
        raise "QC Login Failed."

    qc.open_session()
    exit_code = 0

    almqtyfilter = alm_filter(args.component)

    almqtyfilter.pull_filedmapping(qc)

    try:
        start_datetime = datetime.now()
        print ("Start @ " + str(start_datetime))  
        

        for board, query in filter_conditions:
            query_str = almqtyfilter.parsing_filter_string (query)
            include_folders = almqtyfilter.get_include_folders()
            root = almqtyfilter.get_root_folder()

            campaign_folders =  get_test_set_folder_by_path(qc, root, include_folders)
            for folder in campaign_folders:
                fullfolders = []
                fullfolders.append(folder)
                subfolders = pull_full_folders(qc, folder["id"], root+"/"+folder["name"], folder["name"])
                fullfolders = fullfolders + subfolders
                print "-" * 90
                print "Campaign Folder: ", folder["name"]
                test_sets = pull_full_test_cases (folder, fullfolders, query_str)

                full_csv_name = os.path.join(os.getcwd(), 'CSV', folder["name"] + '.csv')
                generate_csvs(test_sets, full_csv_name)

    except Exception, ex:
        print ex
        exit_code = 1

    finally:
         qc.close_session()
         qc.logout()
         qc.close()

    exit(exit_code)
