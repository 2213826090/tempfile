import os
import sys
import argparse
import xml.etree.ElementTree as ET
import httplib
import base64
import logging
import urllib2
from urllib import pathname2url

from ConfigParser import ConfigParser

import re
from xml.etree.ElementTree import *
from xml.dom.minidom import parseString

reload(sys)
sys.setdefaultencoding('utf-8')
logging.basicConfig(level = logging.DEBUG)

qc = None

class qc_connector:

    DEFAULT_HOST = 'hpalm.intel.com'
    DEFAULT_PORT = 443
    DEFAULT_DOMAIN = 'SSG'
    DEFAULT_PROJECT = 'OTC_Android_Testing'
    DEFAULT_PROTOCOL = 'https'

    DEFAULT_USERNAME = ''
    DEFAULT_PASSWORD = ''

    def __init__(self, host=DEFAULT_HOST, \
                       port=DEFAULT_PORT, \
                       domain=DEFAULT_DOMAIN, \
                       project=DEFAULT_PROJECT, \
                       protocol=DEFAULT_PROTOCOL):

        self.__host = host
        self.__port = port
        self.__protocol = protocol
        self.__domain = domain
        self.__project = project

        if ('https' == self.__protocol):
            self.__conn = httplib.HTTPSConnection(self.__host, self.__port)
        else:
            self.__conn = httplib.HTTPConnection(self.__host, self.__port)

        self.__cookies = {}
        self.cookie_str = None

    def connect(self):
        logging.info("Create connection")
        self.__conn.connect()

    def __update_cookie(self, cookie_str):
        if (self.cookie_str is not None):
            self.cookie_str = self.cookie_str + cookie_str
        else:
            self.cookie_str = cookie_str

    def __get_cookie_str(self):
        return self.cookie_str

    def build_entity_url(self, entity_path):
        return '/qcbin/rest/domains/%s/projects/%s/%s' % \
               (self.__domain, self.__project, entity_path)

    def is_authenticated(self):
        self.__conn.request('GET', '/qcbin/rest/is-authenticated')
        response = self.__conn.getresponse()
        response.read()
        if response.status == httplib.OK:
            return None
        elif response.status == httplib.UNAUTHORIZED:
            auth_point = self.__protocol + '://' + self.__host + '/qcbin/authentication-point/authenticate'
            return auth_point
        else:
            raise 'Network Error. %s %s' %(response.status, response.reason)

    def login(self, username, password):
        if not (username and password):
            logging.debug("Loging with default user & password")
            username, password = self.DEFAULT_USERNAME, self.DEFAULT_PASSWORD
        logging.debug("Loging with user: %s password: %s"%(username, password))
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
        return False

    def logout(self):
        logging.debug("Logout HPQC")
        self.__conn.request('GET', '/qcbin/authentication-point/logout')
        response = self.__conn.getresponse()
        response.read()
        if response.status == httplib.OK:
            return True
        else:
            return False

    def open_session(self):
        logging.debug("Open session")
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
        logging.debug("Close session")
        headers = {}
        cookie_str = self.__get_cookie_str()
        if cookie_str is not None:
            headers['Cookie'] = cookie_str

        self.__conn.request('DELETE', \
                            '/qcbin/rest/site-session?login-form-required=y', \
                            headers=headers)

        response = self.__conn.getresponse()
        response.read()
        if response.status == httplib.OK:
            return True
        else:
            return False

    def do_get(self, url, query, headers={}):
        if query is not None and query.strip() is not '':
            url = '%s?%s' % (url, query)
        cookie_str = self.__get_cookie_str()
        if cookie_str is not None:
            headers['Cookie'] = cookie_str
        self.__conn.request('GET', url, None, headers)
        response = self.__conn.getresponse()
        rs = response.read()
        if response.status == httplib.OK:
            return rs
        else:
            return None

    def do_put(self, url, entity_type, entity_id, data={}, headers={}):
        url = url + "/" + entity_id

        cookie_str = self.__get_cookie_str()
        if cookie_str is not None:
            headers['Cookie'] = cookie_str
        body = self._get_xml_from_pairs(entity_type,data)
        logging.debug(body)
        self.__conn.request('PUT', url, body, headers)
        response = self.__conn.getresponse()
        if response.status == httplib.OK:
            return response.read()
        else:
            return None

    def close(self):
        if self.__conn is not None:
            logging.debug("Close connection")
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
    rs = qc.do_get(qc.build_entity_url('test-folders'), \
                   'query={parent-id[%s]}&fields=id,name' % folderid)

    if rs is None:
        logging.warn('No Test Set under this test set folder, id:' % folderid)
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

        subfolders = pull_full_folders (qc, test_set_folder["id"], \
                                        test_set_folder["fullpath"], \
                                        test_set_folder["scriptpath"])

        if (subfolders is not None):
            test_set_folders = test_set_folders + subfolders
    return test_set_folders

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

def merge_test_sets(test_sets1, test_sets2):
    test_set_ids1 = []
    for test in test_sets1:
        test_set_ids1.append(test['id'])

    for test in test_sets2:
        if not test['id'] in test_set_ids1:
            test_set_ids1.append(test['id'])
            test_sets1.append(test)

    return test_sets1

def pull_full_test_cases (rootfolder, fullfolders, query_str):
    test_sets = []
    counter = 0
    global dict_areq_covers
    for folder in fullfolders:
        folderid = folder["id"]
        #foldername = folder["name"]
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
                test_set["exec_type"] = entity.find('Fields/Field[@Name="user-08"]/Value').text
                description = entity.find('Fields/Field[@Name="description"]/Value').text
                if (description):
                    matchobj = re.match(r'\<html\>\<body\>(.*)\<\/body\>\<\/html\>', description, re.M|re.I|re.DOTALL)
                    if (matchobj):
                        test_set["description"] = matchobj.group(1)
                    else:
                        test_set["description"] = description
                else:
                    test_set["description"] = test_set["name"]

                test_set["fullpath"] = folderfullpath
                test_set["scriptpath"] = scriptpath
                # Add Priority, test category[ST,FT],
                test_set["test_category"] = entity.find('Fields/Field[@Name="user-18"]/Value').text
                test_set["priority"] = entity.find('Fields/Field[@Name="user-04"]/Value').text
                test_set["domain"] = folderfullpath.split("/")[3]
                test_set["ci_ready"] = entity.find('Fields/Field[@Name="user-24"]/Value').text
                test_set["bench_caps"] = entity.find('Fields/Field[@Name="user-21"]/Value').text
                test_set["tc_type"] = entity.find("Fields/Field[@Name='user-03']/Value").text

                # Add AREQ information
                if test_set["id"] in dict_areq_covers:
                    test_set["areq"] = dict_areq_covers[test_set["id"]]
                else:
                    test_set["areq"] = None
                test_sets.append(test_set)
                counter = counter + 1
    logging.info( "%d test cases found in %s" % (counter, rootfolder["name"]))
    return test_sets

def generate_tc_files(test_sets, tc_folder, campaign, longname=False):
    TC_NAME_PREFIX = 'tests.'
    for tc in test_sets:
        full_name = os.path.join(tc_folder,
                                 tc["scriptpath"], tc["name"].split('.')[-1]+'.xml')
        if (longname):
            full_name = os.path.join(tc_folder,
                                     tc["scriptpath"], tc["name"] + '.xml')
        test_case = Element('TestCase')
        if tc["exec_type"] == "CTS":
            tcpath = ".".join(tc["scriptpath"].lower().split("/")) + "." + tc["name"]
            elements = {
                'Phase': 'CORE',
                'Type': 'FUNCTIONAL',
                'Domain': campaign,
                'UseCase':'MEDIA_INSTRUMENT',
                'Discription':tc["description"],
                'b2bIteration':'1',
                'b2bContinuousMode':'True',
                'TcExpectedResult':'PASS',
            }
            params = {
                'TEST_CASE':tcpath,
                'TEST_TIMEOUT': "600",
                'HpalmName': tc['name'],
            }
        else:
            tcpath = ".".join(tc["scriptpath"].split("/")) + "." + tc["name"]
            elements = {
                'Phase': 'CORE',
                'Type': 'FUNCTIONAL',
                'Domain': campaign,
                'UseCase':'PY_UNIT',
                'Discription':tc["description"],
                'b2bIteration':'1',
                'b2bContinuousMode':'True',
                'TcExpectedResult':'PASS',
            }
            params = {
                'TEST_DATA_ROOT':os.path.join('testplan', campaign),
                'TEST_CASE':TC_NAME_PREFIX + tcpath,
                'HpalmName': tc['name'],
            }
        for k, v in elements.items():
            subelem = SubElement(test_case, k)
            subelem.text = v

        parameters = SubElement(test_case, 'Parameters')
        for k, v in params.items():
            parameter = SubElement(parameters, 'Parameter')
            name_elem = SubElement(parameter, 'Name')
            name_elem.text = k
            value_elem = SubElement(parameter, 'Value')
            value_elem.text = v

        content = prettify(test_case)
        logging.debug("Write test case file: %s" % full_name)
        flush_content(full_name, content)
    logging.debug("Finished to write test case files.")

def generate_campaigns(test_sets, full_campaign_name, longname=False):
    logging.debug("Saving into campaign file: %s" %full_campaign_name)
    TC_PATH_PREFIX = '../../../../../TC/TP/TC/'
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
    for tc in test_sets:
        tc_name = tc["name"].split('.')[-1]
        if (longname):
            tc_name = tc["name"]
        tc_path = TC_PATH_PREFIX + os.path.join(tc["scriptpath"], tc_name)
        SubElement(testcases, 'TestCase', Id=tc_path)
    content = prettify(campaign)
    flush_content(full_campaign_name, content)
    logging.info("Finished to write campaign file.")

def get_test_set_folder_by_path(qc_conn, path, include_folders):
    rootpath = path
    path = path + '*' if path[-1] == "/" else path
    ps = path.split("/")
    last_test_set_folder_id = 0
    parent_str = ""
    for p in ps:
        rs = qc.do_get(qc.build_entity_url('test-folders'), \
                       'query={'+ parent_str +"name['" +urllib2.quote(p) +"']}&fields=id,name")
        if rs is None:
            logging.debug( 'Folder : %s Not found' % p)
            return None
        entities = ET.fromstring(rs)
        if not entities.attrib['TotalResults'] > 1:
            logging.debug( 'Found more than 1 folder: %s, first one will be used' % p)
        value = entities.findall('Entity/Fields/Field[@Name="id"]/Value')[0]
        parent_str = "parent-id[{0}];".format(value.text)
        last_test_set_folder_id = value.text

    rs = qc.do_get(qc.build_entity_url('test-folders'), \
                   'query={parent-id[%s]}&fields=id,name' % last_test_set_folder_id)
    if rs is None:
        logging.debug( 'No Test Set under this test set folder')
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

def get_filter_conditions(filter_fd):
    config = ConfigParser()
    config.optionxform = str
    with filter_fd:
        config.readfp(filter_fd)
    return config.items("query")

def update_catalog(test_sets, platform, os_version):
    CATALOG_PATH_PREFIX = "./"
    CATALOG_DEFAULT_NAME = "tc_catalog.xml"
    TC_PATH_PREFIX = "../../../../../TC/TP/TC/"
    #TEMPLATE_PATH_PREFIX = "_ExecutionConfig/OTC/TC/TP/support/plangenerator/"
    TEMPLATE_PATH_PREFIX = "OTC/CAMPAIGN/SystemFunctional/M/MR_0/TP/"
    catalog_filename = CATALOG_PATH_PREFIX + CATALOG_DEFAULT_NAME

    if os.path.exists(catalog_filename):
        try:
            catalog_tree = ET.parse(catalog_filename)
            testcases = catalog_tree.getroot()
        except Exception, ex:
            print ex
            raise Exception("Update test cases catalog failed, for catalog file incorrect")
    else:
        testcases = ET.Element("testcases")
        catalog_tree = ET.ElementTree(testcases)

    for testcase in test_sets:
        tc = testcases.find("testcase[@name='{name}']".format(name=testcase["name"]))
        if tc is None:
            tc_attr = {}
            tc_attr["domain"] = testcase["domain"]
            tc_attr["priority"] = testcase["priority"]
            tc_attr["testcategory"] = testcase["test_category"] if testcase["test_category"] else ""
            tc_attr["runner"] = testcase["exec_type"] if testcase["exec_type"] else ""
            # Translate the value - PyUnit to ACS
            if str.lower(tc_attr["runner"]) == "pyunit":
                tc_attr["runner"] = "ACS"
            tc_attr["ci_ready"] = testcase["ci_ready"] if testcase["ci_ready"] else ""
            tc_attr["template"] = TEMPLATE_PATH_PREFIX + testcase["domain"] + "_" + tc_attr["runner"] +"_template.xml"
            tc_attr["name"] = testcase["name"]
            tc_attr["tctype"] = testcase["tc_type"]
            if tc_attr["runner"] == "ACS":
                tc_attr["id"] = TC_PATH_PREFIX + os.path.join(testcase["scriptpath"], testcase["name"].split(".")[-1])
            else:
                tc_attr["id"] = testcase["name"]
                tc_attr["package"] = ""

            tc = ET.SubElement(testcases, "testcase", tc_attr)

            if testcase["bench_caps"]:
                bench_caps = tc.find("benchcapabilities")
                if bench_caps is None:
                    bench_caps = ET.SubElement(tc, "benchcapabilities")
                caps = testcase["bench_caps"].split(",")
                for cap in caps:
                    ET.SubElement(bench_caps, "benchcapability", {"name":cap})

            if testcase["areq"]:
                requirements = tc.find("requirements")
                if requirements is None:
                   requirements = ET.SubElement(tc, "requirements")
                for r in testcase["areq"]:
                    ET.SubElement(requirements, "requirement", {"name":r})

            targets = ET.SubElement(tc, "targets")
            target = ET.SubElement(targets, "target", {"version":os_version})
            ET.SubElement(target, "variant", {"name":platform})

        else:
            targets = tc.find("targets")
            if targets is None:
                targets = ET.SubElement(tc, "targets")
            target = targets.find("target[@version='{version}']".format(version=os_version))
            if target is None:
                target = ET.SubElement(targets, "target", {"version":os_version})
            variant = target.find("variant[@name='{platform}']".format(platform=platform))
            if variant is None:
                variant = ET.SubElement(target, "variant", {"name":platform})

            if testcase["bench_caps"]:
                bench_caps = tc.find("benchcapabilities")
                if bench_caps is None:
                    bench_caps = ET.SubElement(tc, "benchcapabilities")
                caps = testcase["bench_caps"].split(",")
                for cap in caps:
                    ET.SubElement(bench_caps, "benchcapability", {"name":cap})

    catalog_tree.write(catalog_filename, encoding="UTF-8", xml_declaration=True, method="xml")

def filter_ACS_testcase(test_sets):
    testcases = []
    for testcase in test_sets:
        if testcase["exec_type"] is None or testcase["exec_type"] == "" or str.lower(testcase["exec_type"]) == "acs" or str.lower(testcase["exec_type"]) == "pyunit":
            testcases.append(testcase)
    return testcases

dict_areq_covers = {}
def get_requirements():
    start_index = 1
    page_size = 2000
    print "Get requirements(AREQ)"
    #fetch requirements
    dict_areq = {}
    while True:
        query = "fields=id,name&page-size={page_size}&start-index={start_index}".format(page_size=page_size, start_index=start_index)
        rs = qc.do_get(qc.build_entity_url("requirements"), query)
        actual_page_size = 0
        if rs is not None:
            entities = ET.fromstring(rs).findall("Entity")
            for entity in entities:
                areq_id = entity.find("Fields/Field[@Name='id']/Value").text
                areq_name = entity.find("Fields/Field[@Name='name']/Value").text
                dict_areq[areq_id] = areq_name
            actual_page_size = len(entities)
        if actual_page_size < page_size:
            break
        start_index = start_index + page_size

    global dict_areq_covers
    start_index = 1
    print "Get requirement coverage"
    # fetch requirement coverages
    while True:
        query = "fields=test-id,requirement-id&page-size={page_size}&start-index={start_index}".format(page_size=page_size, start_index=start_index)
        rs = qc.do_get(qc.build_entity_url("requirement-coverages"),query)
        actual_page_size = 0
        if rs is not None:
            entities = ET.fromstring(rs).findall("Entity")
            for entity in entities:
                test_id = entity.find("Fields/Field[@Name='test-id']/Value").text
                areq_id = entity.find("Fields/Field[@Name='requirement-id']/Value").text
                if not test_id in dict_areq_covers:
                    dict_areq_covers[test_id] = []
                dict_areq_covers[test_id].append(dict_areq[areq_id])
            actual_page_size = len(entities)
        if actual_page_size < page_size:
            break
        start_index = start_index + page_size

class alm_filter:
    DEFAULT_BASEFOLDER = "Subject/SSG/System Functional Tests"
    def __init__ (self, component):
        if (component is not None):
            self.components = component.split(',')
        else:
            self.components = []
        self.fieldlist = []
        self.fieldmap = {}

    def handle_custom_filters(self, key, value, filterobj):
        if (key == 'BaseFolder'):
            matchobj = re.match(r'\[(.*)\]', value, re.M|re.I)
            if (matchobj):
                filterobj['basefolder'] = matchobj.group(1)
                filterobj['basefolder'].strip()
        elif (key == 'SubFolders'):
            matchobj = re.match(r'\[(.*)\]', value, re.M|re.I)
            if (matchobj):
                filterobj['include_folders'] = matchobj.group(1).split(' ')

    def get_include_folders(self, filterobj):
        res_folders = None
        if (len(self.components) > 0):
            res_folders = [folder for folder in filterobj['include_folders'] if folder in self.components]
        else:
            res_folders = filterobj['include_folders']
        if (len(res_folders) == 0 and filterobj['include_folders'] is not None):
            res_folders = filterobj['include_folders']
        return res_folders

    def get_root_folder(self, filterobj):
        return filterobj['basefolder']

    def pull_filedmapping(self, qc):
        rs = qc.do_get(qc.build_entity_url('customization/entities/test/fields'),\
                       "query={'name[*]'")
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
        query_group = []
        querylist = filterstr.split('union')
        for query in querylist:
            syntaxparser = query.split('{')
            query_body = None
            if (len(syntaxparser) > 1):
                query_body = query.split('{')[1].split('}')[0]
            else:
                query_body = query
            condition_list = query_body.split(';')
            filterobj = {}
            filterobj['basefolder'] = alm_filter.DEFAULT_BASEFOLDER
            filterobj['include_folders'] = []
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
                    self.handle_custom_filters(ckey[0], cvalue, filterobj)
            query_str = ';'.join(new_filter)
            filterobj['query_str'] = query_str
            query_group.append(filterobj)
        return query_group

def hanlder_args(args):
    parser = argparse.ArgumentParser(description="ACS Campaign Generator For HP ALM")
    parser.add_argument("--project", "-p",
                        type=str, dest="project",
                        required=True, help="Specify HP ALM project name")
    parser.add_argument("--domain", type=str, required=True,
                        dest="domain", default="SSG",
                        help="Specify HP ALM Domain")
    parser.add_argument("--user", "-u",
                        type=str, dest="user", required=True,
                        help="HP ALM user account name")
    parser.add_argument("--password", "-P",
                        type=str, dest="password",
                        required=True, help="Password")
    parser.add_argument("--filterfile", "-f",
                        type=file, dest="filterfile",
                        required=True, help="Test case filter condition file")
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
    parser.add_argument("-k", "--catalog", required=False, default=False, action="store_true", dest="catalog", help="Generate catalog file")

    args = parser.parse_args(args)
    return args

if __name__ == "__main__":
    args = hanlder_args(sys.argv[1:])
    filter_conditions = get_filter_conditions(args.filterfile)
    qc = qc_connector(domain=args.domain, project=args.project)
    almqtyfilter = alm_filter(args.component)
    exit_code = 0
    try:
        qc.connect()
        if not qc.login(args.user, args.password):
            raise Exception("HP ALM Login Failed.")
        qc.open_session()
        almqtyfilter.pull_filedmapping(qc)

        ##get_requirements()

        for board, query in filter_conditions:
            logging.info("Processing campaigns for: %s"%board)
            if board[-2] == "_":
                platform = board[0:-2]
                version = board[-1]
            else:
                platform = board
                version = "M"

            campaign_tcs = {}
            query_group = almqtyfilter.parsing_filter_string(query)
            for query in query_group:
                include_folders = almqtyfilter.get_include_folders(query)
                root = almqtyfilter.get_root_folder(query)
                campaign_folders =  get_test_set_folder_by_path(qc, root, include_folders)
                for folder in campaign_folders:
                    fullfolders = []
                    fullfolders.append(folder)
                    subfolders = pull_full_folders(qc, folder["id"], \
                                                   root+"/"+folder["name"], \
                                                   folder["name"])
                    fullfolders = fullfolders + subfolders
                    logging.info("Processing campaign folder: %s"%folder["name"])
                    # Get the test case list for given campaign folder
                    #logging.info( "Start to pull test case list ...")
                    test_sets = pull_full_test_cases(folder, fullfolders, query['query_str'])
                    if args.catalog:
                        update_catalog(test_sets, platform, version)

                    test_sets = filter_ACS_testcase(test_sets)
                    if (folder["name"] in campaign_tcs.keys()):
                        campaign_tcs[folder["name"]]["test_sets"] = \
                            merge_test_sets(campaign_tcs[folder['name']]['test_sets'], test_sets)
                        #campaign_tcs[folder["name"]]["test_sets"] = test_sets + \
                        #    campaign_tcs[folder["name"]]["test_sets"]
                    else:
                        campaign_tcs[folder["name"]] = {"test_sets":test_sets}

            for campaign in campaign_tcs.keys():
                if (len(campaign_tcs[campaign]['test_sets']) > 0):
                    campaign_name = "%s.%s" % (campaign, board)
                    if (args.tag and args.tag != ''):
                        campaign_name = '%s.%s.%s' % (campaign, args.tag, board)
                    full_campaign_name = os.path.join(args.dest, 'CAMPAIGN', campaign_name + '.xml')
                    # Generate ACS campaign files ...
                    logging.info("Generating ACS campaign files ...")
                    generate_campaigns(campaign_tcs[campaign]['test_sets'], full_campaign_name, args.longname)
                    tc_folder = os.path.join(args.dest, 'TC')
                    # Generate ACS test case xml files ...
                    logging.info( "Generating ACS test case xml files ...")
                    generate_tc_files(campaign_tcs[campaign]['test_sets'], tc_folder, campaign, args.longname)
                else:
                    logging.warn("Skil campaign generation for : %s due to no matched test cases found." % campaign)

            for campaign in campaign_tcs.keys():
                if (len(campaign_tcs[campaign]['test_sets']) == 0):
                    logging.warn("Skiped campaign: %s due no matched test cases found." % campaign)

    except Exception, ex:
        logging.error(str(ex))
        exit_code = 1

    finally:
        qc.close_session()
        qc.logout()
        qc.close()

    exit(exit_code)
