#!/usr/bin/env python
"""
Convert to ACS CAMPAIGN and TC xml files by module and platform
"""

import os
import sys
import json
import os.path
import requests
import argparse
from xml.etree.ElementTree import *
from xml.dom.minidom import parseString
from ConfigParser import ConfigParser

SECTION_NAME = 'query'
HEADER_STR = '[tests]\n'
ROOT_URL = 'https://et03.devtools.intel.com/api/'
SCRIPTS = 'scripts?'
SCRIPTPACKAGES = 'scriptpackages?'
SUB_ROOT_NAME = 'Script Library - System Functional Test'

def error(text):
    """print error msg"""
    print "\033[1;31m[ERR]%s\033[0m" % text


def info(text):
    """print info msg"""
    print "\033[1;32m%s\033[0m" % text


def parse_args():
    """parse parameters"""
    parser = argparse.ArgumentParser(
                        description="generates ACS CAMPAIGN and TC XML files \
                        according to TQL config file")
    parser.add_argument("input_file",
                        type=file,)
    parser.add_argument("-d", "--dest",
                        type=str,
                        required=False,
                        default='../../',
                        help="dest top folder. (default is ../../)",)
    parser.add_argument("-s", "--script-path",
                        type=str,
                        required=True,
                        help='path to oat-project-root, MUST be absolute path',)
    parser.add_argument("-u", "--user",
                        type=str,
                        required=True,
                        help="ET user name",)
    parser.add_argument("-p", "--passwd",
                        type=str,
                        required=True,
                        help="password",)
    parser.add_argument("-c", "--component",
                        type=str,
                        required=False,
                        default='',
                        help="Only generating Campaign/TC for given component",)
    parser.add_argument("-l", "--longname",
                        required=False,
                        default=False,
                        action="store_true",
                        dest="longname",
                        help="Generate long test case xml file name",)

    parser.add_argument("-t", "--tag",
                        type=str,
                        required=False,
                        default='',
                        help="Tag string added into campaign file name",)
    return parser.parse_args()


def parse_config(fd):
    """Parse configuration file content to TQL list per component
    config file is TQL list per platform
    """
    config = ConfigParser()
    config.optionxform = str
    with fd:
        config.readfp(fd)
    return config.items(SECTION_NAME)


class KeyValue(object):
    """Key-Value like object"""
    def __init__(self, key_values):
        super(KeyValue, self).__setattr__('attr', key_values)

    def __getattr__(self, name):
        """return given script attr value"""
        return self.attr[name] if name in self.attr else None

    def __setattr__(self, name, value):
        """set given value to attr"""
        super(KeyValue, self).__setattr__(name, value)


class ScriptPackage(KeyValue):
    """ScriptPackage data structure"""
    def __init__(self, key_values):
        super(ScriptPackage, self).__setattr__('attr', key_values)
        if self.Links is not None:
            links = []
            for link in self.Links:
                links.append(KeyValue(link))
            self.Links = links
        self.Parent = None
        self.Children = []

    def is_parent(self, other):
        """return if is the parent of 'other'"""
        for link in other.Links:
            if self.Self == link.Href and link.Rel == 'Parent':
                return True
        return False

    def full_package_name(self):
        """return dot seperated full package name from root till self"""
        names = [self.Name]
        parent = self.Parent
        while parent:
            names.insert(0, parent.Name)
            parent = parent.Parent
        return '.'.join(names)

    def ancestor_package(self):
        """return ancestor package"""
        if self.Parent is None:
            return self
        return self.Parent.ancestor_package()

def get_all_packages(user_name, passwd):
    """get all ScriptPackage object list"""
    url = '%s%s$Top=1' % (ROOT_URL, SCRIPTPACKAGES)
    res = requests.get(url, auth=(user_name, passwd))
    count = res.json()['Total']
    url = '%s%s$Top=%d' % (ROOT_URL, SCRIPTPACKAGES, count)
    res = requests.get(url, auth=(user_name, passwd))
    items = res.json()['Items']
    return [ScriptPackage(item) for item in items]


def construct_package_tree(script_packages):
    """iterate all script packages and set their Parent and Children"""
    total = len(script_packages)
    for i in xrange(total):
        me = script_packages[i]
        for j in xrange(i + 1, total):
            other = script_packages[j]
            if me.is_parent(other):
                me.Children.append(other)
                other.Parent = me
            if other.is_parent(me):
                other.Children.append(me)
                me.Parent = other


def cut_sub_tree(script_packages, root_name, options):
    """cut sub trees from node whose name is root_name"""
    for node in script_packages:
        parent = node
        while parent:
            if parent.Name == root_name:
                break
            parent = parent.Parent
        else:
            script_packages.remove(node)

    sub_roots = None

    for node in script_packages:
        if node.Name == root_name:
            project_link = None
            for link in node.Links:
                if (link.Rel == 'Project'):
                    project_link = link.Href

            if (project_link):
                print "project_link: %s" % project_link
                res = requests.get(project_link, auth=(options.user, options.passwd))
                projectname = res.json()['Name']
                if (projectname == 'OTC Android Testing'):
                    sub_roots = node.Children
                    script_packages.remove(node)
                    node = None
                    break

    assert sub_roots

    for node in sub_roots:
        node.Parent = None
    return sub_roots


def get_scripts(tql, user_name, passwd, component=None):
    """get all scripts according to given tql"""
    comp_tql = tql
    if (component and component != ''):
        complist = ["\"%s\"" % comp for comp in component.split(',')]
        comptrans = ','.join(complist)
        comp_tql = '%s AND AllPackages IN [%s]' % (tql, comptrans)
    count_tql = '%s%s$top=1&%s' % (ROOT_URL, SCRIPTS, comp_tql)
    print "count_tql: %s" % count_tql
    res = requests.get(count_tql, auth=(user_name, passwd))
    res.encoding = 'iso-8859-1'
    count = res.json()['Total']
    unit = 1000
    dpair = divmod(count, unit)
    times = dpair[0] + 1 if dpair[1] else dpair[0]
    scripts = []
    for i in xrange(times):
        try:
            seq_tql = '%s%s$skip=%d&$top=%d&tql=%s' % (
                        ROOT_URL, SCRIPTS, i * unit, unit, comp_tql)
            print "seq_tql: %s" % seq_tql
            res = requests.get(seq_tql, auth=(user_name, passwd))
            seg = [KeyValue(item) for item in res.json()['Items']]
            scripts.extend(seg)
        except Exception, err:
            error(str(err))
            sys.exit(0)
    return scripts


def prettify(elem):
    """return a pretty-printed xml string for the etree"""
    rough_string = tostring(elem, 'utf-8')
    reparsed = parseString(rough_string)
    return reparsed.toprettyxml(indent='    ')


def dump_campaign(file_path, scripts, longname=False):
    """dump campaign xml file"""
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
    for script in scripts:
        tc_name = script.Name.split('.')[-1]
        if (longname):
            tc_name = script.Name
        tc_path = TC_PATH_PREFIX + os.path.join(script.full_package_name, tc_name)
        SubElement(testcases, 'TestCase', Id=tc_path)

    content = prettify(campaign)
    flush_content(file_path, content)


def dump_testcase(file_path, module, script_path, script):
    """dump testcase xml file"""
    TC_NAME_PREFIX = 'tests.'
    test_case = Element('TestCase')
    elements = {
        'UseCase':'PY_UNIT',
        'Discription':script.Description,
        'b2bIteration':'1',
        'b2bContinuousMode':'True',
        'TcExpectedResult':'PASS',
    }
    for k, v in elements.items():
        subelem = SubElement(test_case, k)
        subelem.text = v
    parameters = SubElement(test_case, 'Parameters')
    params = {
        'TEST_DATA_ROOT':os.path.join('testplan', module),
        'TEST_CASE':TC_NAME_PREFIX + script.full_name,
    }
    for k, v in params.items():
        parameter = SubElement(parameters, 'Parameter')
        name_elem = SubElement(parameter, 'Name')
        name_elem.text = k
        value_elem = SubElement(parameter, 'Value')
        value_elem.text = v

    content = prettify(test_case)
    flush_content(file_path, content)


def flush_content(file_path, content):
    """dump contents to specified file"""
    info("start to create %s" % file_path)
    try:
        if os.path.exists(file_path):
            info('File already exists, overwrite!')
        if not os.path.isdir(os.path.dirname(file_path)):
            os.makedirs(os.path.dirname(file_path))
        with open(file_path, 'w') as fd:
            fd.write(content.encode('UTF-8'))
    except Exception, err:
        error("Write %s file error(%s)" % (file_path, str(err)))

if __name__ == '__main__':
    OPTIONS = parse_args()
    info("Dest       : " + OPTIONS.dest)
    info("Input file : " + str(OPTIONS.input_file))
    info("User name  : " + OPTIONS.user)
    info("Script path: " + OPTIONS.script_path)

    tqls = parse_config(OPTIONS.input_file)
    script_packages = get_all_packages(OPTIONS.user, OPTIONS.passwd)
    construct_package_tree(script_packages)
    sub_roots = cut_sub_tree(script_packages, SUB_ROOT_NAME, OPTIONS)
    script_packages = {item.Id:item for item in script_packages}
    for platform, tql in tqls:
        info("Processing %s %s" % (platform, tql))
        scripts = get_scripts(tql, OPTIONS.user, OPTIONS.passwd, OPTIONS.component)
        module_cases = {node.Name:[] for node in sub_roots}
        for script in scripts:
            package = script_packages[script.PackageId]
            script.full_name = '.'.join([package.full_package_name(), script.Name])
            script.full_package_name = '/'.join(package.full_package_name().split('.'))
            key = package.ancestor_package().Name
            module_cases[key].append(script)

        for module, scripts in module_cases.items():
            if not scripts: # not dump empty one
                continue
            module_platform = "%s.%s" % (module, platform)
            if (OPTIONS.tag and OPTIONS.tag != ''):
                module_platform = '%s.%s.%s' % (module, OPTIONS.tag, platform)
            full_name = os.path.join(OPTIONS.dest, 'CAMPAIGN', module_platform + '.xml')
            dump_campaign(full_name, scripts, OPTIONS.longname)
            for script in scripts:
                full_name = os.path.join(OPTIONS.dest,
                                         'TC',
                                         script.full_package_name, script.Name.split('.')[-1]+'.xml')
                if (OPTIONS.longname):
                    full_name = os.path.join(OPTIONS.dest,
                                             'TC',
                                             script.full_package_name, script.Name+'.xml')
                dump_testcase(full_name, module, OPTIONS.script_path, script)
