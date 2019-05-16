#!/usr/bin/env python
"""
Convert to OAT test plan by module and platform
"""

import os
import sys
import os.path
import requests
import argparse
from ConfigParser import ConfigParser

SECTION_NAME = 'query'
HEADER_STR = '[tests]\n'
ROOT_URL = 'https://et03.devtools.intel.com/api/'
SCRIPTS = 'scripts?'
SCRIPTPACKAGES = 'scriptpackages?'


def error(text):
    """print error msg"""
    print "\033[1;31m[ERR]%s\033[0m" % text

def info(text):
    """print info msg"""
    print "\033[1;32m%s\033[0m" % text

def parse_args():
    """parse parameters"""
    parser = argparse.ArgumentParser(
                        description="generates OAT testplan files \
                        according to TQL config file")
    parser.add_argument("input_file",
                        type=file,)
    parser.add_argument("-d", "--dest",
                        type=str,
                        required=False,
                        default='../../testplan',
                        help="dest top folder. (default is ../../testplan)",)
    parser.add_argument("-u", "--user",
                        type=str,
                        required=True,
                        help="ET user name",)
    parser.add_argument("-p", "--passwd",
                        type=str,
                        required=True,
                        help="password",)
    parser.add_argument("-f", "--postfix",
                        type=str,
                        required=False,
                        default='',
                        help="postfix of output test plan file name (Ex: _reliability)",)
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

def retrieve_script_packages(user, passwd):
    """get leaf packages id-full packages name dict"""
    url = '%s%s$Top=1' % (ROOT_URL, SCRIPTPACKAGES)
    res = requests.get(url, auth=(user, passwd))
    count = res.json()['Total']
    url = '%s%s$Top=%d' % (ROOT_URL, SCRIPTPACKAGES, count)
    res = requests.get(url, auth=(user, passwd))
    items = res.json()['Items']
    full_packages = {item['Id']:item for item in items}

    #def is_leaf(package):
    #    self_id = package['Id']
    #    for _, package in full_packages.items():
    #        links = package['Links']
    #        for link in links:
    #            if link['Rel'] == 'Parent' and \
    #               self_id in link['Href']:
    #               return False
    #    return True

    def is_root(package):
        """check if root package"""
        for link in package['Links']:
            if link['Rel'] == 'Parent':
                return False
        return True

    def parent_package(package):
        """get parent package"""
        for link in package['Links']:
            if link['Rel'] == 'Parent':
                parent_id = link['Href'].split('/')[-1]
                return full_packages[parent_id]
        #Root pacakge only has project
        return None

    def get_full_package_names(package, names):
        """retieve full package name list for a leaf package"""
        names[:0] = [package['Name']]
        if is_root(package):
            return names
        parent = parent_package(package)
        return get_full_package_names(parent, names)

    full_package_names = {}
    for _, package in full_packages.items():
        package_names = get_full_package_names(package, [])
        full_package_names[package['Id']] = '.'.join(package_names)

    return full_package_names

def execute_tql(tql, user, passwd):
    """Execute tql and return result
       result is a dict with below format
       Graphics : [case_name, ...]
       Multimedia_Audio : [case_name, ...]
       Multimedia_Camera : [case_name, ...]
       Multimedia_Image : [case_name, ...]
       Multimedia_Media : [case_name, ...]
       Multimedia_Audio : [case_name, ...]
       ...
    """
    full_package_names = retrieve_script_packages(user, passwd)

    count_tql = '%s%s$top=1&%s' % (ROOT_URL, SCRIPTS, tql)
    res = requests.get(count_tql, auth=(user, passwd))
    res.encoding = 'iso-8859-1'
    count = res.json()['Total']
    unit = 1000
    dpair = divmod(count, unit)
    times = dpair[0] + 1 if dpair[1] else dpair[0]
    items = []
    for i in xrange(times):
        try:
            seq_tql = '%s%s$skip=%d&$top=%d&tql=%s' % (
                        ROOT_URL, SCRIPTS, i * unit, unit, tql)
            res = requests.get(seq_tql, auth=(user, passwd))
            tmp = res.json()['Items']
            items.extend(tmp)
        except Exception, err:
            error(str(err))
            sys.exit(0)

    result = {}
    for _, full_package_name in full_package_names.items():
        if full_package_name.split('.')[0] != \
            'Script Library - System Functional Test':
            continue
        if full_package_name == 'Script Library - System Functional Test':
            continue
        root_module_name = full_package_name.split('.')[1]
        if root_module_name not in result:
            result[root_module_name] = []

    for item in items:
        package_id = item['PackageId']
        full_package_name = full_package_names[package_id]
        full_package_name = '.'.join(full_package_name.split('.')[1:])
        full_case_name = '.'.join([full_package_name, item['Name']])
        root_module_name = full_package_name.split('.')[0]
        if root_module_name not in result:
            continue
        result[root_module_name].append(full_case_name)

    return result

def dump_test_plan(file_path, case_names):
    """dump test plan contents to test plan file"""
    info("start to create %s" % file_path)
    try:
        if os.path.exists(file_path):
            info('File already exists, overwrite!')
        if not os.path.isdir(os.path.dirname(file_path)):
            os.makedirs(os.path.dirname(file_path))
        with open(file_path, 'w') as fd:
            fd.write(HEADER_STR)
            for case_name in case_names:
                fd.write(case_name.encode('UTF-8') + " = 1\n")
    except Exception, err:
        error("Write %s file error(%s)" % (file_path, str(err)))

if __name__ == '__main__':
    OPTIONS = parse_args()
    info("Dest       : " + OPTIONS.dest)
    info("Input file : " + str(OPTIONS.input_file))
    info("User name  : " + OPTIONS.user)

    tqls = parse_config(OPTIONS.input_file)
    for platform, tql in tqls:
        info("Processing %s %s" % (platform, tql))
        module_cases = execute_tql(tql, OPTIONS.user, OPTIONS.passwd)

        for module, case_names in module_cases.items():
            if not case_names:
                continue
            name = 'manifest%s.%s' % (OPTIONS.postfix, platform)
            full_name = os.sep.join([OPTIONS.dest, module, name])
            dump_test_plan(full_name, case_names)
