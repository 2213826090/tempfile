#!/usr/bin/env python
"""
pre-manifest to CAMPAIGN and TC
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

SECTION_NAME = 'tests'

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
    parser.add_argument("input_path",
                        type=str,)
    parser.add_argument("-d", "--dest",
                        type=str,
                        required=False,
                        default='../../',
                        help="dest top folder. (default is ../../)",)
    return parser.parse_args()


def parse_config(fd):
    """return case name list
    """
    config = ConfigParser()
    config.optionxform = str
    with fd:
        config.readfp(fd)
    items = config.items(SECTION_NAME)
    return [item[0] for item in items]

def prettify(elem):
    """return a pretty-printed xml string for the etree"""
    rough_string = tostring(elem, 'utf-8')
    reparsed = parseString(rough_string)
    return reparsed.toprettyxml(indent='    ')


def dump_campaign(file_path, cases):
    """dump campaign xml file"""
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
    ]
    for kv in attributes:
        SubElement(parameters, 'Parameter', attrib=kv)
    testcases = SubElement(campaign, 'TestCases')
    for case in cases:
        SubElement(testcases, 'TestCase', Id=case)

    content = prettify(campaign)
    flush_content(file_path, content)


def dump_testcase(file_path, module, case):
    """dump testcase xml file"""
    test_case = Element('TestCase')
    elements = {
        'UseCase':'PY_UNIT',
        'Discription':"Pre-request test case",
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
        'TEST_CASE':case,
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


def convert_to_case_path(case_name, prefix, module):
    return os.path.join(prefix, module, case_name)


def process_pretest(module, OPTIONS):
    manifest_pretest = os.path.join(OPTIONS.input_path, module, 'manifest_pretest')
    with open(manifest_pretest, 'r') as fd:
        case_names = parse_config(fd)
    pre_fix = '../../../../../TC/TP/TC/'
    case_paths = [convert_to_case_path(case_name, pre_fix, module) for case_name in case_names]

    campaign_full_path = os.path.join(OPTIONS.dest, 'CAMPAIGN', module + '_pretest.xml')
    dump_campaign(campaign_full_path, case_paths)
    case_full_names = ['.'.join(['tests', case_name]) for case_name in case_names]
    for case_full_name in case_full_names:
        full_path = os.path.join(OPTIONS.dest,
                                'TC',
                                module,
                                '.'.join(case_full_name.split('.')[1:])+'.xml')

        dump_testcase(full_path, module, case_full_name)

if __name__ == '__main__':
    OPTIONS = parse_args()
    info("Dest       : " + OPTIONS.dest)
    info("Input path : " + str(OPTIONS.input_path))

    modules = [
       "EnergyManagement_CPS",
       "EnergyManagement_cutter1",
       "EnergyManagement_cutter2",
       "EnergyManagement_cutter3",
       "IRDA_Auto_Detect",
       "Multimedia_Audio",
       "Multimedia_Video",
       "Multimedia_Camera",
       "Multimedia_Image",
       "Graphics_Display",
       "Graphics_RenderApp",
       "Graphics_System",
       "IRDA_OEM_Customization",
       "GOTA",
       "Sensor",
       "AfW",
    ]

    for module in modules:
        process_pretest(module, OPTIONS)
