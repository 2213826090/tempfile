#!/usr/bin/python
# -*- coding: utf-8 -*-
import sys
import json
import requests
import traceback
import tempfile
import os
import re

CTI_SERVER_URL = 'http://cti-bridge.sh.intel.com'
_loadedTestRunnerVariables = None

def loadTestRunnerVariables():
    test_runner_variables_file = "test_runner_variables.json"
    # New location
    conf_file = os.path.expanduser("~", ".buildbot_test_runner", test_runner_variables_file)
    if not os.path.isfile(conf_file):
        # Old location
        conf_file = os.path.expanduser(tempfile.gettempdir(), test_runner_variables_file)
        if not os.path.isfile(conf_file):
            return {}

    with open(conf_file, 'r') as f:
        return json.load(f)

def getTestRunnerVariables():
    global _loadedTestRunnerVariables
    if _loadedTestRunnerVariables is None:
        _loadedTestRunnerVariables = loadTestRunnerVariables()
    return _loadedTestRunnerVariables

def getVariable(variableName):
    return getTestRunnerVariables().get(variableName)

def getImageBuildId():
    url_buildinfo = getVariable('url_buildinfo')
    image_BuildId = url_buildinfo.split('/')[-2]
    return image_BuildId

def getImageSource():
    url_buildinfo = getVariable('url_buildinfo')
    board_name = getVariable('board_type')
    target_name = getVariable('build_target')
    try:
        os.system('rm -rf /tmp/buildinfo.json')
    except Exception as e:
        append_OUTPUT('Fail to delete buildinfo.json')
    os.system('wget %s -O /tmp/buildinfo.json' %url_buildinfo)
    if os.path.isfile('/tmp/buildinfo.json'):
        with open('/tmp/buildinfo.json', 'r') as f:
            runner_data = json.load(f)
    else:
        print 'No buildinfo.json file list'
    image_url = runner_data['hardwares'][board_name]['variants']['%s-userdebug' %target_name]['flashfiles']['fastboot'][0]
    image_source = url_buildinfo.strip('/build_info.json') + image_url
    return image_source

def submitSession():
    source_path = getImageSource()
    build_id = getImageBuildId()
    campaign_name = getVariable('campaign_name')

    data = {'imageSource':source_path.encode('utf-8'), 'imageBuildId':build_id.encode('utf-8'), 'campaignName': campaign_name.encode('utf-8')}
    headers = {'Content-type':'application/json', 'Accept':'application/json'}
    url = '{0}/api/sessions'.format(CTI_SERVER_URL)
    ret = requests.post(url, json.dumps(data), headers=headers)

    if 201 != ret.status_code:                                                    # 201 - indicates success invocation.
        sys.stderr.write('[Error]status_code: {0}\n'.format(ret.status_code))     # Other possible status_codes: 400-parameter missing;500-general invocation failure
        sys.stderr.write('[Error]:{0}\n'.format(ret.text))                        # You may check the error message by ret.text
        return (FAILURE, 'Get wrong invocation')
    else:
        return (SUCCESS, 'No errors')

#if '__main__' == __name__:
(VERDICT,OUTPUT) = submitSession()
