# coding: UTF-8
'''
Created on Sep 4, 2017

@author: Li Zixi
'''
import re
import sys
import time
import base64
import json
import requests
from requests.auth import HTTPBasicAuth
# from testlib.util.log import Logger
import logging
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger()
# logger = Logger.getlogger()


# name = base64.decodestring(name)
# password = base64.decodestring(password)
auth = (name, password)
logger.info("auth=%s" % str(auth))

'''
e.g. :
    payload = {'forcescheduler': 'force_engineering_for_integration_tests_one_android',
               'manifest_override_url': 'https://artidevsh.sh.intel.com/artifactory/cactus-absp-dev_shankang-sh/bisect-for_integration_tests_one_android/5/bisect_1/manifests-generated.json',
               'force_build_changeids': '100\n101',
               'preferred_site': 'sh',
               'reason': 'your reason string',
               'variant_list': ['userdebug', 'user'],
               'target_products_to_build': ['r1_target1', 'r1_target2'],
               'download_list_computation_mode': 'reorder without downloading other patches',
               'checkbox': ['check_external', 'cherry_pick_from_other_gerrit']}
'''

class BuildbotHelper:
    def __init__(self):
        self.url = "https://buildbot.sh.intel.com/absp/builders/bxtp_ivi_m-engineering/force"
        self.payload = {'forcescheduler': 'force_engineering_bxtp_ivi_m',
                   'manifest_override_url': 'https://mcg-depot.intel.com/artifactory/cactus-absp-jf/build/eng-builds/bxtp_ivi_m/PSI/weekly/2017_WW34/manifests-generated.json',
                   'force_build_changeids': '597217',
                   'preferred_site': 'sh',
                   'reason': 'your reason string',
                   'variant_list': ['userdebug'],
                   'target_products_to_build': ['r0_bxtp_abl'],
                   'download_list_computation_mode': 'reorder without downloading other patches'
                   }

    def getFailureReason(self, data):
        l = data.split("\n")
        errString = ""
        output = False
        for s in l:
            if "<pre class=\"alert\">" in s:
                output = True
                continue
            if s.strip() == "</pre>" and output==True:
                break
            if output:
                errString += s
        return errString

    def triggerEBBuilder(self, url="", payload=""):
        if url == "":
            url = self.url
        if payload == "":
            payload = self.payload
        logger.info("url=%s, payload=%s" % (url, str(payload)))
        r = requests.post(url = url,auth=HTTPBasicAuth(auth[0], auth[1]),
                       data=payload,
                       verify=False)
        if "cancel request" in r.text:
            logger.info("Trigger successfully!")
        else:
            fail_reason = self.getFailureReason(r.text)
            logger.info("Trigger failed! As {}".format(fail_reason))
        assert r.reason == 'OK', "Failed to trigger the merge bridge, because %s" %(r.reason)

    def findTriggeredCampaignBuild(self, buildbot_url, builder_name, user,
                                           password, reason):
        url = "{}/json/builders/{}/builds/_all?as_text=1".format(buildbot_url,
                                                                 builder_name)
        logger.info("Trying to find the specified build...")
        r = requests.get(url,
                     auth=HTTPBasicAuth(user, password),
                     verify=False)
        data = json.loads(r.text)
        last_cached_builds = set([])
        while True:
            cached_builds = list(set(data.keys()) - last_cached_builds)
            logger.info("cached_builds:{}".format(cached_builds))
            for id in cached_builds:
                url = "{}/json/builders/{}/builds/{}?as_text=1".\
                    format(buildbot_url, builder_name, id)
                r = requests.get(url,
                                 auth=HTTPBasicAuth(user, password),
                                 verify=False)
                results = json.loads(r.text)
                if reason in results['reason']:
                    return id
            last_cached_builds = last_cached_builds | set(cached_builds)
            time.sleep(5)
            url = "{}/json/builders/{}/builds/_all?as_text=1".format(buildbot_url,
                                                                     builder_name)
            logger.info("Trying to find the specified build...")
            r = requests.get(url,
                         auth=HTTPBasicAuth(user, password),
                         verify=False)
            data = json.loads(r.text)

    def checkBuildIsCompleted(self, buildbot_url, builder_name, id, user, password):
        url = "{}/json/builders/{}/builds/{}?as_text=1".\
            format(buildbot_url, builder_name, id)
        r = requests.get(url,
                         auth=HTTPBasicAuth(user, password),
                         verify=False)
        results = json.loads(r.text)
        result = None
        try:
            logger.info("build id:{} result:{}".format(id, results['text']))
            if 'failed' not in results['text'] and \
                    'exception' not in results['text']:
                result = "Success"
            else:
                result = "Fail"
        except:
            logger.info("build id:{} Unfinished".format(id))
        return result

if __name__ == "__main__":
    build_manifest_url = "https://mcg-depot.intel.com/artifactory/cactus-absp-jf/build/eng-builds/bxtp_ivi_m/PSI/weekly/2017_WW34/manifests-generated.json"
    get_url_date_parttern = re.compile(r"/(20.._WW.*)/")
    reason = get_url_date_parttern.findall(build_manifest_url)[0] + " camera scaling add patch"

    buildbot_helper = BuildbotHelper()
    buildbot_helper.payload["reason"] = reason
    buildbot_helper.payload["manifest_override_url"] = build_manifest_url
#     buildbot_helper.triggerEBBuilder()
    t_id = buildbot_helper.findTriggeredCampaignBuild("https://buildbot.sh.intel.com/absp", "bxtp_ivi_m-engineering", auth[0], auth[1], reason)
    logger.info("t_id:%s" % str(t_id))
    t_id = 10806
    result = ""
    for i in range(10):
        t_result = buildbot_helper.checkBuildIsCompleted("https://buildbot.sh.intel.com/absp", "bxtp_ivi_m-engineering", t_id, auth[0], auth[1])
        if t_result == "Success" or t_result == "Fail":
            result = t_result
            break
        time.sleep(10)
    if result == "":
        logger.info("checkBuildIsCompleted timeout!")
    elif result == "Fail":
        logger.info("Build failed!")
