# -*- coding:utf-8 -*-
import unittest
import urllib2
import re
import os
from testlib.util.config import TestConfig


REPO_URL = os.path.join(
    TestConfig().getConfValue(section="artifactory", key="location"),
    "../latest/")
PKG_NAME = "otcqa-instrumentation"


class MediaInstrumentTest(unittest.TestCase):
    repo_url = REPO_URL
    pkg_dir = "/tmp/%s/" % PKG_NAME
    pkg_regexp = r'href="(%s_.*.tar.gz)"' % PKG_NAME

    def setUp(self):
        super(MediaInstrumentTest, self).setUp()
        self._deploy()

    def tearDown(self):
        super(MediaInstrumentTest, self).tearDown()

    def _deploy(self):
        if os.path.isdir(self.pkg_dir):
            os.system("rm -rf " + self.pkg_dir)
        f = urllib2.urlopen(self.repo_url)
        # find the packages
        urls = []
        for l in f.readlines():
            m = re.search(self.pkg_regexp, l)
            if m:
                urls.append(self.repo_url + m.groups()[0])
        # download packages
        for url in urls:
            local_path = os.path.join('/tmp/', os.path.basename(url))
            print "Download %s to %s" % (url, local_path)
            cmd = "wget -q --no-check-certificate %s -O %s" % (url, local_path)
            os.system(cmd)
            os.system("tar -xvf %s -C /tmp" % local_path)
            os.unlink(local_path)  # delete if uncompress done

    def testRunMediaInstrumentTestSuite(self):
        cmd = " ".join(["python %s/script/main.py" % self.pkg_dir,
                        "-r %s/result" % self.pkg_dir,
                        "-d %s" % self.pkg_dir])
        os.system(cmd)
