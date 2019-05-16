#! /usr/bin/env python
# coding:utf-8

import unittest
from testlib.em.em_impl import EMImpl

class CheckAritifactory(unittest.TestCase):
    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.emImpl = EMImpl()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name

    def test_check_artifactory(self):
        print "[RunTest]: %s" % self.__str__()
        self.emImpl.download_artifactory_content("long_music")

