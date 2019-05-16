#! /usr/bin/env python
# coding:utf-8

import unittest
import os
from testlib.em.tools import ADBTools
from testlib.em.constants_def import BXT_O
from testlib.util.process import shell_command

class ART(unittest.TestCase):
    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.adb = ADBTools()
        self.adb.adb_root()
        self.dirname = os.path.dirname(os.path.abspath(__file__))
        build = self.adb.get_build_version()
        if build.startswith("8"):
            self.art_package = "bundle_OTC_20180410"
            self.art_package_zip = "bundle_OTC_20180410_for_AFT.zip"
        else:
            self.art_package = "bundle_OTC_20160620"
            self.art_package_zip = "bundle_OTC_20160620.zip"

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name

    def check_package_download(self):
        art_package_root = os.path.join(self.dirname, self.art_package)
        if not os.path.exists(art_package_root):
            from testlib.util.repo import Artifactory
            arti_obj = Artifactory("https://mcg-depot.intel.com/artifactory/acs_test_artifacts/OTC_Android_Auto_Test_Suite/resources")
            ret_file = arti_obj.get("ART/%s" % self.art_package_zip)
            cmd = "unzip -q %s -d %s" % (ret_file, self.dirname)
            print cmd
            shell_command(cmd, 180)
            assert os.path.exists(art_package_root), "Download ART package failed!"

    def run_art(self, test_case, timeout_second = 3 * 3600):
        self.check_package_download()
        script_path = os.path.join(self.dirname, "common_run.sh")
        cmd = "%s %s %s | grep PASSED" % (script_path, self.art_package, test_case)
        print cmd
        exit_code_int, result_array = shell_command(cmd, timeout_second)
        #print exit_code_int, result_array[0]
        assert exit_code_int == 0
        assert result_array

    def test_ARTtests(self):
        print "[RunTest]: %s" % self.__str__()
        self.run_art("ARTtests")

    def test_apache_commons_cli(self):
        print "[RunTest]: %s" % self.__str__()
        self.run_art("apache.commons.cli")

    def test_apache_commons_codec(self):
        print "[RunTest]: %s" % self.__str__()
        self.run_art("apache.commons.codec")

    def test_apache_commons_collections_primitives(self):
        print "[RunTest]: %s" % self.__str__()
        self.run_art("apache.commons.collections.primitives")

    def test_apache_commons_collections(self):
        print "[RunTest]: %s" % self.__str__()
        if self.adb.get_product() == BXT_O:
            assert False, "Not applicable"
        self.run_art("apache.commons.collections")

    def test_apache_commons_compress(self):
        print "[RunTest]: %s" % self.__str__()
        if self.adb.get_product() == BXT_O:
            assert False, "Not applicable"
        self.run_art("apache.commons.compress")

    def test_apache_commons_exec(self):
        print "[RunTest]: %s" % self.__str__()
        if self.adb.get_product() == BXT_O:
            assert False, "Not applicable"
        self.run_art("apache.commons.exec")

    def test_apache_commons_io(self):
        print "[RunTest]: %s" % self.__str__()
        self.run_art("apache.commons.io")

    def test_apache_commons_jexl(self):
        print "[RunTest]: %s" % self.__str__()
        self.run_art("apache.commons.jexl")

    def test_apache_commons_lang3(self):
        print "[RunTest]: %s" % self.__str__()
        self.run_art("apache.commons.lang3")

    def test_apache_commons_math3(self):
        print "[RunTest]: %s" % self.__str__()
        if self.adb.get_product() == BXT_O:
            assert False, "Not applicable"
        self.run_art("apache.commons.math3", timeout_second = 5 * 3600)

    def test_apache_commons_net(self):
        print "[RunTest]: %s" % self.__str__()
        self.run_art("apache.commons.net")

    def test_apache_commons_pool(self):
        print "[RunTest]: %s" % self.__str__()
        self.run_art("apache.commons.pool")

    def test_apache_httpcomponents(self):
        print "[RunTest]: %s" % self.__str__()
        self.run_art("apache.httpcomponents")

    def test_apache_httpcore_nio(self):
        print "[RunTest]: %s" % self.__str__()
        self.run_art("apache.httpcore-nio")

    def test_apache_mime4j(self):
        print "[RunTest]: %s" % self.__str__()
        self.run_art("apache.mime4j")

    def test_apache_sanselan(self):
        print "[RunTest]: %s" % self.__str__()
        self.run_art("apache.sanselan")

    def test_beihai_workloads(self):
        print "[RunTest]: %s" % self.__str__()
        self.run_art("beihai-workloads")

    def test_code_google_gson(self):
        print "[RunTest]: %s" % self.__str__()
        self.run_art("code.google.gson")

    def test_code_google_wave_protocol(self):
        print "[RunTest]: %s" % self.__str__()
        self.run_art("code.google.wave-protocol")

    def test_CoverageHelper(self):
        print "[RunTest]: %s" % self.__str__()
        if self.adb.get_product() == BXT_O:
            assert False, "Not applicable"
        self.run_art("CoverageHelper")

    def test_cts_dalviktests_stressed_ART(self):
        print "[RunTest]: %s" % self.__str__()
        self.run_art("cts_dalviktests_stressed_ART")

    def test_dvm_stress(self):
        print "[RunTest]: %s" % self.__str__()
        self.run_art("dvm_stress")

    def test_ejml(self):
        print "[RunTest]: %s" % self.__str__()
        self.run_art("ejml")

    def test_github_dingram(self):
        print "[RunTest]: %s" % self.__str__()
        self.run_art("github_dingram")

    def test_github_jackson(self):
        print "[RunTest]: %s" % self.__str__()
        self.run_art("github_jackson")

    def test_github_json(self):
        print "[RunTest]: %s" % self.__str__()
        self.run_art("github_json")

    def test_github_protostuff(self):
        print "[RunTest]: %s" % self.__str__()
        self.run_art("github_protostuff")

    def test_github_spongycastle(self):
        print "[RunTest]: %s" % self.__str__()
        if self.adb.get_product() == BXT_O:
            assert False, "Not applicable"
        self.run_art("github_spongycastle")

    def test_harmony_stress(self):
        print "[RunTest]: %s" % self.__str__()
        self.run_art("harmony_stress")

    def test_intrinsics_tests(self):
        print "[RunTest]: %s" % self.__str__()
        self.run_art("intrinsics_tests")

    def test_java_concurrency_torture(self):
        print "[RunTest]: %s" % self.__str__()
        self.run_art("java-concurrency-torture")

    def test_jLAPACK(self):
        print "[RunTest]: %s" % self.__str__()
        self.run_art("jLAPACK")

    def test_jmatbench(self):
        print "[RunTest]: %s" % self.__str__()
        self.run_art("jmatbench")

    def test_la4j(self):
        print "[RunTest]: %s" % self.__str__()
        self.run_art("la4j")

    def test_omnidroid(self):
        print "[RunTest]: %s" % self.__str__()
        self.run_art("omnidroid")

    def test_regression_tests(self):
        print "[RunTest]: %s" % self.__str__()
        self.run_art("regression_tests")

    def test_threadtest(self):
        print "[RunTest]: %s" % self.__str__()
        self.run_art("threadtest")

    def test_wave(self):
        print "[RunTest]: %s" % self.__str__()
        self.run_art("wave")

    def test_xstream(self):
        print "[RunTest]: %s" % self.__str__()
        self.run_art("xstream")

