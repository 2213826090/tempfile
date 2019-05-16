#!/usr/bin/python
# -*- coding:utf-8 -*

from testlib.util.instrumentedtestbase import InstrumentationTestBase
from testlib.instrumentation.graphics_impl import ColorModeImpl
from testlib.androidframework.common import EnvironmentUtils
from testlib.graphics.common import pkgmgr, get_resource_from_atifactory


PACKAGE_NAME="android.colormode.cts"


class ColorModeTestBase(InstrumentationTestBase):
    @classmethod
    def setUpClass(cls):
        """
        install apk
        """
        super(ColorModeTestBase, cls).setUpClass()
        cls._install_apk()
        cls.clm = ColorModeImpl()
        cls.clm.intialize()

    @classmethod
    def tearDownClass(cls):
        """
        uninstall apk
        """
        cls.clm.finalize()
        super(ColorModeTestBase, cls).tearDownClass()

    @classmethod
    def _install_apk(self):

        if not pkgmgr._package_installed(pkgName=PACKAGE_NAME):
            android_version = EnvironmentUtils().get_android_version()
            selection = {'L': 'apk_l',
                         'M': 'apk_m',
                         'N': 'apk_n',
                         'O-MR0': 'apk_omr0',
                         'O-MR1': 'apk_omr1'}
            try:
                file_path = get_resource_from_atifactory('tests.tablet.artifactory.conf',
                                                         'content_colormode',
                                                         selection[android_version])
                pkgmgr.apk_install(file_path)
            except KeyError, e:
                print "ColorMode app version not match with android os: %s" % str(e)
            # Need to update above when new version is released.
