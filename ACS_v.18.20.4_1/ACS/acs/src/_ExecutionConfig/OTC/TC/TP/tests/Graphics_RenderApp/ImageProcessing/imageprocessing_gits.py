# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 08/24/2015
@author: Xiangyi Zhao
'''

from testlib.graphics.test_template.render_app_testbase import RenderAppTestBase
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.common import g_common_obj
from testlib.graphics.imageprocessing import ImageProcessingTest

class ImageProcessing(RenderAppTestBase):

    @classmethod
    def setUpClass(self):
        """
        install apk
        """
        super(ImageProcessing, self).setUpClass()
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        cfg = config.read(cfg_file, 'content_imageprocessing')
        arti = Artifactory(cfg_arti.get('location'))
        apk_name = cfg.get("name")
        file_path = arti.get(apk_name)
        result = config_handle.check_apps("com.android.rs.image")
        if result == 0:
            g_common_obj.adb_cmd_common('install ' + file_path)
        self.imageprocessing = ImageProcessingTest()

    def setUp(self):
        super(ImageProcessing, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.imageprocessing.change_to_vertical()

    @classmethod
    def tearDownClass(self):
        """
        uninstall apk
        """
        super(ImageProcessing, self).tearDownClass()
        self.imageprocessing.uninstall_app()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ImageProcessing, self).tearDown()
        self.imageprocessing.remove_file()

    def test_Imageprocessing_blur_radius_25(self):
        """ Test Imageprocessing blur-radius-25 with gits renderscript
        """
        print "[RunTest]: %s" % self.__str__()
        self.imageprocessing.launch_app_am()
        self.imageprocessing.check_picture("Blur radius 25")
        self.imageprocessing.stop_app_am()

    def test_Imageprocessing_colormaxtrix(self):
        """ Test Imageprocessing colormaxtrix with gits renderscript
        """
        print "[RunTest]: %s" % self.__str__()
        self.imageprocessing.launch_app_am()
        self.imageprocessing.check_picture("ColorMatrix")
        self.imageprocessing.stop_app_am()

    def test_Imageprocessing_convolve_3x3(self):
        """ Test Imageprocessing convolve-3x3 with gits renderscript
        """
        print "[RunTest]: %s" % self.__str__()
        self.imageprocessing.launch_app_am()
        self.imageprocessing.check_picture("Convolve 3x3")
        self.imageprocessing.stop_app_am()

    def test_Imageprocessing_convolve_5x5(self):
        """ Test Imageprocessing stream-convolve-5x5 with gits renderscript
        """
        print "[RunTest]: %s" % self.__str__()
        self.imageprocessing.launch_app_am()
        self.imageprocessing.check_picture("Convolve 5x5")
        self.imageprocessing.stop_app_am()

    def test_Imageprocessing_copy(self):
        """ Test Imageprocessing copy with gits renderscript
        """
        print "[RunTest]: %s" % self.__str__()
        self.imageprocessing.launch_app_am()
        self.imageprocessing.check_picture("Copy")
        self.imageprocessing.stop_app_am()

    def test_Imageprocessing_crossprocess_lut(self):
        """ Test Imageprocessing crossprocess-lut with gits renderscript
        """
        print "[RunTest]: %s" % self.__str__()
        self.imageprocessing.launch_app_am()
        self.imageprocessing.check_picture("CrossProcess (using LUT)")
        self.imageprocessing.stop_app_am()

    def test_Imageprocessing_fisheye_approximate_relaxed(self):
        """ Test Imageprocessing fisheye-approximate-relaxed with gits renderscript
        """
        print "[RunTest]: %s" % self.__str__()
        self.imageprocessing.launch_app_am()
        self.imageprocessing.check_picture("Fisheye Approximate Relaxed")
        self.imageprocessing.stop_app_am()

    def test_Imageprocessing_fisheye_relaxed(self):
        """ Test Imageprocessing fisheye-relaxed with gits renderscript
        """
        print "[RunTest]: %s" % self.__str__()
        self.imageprocessing.launch_app_am()
        self.imageprocessing.check_picture("Fisheye Relaxed")
        self.imageprocessing.stop_app_am()

    def test_Imageprocessing_grain(self):
        """ Test Imageprocessing grain with gits renderscript
        """
        print "[RunTest]: %s" % self.__str__()
        self.imageprocessing.launch_app_am()
        self.imageprocessing.check_picture("Grain")
        self.imageprocessing.stop_app_am()

    def test_Imageprocessing_greyscale(self):
        """ Test Imageprocessing greyscale with gits renderscript
        """
        print "[RunTest]: %s" % self.__str__()
        self.imageprocessing.launch_app_am()
        self.imageprocessing.check_picture("Greyscale")
        self.imageprocessing.stop_app_am()

    def test_Imageprocessing_group_test_emulated(self):
        """ Test Imageprocessing group-test-emulated with gits renderscript
        """
        print "[RunTest]: %s" % self.__str__()
        self.imageprocessing.launch_app_am()
        self.imageprocessing.check_picture("Group Test (emulated)")
        self.imageprocessing.stop_app_am()

    def test_Imageprocessing_group_test_native(self):
        """ Test Imageprocessing group-test-native with gits renderscript
        """
        print "[RunTest]: %s" % self.__str__()
        self.imageprocessing.launch_app_am()
        self.imageprocessing.check_picture("Group Test (native)")
        self.imageprocessing.stop_app_am()

    def test_Imageprocessing_intrinsic_blur_radius_25(self):
        """ Test Imageprocessing intrinsic-blur-radius-25 with gits renderscript
        """
        print "[RunTest]: %s" % self.__str__()
        self.imageprocessing.launch_app_am()
        self.imageprocessing.check_picture("Intrinsic Blur radius 25")
        self.imageprocessing.stop_app_am()

    def test_Imageprocessing_intrinsics_blend(self):
        """ Test Imageprocessing intrinsics-blend with gits renderscript
        """
        print "[RunTest]: %s" % self.__str__()
        self.imageprocessing.launch_app_am()
        self.imageprocessing.check_picture("Intrinsics Blend")
        self.imageprocessing.stop_app_am()

    def test_Imageprocessing_intrinsics_colormaxtrix(self):
        """ Test Imageprocessing intrinsics-colormaxtrix with gits renderscript
        """
        print "[RunTest]: %s" % self.__str__()
        self.imageprocessing.launch_app_am()
        self.imageprocessing.check_picture("Intrinsics ColorMatrix")
        self.imageprocessing.stop_app_am()

    def test_Imageprocessing_intrinsics_colormaxtrix_grey(self):
        """ Test Imageprocessing intrinsics-colormaxtrix-grey with gits renderscript
        """
        print "[RunTest]: %s" % self.__str__()
        self.imageprocessing.launch_app_am()
        self.imageprocessing.check_picture("Intrinsics ColorMatrix Grey")
        self.imageprocessing.stop_app_am()

    def test_Imageprocessing_intrinsics_convolve_3x3(self):
        """ Test Imageprocessing intrinsics-convolve-3x3 with gits renderscript
        """
        print "[RunTest]: %s" % self.__str__()
        self.imageprocessing.launch_app_am()
        self.imageprocessing.check_picture("Intrinsics Convolve 3x3")
        self.imageprocessing.stop_app_am()

    def test_Imageprocessing_intrinsics_convolve_5x5(self):
        """ Test Imageprocessing intrinsics-convolve-5x5 with gits renderscript
        """
        print "[RunTest]: %s" % self.__str__()
        self.imageprocessing.launch_app_am()
        self.imageprocessing.check_picture("Intrinsics Convolve 5x5")
        self.imageprocessing.stop_app_am()

    def test_Imageprocessing_levels_vec3_full(self):
        """ Test Imageprocessing levels-vec3-full with gits renderscript
        """
        print "[RunTest]: %s" % self.__str__()
        self.imageprocessing.launch_app_am()
        self.imageprocessing.check_picture("Levels Vec3 Full")
        self.imageprocessing.stop_app_am()

    def test_Imageprocessing_levels_vec3_relaxed(self):
        """ Test Imageprocessing levels-vec3-relaxed with gits renderscript
        """
        print "[RunTest]: %s" % self.__str__()
        self.imageprocessing.launch_app_am()
        self.imageprocessing.check_picture("Levels Vec3 Relaxed")
        self.imageprocessing.stop_app_am()

    def test_Imageprocessing_levels_vec4_full(self):
        """ Test Imageprocessing levels-vec4-full with gits renderscript
        """
        print "[RunTest]: %s" % self.__str__()
        self.imageprocessing.launch_app_am()
        self.imageprocessing.check_picture("Levels Vec4 Full")
        self.imageprocessing.stop_app_am()

    def test_Imageprocessing_levels_vec4_relaxed(self):
        """ Test Imageprocessing levels-vec4-relaxed with gits renderscript
        """
        print "[RunTest]: %s" % self.__str__()
        self.imageprocessing.launch_app_am()
        self.imageprocessing.check_picture("Levels Vec4 Relaxed")
        self.imageprocessing.stop_app_am()

    def test_Imageprocessing_mandelbrot(self):
        """ Test Imageprocessing mandelbrot with gits renderscript
        """
        print "[RunTest]: %s" % self.__str__()
        self.imageprocessing.launch_app_am()
        self.imageprocessing.check_picture("Mandelbrot fp32")
        self.imageprocessing.stop_app_am()

    def test_Imageprocessing_vignette_approximate_full(self):
        """ Test Imageprocessing vignette-approximate-full with gits renderscript
        """
        print "[RunTest]: %s" % self.__str__()
        self.imageprocessing.launch_app_am()
        self.imageprocessing.check_picture("Vignette Approximate Full")
        self.imageprocessing.stop_app_am()

    def test_Imageprocessing_vignette_approximate_relaxed(self):
        """ Test Imageprocessing vignette-approximate-relaxed with gits renderscript
        """
        print "[RunTest]: %s" % self.__str__()
        self.imageprocessing.launch_app_am()
        self.imageprocessing.check_picture("Vignette Approximate Relaxed")
        self.imageprocessing.stop_app_am()

    def test_Imageprocessing_vignette_full(self):
        """ Test Imageprocessing vignette-full with gits renderscript
        """
        print "[RunTest]: %s" % self.__str__()
        self.imageprocessing.launch_app_am()
        self.imageprocessing.check_picture("Vignette Full")
        self.imageprocessing.stop_app_am()

    def test_Imageprocessing_vignette_relaxed(self):
        """ Test Imageprocessing vignette-relaxed with gits renderscript
        """
        print "[RunTest]: %s" % self.__str__()
        self.imageprocessing.launch_app_am()
        self.imageprocessing.check_picture("Vignette Relaxed")
        self.imageprocessing.stop_app_am()
