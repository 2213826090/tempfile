from testlib.util.uiatestbase import UIATestBase
from testlib.security.security_impl import SecurityImpl
from testlib.security.flash_image import FlashImage

class NegativeTipcTest(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        self.securityImpl = SecurityImpl()
        self.flashImage = FlashImage()
        self.securityImpl.check_pre_build_test_or_ebimage()
        print
        print "[Setup]: %s" % self._test_name
        super(NegativeTipcTest, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(NegativeTipcTest, self).tearDown()


    def test_eb_image_negative_tipc_test(self):
        # HP desc: need use userdebug image
        print "[RunTest]: %s" % self.__str__()
        self.securityImpl.logger.info("")
        option_file = None
        tar_dut_path = "/data/local/tmp/"
        product = self.securityImpl.check_product()
        if "gordon_peak" in product:
            mem_size = self.securityImpl.check_memery_2G_4G_8G_board()
            if "2G" == mem_size:
                option_file = "negative_omr1_2g"
            else:
                option_file = "negative_omr1"
        elif "bxt" in product:
            option_file = "negative_m"
        else:
            print "Other product: {}".format(product)
            raise EnvironmentError, "pls setup DUT..."
        self.securityImpl.logger.info(option_file)
        res_eb_image = self.flashImage.get_download_file_res(option_file, "negative-flashfiles")
        eb_image = self.flashImage.download_image_file(option_file, res_eb_image)
        self.flashImage.auto_flash_image(eb_image)
        file_path = self.securityImpl.push_to_dut_file_add_permission("negative_test", tar_dut_path)
        self.securityImpl.run_negative_test(file_path)

