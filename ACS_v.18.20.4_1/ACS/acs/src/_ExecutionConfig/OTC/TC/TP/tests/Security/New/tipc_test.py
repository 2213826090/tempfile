from testlib.util.uiatestbase import UIATestBase
from testlib.security.security_impl import SecurityImpl
from testlib.security.flash_image import FlashImage

class TipcTest(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        self.securityImpl = SecurityImpl()
        self.flashImage = FlashImage()
        self.securityImpl.check_pre_build_test_or_ebimage()
        self.tar_dut_path = "/data/local/tmp/"
        self.push_file = "tipc-test32"
        print
        print "[Setup]: %s" % self._test_name
        super(TipcTest, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(TipcTest, self).tearDown()

    def auto_flash_eb_image(self):
        product = self.securityImpl.check_product()
        if "gordon_peak" in product:
            mem_size = self.securityImpl.check_memery_2G_4G_8G_board()
            if "2G" == mem_size:
                option_file = "tipc_omr1_2g"
            else:
                option_file = "tipc_omr1"
        elif "bxt" in product:
            option_file = "tipc_m"
        else:
            print "Other product: {}".format(product)
            raise EnvironmentError, "pls setup DUT..."
        self.securityImpl.logger.info(option_file)
        res_eb_image = self.flashImage.get_download_file_res(option_file, "tipc-flashfiles")
        eb_image = self.flashImage.download_image_file(option_file, res_eb_image)
        self.flashImage.auto_flash_image(eb_image)

    def whether_flash_eb_image(self):
        if self.securityImpl.check_eb_image_file_exists(self.tar_dut_path, self.push_file) is False:
            print "[info]---res file %s already not existed, Need to flash image..." % self.push_file
            self.auto_flash_eb_image()

    def test_eb_image_tipc_test(self):
        # HP desc: need use userdebug image
        print "[RunTest]: %s" % self.__str__()
        self.securityImpl.logger.info("")
        self.whether_flash_eb_image()
        file_path = self.securityImpl.push_to_dut_file_add_permission("tipc_test", self.tar_dut_path)
        self.securityImpl.run_tipc_test(file_path)

    def test_eb_image_tipc_test_app(self):
        # HP desc: need use userdebug image
        print "[RunTest]: %s" % self.__str__()
        self.securityImpl.logger.info("")
        self.whether_flash_eb_image()
        file_path = self.securityImpl.push_to_dut_file_add_permission("tipc_test", self.tar_dut_path)
        self.securityImpl.run_tipc_test(file_path)
