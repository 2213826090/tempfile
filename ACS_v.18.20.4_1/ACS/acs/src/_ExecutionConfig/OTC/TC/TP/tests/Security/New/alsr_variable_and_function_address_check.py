from testlib.util.uiatestbase import UIATestBase
from testlib.security.security_impl import SecurityImpl
from testlib.security.flash_image import FlashImage

class ALSRAddressCheck(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        self.securityImpl = SecurityImpl()
        self.flashImage = FlashImage()
        self.securityImpl.check_pre_build_test_or_ebimage()
        self.tar_dut_path = "/data/local/tmp/"
        self.push_file = "alsr_exists"
        print
        print "[Setup]: %s" % self._test_name
        super(ALSRAddressCheck, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ALSRAddressCheck, self).tearDown()

    def auto_flash_eb_image(self):
        product = self.securityImpl.check_product()
        if "gordon_peak" in product:
            mem_size = self.securityImpl.check_memery_2G_4G_8G_board()
            if "2G" == mem_size:
                option_file = "alsr_omr1_2g"
            else:
                option_file = "alsr_omr1"
        elif "bxt" in product:
            option_file = "alsr_m"
        else:
            print "Other product: {}".format(product)
            raise EnvironmentError, "pls setup DUT..."
        self.securityImpl.logger.info(option_file)
        res_eb_image = self.flashImage.get_download_file_res(option_file, "ALSR_address-flashfiles")
        eb_image = self.flashImage.download_image_file(option_file, res_eb_image)
        self.flashImage.auto_flash_image(eb_image)
        self.securityImpl.push_to_dut_file_add_permission(self.push_file, self.tar_dut_path)

    def whether_flash_eb_image(self):
        if self.securityImpl.check_eb_image_file_exists(self.tar_dut_path, self.push_file) is False:
            print "[info]---res file %s already not existed, Need to flash image..." % self.push_file
            self.auto_flash_eb_image()

    def test_eb_image_alsr_variable_function_address_check(self):
        # HP desc: need use userdebug image
        print "[RunTest]: %s" % self.__str__()
        self.securityImpl.logger.info("")
        self.whether_flash_eb_image()
        self.securityImpl.check_ALSR_variable_address()

