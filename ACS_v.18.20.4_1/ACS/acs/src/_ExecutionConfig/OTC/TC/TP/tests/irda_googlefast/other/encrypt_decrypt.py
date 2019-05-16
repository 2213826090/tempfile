from testlib.util.uiatestbase import UIATestBase
from testlib.encrypt.encrypt_impl import Encrypt_Impl

class Encrypt(UIATestBase):
    """
    @summary: Test encrypt and decrypt
    """
    def setUp(self):
        super(Encrypt, self).setUp()
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.encrypt = Encrypt_Impl()

    def tearDown(self):
        self.encrypt.cancel_password()
        super(Encrypt, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testEncrypt(self):
        """
        This test used to test encrypt and decrypt.
        The test case spec is following:

        1. Launch the "settings" app
        2. Set password
        3. Cancel password
        """
        print "[RunTest]: %s" % self.__str__()

        self.encrypt.set_password()
        self.encrypt.decrypt_device()
