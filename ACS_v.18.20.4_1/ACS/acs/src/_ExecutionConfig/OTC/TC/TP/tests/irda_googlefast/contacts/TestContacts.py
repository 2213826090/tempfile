"""
@summary: Test Contact app
@since: 09/12/2014
@author: Sam Lan (samx.lan@intel.com)
"""

import os
from testlib.util.uiatestbase import UIATestBase
from testlib.contacts.contacts_impl import ContactsImpl


class TestContacts(UIATestBase):
    """
    @summary: Test Contact functionalities
    """
    def setUp(self):
        """
        @summary: set up
        @return: None
        """
        super(TestContacts, self).setUp()
        cfg_file = os.path.join(os.environ.get('TEST_REPO_ROOT', ''), 'tests.tablet.google_fast.contacts.conf')
        self.contacts = ContactsImpl(self.config.read(cfg_file, 'contacts'))

    def tearDown(self):
        """
        @summary: tear tearDown
        @return: None
        """
        super(TestContacts, self).tearDown()
        self.contacts = None

    def testContacts(self):
        """
        This test used to test contacts app

        The test case spec is following:
        1. launch contact app
        2. import a contact list from a cvf file.
        """

        self.contacts.launch_by_am()
        self.contacts.jump_add_account_screen()
        self.contacts.add_a_new_contact("AAAA")
        self.contacts.export_contact_list()
        self.contacts.delete_all_contacts()
        self.contacts.import_contact_list(self.contacts.vcf_file_name)
        self.contacts.import_contact_list_successfully("AAAA")
        self.contacts.delete_a_contact("AAAA")
        self.contacts.delete_the_exproted_vcf_file(self.contacts.vcf_file_name)
