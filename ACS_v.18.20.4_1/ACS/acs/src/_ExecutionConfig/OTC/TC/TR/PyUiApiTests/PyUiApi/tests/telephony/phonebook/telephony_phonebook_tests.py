from PyUiApi.common.test_utils import *
from PyUiApi.app_utils.contacts_utils import Contacts
from PyUiApi.app_utils.dialer_utils import Dialer
from PyUiApi.common.status_bar import StatusBar
from PyUiApi.multi_dut_support.dut_manager import *
from random import choice


class TelephonyPhoneBookTests(unittest.TestCase):
    TAG = "TelephonyPhoneBookTests"

    contacts_to_delete = []
    flight_mode = False

    def setUp(self):
        UiAutomatorUtils.unlock_screen()
        UiAutomatorUtils.close_all_tasks()
        self.initial_orientation = d.orientation
        if self.flight_mode:
            StatusBar.open_notifications(2)
            StatusBar.toggle_airplane_mode('OFF')
            StatusBar.close_notifications()
            self.flight_mode = False

    def tearDown(self):
        if self.contacts_to_delete:
            if not Contacts.is_contacts_open():
                LOG.info('Launch Contacts app.')
                Contacts.launch()
                LOG.info('Go to All Contacts view.')
                Contacts.go_to_all_contacts()
            LOG.info('Delete contacts: %s' % self.contacts_to_delete)
            Contacts.delete_contacts(self.contacts_to_delete)
        self.contacts_to_delete = []

        UiAutomatorUtils.close_all_tasks()
        if d.orientation != self.initial_orientation:
            d.orientation = self.initial_orientation
            d.freeze_rotation(False)
        if self.flight_mode:
            StatusBar.open_notifications(2)
            StatusBar.toggle_airplane_mode('OFF')
            StatusBar.close_notifications()
            self.flight_mode = False

    def test_import_one_contact_from_sim(self):
        """
        TestCaseID:      ST_TELEPHONY_SIM_PB_001
        TestDefinition:  Phonebook - Import one contact from SIM to phonebook
        TestPurpose:     To ensure that user can import a contact from the SIM phonebook to the device phonebook
        TestEnvironment: Cellular live simulator testing
        InitialState:    DUT is in Idle mode, SIM inserted; SIM is filled with contacts.
        ExpectedResult:  Contact is successfully imported from SIM phonebook.
        """

        LOG.info('Open Contacts app.')
        ret = Contacts.launch()

        if not ret:
            # When opened for the first time the 'Add your account' window pops up
            if Contacts.is_add_your_account_open():
                LOG.info("'Add your account' window popped up.")
                d.press.back()

        LOG.info('Check that Contacts app is open.')
        ret = Contacts.is_contacts_open()
        self.assertTrue(ret, 'Failed to launch the Contacts app.')

        LOG.info('Get all contacts from Contacts app.')
        all_contacts = Contacts.get_all_contacts()

        if not all_contacts:
            LOG.info('No contacts found in PhoneBook.')

            LOG.info('Open Import from sim card (button).')
            ret = Contacts.open_import_from_sim_card()
            self.assertTrue(ret, 'Failed to open Import from sim card (button).')
        else:
            LOG.info('Found some contacts in PhoneBook.')

            LOG.info('Open More Options menu.')
            ret = Contacts.open_more_options_menu()
            self.assertTrue(ret, 'Failed to open More Options menu.')

            LOG.info('Open Import/export contacts popup.')
            ret = Contacts.open_import_export_contacts_popup()
            self.assertTrue(ret, 'Failed to open Import/export contacts popup.')

            LOG.info('Open Import from sim card (menu).')
            ret = Contacts.open_import_from_sim_card()
            self.assertTrue(ret, 'Failed to open Import from sim card (menu).')

        LOG.info('Check that the Phone app (Select contacts to import ..) is open.')
        ret = Dialer.is_import_from_sim_open()
        self.assertTrue(ret, 'Phone app (Select contacts to import ..) was not open.')

        LOG.info('Get all contacts from sim card.')
        sim_contacts = Dialer.get_all_contacts_from_sim()
        self.assertNotEqual(0, len(sim_contacts), 'No contacts found on the sim card.')

        contact = choice(sim_contacts)
        LOG.info('Get a random contact from sim card: %s' % contact)

        LOG.info('Verify contact %s is not already present in PhoneBook before importing.' % contact)
        # When importing contacts, the '.' is replaced with ' '
        fixed_contact = contact.replace('.', ' ')
        self.assertNotIn(fixed_contact, all_contacts)

        LOG.info('Import contact %s from sim card.' % contact)
        ret = Dialer.import_contact_from_sim(contact)
        self.assertTrue(ret, 'Failed to import contact %s from sim.' % contact)

        LOG.info('Go back to Contacts app (by pressing back).')
        d.press.back()
        self.contacts_to_delete.append(fixed_contact)

        LOG.info('Check that Contacts app is open.')
        ret = Contacts.is_contacts_open()
        self.assertTrue(ret, 'Contacts app is not open.')

        LOG.info('Verify if contact %s was successfully imported.' % contact)
        ret = Contacts.get_contact(fixed_contact)
        self.assertIsNotNone(ret, 'Failed to find contact %s in phonebook after import.' % contact)

    def test_import_all_contacts_from_sim(self):
        """
        TestCaseID:      ST_TELEPHONY_SIM_PB_002
        TestDefinition:  Phonebook - Import all contacts from SIM to phonebook
        TestPurpose:     To ensure that user can import all contacts from the SIM phonebook to the device phonebook
        TestEnvironment: Cellular live simulator testing
        InitialState:    DUT is in Idle mode, SIM inserted; SIM is filled with at least 50 contacts.
        ExpectedResult:  SIM phonebook is successfully imported.
        """

        LOG.info('Open Contacts app.')
        ret = Contacts.launch()

        if not ret:
            # When opened for the first time the 'Add your account' window pops up
            if Contacts.is_add_your_account_open():
                LOG.info("'Add your account' window popped up.")
                d.press.back()

        LOG.info('Check that Contacts app is open.')
        ret = Contacts.is_contacts_open()
        self.assertTrue(ret, 'Failed to launch the Contacts app.')

        LOG.info('Get all contacts from Contacts app.')
        all_contacts = Contacts.get_all_contacts()

        if not all_contacts:
            LOG.info('No contacts found in PhoneBook.')

            LOG.info('Open Import from sim card (button).')
            ret = Contacts.open_import_from_sim_card()
            self.assertTrue(ret, 'Failed to open Import from sim card (button).')
        else:
            LOG.info('Found some contacts in PhoneBook.')

            LOG.info('Open More Options menu.')
            ret = Contacts.open_more_options_menu()
            self.assertTrue(ret, 'Failed to open More Options menu.')

            LOG.info('Open Import/export contacts popup.')
            ret = Contacts.open_import_export_contacts_popup()
            self.assertTrue(ret, 'Failed to open Import/export contacts popup.')

            LOG.info('Open Import from sim card (menu).')
            ret = Contacts.open_import_from_sim_card()
            self.assertTrue(ret, 'Failed to open Import from sim card (menu).')

        LOG.info('Check that the Phone app (Select contacts to import ..) is open.')
        ret = Dialer.is_import_from_sim_open()
        self.assertTrue(ret, 'Phone app (Select contacts to import ..) was not open.')

        LOG.info('Get all contacts from sim card.')
        sim_contacts = Dialer.get_all_contacts_from_sim()
        self.assertNotEqual(0, len(sim_contacts), 'No contacts found on the sim card.')

        # When importing contacts, the '.' is replaced with ' '
        fixed_sim_contacts = [c.replace('.', ' ') for c in sim_contacts]
        LOG.info('Verify none of the sim contacts are already in the PhoneBook.')
        self.assertEqual(0, len(set(fixed_sim_contacts).intersection(set(all_contacts))),
                         'Found contacts from sim card already in the PhoneBook.')

        LOG.info('Import all contacts from sim card.')
        ret = Dialer.import_all_contacts_from_sim()
        self.assertTrue(ret, 'Failed to import all contacts from sim card.')

        LOG.info('Check that Contacts app is open.')
        ret = Contacts.is_contacts_open()
        self.assertTrue(ret, 'Contacts app is not open.')

        LOG.info('Get all contacts from Contacts app.')
        all_contacts = Contacts.get_all_contacts()
        self.contacts_to_delete.extend(fixed_sim_contacts)

        LOG.info('Verify if all sim contacts were successfully imported.')
        for c in fixed_sim_contacts:
            self.assertIn(c, all_contacts, 'Contact %s was not successfully imported.' % c)

    def test_import_contacts_from_sim_flight_mode_on(self):
        """
        TestCaseID:      ST_TELEPHONY_REG_FlightMode_117
        TestDefinition:  Flight Mode ON - Import contacts stored on the SIM card not allowed
        TestPurpose:     The purpose is to check that it is not possible to import contacts
                         stored in the SIM card when the flight mode is activated.
        TestEnvironment: Configuration1 : Cellular live network testing
        InitialState:    - Flight mode is set to on. - One (or more) contacts are stored into SIM card.
        ExpectedResult:  Import contacts stored on the SIM card not allowed in Flight Mode
        """

        LOG.info('Open Notifications.')
        StatusBar.open_notifications(2)

        LOG.info('Set Flight mode to ON')
        ret = StatusBar.toggle_airplane_mode('ON')
        self.assertTrue(ret, 'Failed to set Flight mode to ON.')
        self.flight_mode = True

        StatusBar.close_notifications()
        LOG.info('Open Contacts app.')
        ret = Contacts.launch()

        if not ret:
            # When opened for the first time the 'Add your account' window pops up
            if Contacts.is_add_your_account_open():
                LOG.info("'Add your account' window popped up.")
                d.press.back()

        LOG.info('Check that Contacts app is open.')
        ret = Contacts.is_contacts_open()
        self.assertTrue(ret, 'Failed to launch the Contacts app.')

        LOG.info('Get all contacts from Contacts app.')
        all_contacts = Contacts.get_all_contacts()

        if not all_contacts:
            LOG.info('No contacts found in PhoneBook.')

            LOG.info('Open Import from sim card (button).')
            ret = Contacts.open_import_from_sim_card()
            self.assertTrue(ret, 'Failed to open Import from sim card (button).')
        else:
            LOG.info('Found some contacts in PhoneBook.')

            LOG.info('Open More Options menu.')
            ret = Contacts.open_more_options_menu()
            self.assertTrue(ret, 'Failed to open More Options menu.')

            LOG.info('Open Import/export contacts popup.')
            ret = Contacts.open_import_export_contacts_popup()
            self.assertTrue(ret, 'Failed to open Import/export contacts popup.')

            LOG.info('Open Import from sim card (menu).')
            ret = Contacts.open_import_from_sim_card()
            self.assertTrue(ret, 'Failed to open Import from sim card (menu).')

        LOG.info('Check that the Phone app (Select contacts to import ..) is open.')
        ret = Dialer.is_import_from_sim_open()
        self.assertTrue(ret, 'Phone app (Select contacts to import ..) was not open.')

        LOG.info('Get all contacts from sim card.')
        sim_contacts = Dialer.get_all_contacts_from_sim()
        self.assertNotEqual(0, len(sim_contacts), 'No contacts found on the sim card.')

        contact = choice(sim_contacts)
        LOG.info('Get a random contact from sim card: %s' % contact)

        LOG.info('Verify contact %s is not already present in PhoneBook before importing.' % contact)
        # When importing contacts, the '.' is replaced with ' '
        fixed_contact = contact.replace('.', ' ')
        self.assertNotIn(fixed_contact, all_contacts)

        # NOTE: I managed to import a contact from sim with FlightMode ON
        #       I cannot reproduce the expected behaviour
        #       This step should fail?
        LOG.info('Try to import contact %s from sim card.' % contact)
        ret = Dialer.import_contact_from_sim(contact)
        self.assertTrue(ret, 'Failed to import contact %s from sim.' % contact)

        LOG.info('Go back to Contacts app (by pressing back).')
        d.press.back()
        self.contacts_to_delete.append(fixed_contact)

        LOG.info('Check that Contacts app is open.')
        ret = Contacts.is_contacts_open()
        self.assertTrue(ret, 'Contacts app is not open.')

        LOG.info('Verify contact %s was not imported.' % contact)
        ret = Contacts.get_contact(fixed_contact)
        self.contacts_to_delete = [] if not ret else self.contacts_to_delete
        self.assertIsNone(ret, 'Found contact %s in phonebook after import.' % contact)

        # The other steps are already covered in ST_TELEPHONY_SIM_PB_001
        # 2. Disable Airplane mode and wait for the DUT register on its HPLMN
        # --> Check that the DUT is correctly registered to on its HPLMN
        # 3. From DUT menu, try to import contacts stored in the SIM card to DUT phone book
        # --> Check that we can import contacts from the SIM card.
        LOG.info('Open Notifications.')
        StatusBar.open_notifications(2)

        LOG.info('Set Flight mode to OFF')
        ret = StatusBar.toggle_airplane_mode('OFF')
        self.assertTrue(ret, 'Failed to set Flight mode to OFF.')
        self.flight_mode = False
        StatusBar.close_notifications()
        self.test_import_one_contact_from_sim()
