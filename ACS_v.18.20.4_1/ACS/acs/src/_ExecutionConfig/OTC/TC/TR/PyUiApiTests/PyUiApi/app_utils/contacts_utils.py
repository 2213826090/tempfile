from PyUiApi.common.uiautomator_utils import *


class Contacts(object):

    @staticmethod
    def launch():
        UiAutomatorUtils.launch_app_from_apps_menu(CONTACTS_SHORTCUT_NAME)
        # When first opened the 'Add your account' windows pops on top
        if Contacts.is_add_your_account_open():
            return False
        return Contacts.is_contacts_open()

    @staticmethod
    def click_first_contact():
        if d(resourceId=CONTACTS_CONTACT_LIST_ENTRY_RESID).wait.exists(timeout=3000):
            d(resourceId=CONTACTS_CONTACT_LIST_ENTRY_RESID).click()
        time.sleep(1)

    @staticmethod
    def search(search_string):
        if d(resourceId=CONTACTS_MENU_SEARCH_RESID).wait.exists(timeout=3000):
            d(resourceId=CONTACTS_MENU_SEARCH_RESID).click()
        if d(textContains=CONTACTS_SEARCH_VIEW_TXT).wait.exists(timeout=3000):
            d(textContains=CONTACTS_SEARCH_VIEW_TXT).set_text(search_string)
        time.sleep(2)

    @staticmethod
    def import_contacts_from_google_account():
        Contacts.launch()
        if d(resourceId=CONTACTS_ADD_CONTACT_BUTTON_RESID).wait.exists(timeout=5000):
            d(resourceId=CONTACTS_ADD_CONTACT_BUTTON_RESID)
        if d(text=CONTACTS_OK_TXT).wait.exists(timeout=5000):
            d(text=CONTACTS_OK_TXT).click()

    @staticmethod
    def is_contacts_open():
        return d(text=CONTACTS_SHORTCUT_NAME).wait.exists(timeout=5000)

    @staticmethod
    def is_add_your_account_open():
        return d(packageName=ADD_ACCOUNT_PACKAGE_NAME).wait.exists(timeout=5000)

    @staticmethod
    def go_to_all_contacts():
        vn = ViewNavigator()
        vn.navigate_views([CONTACTS_ALL_CONTACTS_TXT])
        return True

    @staticmethod
    def go_to_favorites():
        vn = ViewNavigator()
        vn.navigate_views([CONTACTS_FAVORITES_TXT])
        return True

    @staticmethod
    def open_more_options_menu():
        if not d(description=CONTACTS_MORE_OPTIONS_TXT,
                 className=ANDROID_WIDGET_IMAGE_BUTTON).wait.exists(timeout=5000):
            LOG.info("Failed to find '%s' menu in a timely manner." % CONTACTS_MORE_OPTIONS_TXT)
            return None
        d(description=CONTACTS_MORE_OPTIONS_TXT, className=ANDROID_WIDGET_IMAGE_BUTTON).click()
        # TODO: Check that the menu popped up
        time.sleep(2)
        return True

    @staticmethod
    def get_options_from(resid, clsname):
        if not d(resourceId=resid, className=clsname).wait.exists(timeout=5000):
            LOG.info("Failed to get options.")
            return None
        # TODO: check opts
        opts = d(resourceId=resid, className=clsname)
        ret_list = []
        for i in opts:
            ret_list.append(i.info['text'])
        d.press.back()
        return ret_list

    @staticmethod
    def get_options_from_more_options_menu():
        return Contacts.get_options_from(TITLE_ELEMENT_RESID, ANDROID_WIDGET_TEXT_VIEW)

    @staticmethod
    def open_import_export_contacts_popup():
        if not d(text=CONTACTS_IMPORT_EXPORT_TXT).wait.exists(timeout=5000):
            LOG.info("Failed to find '%s' option." % CONTACTS_IMPORT_EXPORT_TXT)
            return None
        d(text=CONTACTS_IMPORT_EXPORT_TXT).click()
        if not d(text=CONTACTS_IMPORT_EXPORT_TITLE_TXT,
                 resourceId=ANDROID_WIDGET_ALERT_TITLE).wait.exists(timeout=5000):
            LOG.info("'%s' popup did not show up." % CONTACTS_IMPORT_EXPORT_TITLE_TXT)
            return None
        return True

    @staticmethod
    def get_options_from_import_export_contacts_popup():
        return Contacts.get_options_from(IMPORT_FROM_SIM_CONTACT_RESID, ANDROID_WIDGET_TEXT_VIEW)

    @staticmethod
    def open_import_from_sim_card():
        # When no contact is in the phonebook the More Option button is not available
        if d(resourceId=CONTACTS_IMPORT_CONTACTS_BTN_RESID).wait.exists(timeout=3000):
            LOG.info("Found 'Import Contacts' button.")
            d(resourceId=CONTACTS_IMPORT_CONTACTS_BTN_RESID).click()
        if not d(text=CONTACTS_IMPORT_FROM_SIM_TXT).wait.exists(timeout=5000):
            LOG.info("Failed to find '%s' option." % CONTACTS_IMPORT_FROM_SIM_TXT)
            return None
        d(text=CONTACTS_IMPORT_FROM_SIM_TXT).click()
        # NOTE: This actually launches the phone app
        return True

    @staticmethod
    def get_all_contacts():
        Contacts.go_to_all_contacts()
        return Contacts.get_all_contacts_from(CONTACTS_CONTACT_LIST_ENTRY_RESID)

    @staticmethod
    def get_contact(contact_name):
        """
        Get the contact from the Contacts list
        :param contact_name: string
        :return: The matching object or None
        """
        Contacts.go_to_all_contacts()
        ret = None
        try:
            ret = d(className=ANDROID_WIDGET_LIST_VIEW, resourceId=ANDROID_WIDGET_LIST_RESID)\
                .child_by_text(contact_name, resourceId=CONTACTS_CONTACT_LIST_ENTRY_RESID)
        except:
            LOG.info('Contact "%s" not found.' % contact_name)
        return ret

    @staticmethod
    def delete_contacts(contact_list):
        Contacts.go_to_all_contacts()
        not_found_contacts = []
        select_mode = False
        for c in contact_list:
            try:
                contact = d(className=ANDROID_WIDGET_LIST_VIEW, resourceId=ANDROID_WIDGET_LIST_RESID)\
                    .child_by_text(c, resourceId=CONTACTS_CONTACT_LIST_ENTRY_RESID)
            except:
                LOG.info("Contact %s not found in order to be deleted." % c)
                not_found_contacts.append(c)
                continue
            if not select_mode:
                UiAutomatorUtils.long_click(contact)
                select_mode = True
            check_mark = contact.sibling(className=ANDROID_CHECKBOX_CLASSNAME)
            if not check_mark.info['checked']:
                check_mark.click()
        if not Contacts.open_more_options_menu():
            LOG.info('Failed to open More Options menu.')
            return None
        if not d(text=CONTACTS_DELETE_TXT).wait.exists(timeout=3000):
            LOG.info("Delete button not found in 'More Option' menu.")
            return None
        d(text=CONTACTS_DELETE_TXT).click()
        if not d(text=CONTACTS_OK_TXT).wait.exists(timeout=3000):
            LOG.info("Delete confirmation dialog din not show up.")
            return None
        d(text=CONTACTS_OK_TXT).click()
        time.sleep(2)
        if not_found_contacts:
            LOG.info('Contacts not found in order to be deleted: %s' % not_found_contacts)
            return False
        return True

    @staticmethod
    def get_all_contacts_from(resid):
        AppsScroller.scroll_to_begining()
        ret_list = []
        # Check if there are no contacts in the PhoneBook
        if d(text=CONTACTS_NO_CONTACTS_TXT).wait.exists(timeout=2000):
            LOG.info('No contacts found.')
            return ret_list
        last_elems = []
        abort_after = 5 * 60
        start_time = time.time()
        while True:
            delta = time.time() - start_time
            if delta >= abort_after:
                LOG.info('TIMEOUT after %s seconds.' % abort_after)
                break
            if not d(resourceId=resid).wait.exists(timeout=20000):
                LOG.info('No contacts found.')
                break
            contact_list = d(resourceId=resid)
            for i in contact_list:
                ret_list.append(i.info['text'])
            last_elems.append(ret_list[-1])
            # Check if the last 2 elements from the list have the same value --> hit the bottom of the list
            if len(last_elems) >= 2 and last_elems[-1] == last_elems[-2]:
                break
            else:
                AppsScroller.scroll_forward()
        ret_list = sorted(list(set(ret_list)))
        return ret_list
