#!/usr/bin/env python

#######################################################################
#
# @filename:    i18n_steps.py
# @description: I18n test steps
# @author:      tudorx.a.simion@intel.com
#
#######################################################################

from testlib.scripts.android.ui.ui_step import step as ui_step
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.adb.adb_step import step as adb_step
from testlib.scripts.android.ui.ui_steps import *

class open_language_section(ui_step):

    """ description:
            Opens the Laguage section from the Settings App
        usage:
            i18n_steps.open_language_section("United States")()
        tags:
            i18n, android, Language
    """

    def __init__(self, language_section = "United States", **kwargs):
        ui_step.__init__(self, **kwargs)
        self.language_section = language_section

    def do(self):
        ui_steps.open_settings_app(
            serial = self.serial,
            print_error = "Error - Quick settings page was not displayed",
            view_to_find = {"textContains": "Language"})()
        self.uidevice(text = "Language").exists
        ui_steps.click_button(
            serial = self.serial,
            print_error = "Error - Language was not displayed",
            blocking = True,
            view_to_find = {"text": "Language"})()


    def check_condition(self):
        return self.uidevice(textContains = "Afrikaans").exists


class set_unset_desired_language(ui_step):

    """ description:
            This function switches between the 2 languages that
            were sent as input parameters. The parameter should be
            the Country name.
        usage:
            i18n_steps.set_unset_desired_language(
                desired_language_key_word='xxxx'
                desired_original_key_word = "United Kingdom"
                )()
        tags:
            i18n, android, Language, set, unset
    """

    def __init__(self, desired_language_key_word,
            desired_language_check_word,
            desired_original_key_word = "United States",
            desired_original_check_word = "Language",
            **kwargs):
        ui_step.__init__(self, **kwargs)
        self.desired_language_key_word = desired_language_key_word
        self.desired_language_check_word = desired_language_check_word
        self.desired_original_key_word = desired_original_key_word
        self.desired_original_check_word = desired_original_check_word

    def do(self):
        self.uidevice(scrollable = True).scroll.to(textContains =\
                                              self.desired_language_key_word)
        ui_steps.click_button(
            serial = self.serial,
            print_error = "Error - Language was not displayed",
            blocking = True,
            view_to_find = {"textContains": self.desired_language_key_word},
            view_to_check = {"textContains": self.desired_language_check_word})()
        ui_steps.click_button(
            serial = self.serial,
            print_error = "Error - Language was not displayed",
            blocking = True,
            view_to_find = {"text": self.desired_language_check_word},
            view_to_check = {"text": "Afrikaans"})()

        self.uidevice(scrollable = True).scroll.to(textContains =\
                                              self.desired_original_key_word)
        ui_steps.click_button(
            serial = self.serial,
            print_error = "Error - Language was not displayed",
            blocking = True,
            view_to_find = {"textContains": self.desired_original_key_word},
            wait_time = 3000,
            view_to_check = {"text": self.desired_original_check_word}
            )()

    def check_condition(self):
        return self.uidevice(text = self.desired_original_check_word).exists


class check_desired_language(ui_step):

    """ description:
            This function checks that the system language changes for
            all the apps installed.
                desired_language_key_word = The language to which the
                    switch is done
                desired_os_key_word = A key word in Settings that is
                    translated in the desired language
                desired_os_language_translation = the word "Language"
                    translated in the target language
                language_section = defines the language section in
                    settings ex: Language(United States)
        usage:
            i18n_steps.check_desired_language(
                desired_language_key_word='xxxx'
                desired_os_key_word = 'xxxx'
                desired_os_language_translation = "xxxxx",
                language_section = "United States"
                )()
        tags:
            i18n, android, Language, check language
    """

    desired_language_key_word = None
    desired_os_key_word = None
    desired_os_language_translation = None
    language_section = None

    def __init__(self, desired_language_key_word= "France",
            desired_os_key_word = "Localisation",
            desired_os_language_translation = "Langue",
            language_section = "United States", **kwargs):
        ui_step.__init__(self, **kwargs)

        self.desired_language_key_word = desired_language_key_word
        self.desired_os_key_word = desired_os_key_word
        self.desired_os_language_translation = desired_os_language_translation
        self.language_section = language_section

    def do(self):
        open_language_section(serial = self.serial,
                              language_section = self.language_section,
                              print_error =\
                    "Error - Language section was not displayed")()
        self.uidevice(scrollable = True).scroll.to(textContains =\
                                    self.desired_language_key_word)
        ui_steps.click_button(
            serial = self.serial,
            print_error = "Error - Language was not displayed",
            blocking = True,
            view_to_find = {"textContains": self.desired_language_key_word})()
        ui_steps.click_button(
            serial = self.serial,
            print_error = "Error - Language was not displayed",
            blocking = True,
            view_to_find = {"textContains":\
                self.desired_os_language_translation})()

    def check_condition(self):
        return self.uidevice(textContains =\
                            self.desired_os_language_translation)


class return_to_default_language(ui_step):

    """ description:
            This function checks that the system language can be reverted
            to the original one for all the apps installed.
                desired_language_key_word = The language to which the
                    switch is done
                desired_key_word = A key word in Settings that is
                    translated in the desired language
                original_country = the original language Country name
        usage:
            i18n_steps.return_to_default_language(
                desired_language_key_word='xxxx'
                desired_key_word = 'xxxx'
                original_country = "xxxxx"
                )()
        tags:
            i18n, android, Language, default language
    """

    desired_language_key_word = None
    desired_key_word = None
    original_country = None

    def __init__(self, desired_language_key_word = "United States",
                 desired_os_key_word = "Langue", original_country = "France",
                 **kwargs):
        ui_step.__init__(self, **kwargs)
        self.desired_language_key_word = desired_language_key_word
        self.desired_os_key_word = desired_os_key_word
        self.original_country = original_country

    def do(self):
        if self.uidevice(textContains = self.desired_os_key_word).exists:
            ui_steps.click_button(
                serial = self.serial,
                print_error = "Error - Language was not displayed",
                blocking = True,
                view_to_find = {"textContains": self.desired_os_key_word})()
        else :
            self.uidevice(scrollable = True).scroll.to(textContains =\
                                                self.desired_os_key_word)
        ui_steps.click_button(
            serial = self.serial,
            print_error = "Error - Language was not displayed",
            blocking = True,
            view_to_find = {"textContains": self.original_country})()
        self.uidevice(scrollable = True).scroll.to(textContains =\
                                        self.desired_language_key_word)
        ui_steps.click_button(
            serial = self.serial,
            print_error = "Error - Language was not displayed",
            blocking = True,
            view_to_find = {"textContains": self.desired_language_key_word})()

    def check_condition(self):
        return self.uidevice(textContains =\
                            self.desired_language_key_word)


class toogle_through_languages(ui_step):

    """ description:
            This function toogles between the languages supplied.
                desired_language_key_word = The language to which the
                    switch is done
                desired_original_key_word = A key word in Settings that
                    is translated in the original language
        usage:
            i18n_steps.toogle_through_languages(
                desired_language_key_word='xxxx',
                desired_language_check_word = 'xxxx',
                desired_original_key_word = 'xxxx',
                desired_original_check_word = 'xxxx'
                )()
        tags:
            i18n, android, Language, toogle
    """

    def __init__(self, desired_language_key_word,
                 desired_language_check_word,
                 desired_original_key_word = "United States",
                 desired_original_check_word = "Language",
                 **kwargs):
        ui_step.__init__(self, **kwargs)
        self.desired_language_key_word = desired_language_key_word
        self.desired_language_check_word = desired_language_check_word
        self.desired_original_key_word = desired_original_key_word
        self.desired_original_check_word = desired_original_check_word

    def do(self):
        open_language_section(serial = self.serial)()
        set_unset_desired_language(serial = self.serial,
                                   desired_language_key_word = self.desired_language_key_word,
                                   desired_language_check_word = desired_language_check_word,
                                   desired_original_key_word = self.desired_original_key_word,
                                   desired_original_check_word = self.desired_original_check_word)()

    def check_condition(self):
        # check is done by the last step
        return True


class os_language_changes_apps_language(ui_step):

    """ description:
            This function toogles between the supplied input languages
                desired_language_key_word = The language to which the
                    switch is done
                desired_original_key_word = the original language
                    Country name
        usage:
            i18n_steps.return_to_default_language(
                desired_language_key_word='xxxx',
                desired_language_check_word = 'xxxx',
                desired_original_key_word = 'xxxx',
                desired_original_check_word = 'xxxx'
                )()
        tags:
            i18n, android, Language, change language
    """

    desired_language_key_word = None
    desired_language_check_word = None
    desired_original_key_word = None
    desired_original_check_word = None

    def __init__(self, desired_language_key_word,
                 desired_language_check_word,
                 desired_original_key_word = "United States",
                 desired_original_check_word = "Language",
                 **kwargs):
        ui_step.__init__(self, **kwargs)
        self.desired_language_key_word = desired_language_key_word
        self.desired_original_key_word = desired_original_key_word

    def do(self):
        open_language_section(serial = self.serial,)()
        set_unset_desired_language(serial = self.serial,
                                   desired_language_key_word = self.desired_language_key_word,
                                   desired_language_check_word = self.desired_language_check_word,
                                   desired_original_key_word = self.desired_original_key_word,
                                   desired_original_check_word = self.desired_original_check_word)()

    def check_condition(self):
        # check is done by the last step
        return True

