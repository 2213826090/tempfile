#!/usr/bin/env python

##############################################################################
#
# @filename:    playstore_steps.py
# @description: UI Playstore test steps
# @author:      ion-horia.petrisor@intel.com
#
##############################################################################

from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.android.ui.ui_step import step as ui_step
from testlib.scripts.gms.books import books_utils
import random


class open_my_library(ui_step):

    """ description:
            opens the "My Library" page from Google Books

        usage:
            books_steps.open_my_library()()

        tags:
            ui, android, gms, books, open, library
    """

    def do(self):
        ui_steps.open_google_books(serial = self.serial)()
        books_utils.books_home(serial = self.serial)
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"description": "Show navigation drawer"},
                              view_to_check = {"text": "My Library"})()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"text": "My Library"})()

    def check_condition(self):
        self.uidevice.wait.idle()
        return self.uidevice(text = "ALL BOOKS").exists and \
               self.uidevice(text = "PURCHASES").exists and \
               self.uidevice(text = "SAMPLES").exists


class open_purchases_from_library(ui_step):

    """ description:
            opens the "Purchases" tab from m"My Library".
            it checks that none of the books does not have "Sample" attribute

        usage:
            books_steps.open_purchases_from_library()()

        tags:
            ui, android, gms, books, open, library, purchases
    """

    def do(self):
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"text": "PURCHASES"})()

    def check_condition(self):
        for i in range(6):
            try:
                self.uidevice(resourceId = "com.google.android.apps.books:id/li_infobox",\
                              instance = i).child("com.google.android.apps.books:id/li_price")
                return False
            except Exception, e:
                continue
        return True


class open_samples_from_library(ui_step):

    """ description:
            opens the "Purchases" tab from "My Library".
            it checks that all books have "Sample" attribute

        usage:
            books_steps.open_samples_from_library()()

        tags:
            ui, android, gms, books, open, library, samples
    """

    def do(self):
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"text": "SAMPLES"})()

    def check_condition(self):
        for i in range(6):
            book_view = self.uidevice(resourceId = "com.google.android.apps.books:id/li_price",\
                                      instance = i)
            if book_view.info["text"] == "SAMPLE":
                continue
            else:
                return False
        return True


class open_all_from_library(ui_step):

    """ description:
            opens the "All Books" tab from "My Library".
            it checks that the first row of books have "Sample" attribute
            and that the second row does not

        usage:
            books_steps.open_samples_from_library()()

        tags:
            ui, android, gms, books, open, library, samples
    """

    def do(self):
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"text": "ALL BOOKS"})()

    def check_condition(self):
        for i in range(3):
            book_view = self.uidevice(resourceId = "com.google.android.apps.books:id/li_price",
                                      instance = i)
            if book_view.info["text"] == "SAMPLE":
                continue
            else:
                return False
        for i in range(3,6):
            try:
                self.uidevice(resourceId = "com.google.android.apps.books:id/li_infobox",\
                              instance = i).child("com.google.android.apps.books:id/li_price")
                return False
            except Exception, e:
                continue
        return True
        # check first three from purchase and first three fr, samples


class book_exists(ui_step):

    """ description:
            checks that the book with <title> title and <author> author
            exists in <category> category from "My Lirary"
            category = "ALL BOOKS"/"PURCHASES"/"SAMPLES"

        usage:
            books_steps.book_exists(serial = self.serial,
                                    category = "PURCHASES",
                                    title = "Great Expectations",
                                    author = "Charles Dickens")()

        tags:
            ui, android, gms, books, exists, library
    """

    def __init__(self, category, title, author, **kwargs):
        self.category = category
        self.title = title
        self.author = author
        ui_step.__init__(self, **kwargs)

    def do(self):
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"text": self.category})()

    def check_condition(self):
        title_check = self.uidevice(resourceId = "com.google.android.apps.books:id/li_title",
                                    text = self.title).exists
        author_check = self.uidevice(resourceId = "com.google.android.apps.books:id/li_subtitle",
                                     text = self.author).exists
        return title_check and author_check


class open_first_page_book(ui_step):

    """ description:
            opens (randomly) a book from the first page

        usage:
            books_steps.open_first_page_book()()

        tags:
            ui, android, gms, books, exists, library
    """

    def do(self):
        ui_steps.open_google_books(serial = self.serial)()
        books_utils.books_home(serial = self.serial)
        book_no = random.randint(0,2)
        self.uidevice(text = "Recommended for you").wait.exists(timeout = 10000)
        ui_steps.dump(out_file = "gigi.xml")()

    def check_condition(self):
        return True
