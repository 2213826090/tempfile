#!/usr/bin/env python

from testlib.scripts.android.ui import ui_steps
from testlib.base import base_utils
from testlib.scripts.android.ui.ui_step import step as ui_step
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.android.adb.adb_step import step as adb_step
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.browser import browser_utils
from testlib.scripts.android.adb import adb_utils
from testlib.scripts.gms.docs import docs_utils
import time

class open_docs(ui_step):
    """
        description:
            opens docs app

        usage:
            open_docs()()

        tags:
            android, docs
    """
    def do(self):
        ui_steps.press_home()()
        ui_steps.open_app_from_allapps(serial = self.serial,
                                    view_to_find = {"textContains": "Docs"})()
        self.uidevice(resourceId="com.google.android.apps.docs.editors.\
            docs:id/doc_list_syncing_spinner").wait.gone(timeout=20000)
        if self.uidevice(text="Skip").wait.exists(timeout = 20000):
            ui_steps.click_button(serial = self.serial,
                                view_to_find = {"text":"Skip"},
                                view_to_check = {"description" : "Create new document"})()
        docs_utils.exit_search()
        self.uidevice(resourceId="com.google.android.apps.docs.editors.\
        docs:id/doc_list_syncing_spinner").wait.gone(timeout=20000)

    def check_condition(self):
        return self.uidevice(description="Create new document").\
            wait.exists(timeout = 20000)


class add_new_document(ui_step):
    """
        description:
            Adds a new document and writes some words in it.
            Check undo - redo functionality with undo=True.
        
        usage:
            add_new_document(serial = serial,
                            title = "New document",
                            content = "This is the content of my new document")()

        tags:
            android, docs, new doc
    """
    def __init__(self, title = "New document", undo = False,
                content = "This is the content of my new document", **kwargs):
            self.content = content
            self.title = title
            self.undo = undo
            ui_step.__init__(self, **kwargs)
    def do(self):
        open_docs(serial = self.serial)()
        ui_steps.click_button(serial = self.serial,
                                view_to_find = {"description" : "Create new document"},
                                view_to_check = {"description" : "Done"})()
        ui_steps.edit_text(serial = self.serial,
                        view_to_find = {"resourceId":"com.google.android.apps.docs.editors.docs:id/editor"},
                        value = self.content)()
        if self.undo:
            ui_steps.click_button(serial = self.serial,
                        view_to_find = {"description":"Undo"})()
            
            ui_steps.click_button(serial = self.serial,
                        view_to_find = {"description":"Redo"},
                        view_to_check = {\
            "resourceId":"com.google.android.apps.docs.editors.docs:id/editor",
            "text":self.content+'\n'})()

        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"description":"Done"},
                            view_to_check = {"description":"Navigate up"})()
        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"text":"Untitled document"},
                            view_to_check = {"text":"Rename document"})()
        ui_steps.edit_text(serial = self.serial,
                        view_to_find = {"resourceId":"com.google.android.apps.docs.editors.docs:id/new_name"},
                        value = self.title)()
                        
        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"text":"OK"},
                            view_to_check = {"text":self.title,
                                        "resourceId":"com.google.android.apps.docs.editors.docs:id/editor_action_bar_title"})()
        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"description":"Navigate up"},
                            view_to_check = {"text":"Docs"})()

    def check_condition(self):
        docs_utils.search_by(serial = self.serial, name = self.title)
        stat = ui_utils.view_exists(serial = self.serial, view_to_find=\
            {"resourceId":"com.google.android.apps.docs.editors.docs:id/title",
            "text":self.title})
        docs_utils.exit_search()
        return stat

class delete_document(ui_step):
    """
        description:
            Deletes a document
            
        usage:
            delete_document(serial = serial,
                            title = "New document")()

        tags:
            android, docs, delete document
    """
    def __init__(self, title = "New document", no_check = False, **kwargs):
        self.title = title
        self.no_check = no_check
        ui_step.__init__(self, **kwargs)
    def do(self):
        open_docs(serial = self.serial)()
        docs_utils.search_by(serial = self.serial, name = self.title)
        ui_utils.view_exists(serial = self.serial,
                            view_to_find = {"text":self.title})
        attempts = 0
        while not self.uidevice(text="Remove").wait.exists(timeout=5000)\
            and attempts < 4:
            self.uidevice(text=self.title).\
                sibling(resourceId = "com.google.android.apps.docs.editors.docs:id/more_actions_button").click()
            attempts += 1
        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"text":"Remove"},
                            view_to_check = {"text":"Do you really want to remove this file?"})()
        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"text":"Remove"},
                            view_to_check = {"description" : "Close search"})()

    def check_condition(self):
        if self.no_check:
            print "deleted one"
            stat = True
        else:
            stat = self.uidevice(resourceId="com.google.android.apps.docs.editors.docs:id/title",\
                                text=self.title).wait.gone(timeout=10000)
        docs_utils.exit_search()
        return stat
class rename_document(ui_step):
    """
        description:
            Renames an existing document.
            
        usage:
            rename_document(serial = serial,
                            title = "New document",
                            new_title = "Updated document")()

        tags:
            android, docs, new doc
    """
    def __init__(self, title = "New document", cancel = False,
                new_title = "Updated document", **kwargs):
            self.new_title = new_title
            self.title = title
            self.cancel = cancel
            ui_step.__init__(self, **kwargs)
    def do(self):
        open_docs(serial = self.serial)()
        docs_utils.search_by(self.title, serial = self.serial)
        attempts = 0
        while not self.uidevice(text="Rename").wait.exists(timeout=5000)\
            and attempts < 4:
            self.uidevice(text=self.title).\
                sibling(resourceId = "com.google.android.apps.docs.editors.docs:id/more_actions_button").click()
            attempts += 1
        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"text":"Rename"},
                            view_to_check = {"text":"Rename document"})()
        ui_steps.edit_text(serial = self.serial,
                        view_to_find = {"resourceId":"com.google.android.apps.docs.editors.docs:id/new_name"},
                        value = str(self.new_title))()
        if self.cancel:
            ui_steps.click_button(serial = self.serial,
                            view_to_find = {"text":"Cancel"},
                            view_to_check = {"description" : "Close search"})()
        else:
            ui_steps.click_button(serial = self.serial,
                            view_to_find = {"text":"OK"},
                            view_to_check = {"description" : "Close search"})()
        
    def check_condition(self):
        docs_utils.exit_search()
        if self.cancel:
            to_check = self.title
        else:
            to_check = self.new_title
        docs_utils.search_by(name = to_check, serial = self.serial)
        stat = ui_utils.view_exists(view_to_find = {"text":to_check})
        docs_utils.exit_search()
        return stat
class replace_in_document(ui_step):
    """
        description:
            Replaces a word in a document. Attempts to replace a number of 
            <count> occurences.
            
        usage:
            rename_document(serial = serial,
                            title = "New document",
                            to_replace = "word",
                            replaced_by = "new",
                            count = 1,
                            undo = True)()

        tags:
            android, docs, new doc
    """
    def __init__(self, title, to_replace, replaced_by, count = 1,
                undo = True,**kwargs):
            self.count = count
            self.title = title
            self.to_replace = to_replace
            self.replaced_by = replaced_by
            self.undo = undo
            ui_step.__init__(self, **kwargs)
    def do(self):
        open_docs(serial = self.serial)()
        docs_utils.search_by(self.title, serial = self.serial)
        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"resourceId":"com.google.android.apps.docs.editors.docs:id/title",
                                            "text":self.title},
                            view_to_check = {"description":"More options"})()
        self.initial_count = self.uidevice(\
            resourceId = "com.google.android.apps.docs.editors.docs:id/editor").\
            info["text"].count(self.to_replace)
        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"description":"More options"},
                            view_to_check = {"textContains":"Find and replace"})()
        ui_steps.click_button(serial = self.serial,
                    view_to_find = {"textContains":"Find and replace"},
                    view_to_check = {"textContains":"Find"})()
        ui_steps.edit_text(serial = self.serial,
                        view_to_find = {"textContains":"Find"},
                        value = self.to_replace)()
        self.uidevice.press("enter")
        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"description":"More options"},
                            view_to_check = {"text":"Replace"})()
        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"text":"Replace"},
                            view_to_check = {"resourceId":"com.google.android.apps.docs.editors.docs:id/findreplace_replace_text"})()
        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"textContains":self.to_replace},
                            view_to_check = {"resourceId":"com.google.android.apps.docs.editors.docs:id/findreplace_replace_text"})()
        ui_steps.edit_text(serial = self.serial,
                        view_to_find = {"resourceId":"com.google.android.apps.docs.editors.docs:id/findreplace_replace_text"},
                        value = self.replaced_by)()
        self.current_count = self.initial_count
        while (self.initial_count - self.current_count) < self.count and self.current_count > 0:
            ui_steps.click_button(serial = self.serial,
                            view_to_find = {"text":"Replace"})()
            self.current_count = self.uidevice(\
            resourceId = "com.google.android.apps.docs.editors.docs:id/editor").\
            info["text"].count(self.to_replace)
        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"description":"Done"},
                            view_to_check = {"resourceIdMatches":".*/start_editing_button"})()
        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"resourceId":"com.google.android.apps.docs.editors.docs:id/editor"})()
        self.uidevice(resourceIdMatches=".*/start_editing_button").wait.gone(timeout=10000)
        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"resourceId":"com.google.android.apps.docs.editors.docs:id/editor"},
                            view_to_check = {"description":"Navigate up"})()
        docs_utils.exit_search()
    def check_condition(self):
        return (self.current_count+self.count == self.initial_count or self.current_count == 0)
class insert_comment(ui_step):
    """
        description:
            Inserts a comment in a specified document. You may choose to cancel
            adding it by setting cancel to True
            
        usage:
            insert_comment(serial = serial,
                            title = "New document",
                            comment = "comment",
                            cancel = False)()

        tags:
            android, docs, comment
    """
    def __init__(self, title, comment, cancel = False, **kwargs):
        self.comment = comment
        self.title = title
        self.cancel = cancel
        ui_step.__init__(self, **kwargs)
    def do(self):
        open_docs(serial = self.serial)()
        docs_utils.search_by(self.title, serial = self.serial)
        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"resourceId":"com.google.android.apps.docs.editors.docs:id/title",
                                            "text":self.title},
                            view_to_check = {"descriptionContains":"rename button"})()
        ui_steps.click_button(serial = self.serial,
            view_to_find = {"resourceId":"com.google.android.apps.docs.editors.docs:id/editor"},
            view_to_check = {"description":"Insert"})()
        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"description":"Insert"},
                            view_to_check = {"text":"Comments"})()
        ui_steps.click_button(serial = self.serial,
            view_to_find = {"text":"Comments"},
            view_to_check = {"resourceId":"com.google.android.apps.docs.editors.docs:id/comment_text"})()
        ui_steps.edit_text(serial = self.serial,
            view_to_find = {"resourceId":"com.google.android.apps.docs.editors.docs:id/comment_text"},
            value = self.comment)()
        if self.cancel:
            ui_steps.click_button(serial = self.serial,
                            view_to_find = {"text":"Cancel"},
                            view_to_check = {"description":"Navigate up"})()
        else:
            ui_steps.click_button(serial = self.serial,
                            view_to_find = {"text":"Comment"},
                            view_to_check = {"text":"Resolve"})()

    def check_condition(self):
        return self.cancel != self.uidevice(text="Resolve").exists
class keep_document(ui_step):
    """
        description:
            Sets document keep on device property to a specified value.
            
        usage:
            keep_document(serial = serial,
                            title = "New document",
                            keep_on_device = True)()

        tags:
            android, docs, keep on device
    """
    def __init__(self, title = "New document", keep_on_device = False, **kwargs):
            self.title = title
            self.keep_on_device = keep_on_device
            ui_step.__init__(self, **kwargs)
    def do(self):
        open_docs(serial = self.serial)()
        docs_utils.search_by(self.title, serial = self.serial)
        self.uidevice.press("back")
        self.uidevice(text=self.title).\
            sibling(description = "Open the document actions menu").click()
        keep = self.uidevice(text="Keep offline",\
                            description = "Unpin from device").exists
        if (keep and not self.keep_on_device) or \
            (not keep and self.keep_on_device):
            ui_steps.click_button(serial = self.serial,
                            view_to_find = {"text":"Keep offline"},
                            view_to_check = {"description":"Close search"})()
        else:
            self.uidevice.press("back")
        docs_utils.exit_search()
    def check_condition(self):
        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"description":"Open navigation drawer"},
                            view_to_check = {"text":"Offline"})()
        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"text":"Offline"},
                            view_to_check = {"description":"Open navigation drawer"})()
        document_exists = ui_utils.view_exists(view_to_find = {"text":self.title})
        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"description":"Open navigation drawer"},
                            view_to_check = {"text":"Recent"})()
        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"text":"Recent"},
                            view_to_check = {"description":"Open navigation drawer"})()
        return self.keep_on_device == document_exists
