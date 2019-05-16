#!/usr/bin/env python

from testlib.utils.connections.adb import Adb
from testlib.utils.ui import uiandroid
from testlib.scripts.android.adb import adb_utils

def evaluate_search(serial, keyword, wait_time = 20000):
    """ description:
           Attempts a very light evaluation of the search results.
           Only considers the presence of a keyword in the dump xml.

        usage:
            google_search_utils.evaluate_search(serial = serial,
                                                keyword = "somestring")

        tags:
            ui, android, click, app, scroll
    """
    if serial:
        uidevice = uiandroid.UIDevice(serial = serial)
    else:
        uidevice = uiandroid.UIDevice()
    xml_dump_lcase = uidevice.dump(compressed = False).lower()
    print xml_dump_lcase
    print xml_dump_lcase.count(keyword.lower())
    return xml_dump_lcase.count(keyword.lower()) > 1 or \
            uidevice(text = "Can't load search results").wait.exists(timeout = wait_time)
