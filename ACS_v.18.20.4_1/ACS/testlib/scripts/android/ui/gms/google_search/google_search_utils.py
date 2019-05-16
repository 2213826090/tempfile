#!/usr/bin/env python

from testlib.utils.ui.uiandroid import UIDevice as ui_device
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
        uidevice = ui_device(serial = serial)
    else:
        uidevice = ui_device()
    xml_dump_lcase = uidevice.dump(compressed = False).lower()
    return xml_dump_lcase.count(keyword.lower()) > 1 or \
            uidevice(text = "Can't load search results").wait.exists(timeout = wait_time)
