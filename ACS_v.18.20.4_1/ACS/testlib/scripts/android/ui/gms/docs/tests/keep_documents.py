import sys

from testlib.base.base_utils import get_args
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.gms.docs import docs_steps
from testlib.scripts.gms.docs import docs_utils
from testlib.scripts.gms import gms_utils


################################################################################
# Fetch parameters passed to the script
################################################################################
args = get_args(sys.argv)
globals().update(vars(args))
globals().update(eval(script_args[0]))
adb_steps.root_connect_device(serial = serial, port = adb_server_port)()
globals().update({"version": adb_utils.get_android_version()})

TITLE = "kd_{0}".format(serial)
ACCOUNT = account
PASSWORD = password
WHERE = 'name = "{0}"'.format(ACCOUNT)

account_exists = ui_utils.google_account_exists(serial = serial, where = WHERE)
total_account_no = gms_utils.get_google_account_number(serial = serial)

if account_exists:
    account_synced = ui_steps.sync_google_account(serial = serial,
                            account = ACCOUNT,
                            password = PASSWORD)()

if (total_account_no >= 2) or (not account_exists) or (account_exists and not account_synced):
    ui_steps.remove_all_google_accounts(serial = serial)()
    ui_steps.add_google_account_for_L(serial = serial,
                                account = ACCOUNT,
                                password = PASSWORD,
                                prefer_sync = True)()

# remove document if exists
docs_steps.open_docs(serial = serial)()
docs_utils.search_by(serial = serial, name = TITLE)
while ui_utils.view_exists({"text":TITLE, "resourceIdMatches":".*title"}, serial = serial):
    print "found one"
    docs_steps.delete_document(serial = serial,
                            title = TITLE,
                            no_check = True)()
    docs_utils.search_by(serial = serial, name = TITLE)
docs_utils.exit_search()

# create new document
docs_steps.add_new_document(title = TITLE,
                            undo = True)()

# set keep to true
docs_steps.keep_document(title = TITLE,
                        keep_on_device = True)()

# set keep to true
docs_steps.keep_document(title = TITLE,
                        keep_on_device = False)()

# delete the document
docs_steps.delete_document(title = TITLE)()
ui_steps.close_all_app_from_recent(serial =  serial)()
