import sys

from testlib.base.base_utils import get_args
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils
from testlib.scripts.gms.PlayGames import play_games_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.gms import gms_utils


################################################################################
# Fetch parameters passed to the script
################################################################################
args = get_args(sys.argv)
globals().update(vars(args))
globals().update(eval(script_args[0]))
adb_steps.root_connect_device(serial = serial, port = adb_server_port)()
globals().update({"version": adb_utils.get_android_version()})

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
play_games_steps.open_play_games_settings(serial = serial)()
ui_steps.close_all_app_from_recent(serial = serial)()
