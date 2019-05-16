from _prerequisites import *
from PyUiApi.common.system_utils import AccountAdder
from PyUiApi.common.uiautomator_utils import UiAutomatorUtils
import sys

if len(sys.argv) < 3:
    LOG.info("this test script has to be passed a valid google user account and password")
    sys.exit(2)

user = sys.argv[1]
password = sys.argv[2]
LOG.info("google user account:" + user)
LOG.info("google user password:" + password)

account_adder = AccountAdder()

try:
    UiAutomatorUtils.unlock_screen()
    if AccountAdder.is_google_account_active():
        if not account_adder.remove_current_google_account():
            raise Exception("could not remove google account")
    account_adder.start_add_account()\
                 .enter_email(user)\
                 .enter_pass(password)\
                 .accept_terms()\
                 .agree_backup()
    print "TEST_PASSED"
except:
    LOG.error("There was an error while setting up a google account")
finally:
    UiAutomatorUtils.close_all_tasks()