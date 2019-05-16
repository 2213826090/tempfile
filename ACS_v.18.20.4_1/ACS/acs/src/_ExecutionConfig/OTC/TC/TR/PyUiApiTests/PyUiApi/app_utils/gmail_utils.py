from PyUiApi.common.uiautomator_utils import *


class Gmail(object):

    @staticmethod
    def select_sync_account_now():
        if d(resourceId=GMAIL_SYNC_NOW_RESID).wait.exists(timeout=3000):
            d(resourceId=GMAIL_SYNC_NOW_RESID).click()
        time.sleep(2)
