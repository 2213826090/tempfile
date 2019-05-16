from uiautomator_utils import *


class StatusBar(object):
    @staticmethod
    def open_notifications(nr_of_swipes=1):
        width, height = UiAutomatorUtils.get_screen_dims()
        sx = width / 2
        sy = height / 100
        ex = sx
        ey = int(height * 0.6)
        for i in range(nr_of_swipes):
            d.swipe(sx, sy, ex, ey)
            time.sleep(0.5)
        time.sleep(2)

    @staticmethod
    def close_notifications():
        # notifications are at most 2 back presses away from closing
        if not d(resourceId=NOTIFICATIONS_BATTERY_RESID).wait.exists(timeout=10000):
            LOG.info('Failed to find %s' % NOTIFICATIONS_BATTERY_RESID)
            return False
        d.press.back()
        if not d(resourceId=NOTIFICATIONS_BATTERY_RESID).wait.gone(timeout=5000):
            d.press.back()
        return d(resourceId=NOTIFICATIONS_BATTERY_RESID).wait.gone(timeout=5000)

    @staticmethod
    def get_nr_of_notifications():
        if d(resourceId=NOTIFICATIONS_MAIN_COLUMN_RESID).wait.exists(timeout=2000):
            return d(resourceId=NOTIFICATIONS_MAIN_COLUMN_RESID).count
        elif d(resourceId=NOTIFICATIONS_NOTIFICATION_TITLE_RESID).wait.exists(timeout=2000):
            return d(resourceId=NOTIFICATIONS_NOTIFICATION_TITLE_RESID).count
        else:
            return 0

    @staticmethod
    def click_on_first_status_element():
        if d(resourceId=NOTIFICATIONS_MAIN_COLUMN_RESID).wait.exists(timeout=2000):
            return d(resourceId=NOTIFICATIONS_MAIN_COLUMN_RESID).click()
        elif d(resourceId=NOTIFICATIONS_NOTIFICATION_TITLE_RESID).wait.exists(timeout=2000):
            return d(resourceId=NOTIFICATIONS_NOTIFICATION_TITLE_RESID).click()
        else:
            return 0

    @staticmethod
    def get_first_status_element():
        if d(resourceId=NOTIFICATIONS_MAIN_COLUMN_RESID).wait.exists(timeout=3000):
            return d(resourceId=NOTIFICATIONS_MAIN_COLUMN_RESID)[0].text
        elif d(resourceId=NOTIFICATIONS_NOTIFICATION_TITLE_RESID).wait.exists(timeout=2000):
            return d(resourceId=NOTIFICATIONS_NOTIFICATION_TITLE_RESID)[0].text
        else:
            return 0

    @staticmethod
    def clear_notifications():
        if d(resourceId=NOTIFICATIONS_CLEAR_ALL_RESID).wait.exists(timeout=2000):
            d(resourceId=NOTIFICATIONS_CLEAR_ALL_RESID).click()

    @staticmethod
    def hang_phone():
        if d(text=DIALER_HANG_UP_TXT).wait.exists(timeout=5000):
            d(text=DIALER_HANG_UP_TXT).click()

    @staticmethod
    def answer_phone():
        if not d(text=DIALER_ANSWER_TXT).wait.exists(timeout=30000):
            LOG.info('Failed to find %s' % DIALER_ANSWER_TXT)
            return False
        time.sleep(1)  # Wait for the UI to sync
        d(text=DIALER_ANSWER_TXT).click()
        if not d(text=DIALER_ANSWER_TXT).wait.gone(timeout=10000):
            LOG.info('Failed to answer.')
            return False
        return True

    @staticmethod
    def dismiss_phone():
        if not d(text=DIALER_DISMISS_TXT).wait.exists(timeout=30000):
            LOG.info('Failed to find %s' % DIALER_DISMISS_TXT)
            return False
        time.sleep(2)
        d(text=DIALER_DISMISS_TXT).click()
        if not d(text=DIALER_DISMISS_TXT).wait.gone(timeout=10000):
            LOG.info('Failed to dismiss.')
            return False
        return True

    @staticmethod
    def toggle_airplane_mode(mode):
        """
        Toggle airplane mode ON/OFF
        :param mode: ON/OFF
        :return: True if success else None
        """
        # TODO: this should be moved outside this function when more
        #       toggles are added to it
        notifications_toggles = {'Airplane mode': {'ON': NOTIFICATIONS_AIRPLANE_ON_DESC,
                                                   'OFF': NOTIFICATIONS_AIRPLANE_OFF_DESC}}

        def negate_mode(mode):
            return 'ON' if mode == 'OFF' else 'OFF'

        if mode not in notifications_toggles['Airplane mode'].keys():
            LOG.info("'%s' not a valid option (ON/OFF)." % mode)
            return None
        if mode == StatusBar.get_airplane_mode():
            LOG.info("Already on mode %s." % mode)
            return True
        else:
            d(description=notifications_toggles['Airplane mode'][negate_mode(mode)]).click.wait()
        # Check if toggle switched states
        return True if mode == StatusBar.get_airplane_mode() else None

    @staticmethod
    def get_airplane_mode():
        """
        Get airplane mode status
        :return: ON/OFF
        """
        if d(description=NOTIFICATIONS_AIRPLANE_ON_DESC).wait.exists(timeout=3000):
            return 'ON'
        elif d(description=NOTIFICATIONS_AIRPLANE_OFF_DESC).wait.exists(timeout=3000):
            return 'OFF'
        else:
            LOG.info('Failed to determine Airplane mode.')
            return None

