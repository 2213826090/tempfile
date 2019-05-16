"""
@summary: Class for notification panel functionality
@since: 07/09/2014
@author: yongzoux (yongx.zou@intel.com)
"""
import time
from testlib.util.common import g_common_obj

class VPNInNofifyPanel(object):
    ''' class for manipulate VPN in notification panel '''


    def __init__(self, cfg={}):
        """
        init
        """
        self.d = g_common_obj.get_device()
        self.cfg = cfg

    @property
    def __vpn_in_notify(self):
        """ UI object display vpn status in notification panel """
        return self.d(textContains="Network may be monitored")

    @property
    def __btn_disconnect(self):
        """ UI button 'Disconnect' """
        return self.d(text='Disconnect VPN')

    @property
    def __btn_cancel(self):
        """ UI button 'Cancel' """
        return self.d(text='Cancel')

    def discon_vpn(self):
        """ Disconnect VPN in notification panel """
        if self.__vpn_in_notify.exists is False:
            return False
        self.__vpn_in_notify.click()
        time.sleep(1)
        self.__btn_disconnect.click()
        return True


class NotifyPanelImpl(object):
    ''' Notification panel
    '''

    def __init__(self, cfg={}):
        self._device = g_common_obj.get_device()
        self.cfg = cfg

    @property
    def __notification_panel(self):
        """
            notification panel
        """
        return self._device(
            resourceId="com.android.systemui:id/notification_panel")

    @property
    def __notification_list(self):
        """
            notification list
        """
        return self._device(resourceId="com.android.systemui:id/latestItems")

    def __notification(self, message):
        """ UI notification appeared in Notification Bar """
        return self._device(textContains=message)

    @property
    def __btn_clear_all_notifications(self):
        """ UI image button 'Clear all notifications.' in notification bar """
        return self._device(description="Clear all notifications.")

    def open_notification_panel(self):
        """ Open notification panel """
        self._device.open.quick_settings()
        time.sleep(2)
        self._device.open.quick_settings()

    def hidden_notification_panel(self):
        """ Hidden notificaion panel """
        self.__notification_panel.swipe.up()

    @staticmethod
    def check_music_player_icon_show():
        """ Check music player icon show on status bar
        """
        print "[INFO] Check music player icon show on status bar"
        cmd = 'dumpsys statusbar |\
        grep "StatusBarNotification(pkg=com.google.android.music"; echo $?'
        loop = 10
        find = False
        while (loop > 0):
            res_list = g_common_obj.adb_cmd_capture_msg(cmd).split('\r\n')
            ret = int(res_list[-1].strip())
            if ret == 0:
                find = True
                break
            time.sleep(3)
            loop -= 1
        assert find, "[ERROR] Music player icon not show on status bar"

    def check_density(self):
        """ Check density
        """
        print "[INFO] Check density"
        cmd = 'getprop | grep density; echo $?'
        res_list = g_common_obj.adb_cmd_capture_msg(cmd).split('\r\n')
        ret = int(res_list[-1].strip())
        assert ret == 0, "[ERROR] Check device density fail"
        print "[INFO] Device actual density %s" % res_list[0]
        # get densify from config file
        densify_in_config = self.cfg.get("density")
        print "[INFO] Density in config file: %s" % densify_in_config
        message = "[ERROR] Device's density is not %d" % densify_in_config
        assert densify_in_config in res_list[0], message

    def open_andftp(self):
        """ Open AndFTP from notification panel """
        print "[INFO] Open AndFTP from Notification Bar"
        self.open_notification_panel()
        self._device(textContains="AndFTP").click()

    def check_notification_exists(self, message):
        """ Check specified notification appear in Notification Bar
        @param message:(string)  message appeared in Notification Bar
        @return: True = appear, False = not exists
        """
        print "[INFO] Check Notification: '%s'" % message
        self.open_notification_panel()
        time.sleep(5)
        exists = self.__notification(message).exists
        self.hidden_notification_panel()
        return exists

    def clear_all_notifications(self):
        """ Clear all notifications """
        print "[INFO] Clear all notifications"
        self._device.open.notification()
        time.sleep(2)
        if self.__btn_clear_all_notifications.exists:
            self.__btn_clear_all_notifications.click()
        else:
            if self.__notification_panel.exists:
                self.hidden_notification_panel()

    def clear_download_notification(self):
        """ Clear all Download complete notification in Notification Bar """
        print "[INFO] Clear all 'Download complete' notification"
        self.open_notification_panel()
        time.sleep(2)
        for _ in range(0, 10):
            for item in self.__notification_list.\
            child(className="android.widget.FrameLayout"):
                if item.sibling(textContains="Download complete").exists:
                    item.swipe.right()
        self.hidden_notification_panel()

    def wait_download_complete_in_notifiybar(self, timeout):
        """ Check 'Download complete' appear in Notification Bar
        @param timeout: seconds to wait
        """
        print "[INFO] Wait Notification: 'Download complete'"
        appear = False
        start_time = time.time()
        _timeout = timeout
        msg = "[ERROR] Notification: \
        'Download complete' not appear in %d seconds" % _timeout
        while timeout > 0:
            # check 'Download failed' in Notification Bar
            if self.check_notification_exists("Download failed"):
                msg = "[ERROR] 'Download failed' appear in Notification Bar"
                break
            # check 'Download complete' in Notification Bar
            if self.check_notification_exists("Download complete"):
                msg = "[INFO] 'Download complete' appear in Notification Bar"
                appear = True
                break
            elapse_time = time.time() - start_time
            # if over timeout
            if elapse_time > timeout:
                break
            time.sleep(10)
            timeout -= 10
        assert appear, msg
