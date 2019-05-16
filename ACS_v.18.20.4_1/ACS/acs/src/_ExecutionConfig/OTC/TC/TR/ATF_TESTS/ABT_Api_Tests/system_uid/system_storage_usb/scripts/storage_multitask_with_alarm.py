from datetime import datetime, timedelta
import os.path
from time import sleep
import sys
from multiprocessing import Pool
from test_utils import *

if os.getenv('ACS_EXECUTION_CONFIG_PATH') is not None:
    sys.path.insert(0, (os.getenv('ACS_EXECUTION_CONFIG_PATH') + '/TC/TP'))
else:
    sys.path.insert(0,(os.getcwd() + '/_ExecutionConfig/OTC/TC/TP'))

from testlib.util.common import g_common_obj
from testlib.androidframework.common import *
from testlib.androidframework.dut_manager import dut_manager
from adb_helper.adb_utils import *

class StorageCheckMultiTask(unittest.TestCase):
    '''
    Description : Check for the device multitasking/background activity capability
    Usage       : Set an Alarm and check for the apps operation when the alarm
                notification is shown.
    ToDo        :
    '''

    TAG = "StorageCheckMultiTask"

    def setUp(self):
        print self.TAG, "setUp"
        self.d = g_common_obj.get_device(dut_manager.active_uiautomator_device_serial)

    def tearDown(self):
        print self.TAG, "tearDown"


    def test_multitasking_while_alarm_expire(self):
        if self.launch_clock():
            self.setting_alarm_time()
            self.task_ops()
            self.d.wait.update()
            self.check_alarm_notification()

        else:
            self.assertFalse("Clock App failed to launch")

    def task_ops(self):
        self.d.press.home()
        self.launch_camera()
        self.d(resourceId="com.android.camera2:id/shutter_button",packageName="com.android.camera2").click()
        sleep(2)
        self.d(packageName="com.android.camera2", resourceId = "com.android.camera2:id/rounded_thumbnail_view").click()
        self.d(packageName="com.android.camera2", resourceId = "com.android.camera2:id/filmstrip_bottom_control_delete").click()
        sleep(1)
        del_btn = self.d(packageName="com.android.camera2", text= "Deleted")
        if not del_btn.exists:
            self.assertTrue("Image could not be deleted")
        del_btn.click()
        self.launch_video()
        self.start_stop_cam()
        sleep(30)
        self.start_stop_cam()
        self.launch_photos()
        self.d(packageName='com.google.android.apps.photos', description='Video').click()
        self.d(packageName='com.google.android.apps.photos', resourceId='com.google.android.apps.photos:id/video_player_controller_fragment_container').click()
        self.d(resourceId='com.google.android.apps.photos:id/delete_device_copy').click()
        self.d(className='android.widget.Button',packageName='com.google.android.apps.photos',text='Delete').click()

    def setting_alarm_time(self):
        cur_time = AdbUtils.run_adb_cmd('date')
        cur_time = cur_time.split(' ')[3]
        fr_cur_time = datetime.datetime.strptime(cur_time, '%H:%M:%S')
        alarm_time = fr_cur_time + timedelta(seconds=180)
        hour_hand = int(alarm_time.time().strftime('%I'))
        minute_hand = int(alarm_time.time().strftime('%M'))
        minute_hand = int(minute_hand + 5 - (minute_hand % 5))
        am_pm_label = alarm_time.time().strftime('%p')
        print "Setting Alarm to {0}:{1} {2}".format(hour_hand, minute_hand, am_pm_label)
        self.set_alarm(hour_hand, minute_hand, am_pm_label)

    def set_alarm(self,hour_hand,minute_hand,am_pm_label):
        self.d(packageName="com.google.android.deskclock", description="Alarm").click()
        self.clear_alarm()
        self.d(packageName="com.google.android.deskclock", description="Add alarm").click()
        #set Hours
        self.d(packageName='com.google.android.deskclock', resourceId="android:id/hours").click()
        self.d(packageName='com.google.android.deskclock', index=hour_hand-1).click()
        #set Minutes
        self.d(packageName='com.google.android.deskclock', description=minute_hand).click()
        #set AM or PM
        self.d(packageName='com.google.android.deskclock', text=am_pm_label).click()
        #click OK
        self.d(packageName='com.google.android.deskclock', text='OK').click()
        set_time = self.d(packageName='com.google.android.deskclock', resourceId="com.android.deskclock:id/digital_clock").text
        print "set_time: ",set_time
        import unicodedata
        a_time = unicodedata.normalize('NFKD', set_time).encode('ascii', 'ignore')
        s_time = datetime.datetime.strptime(a_time, '%I:%M %p')
        print "Alarm Set to ",s_time.time()
        hr,mn,label = s_time.time().strftime('%I'),s_time.time().strftime('%M'),s_time.time().strftime('%p')
        if not ((int(hr)==hour_hand) and (int(mn)==minute_hand) and (str(label)==am_pm_label)):
            self.assertFalse("Alarm set to Wrong time")

    def clear_alarm(self):
        arrow = self.d(packageName='com.google.android.deskclock',resourceId='com.android.deskclock:id/arrow')
        ar_del = self.d(packageName='com.google.android.deskclock',resourceId='com.android.deskclock:id/delete')
        if arrow.exists:
            arrow_count = arrow.count
            for count in range(arrow_count):
                arrow.click()
                ar_del.click()
            print "Previous Alarm Deleted"
        else:
            print "No previous alarm found to clear"


    def check_alarm_notification(self):
        if (self.d(text='Alarm').wait.exists(timeout=3000)):
            self.d(text='Dismiss').click()
            sleep(10)

    def launch_clock(self):
        cmd = "am start com.google.android.deskclock"
        AdbUtils.run_adb_cmd(cmd)
        return True

    def launch_video(self):
        cmd= "am start -a android.media.action.VIDEO_CAPTURE"
        AdbUtils.run_adb_cmd(cmd)
        return True

    def launch_camera(self):
        cmd = "am start -a android.media.action.IMAGE_CAPTURE"
        AdbUtils.run_adb_cmd(cmd)
        return True

    def start_stop_cam(self):
        cmd = "input keyevent KEYCODE_CAMERA"
        AdbUtils.run_adb_cmd(cmd)
        return True

    def launch_photos(self):
        cmd = "am start com.google.android.apps.photos"
        AdbUtils.run_adb_cmd(cmd)
        return True
