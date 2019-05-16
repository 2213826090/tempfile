# coding: UTF-8
import commands
import os
import time
from testlib.util.common import g_common_obj

class android_framework_Impl:
    """
    Implements System android framework actions.
    """

    def __init__ (self, cfg = None):
        self.d = g_common_obj.get_device()
    
    
    """
    check if density is 213
    """
    def check_density(self,density):
        density_cmd="adb shell getprop | grep density_info"
        getprop_density=os.popen(density_cmd).read()
        print  "density : %s"% getprop_density 
        result=getprop_density.count(density)
        assert result>0, "[ERROR]: density is not "+density
        print "[INFO]: density is "+density
        
    """
    check if "adb shell halctl -l" is success
    """   
    def check_halctl_list(self):
        halctl_cmd="adb shell halctl -l"
        halctl_list=os.popen(halctl_cmd).read()
        print  "density : %s"% halctl_list 
        result=halctl_list.count("modalias")
        assert result>100, "[ERROR]: halctl -list is fail"
        print "[INFO]: halctl -list is success"
        
    """
    check if "adb shell kmod -n module" is success
    kmod list =adb shell halctl -l | grep kmod
    xuyuhui@xuyuhui:~/work/oat-L$ adb shell halctl -l | grep kmod
    kmod:"bmm050"
    kmod:"bmc150_accel"
    kmod:"jsa1127"
    kmod:"bmg160"
    kmod:"mt9m114"
    kmod:"ov5693"
    kmod:"rfkill_gpio"
    kmod:"rfkill_gpio"
    kmod:"atomisp_css2400b0_v21"
    kmod:"8723bs"
    kmod:"8250_dw"
    kmod:"i2c_hid"
    kmod:"spi_pxa2xx_platform"
    """    
    def check_kmod_n(self):
#         res = commands.getoutput('adb root')
#         print res
        search_module_str="adb shell halctl -l | grep kmod"
        kmod_list=os.popen(search_module_str).read()
        kmod=kmod_list.split("kmod:")
        lines = [x.strip() for x in kmod]
#         kmod=kmod_list.replace("\n","")
#         print kmod
#         kmod=kmod.replace("\r","")
#         print kmod
#         kmod=kmod.replace("\t","")
        print lines
        kmod_value=os.popen("adb shell kmod -n %s"%lines[3]).read()
        print kmod_value
        kmod_result=kmod_value.find("filename")
        print kmod_result
        assert kmod_result>-1, "[ERROR]: kmod -n %s is fail"%lines[3]
        print "[INFO]: kmod -n %s is success"%lines[3]
#         print kmod
#         traversal kmod
#         del kmod[0]
#         for ele in kmod:    
#             kmod_value=os.popen("adb shell kmod -n %s"%ele).read()
#             print kmod_value
#         kmod1=kmod
#         print "kmd /n",kmod1
        
#         result=halctl_list.count("modalias")
#         assert result>100, "[ERROR]: halctl -list is fail"
#         print "[INFO]: halctl -list is successful"

    def launch_settings(self):
        '''
        Launch Settings app.
        '''
        print "[Info] ---Launch Settings app."
        g_common_obj.launch_app_am("com.android.settings", ".Settings")
        for i in range(10):
            if self.d(text="Settings").exists:
                break
            time.sleep(1)
        assert self.d(text="Settings").exists

    def set_screen_lock(self, option, curpasswd="abcd", newpasswd="abcd"):
        '''
        Set screen lock.
        '''
        print "[Info] ---Set screen lock %s." % option
        if not self.d(text="Screen lock").exists:
            self.d(text = "Security").click.wait()
        self.d(text = "Screen lock").click.wait()
        if self.d(resourceId = "com.android.settings:id/password_entry").exists:
            self.d(resourceId = "com.android.settings:id/password_entry").set_text(curpasswd)
            self.d(text = "Continue").click.wait()
        self.d(text = option).click.wait()
        if option == "Password":
            self.d(resourceId = "com.android.settings:id/password_entry").set_text(newpasswd)
            self.d(text = "Continue").click.wait()
            self.d(resourceId = "com.android.settings:id/password_entry").set_text(newpasswd)
            self.d(text = "OK").click.wait()
            self.d(text = "Done").click.wait()
        assert self.d(text="Screen lock").down(text=option) != None
        self.d.press.back()

    def lock_screen(self):
        '''
        Lock screen.
        '''
        print "[Info] ---Lock screen."
        self.d.sleep()
        self.d.wakeup()
        assert self.dsystemdomainsImpl(description="Unlock").exists

    def unlock_screen(self, passwd="abcd", status=True):
        '''
        Unlock screen.
        '''
        print "[Info] ---Unlock screen."
        self.d.wakeup()
        if self.d(description="Unlock").exists:
            self.d(description="Unlock").drag.to(resourceId = "com.android.systemui:id/clock_view")
        assert not self.d(description="Unlock").exists
        if self.d(resourceId = "com.android.systemui:id/passwordEntry").exists:
            self.d(resourceId = "com.android.systemui:id/passwordEntry").click()
            self.d(resourceId = "com.android.systemui:id/passwordEntry").set_text(passwd)
            self.d.click(700,1000)
            if status:
                assert not self.d(resourceId = "com.android.systemui:id/passwordEntry").exists
            else:
                assert self.d(resourceId = "com.android.systemui:id/passwordEntry").exists \
                        or self.d(textStartsWith = "You have incorrectly typed your password").exists

    def verify_cannot_input_chars(self):
        '''
        Verify cannot input chars.
        '''
        print "[Info] ---Verify cannot input chars."
        chars="a1A!"
        if self.d(text = "OK").exists:
            self.d(text = "OK").click.wait()
        self.d(resourceId = "com.android.systemui:id/passwordEntry").click()
        self.d(resourceId = "com.android.systemui:id/passwordEntry").set_text(chars)
        self.d.click(700,1000)
        assert self.d(textStartsWith = "Try again in").exists
        assert not self.d(text = "Wrong Password").exists

    def set_wallpaper_systemdomainsImplrandomly(self):
        '''
        Set wallpaper randomly.
        '''
        from random import randrange
        self.d(text = "Display").click.wait()
        self.d(text = "Wallpaper").click.wait()
        self.d(text = "Live Wallpapers").click.wait()
        i = randrange(5)
        print "[Info] ---Set wallpaper index %d." % i
        self.d(packageName = "com.android.wallpaper.livepicker", index=i).click.wait()
        self.d(text = "Set wallpaper").click.wait()
        assert not self.d(text = "Set wallpaper").exists
        self.d.press.back()
        self.d.press.back()

    def set_airplane_mode(self, status="OFF"):
        '''
        Set airplane mode status.
        '''
        print "[Info] ---Set airplane mode status %s." % status
        if not self.d(text = "Airplane mode").exists:
            self.d(text = "More…").click.wait()
        if not self.d(resourceId = "android:id/switchWidget").text == status:
            self.d(resourceId = "android:id/switchWidget").click.wait()
        assert self.d(resourceId = "android:id/switchWidget").text == status

system_domains2=android_framework_Impl(g_common_obj)
