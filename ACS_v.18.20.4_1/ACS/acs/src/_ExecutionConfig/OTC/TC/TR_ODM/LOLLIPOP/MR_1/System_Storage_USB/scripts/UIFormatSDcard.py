#Storage.USB_SDCard_Format - UI interaction
import time
import sys
from uiautomator import device as d


def EraseSDcard():
        try:
                d(text="Erase SD card")[0].click()
        except:
                sys.exit("FAIL - couldn't press the Erase SD card option from Storage menu")
        try:
                d(text="Erase SD card")[1].click()
        except:
                sys.exit("FAIL - couldn't press the Erase SD card button from Erase SD card window")
        try:
                d(text="Erase everything").click()
        except:
                sys.exit("FAIL - couldn't press the Erase Everything button from Erase SD card window")

def DisableMTP():
        #MTP or PTP must be disabled
        try:
                d.press.menu()
                d(text="USB computer connection").click()
        except:
                sys.exit("FAIL - to open the USB computer connection menu")
        if d(className="android.widget.CheckBox")[0].info['checked']:
                try:
                        d(className="android.widget.CheckBox")[0].click()
                except:
                        #MTP was deactivated, wait 5 seconds for reconnection
                        time.sleep(5)
                        d.press.back()
        if d(className="android.widget.CheckBox")[1].info['checked']:
                try:
                        d(className="android.widget.CheckBox")[1].click()
                except:
                        #PTP was deactivated, wait 5 seconds for reconnection
                        time.sleep(5)
                        d.press.back()

if d(packageName="com.android.settings").wait.exists(timeout=10000):
        while not d(text="Erase SD card").exists:
                d(scrollable=True).scroll(steps=10)
        if d(text="Erase SD card").info['enabled']:
                EraseSDcard()
                time.sleep(5) # wait 5 seconds for the unmount - erase - mount operation to finish
                sys.exit(0)
        else:
                try:
                        DisableMTP()
                except:
                        time.sleep(1) #treating exception because of MTP deactivation
                EraseSDcard()
                time.sleep(5) # wait 5 seconds for the unmount - erase - mount operation to finish
                sys.exit(0)
else:
        sys.exit("FAIL - Settings not opened")
