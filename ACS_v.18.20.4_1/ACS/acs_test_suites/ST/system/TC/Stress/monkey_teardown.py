import os

os.system("adb shell killall com.android.commands.monkey")
for i in range(0,5):
	os.system("adb shell input keyevent 4")
print "0"