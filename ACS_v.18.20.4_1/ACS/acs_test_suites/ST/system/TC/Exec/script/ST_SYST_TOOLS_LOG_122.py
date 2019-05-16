import os
import time
import string
import threading
import unittest

class ThreadClass(threading.Thread):
	def run(self):
		time.sleep(1)
		os.system("adb shell pkill logcat")

class ST_SYST_TOOLS_LOG_122(unittest.TestCase):
	logfile = os.getcwd() + "/ST_SYST_TOOLS_LOG_122.log"
	
	def setUp(self):
		pass
	
	def tearDown(self):
		pass

	def logError(self,msg):
		curTime = time.strftime("%m-%d %H:%M:%S ")
		os.system("echo " + curTime + "Error/" + msg + " >> " + self.logfile)

	def log(self,msg):
		curTime = time.strftime("%m-%d %H:%M:%S ")
		os.system("echo " + curTime + "Infor/" + msg + " >> " + self.logfile)
	
	def adb_logcat(self):
		t = ThreadClass()
		t.start()
		os.system("adb shell logcat > logcat_file")
		if 0 < os.path.getsize("logcat_file"):
			return True
		else:
			return False

	def emmcap_sdap_aplog(self):
		os.system("adb shell setprop persist.service.aplogfs.enable 0")
		time.sleep(2)
		os.system("adb shell setprop persist.service.aplogsd.enable 0")
		time.sleep(2)
		os.system("adb shell ls /logs/aplog > 1.txt")
		os.system("adb shell ls /sdcard/aplog > 2.txt")
		file = open("1.txt","r")
		line = file.readlines()
		for line1 in line:
			if 0 > line1.find("aplog:") and -1 < line1.find("aplog"):
				os.system("adb shell rm /logs/aplog")
				break
		file.close()
		os.remove("1.txt")
		file = open("2.txt","r")
		line = file.readlines()
		for line1 in line:
			if 0 > line1.find("aplog:") and -1 < line1.find("aplog"):
				os.system("adb shell rm /sdcard/aplog")
				break
		file.close()
		os.remove("2.txt")
		os.system("adb shell setprop persist.service.aplogfs.enable 1")
		time.sleep(2)
		os.system("adb shell setprop persist.service.aplogsd.enable 1")
		time.sleep(2)
		os.system("adb shell ls /logs/aplog > 3.txt")
		os.system("adb shell ls /sdcard/aplog > 4.txt")
		file = open("3.txt","r")
		line = file.readlines()
		file.close()
		os.remove("3.txt")
		existflag = False
		for line1 in line:
			if 0 >  line1.find("aplog:") and -1 < line1.find("aplog"):
				self.log("/logs/aplog exist")
				os.system("adb shell rm /logs/aplog")
				existflag = True
				break
		existflag1 = True
		if False == existflag:
			self.logError("/logs/aplog does not exist")
			existflag1 = False
		file = open("4.txt","r")
		line = file.readlines()
		file.close()
		os.remove("4.txt")
		existflag = False
		for line1 in line:
			if 0 >  line1.find("aplog:") and -1 < line1.find("aplog"):
				self.log("/sdcard/aplog exist")
				existflag = True
				os.system("adb shell rm /sdcard/aplog")
				break
		existflag2 = True
		if False == existflag:
			self.logError("/sdcard/aplog does not exist")
			existflag2 = False
		if False == existflag1 or False == existflag2:
			return False
		return True

	def test_emmcap_sdap_aplog(self):
		self.log("ST_SYST_TOOLS_LOG_122: start!")
		if not self.emmcap_sdap_aplog():
			self.log("ST_SYST_TOOLS_LOG_122: fail")
			print "ST_SYST_TOOLS_LOG_122: fail"
		else:
			self.log("ST_SYST_TOOLS_LOG_122: pass")
			print "ST_SYST_TOOLS_LOG_122: pass"
		self.log("ST_SYST_TOOLS_LOG_122: end!")
		
if __name__ == '__main__':
	if os.path.exists(r'ST_SYST_TOOLS_LOG_122.log'):
		os.remove(r'ST_SYST_TOOLS_LOG_122.log')
	os.system("adb shell ls /logs/aplog > 1.txt")
	file = open("1.txt")
	line = file.readlines()
	file.close()
	for line1 in line:
		if 0 > line1.find("aplog:") and -1 < line1.find("aplog"):
			os.system("adb shell rm /logs/aplog")
			break
	os.remove("1.txt")
	os.system("adb shell ls /sdcard/aplog > 1.txt")
	file = open("1.txt")
	line = file.readlines()
	file.close()
	for line1 in line:
		if 0 > line1.find("aplog:") and -1 < line1.find("aplog"):
			os.system("adb shell rm /sdcard/aplog")
			break
	os.remove("1.txt")
	unittest.main()