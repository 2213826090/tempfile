import os
import time
import string
import threading
import unittest

class ThreadClass(threading.Thread):
	def run(self):
		time.sleep(1)
		os.system("adb shell pkill logcat")

class ST_SYST_TOOLS_LOG_009(unittest.TestCase):
	logfile = os.getcwd() + "/ST_SYST_TOOLS_LOG_009.log"
	
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

	def sd_aplogTimestamp(self):
		os.system("adb shell setprop persist.service.aplogsd.enable 0")
		time.sleep(2)
		os.system("adb shell ls /sdcard/aplog > 1.txt")
		file = open("1.txt","r")
		line = file.readlines()
		for line1 in line:
			if 0 > line1.find("aplog:") and -1 <  line1.find("aplog"):
				os.system("adb shell rm /sdcard/aplog")
				break
		file.close()
		os.remove("1.txt")
		os.system("adb shell setprop persist.service.aplogsd.enable 1")
		time.sleep(2)
		os.system("adb shell ls /sdcard/aplog > 2.txt")
		file = open("2.txt","r")
		line = file.readlines()
		file.close()
		os.remove("2.txt")
		existflag = False
		for line1 in line:
			if 0 >  line1.find("aplog:") and -1 < line1.find("aplog"):
				self.log("/sdcard/aplog exist")
				existflag = True
				break
		if False == existflag:
			self.log("/sdcard/aplog does not exist")
			return False
		os.system("adb pull /sdcard/aplog .")
		os.system("adb shell rm /sdcard/aplog")
		file = open("aplog")
		line = file.readlines()
		file.close()
		os.remove("aplog")
		line2 = line[1]
		line3 = line[2]
		if '-' == line2[2] and ':' == line2[8] == line2[11] and '.' == line2[14]:
			self.log("aplog file contains timestamp")
			return True
		elif '-' == line3[2] and ':' == line3[8] == line3[11] and '.' == line3[14]:
			self.log("aplog file contains timestamp")
			return True
		else:
			self.logError("aplog file does not contains timesamp")
			return False	
	
	def test_sd_aplogTimestamp(self):
		self.log("ST_SYST_TOOLS_LOG_009: start!")
		if not self.sd_aplogTimestamp():
			self.logError("ST_SYST_TOOLS_LOG_009: fail")
			print "ST_SYST_TOOLS_LOG_009: fail"
		else:
			self.log("ST_SYST_TOOLS_LOG_009: pass")
			print "ST_SYST_TOOLS_LOG_009: pass"
		self.log("ST_SYST_TOOLS_LOG_009: end!")
		
if __name__ == '__main__':
	if os.path.exists(r'ST_SYST_TOOLS_LOG_009.log'):
		os.remove(r'ST_SYST_TOOLS_LOG_009.log')
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