import os
import time
import string
import threading
import unittest

class ThreadClass(threading.Thread):
	def run(self):
		time.sleep(1)
		os.system("adb shell pkill logcat")

class ST_SYST_TOOLS_LOG_014(unittest.TestCase):
	logfile = os.getcwd() + "/ST_SYST_TOOLS_LOG_014.log"
	
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

	def adb_test(self):
		os.system("adb shell ls > adb_test1.txt")
		file = open("adb_test1.txt","r")
		lines = file.readlines()
		file.close()
		os.remove("adb_test1.txt")
		for line in lines:
			if -1 < line.find("sdcard"):
				return True
		return False
			
	def test_adb_test(self):
		self.log("ST_SYST_TOOLS_LOG_014: start!")
		if not self.adb_test():
			self.log("ST_SYST_TOOLS_LOG_014: fail")
			print "ST_SYST_TOOLS_LOG_014: fail"
		else:
			self.log("ST_SYST_TOOLS_LOG_014: pass")
			print "ST_SYST_TOOLS_LOG_014: pass"
		self.log("ST_SYST_TOOLS_LOG_014: end!")
		
if __name__ == '__main__':
	if os.path.exists(r'ST_SYST_TOOLS_LOG_014.log'):
		os.remove(r'ST_SYST_TOOLS_LOG_014.log')
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