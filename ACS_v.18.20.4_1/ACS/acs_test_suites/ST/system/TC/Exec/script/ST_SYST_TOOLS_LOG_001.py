import os
import time
import string
import threading
import unittest

class ThreadClass(threading.Thread):
	def run(self):
		time.sleep(1)
		os.system("adb shell pkill logcat")

class ST_SYST_TOOLS_LOG_001(unittest.TestCase):
	logfile = os.getcwd() + "/ST_SYST_TOOLS_LOG_001.log"
	
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
		time.sleep(5)
		if 0 < os.path.getsize("logcat_file"):
			return True
		else:
			return False
	
	def test_adb_logcat(self):
		self.log("ST_SYST_TOOLS_LOG_001: start!")
		if not self.adb_logcat():
			self.log("ST_SYST_TOOLS_LOG_001: fail")
			print "ST_SYST_TOOLS_LOG_001: fail"
		else:
			self.log("ST_SYST_TOOLS_LOG_001: pass") 
			print "ST_SYST_TOOLS_LOG_001: pass"
		self.log("ST_SYST_TOOLS_LOG_001: end!")
		
if __name__ == '__main__':
	if os.path.exists(r'ST_SYST_TOOLS_LOG_001.log'):
		os.remove(r'ST_SYST_TOOLS_LOG_001.log')
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