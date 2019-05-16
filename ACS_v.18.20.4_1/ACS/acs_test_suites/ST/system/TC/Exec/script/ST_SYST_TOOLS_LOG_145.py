import os
import time
import string
import threading
import unittest

class ThreadClass(threading.Thread):
	def run(self):
		time.sleep(1)
		os.system("adb shell pkill logcat")

class ST_SYST_TOOLS_LOG_145(unittest.TestCase):
	logfile = os.getcwd() + "/ST_SYST_TOOLS_LOG_145.log"
	
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

	def emmcap_sdap_aplog_1h(self):
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
				break                          
		existflag2 = True              
		if False == existflag:         
			self.logError("/sdcard/aplog does not exist")
			existflag2 = False             
		if False == existflag1 or False == existflag2:
			return False                   
		
		os.system("adb pull /logs/aplog aplog_data")
		os.system("adb pull /sdcard/aplog aplog_sdcard")
		ap_data_size_old = os.path.getsize("aplog_data")
		ap_sd_size_old = os.path.getsize("aplog_sdcard")
		num_min  = 30
		while 0 < num_min:
			os.system("adb pull /logs/aplog aplog_data")
			os.system("adb pull /sdcard/aplog aplog_sdcard")
			ap_data_size = os.path.getsize("aplog_data")
			print "ap_data_size:",ap_data_size
			data_ok = True
			if ap_data_size < ap_data_size_old:
				data_ok = False
			ap_data_size_old = ap_data_size
			ap_sd_size = os.path.getsize("aplog_sdcard")
			print "ap_sd_size:",ap_sd_size
			sd_ok = True
			if ap_sd_size < ap_sd_size_old:
				sd_ok = False
			ap_sd_size_old = ap_sd_size
			time.sleep(5)
			num_min = num_min-1
			if not data_ok and not sd_ok:
				self.log("aplog in data or sdcard no growth")
				return False
		os.remove("aplog_sdcard")
		os.remove("aplog_data")
		os.system("adb shell rm /sdcard/aplog")
		os.system("adb shell rm /logs/aplog")
		return True

	def test_emmcap_sdap_aplog_1h(self): 
		self.log("ST_SYST_TOOLS_LOG_145: start!")
		if not self.emmcap_sdap_aplog_1h(): 
			self.log("ST_SYST_TOOLS_LOG_145: fail")
			print "ST_SYST_TOOLS_LOG_145: fail"           
		else:
			self.log("ST_SYST_TOOLS_LOG_145: pass")
			print "ST_SYST_TOOLS_LOG_145: pass"
		self.log("ST_SYST_TOOLS_LOG_145: end!")
		
if __name__ == '__main__':
	if os.path.exists(r'ST_SYST_TOOLS_LOG_145.log'):
		os.remove(r'ST_SYST_TOOLS_LOG_145.log')
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