import os
import time
import shutil
import md5

def log(msg):
	curTime = time.strftime("%Y-%m-%d/%H:%M:%S ")
	os.system("echo " + curTime + msg + " >> " + logfile)

def create_file_client(file_name, file_size):
	filename = str(file_name)
	filesize = int(file_size)
	os.system("adb shell \"ls -l /data/ >> /data/%s\"" %filename)
	lsResponce = os.popen("adb shell ls /data/%s" %filename)
	lsOutput = lsResponce.read().strip()
	if lsOutput.find("No such file or directory") != -1:
		log("file create in client failed")
		return False
	else:
		duResponce = os.popen("adb shell du /data/%s" %filename)
		duOutput = duResponce.read().strip()
		duResult = int(duOutput.split()[0])
		while duResult < filesize:
			os.system("adb shell \"ls -l /data/ >> /data/%s\"" %filename)
			duResponce = os.popen("adb shell du /data/%s" %filename)
			duOutput = duResponce.read().strip()
			duResult = int(duOutput.split()[0])
		duResponce = os.popen("adb shell du /data/%s" %filename)
		duOutput = duResponce.read().strip()
		duResult = int(duOutput.split()[0])
		if duResult < filesize:
			log("file size less than expected")
			return False
		else:
			return True

def copy_file_client(file_a, file_b):
	filea = str(file_a)
	fileb = str(file_b)
	os.system("adb shell \"cp /data/%s /data/%s\"" %(file_a, file_b))
	lsResponce = os.popen("adb shell ls /data/%s" %file_b)
	lsOutput = lsResponce.read().strip()
	if lsOutput.find("No such file or directory") != -1:
		log("file copy in client failed")
		return False
	else:
		return True

def move_file_client(file_a, file_b):
	filea = str(file_a)
	fileb = str(file_b)
	os.system("adb shell \"mv /data/%s /data/%s\"" %(file_a, file_b))
	lsResponce = os.popen("adb shell ls /data/%s" %file_b)
	lsOutput = lsResponce.read().strip()
	if lsOutput.find("No such file or directory") != -1:
		log("file copy in client failed")
		return False
	else:
		return True
		
def compare_file(file_a, file_b):
	filea = str(file_a)
	fileb = str(file_b)
	fileap = open(filea,'rb')
	filebp = open(fileb,'rb')
	if md5.new(fileap.read()).digest() == md5.new(filebp.read()).digest():
		return True
	else:
		return False
	
def check_md5():
	if not create_file_client("file_a.txt", 1024):
		log("md5 test failed")
		print "md5 test failed"
	else:
		if not copy_file_client("file_a.txt", "file_b.txt"):
			log("md5 test failed")
			print "md5 test failed"
		else:
			for i in range(1, 10001):
				if not move_file_client("file_b.txt", "file_c.txt"):
					log("md5 test failed")
					print "md5 test failed"
				else:
					if not move_file_client("file_c.txt", "file_b.txt"):
						log("md5 test failed")
						print "md5 test failed"
					else:
						log("file copy between emmc -- time %s" %i)
			os.system("adb pull data/file_a.txt .")
			os.system("adb pull data/file_b.txt .")
			if compare_file("file_a.txt", "file_b.txt"):
				log("md5 test passed")
				print "md5 test passed"
			else:
				log("md5 test failed")
				print "md5 test failed"
	os.remove("file_a.txt")
	os.remove("file_b.txt")
	
if __name__ == '__main__':
	if os.path.exists(r'ST_SYST_FS_eMMC_157.log'):
		os.remove(r'ST_SYST_FS_eMMC_157.log')
	logfile = os.getcwd() + "/ST_SYST_FS_eMMC_157.log"
	
	check_md5()
	