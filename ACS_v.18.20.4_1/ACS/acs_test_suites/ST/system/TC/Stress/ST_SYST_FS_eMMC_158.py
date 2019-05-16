import os
import time
import shutil
import md5

def log(msg):
	curTime = time.strftime("%Y-%m-%d/%H:%M:%S ")
	os.system("echo " + curTime + msg + " >> " + logfile)

def create_file_host(file_name, file_size):
	filename = str(file_name)
	filesize = long(file_size)
	filedir = os.getcwd()
	filepath = os.path.join(filedir, filename)
	file = open(filepath, 'w')
	if not os.path.exists(filename):
		log("file create in host failed")
		return False
	else:	
		while os.path.getsize(filename) < filesize:
			file.write("This is a temp file for md5 check......\n")
		if os.path.getsize(filename) < filesize:
			log("file size less than expected")
			return False
		else:
			return True

def copy_file_host(file_a, file_b):
	filea = str(file_a)
	fileb = str(file_b)
	filedir = os.getcwd()
	if os.path.exists(fileb):
		os.remove(fileb)
	shutil.copyfile(filea, fileb)
	if not os.path.exists(fileb):
		log("file copy in host failed")
		return False
	else:
		return True
		
def copy_file_host_to_client(file_a, file_b):
	src = os.getcwd() + os.sep + str(file_a)
	dst = "data/" + str(file_b)
	os.system("adb push %s %s" %(src, dst))
	lsOutput = os.popen("adb shell ls data/")
	if (lsOutput.read().strip().find(str(file_b)) == -1):
		log("file copy from host to client failed")
		return False
	else:
		return True
		
def copy_file_client_to_host(file_a, file_b):
	src = "data/" + str(file_a)
	dst = os.getcwd() + os.sep + str(file_b)
	os.system("adb pull %s %s" %(src, dst))
	os.system("dir > temp.txt")
	temppoint = open("temp.txt")
	templist = temppoint.readlines()
	temppoint.close()
	os.remove("temp.txt")
	for line in templist:
		if (line.find(str(file_b)) != -1):
			return True
	log("file copy from client to host failed")
	return False

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
	if not create_file_host("file_a.txt", 1024000):
		log("md5 test failed")
		print "md5 test failed"
	else:
		if not copy_file_host("file_a.txt", "file_b.txt"):
			log("md5 test failed")
			print "md5 test failed"
		else:
			for i in range(1, 10001):
				if not copy_file_host_to_client("file_b.txt", "file_c.txt"):
					log("md5 test failed")
					print "md5 test failed"
				else:
					if not copy_file_client_to_host("file_c.txt", "file_b.txt"):
						log("md5 test failed")
						print "md5 test failed"
					else:
						log("file copy between host and client -- time %s" %i)
			os.system("adb pull data/file_c.txt .")
			if compare_file("file_a.txt", "file_b.txt") and compare_file("file_b.txt", "file_c.txt") and compare_file("file_c.txt", "file_a.txt"):
				log("md5 test passed")
				print "md5 test passed"
			else:
				log("md5 test failed")
				print "md5 test failed"
	os.remove("file_a.txt")
	os.remove("file_b.txt")
	os.remove("file_c.txt")
	
	
if __name__ == '__main__':
	if os.path.exists(r'ST_SYST_FS_eMMC_158.log'):
		os.remove(r'ST_SYST_FS_eMMC_158.log')
	logfile = os.getcwd() + "/ST_SYST_FS_eMMC_158.log"
	
	check_md5()
	