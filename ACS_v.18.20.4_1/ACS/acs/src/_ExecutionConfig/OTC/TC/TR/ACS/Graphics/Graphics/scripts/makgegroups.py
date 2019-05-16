#!/usr/bin/python
import os

def main():
	group_files = ["ES2-CTS-groups.txt", "ES3-CTS-groups.txt", "ES31-CTS-groups.txt"]
	path='/home/mihaela/acs_repo_sync/acs_test_suites/OTC/TC/ACS/Graphics/Graphics/scripts'
	for root, dirs, files in os.walk(r'%s' %path):
        	for file in files:
                	for gf in group_files:
                        	tf_d = open(path + "/" + file, 'r')
                        	gf_d = open(path + "/" + gf, 'w')
                            	group_run = None
                            	for line in tf_d:
                                	if "GROUP" in line:
                                    		group_run = line.split(" ")[1]

                                	if ("TEST" in line) and group_run:
                                    		gf_d.write(group_run.strip('\n') + '.*' + '\n')
                                    		group_run = None
                            	tf_d.close()
                            	gf_d.close()

if __name__ == "__main__":
    main()
