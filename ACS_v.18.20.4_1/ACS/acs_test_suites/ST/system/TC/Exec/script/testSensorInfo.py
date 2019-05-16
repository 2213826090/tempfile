#!/usr/bin/env python

import fileinput
import re
import sys
import os

#######################################################

t_result='fail'

if len(sys.argv) < 8: 
  print "Usage:        " +sys.argv[0]+ " testcatagory sensorname result deviceserial logfile para1 para2\n"
  print "\n"            
  print "testcatagory:  Which kind of test will be executed. They are: \n"  
  print "               - list, test the list of sensor.\n"  
  print "               - frequency, test the frequency of sensor data.\n" 
  print "                 para1 (as data speed type, fastest, game, ui, normal) needed.\n"   
  print "               - range, test the max range of sensor data.\n"
  print "               - delay, test the delay between sensor data collected and client received.\n"
  print "                 para1 (as data speed type, fastest, game, ui, normal) needed.\n"  
  print "               - concurrent, 5 instance of the same sensor client are activated. \n"
  print "                 Fastest frequency and data delay is checke. \n"
  print "                 result is the target frequency(Hz), and para1 is the target delay value(ms).\n"    
  print "sensorname:    the sensor name to be tested, can be:\n"
  print "               accelerometer, compass, gyroscope, orientation, \n"
  print "               barometer, proximity, light, Linear_accelerometer, \n"
  print "               Gravity, Rotation_vector. For concurrent and frequency, no light and proximity.\n"
  print "result:        the target sensor test result.\n"
  print "               - List, integer value.\n"
  print "               - Range, float value.\n"  
  print "               - frequency, integer value(Hz). \n"
  print "               - delay, integer value(ms).\n"
  print "               - concurrent, integer value, means frequency(Hz).\n"
  print "deviceserial:  the target device serial No. If only one device, default can be used.\n"
  print "logfile:       the logfile name with full path. Otherwise, the log file will saved\n"
  print "               based on the current path\n"
  print "para1:         different test has different meaning. If not used, 0 is recommended.\n"
  print "               - frequency test, means data speed type (fastest, game, ui, normal).\n"
  print "               - delay test, means data speed type (fastest, game, ui, normal).\n" 
  print "               - concurrent test, means data speed type (fastest, game, ui, normal).\n" 
  print "para2:         different test has different meaning. If not used, 0 is recommended.\n"
  print "               - concurrent test, means target delay (ms), integer value.\n" 
  print "\n"
  print "For example: " +sys.argv[0]+ " list accelerometer 1 default log.txt 0 0\n"
  print t_result
  sys.exit()

################################################################
# open the log file
try:
  logf = file(sys.argv[5], 'a')
except:
  print 'log file:'+sys.argv[5]+" can't be opened."
  sys.exit()
  
################################################################
# Find out the test catagory, and check the needed parameter  
# a swith-case implementation, get(,) is default branch.
t_testfun = {
  'list': 'testListGet',
  'frequency': 'testFrequencyGet',
  'range': 'testRangeGet',
  'delay': 'testDelayGet',
  'concurrent': 'testConcurrent'
}.get(sys.argv[1], 'null')

if t_testfun=='null':
  logf.write( 'E:--Invalid test catagory in arg1.\n')
  logf.close()
  print t_result
  sys.exit()

#################################################################
# Find the device. The input device maybe default
t_strdev = os.popen('adb devices').readlines()

m_device=sys.argv[4]
t_bool=0

p=re.compile('(\S+)\s+device\s')
for i in t_strdev:
  m=p.match(i)
# matched, at least one device
  if m: 
    if 'default'==m_device:
      m_device=m.group(1)
      t_bool=1
      break
    elif m.group(1)==m_device:
      t_bool=1
      break
    
logf.write(sys.argv[0]+' '+sys.argv[1]+' '+sys.argv[2]+' '+sys.argv[3]+' '+sys.argv[4]+' '\
  +sys.argv[5]+' '+sys.argv[6]+' '+sys.argv[7]+': device is: '+m_device+'\n')
if 0==t_bool:
  logf.write('E:--But the device not exist.\n')
  print t_result
  sys.exit()

#####################################################################
# execute the am command

t_strdev = os.popen('adb -s '+m_device+' shell am instrument \
  -e class android.intel.umg.sensor.test.TSensorTestTest#'+t_testfun+' \
  -e testitem '+sys.argv[2]+' -e testresult '+sys.argv[3]+' -e testresultpara1 '\
  +sys.argv[6]+' -e testresultpara2 '+sys.argv[7]+' \
  -w android.intel.umg.sensor.test/android.intel.umg.sensor.test.STIOInstrumentationTestRunner').readlines()

p=re.compile('.+for\s+STIOInstrumentationTestRunner=\.\s.*')
for i in t_strdev:
  logf.write('  --'+i)
  m=p.match(i)
  if m: # matched
    t_result='pass'

####################################################
# close log file and return
logf.close()
print t_result
sys.exit()


  
  
