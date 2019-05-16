#!/usr/bin/python

import sys
import subprocess
import time

test_script = """
    trap "echo FAIL" EXIT
    set -e
    g1={gpio_pin_1}
    g2={gpio_pin_2}
    G=/sys/class/gpio
    E=$G/export
    U=$G/unexport
    G1=$G/gpio$g1
    G2=$G/gpio$g2

    # Clean up any old exports
    set +e
    echo $g1 > $U
    echo $g2 > $U
    set -e

    echo $g1 > $E
    echo $g2 > $E

    echo out > $G1/direction 
    echo in  > $G2/direction 
    
    echo 0   > $G1/value
    v0=$(cat $G2/value)
    echo 1   > $G1/value
    v1=$(cat $G2/value)
    
    if [ $v1 -ne 1 -o $v0 -ne 0 ]
    then
      rc1='FAIL'
    else
      rc1='PASS'
    fi
    echo "$rc1 (out: $g1, in: $g2, ($v0, $v1)"
    
    echo out > $G2/direction 
    echo in  > $G1/direction 
    
    echo 0   > $G2/value
    v0=$(cat $G1/value)
    echo 1   > $G2/value
    v1=$(cat $G1/value)

    if [ $v1 -ne 1 -o $v0 -ne 0 ]
    then
      rc2='FAIL'
    else
      rc2='PASS'
    fi
    echo "$rc2 (out: $g2, in: $g1, ($v0, $v1)"
    trap "" EXIT
"""
gpio_pairs = [
  [ "J18",  (12,7), (13,1) ],
  [ "J19,J20",  (14,9), (15,7) ],
  [ "J19,J20",  (44,4), (45,4) ],
  ]

def main():
    ret = 0

    if subprocess.call(["adb", "root"]):
	sys.exit(1)

    time.sleep(5)
    log = open('testfile.log', 'w+')

    for index, pair in enumerate(gpio_pairs):
        connector, gpio1, gpio2 = pair
        gpio1_linux, gpio1_pin = gpio1
        gpio2_linux, gpio2_pin = gpio2
#  print "\ntesting Connector: {con}, pins: {p1}, {p2}, via linux gpios: {g1}, {g2}".format(con=connector, p1=gpio1_pin, p2=gpio2_pin, g1=gpio1_linux, g2=gpio2_linux)
        testfile = open('testfile.sh', 'w')
        testfile.write(test_script.format(gpio_pin_1=gpio1_linux, gpio_pin_2=gpio2_linux))
        testfile.close()

        subprocess.call(["adb", "push", "testfile.sh", "/data/data/testfile.sh"])
        ret = subprocess.call(["adb", "shell", "sh /data/data/testfile.sh"],stderr=subprocess.STDOUT, stdout=log)
        if ret != 0:
            break
    
    log.seek(0,0)
    if 'FAIL' in log.read():
       ret = 1	

    log.seek(0,0)
    print(log.read())

    log.close()
    sys.exit(ret) 

if __name__ == "__main__":
	main()
