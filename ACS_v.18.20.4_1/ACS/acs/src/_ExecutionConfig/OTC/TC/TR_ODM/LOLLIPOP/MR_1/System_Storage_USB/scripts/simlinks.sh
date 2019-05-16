#! /bin/bash
# Returns PASS if the simlink searched (param 2) points to the right folder
# (param 3) after listing the inputted folder (param 1).
# This is an auxiliary script.
# Author: Diana Dumitrescu

# This portion verifies the usage of the script according to the description above
if  [ -z $1 ]
then
        if [ -z $2 -o -z $3 ]
        then
                echo "Usage ./mounted_simlinks.sh <folder_path> <simlink_name> <simlink_dest_file>"
                exit 1
        fi
fi

# The commands are used to parse the ls -l command from DUT
adb shell ls -l $1 | grep lr | awk '{print $6}' | tr -d '\r' > file1.out
adb shell ls -l $1 | grep lr | awk '{print $8}' | tr -d '\r' > file2.out

# The outputs are introduced in arrays for better handling
readarray a < file1.out
readarray b < file2.out

# This loops the 2 arrays and searches for the inputted simlink (param 2) in the array
# a, then compares the second array element on the same position for the right pointed
# folder (param 3).
i=0
while [ $i -lt 10 ]
do
        if [ ${a[i]} = $2 ]
        then
                if [ ${b[i]} = $3 ]
                then
                        echo "PASS"
                        break
                else
                        echo "FAIL"
                        break
                fi
        fi
        i=`expr $i + 1`;
done

# The cleanup after the execution of this script
rm -f file1.out
rm -f file2.out
