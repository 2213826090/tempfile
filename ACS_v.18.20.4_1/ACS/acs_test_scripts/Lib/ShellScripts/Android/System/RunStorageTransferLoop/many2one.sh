#!/system/bin/sh
# many2one: Start ~

dir=$stdir

echo -e "\n\n\n"
if [ -z "$1" ]; then echo "Usage: $dir/many2one.sh <number of loops>"; exit 1; fi
echo "Start many2one.sh test, $(date)" | tee -a $dir/many2one.log
looping=$1
counter=1
result=0

echo "		Clear all content from previous test in the device..."
size=0
sdloop=1
while [ $sdloop -le $stloop ]; do
	rm -r $dir/cv$sdloop/tempfile$sdloop 2> /dev/null
	mkdir $dir/cv$sdloop/tempfile$sdloop 2> /dev/null

	spaces=$(df -m | grep cv$sdloop | awk '{ print($4) }')

	if [ $size -eq 0 ]; then
		size=$spaces
	elif [ $size -ge $spaces ]; then
		size=$spaces
	fi

	sdloop=$(($sdloop+1))
done
sync

if [ $size -ge $((2000*$stloop)) ]; then
	sourcemedia=$mediapath
elif [ $size -ge $((1000*$stloop)) ]; then
	sourcemedia=$mediapath/1GBFiles
elif [ $size -ge $((512*$stloop)) ]; then
	sourcemedia=$mediapath/1GBFiles/512MBFiles
elif [ $size -ge $((256*$stloop)) ]; then
	sourcemedia=$mediapath/1GBFiles/512MBFiles/256MBFiles
else
	echo "many2one: One of device not enough space, $(date)" | tee -a $dir/error.log
	exit 1
fi

echo "	Source media directory is $sourcemedia/" | tee -a $dir/many2one.log
echo "		Please wait to initialize the device under test..."
sdloop=1
while [ $sdloop -le $stloop ]; do
	cp -rf $sourcemedia $dir/cv$sdloop/tempfile$sdloop/src$sdloop
	echo "		cv$sdloop done!!!"
	sdloop=$(($sdloop+1))
done
sync

while [ $counter -le $looping ]; do 
	echo "Loop #$counter : many2one" | tee -a $dir/many2one.log
	sdloop=1
	while [ $sdloop -le $stloop ]; do
		tmploop=1
		while [ $tmploop -le $stloop ]; do 
			if [ $tmploop -ne $sdloop ] ; then 
				rm -r $dir/cv$sdloop/tempfile$sdloop/src$tmploop/ 2> /dev/null &
			fi
			tmploop=$(($tmploop+1))
		done
		wait
		sync
		sdloop=$(($sdloop+1))
	done

#Start the massive transfer from here
	targetdev=1
	while [ $targetdev -le $stloop ]; do
		echo "	Target directory is $dir/cv$targetdev/" | tee -a $dir/many2one.log
		sdloop=1
		start=$(date +%s)
		while [ $sdloop -le $stloop ]; do
			if [ $sdloop -ne $targetdev ]; then
				echo "		Copying files from $dir/cv$sdloop/tempfile$sdloop/src$sdloop/ to $dir/cv$targetdev/tempfile$targetdev/src$sdloop/..." | tee -a $dir/many2one.log		
				cp -rf $dir/cv$sdloop/tempfile$sdloop/src$sdloop/ $dir/cv$targetdev/tempfile$targetdev/src$sdloop/&
			fi
			sdloop=$(($sdloop+1))
		done
		echo "		Copy processes sent to background. Please wait..." | tee -a $dir/many2one.log
		wait
		sync
		echo "		Copy process done!"
		stop=$(date +%s)
		duration=$((($stop-$start)/60+1))
		echo "		Total time for the copy process = $duration minutes" | tee -a $dir/many2one.log

		sdloop=1
		while [ $sdloop -le $stloop ]; do
			if [ $sdloop -ne $targetdev ]; then
				echo "		diff $sourcemedia and $dir/cv$sdloop/tempfile$sdloop/src$sdloop" | tee -a $dir/many2one.log	
				diff -r $sourcemedia $dir/cv$sdloop/tempfile$sdloop/src$sdloop >> $dir/check_if_fail/error_many2one.log
				if [ $? -ne 0 ]; then
					echo "****** Error during the diff process, $(date) ******" | tee -a $dir/many2one.log
					echo "Failed in many2one.sh test, $(date)" >> $dir/error.log
					result=1
				else
					echo "		No error found." | tee -a $dir/many2one.log
					echo "		Done diff process, $(date)." | tee -a $dir/many2one.log
				fi
			fi
			sdloop=$(($sdloop+1))
		done
		targetdev=$(($targetdev+1))
	done
	counter=$(($counter+1))
done

if [ ! -s $dir/check_if_fail/error_many2one.log ]; then
	rm -r $dir/check_if_fail/error_many2one.log 2> /dev/null
fi

echo "Completed many2one.sh, $(date)" | tee -a $dir/many2one.log

if [ $result -eq 0 ]; then
	echo -n "	[PASS] - " | tee -a $dir/logfile.log
else		
	echo -n "	[FAIL] - " | tee -a $dir/logfile.log
fi
# many2one.sh: End ~#!/bin/sh

