#!/system/bin/sh
# concurrency: Start ~

dir=$stdir

echo -e "\n\n\n"
if [ -z "$1" ]; then echo "Usage: $dir/concurrency.sh <number of loops>"; exit 1; fi
echo "Start concurrency.sh test, $(date)" | tee -a $dir/concurrency.log
looping=$1
counter=1
result=0

##### Functions
compare(){
	echo "		Comparing the source file $1 and $2" | tee -a $dir/concurrency.log
	diff -r $1 $2 >> $dir/error_concurrency.log
	if [ $? -ne 0 ]; then
		echo "****** Error during the diff process, $(date) ******" | tee -a $dir/concurrency.log
		echo "Failed in concurrency.sh test, $(date)" >> $dir/error.log
		result=1
	else
		echo "		No error found." | tee -a $dir/concurrency.log
		echo "		Done diff process, $(date)." | tee -a $dir/concurrency.log	
	fi
}

if [ $size_rank -eq 1 ]; then
	sourcemedia=$mediapath
elif [ $size_rank -eq 2 ]; then
	sourcemedia=$mediapath/1GBFiles
elif [ $size_rank -eq 4 ]; then
	sourcemedia=$mediapath/1GBFiles/512MBFiles
elif [ $size_rank -eq 8 ]; then
	sourcemedia=$mediapath/1GBFiles/512MBFiles/256MBFiles
else
	echo "Concurrency: One of the devices does not have enough space.\nIt has $(size) MB and at least 512 MB is needed, $(date)" | tee -a $dir/error.log
	exit 1
fi

echo "		Please wait to initialize the device under test..." | tee -a $dir/concurrency.log
sdloop=1
#stloop=1
while [ $sdloop -le $stloop ]; do
	cp -rf $sourcemedia $dir/cv$sdloop/tempfile$sdloop/src$sdloop
	echo "		cv$sdloop done!!!"
	sdloop=$(($sdloop+1))
done
sync

while [ $counter -le $looping ]; do 
	echo "Loop #$counter : concurrency" | tee -a $dir/concurrency.log
	
	#Start the concurrency transfer from here
	sdloop=1
	while [ $sdloop -le $stloop ]; do
		targetdev=$(($sdloop+1))
		
		start=$(date +%s)
		while [ $targetdev -le $stloop ]; do
			echo "		Copying files between $dir/cv$sdloop and $dir/cv$targetdev" | tee -a $dir/concurrency.log
			cp -rf $dir/cv$sdloop/tempfile$sdloop/src$sdloop/ $dir/cv$targetdev/tempfile$targetdev/src$sdloop/&
			cp -rf $dir/cv$targetdev/tempfile$targetdev/src$targetdev/ $dir/cv$sdloop/tempfile$sdloop/src$targetdev/&

			echo "		Copy processes sent to background. Please wait..." | tee -a $dir/concurrency.log
			wait
			sync
			echo "		Copy process done!"
			stop=$(date +%s)
			duration=$((($stop-$start)/60+1))
			echo "		Total time for the copy process = $duration minutes" | tee -a $dir/concurrency.log
			
			compare $sourcemedia $dir/cv$targetdev/tempfile$targetdev/src$sdloop
			wait
			compare $sourcemedia $dir/cv$sdloop/tempfile$sdloop/src$targetdev
			wait
			echo "		Cleaning the contents in the device under test..." | tee -a $dir/concurrency.log
			rm -r $dir/cv$targetdev/tempfile$targetdev/src$sdloop/ 2> /dev/null
			rm -r $dir/cv$sdloop/tempfile$sdloop/src$targetdev/ 2> /dev/null
			sync
			targetdev=$(($targetdev+1))
		done
		sdloop=$(($sdloop+1))
	done
	counter=$(($counter+1))
done

if [ ! -s $dir/error_concurrency.log ]; then
	rm -r $dir/error_concurrency.log 2> /dev/null
fi

echo "Completed concurrency.sh, $(date)" | tee -a $dir/concurrency.log

if [ $result -eq 0 ]; then
	echo -n "	[PASS] - " | tee -a $dir/logfile.log
else		
	echo -n "	[FAIL] - " | tee -a $dir/logfile.log
fi
# concurrency.sh: End ~#
