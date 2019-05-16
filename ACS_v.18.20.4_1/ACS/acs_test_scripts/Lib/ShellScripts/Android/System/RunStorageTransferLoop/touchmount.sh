#!/system/bin/sh
# Touch & Mount: Start~

dir=$stdir

echo -e "\n\n\n"
echo "Start touchmount.sh test on $(date)" | tee -a $dir/touchmount.log
echo "		Clearing all contents from previous test in the device..."

result=0

sdloop=1
while [ $sdloop -le $stloop ] 
do
	rm -r $dir/cv$sdloop/tempfile$sdloop 2> /dev/null
	mkdir -p $dir/cv$sdloop/tempfile$sdloop/src$sdloop 2> /dev/null
	sdloop=$(($sdloop+1))
done
sync

sdloop=1
while read line    #reading from devices.txt file [sda1,sda2,etc...]
do
	echo "	Working on $line device mounted to $dir/cv$sdloop" | tee -a $dir/touchmount.log
	echo "		Remove everthing in $dir/cv$sdloop/tempfile$sdloop/src$sdloop/ now..." | tee -a $dir/touchmount.log
	rm -r $dir/temp 2> /dev/null
	sync
	mkdir $dir/temp 2> /dev/null
	umount $dir/cv$sdloop 2> /dev/null
	wait
	counter=1
	start=$(date +%s)
	while [ $counter -le 100 ]; do
		mount -t ext3 $line $dir/cv$sdloop 2> /dev/null
                mount -t vfat $line $dir/cv$sdloop 2> /dev/null
                mount -t ext2 $line $dir/cv$sdloop 2> /dev/null
                mount -t ext4 $line $dir/cv$sdloop 2> /dev/null
		wait
		mnt=$(df | grep $dir/cv$sdloop)
		if [ -z "$mnt" ]; then
			echo "Failed in $line device mounted to $dir/cv$sdloop, $(date)" | tee -a $dir/touchmount.log
			echo "Failed in $line device mounted to $dir/cv$sdloop, $(date)" >> $dir/error.log
			result=1
		fi

		touch $dir/cv$sdloop/tempfile$sdloop/src$sdloop/test$counter
		touch $dir/temp/test$counter
		sleep 1
		echo -n "		Writing to test$counter   " | tee -a $dir/touchmount.log
		start1=$(date +%s)
		cat $dir/media/input.txt >> $dir/temp/test$counter 
		cat $dir/media/input1.txt >> $dir/temp/test$counter 
		cat $dir/media/input.txt >> $dir/cv$sdloop/tempfile$sdloop/src$sdloop/test$counter
		cat $dir/media/input1.txt >> $dir/cv$sdloop/tempfile$sdloop/src$sdloop/test$counter
		stop1=$(date +%s)
		duration1=$(($stop1-$start1))
		echo "	$duration1 second(s)" | tee -a $dir/touchmount.log
		wait
		sync
		umount $dir/cv$sdloop > /dev/null
		wait
		sleep 1
		counter=$(($counter+1))
	done 
	stop=$(date +%s)
	duration=$((($stop-$start)/60+1))
	echo "		Total time for the cp, mount&umount = $duration minutes" | tee -a $dir/touchmount.log
	mount -t ext3 $line $dir/cv$sdloop 2> /dev/null 
        mount -t vfat $line $dir/cv$sdloop 2> /dev/null 
        mount -t ext2 $line $dir/cv$sdloop 2> /dev/null
	mount -t ext4 $line $dir/cv$sdloop 2> /dev/null
	wait
	mnt=$(df | grep $dir/cv$sdloop)
	if [ -z "$mnt" ]; then
		echo "Failed in $line device mounted to $dir/cv$sdloop, $(date)" | tee -a $dir/touchmount.log
		echo "Failed in $line device mounted to $dir/cv$sdloop, $(date)" >> $dir/error.log
		result=1
	fi

	echo "		diff $dir/cv$sdloop/tempfile$sdloop/src$sdloop and $dir/temp/" | tee -a $dir/touchmount.log
	diff -r $dir/cv$sdloop/tempfile$sdloop/src$sdloop $dir/temp >> $dir/check_if_fail/error_touchmount.log
	if [ $? -ne 0 ]; then
		echo "****** Error during the diff process, $(date) ******" | tee -a $dir/touchmount.log
		echo "Failed in touchmount.sh test, $(date)" >> $dir/error.log
		result=1
	else
		echo "		No error found." | tee -a $dir/touchmount.log
		echo "		Done diff process."| tee -a $dir/touchmount.log	
	fi
	sleep 3
	sdloop=$(($sdloop+1)) 
done < $dir/devices.txt
rm -r $dir/temp 2> /dev/null
sync

if [ ! -s $dir/check_if_fail/error_touchmount.log ]; then
	rm -r $dir/check_if_fail/error_touchmount.log 2> /dev/null
fi

echo "Completed touchmount.sh, $(date)" | tee -a $dir/touchmount.log

if [ $result -eq 0 ]; then
	echo -n "	[PASS] - " | tee -a $dir/logfile.log
else		
	echo -n "	[FAIL] - " | tee -a $dir/logfile.log
fi
# touchmount: End~
