#!/system/bin/sh
# one2many: Start ~

dir=$stdir

echo -e "\n\n\n"
if [ -z "$1" ]; then echo "Usage: $dir/one2many.sh <number of loops>"; exit 1; fi
echo "Start one2many.sh test, $(date)" | tee -a $dir/one2many.log
counter=1
looping=$1
result=0
echo "		Clearing all contents from previous test in the device..."
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

if [ $size -ge 4000 ]; then
	sourcemedia=$mediapath
elif [ $size -ge 2000 ]; then
	sourcemedia=$mediapath/1GBFiles
elif [ $size -ge 1000 ]; then
	sourcemedia=$mediapath/1GBFiles/512MBFiles
elif [ $size -ge 512 ]; then
	sourcemedia=$mediapath/1GBFiles/512MBFiles/256MBFiles
else
	echo "one2many: One of device not enough space, $(date)" | tee -a $dir/error.log
	exit 1
fi

echo "	Source media directory is $sourcemedia/" | tee -a $dir/one2many.log
echo "		Please wait to initialize the device under test..."
sdloop=1
while [ $sdloop -le $stloop ]; do
	cp -rf $sourcemedia $dir/cv$sdloop/tempfile$sdloop/src$sdloop/
	echo "		cv$sdloop done!!!"
	sdloop=$(($sdloop+1))
done
sync

while [ $counter -le $looping ]; do 
	echo "Loop #$counter : one2many" | tee -a $dir/one2many.log
	
#Start the massive transfer from here
	sourcedev=1
	while [ $sourcedev -le $stloop ]; do
		#Clearing all contents from previous test in the device...
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

		echo "	Source directory is $dir/cv$sourcedev/tempfile$sourcedev/src$sourcedev." | tee -a $dir/one2many.log
		sdloop=1
		start=$(date +%s)
		while [ $sdloop -le $stloop ]; do
			if [ $sdloop -ne $sourcedev ]; then
				echo "		Copying files from $dir/cv$sourcedev/tempfile$sourcedev/src$sourcedev to $dir/cv$sdloop/tempfile$sdloop/src$sourcedev/..." | tee -a $dir/one2many.log		
				cp -rf $dir/cv$sourcedev/tempfile$sourcedev/src$sourcedev $dir/cv$sdloop/tempfile$sdloop/src$sourcedev/&
			fi
			sdloop=$(($sdloop+1))
		done
		echo "		Copy processes sent to background. Please wait..." | tee -a $dir/one2many.log
		wait
		sync
		echo "		Copy process done!"
		stop=$(date +%s)
		duration=$((($stop-$start)/60+1))
		echo "		Total time for the copy process = $duration minutes" | tee -a $dir/one2many.log

		sdloop=1
		while [ $sdloop -le $stloop ]; do
			if [ $sdloop -ne $sourcedev ]; then
				echo "		diff $sourcemedia and $dir/cv$sdloop/tempfile$sdloop/src$sourcedev" | tee -a $dir/one2many.log	
				diff -r $sourcemedia $dir/cv$sdloop/tempfile$sdloop/src$sourcedev >> $dir/check_if_fail/error_one2many.log
				if [ $? -ne 0 ]; then
					echo "****** Error during the diff process, $(date) ******" | tee -a $dir/one2many.log
					echo "Failed in many2one.sh test, $(date)" >> -a $dir/error.log
					result=1
				else
					echo "		No error found." | tee -a $dir/one2many.log
					echo "		Done diff process, $(date)." | tee -a $dir/one2many.log
				fi
			fi
			sdloop=$(($sdloop+1))
		done
		sourcedev=$(($sourcedev+1))
	done
	counter=$(($counter+1))
done

if [ ! -s $dir/check_if_fail/error_one2many.log ]; then
	rm -r $dir/check_if_fail/error_one2many.log 2> /dev/null
fi

echo "Completed one2many.sh, $(date)" | tee -a $dir/one2many.log

if [ $result -eq 0 ]; then
	echo -n "	[PASS] - " | tee -a $dir/logfile.log
else		
	echo -n "	[FAIL] - " | tee -a $dir/logfile.log
fi
# one2many.sh: End ~
