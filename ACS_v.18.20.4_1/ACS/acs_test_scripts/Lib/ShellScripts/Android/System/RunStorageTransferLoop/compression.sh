#!/system/bin/sh
# Compression: Start ~

dir=$stdir

echo -e "\n\n\n"
if [ -z "$1" ]; then echo "Usage: $dir/compression.sh <number of loops>"; exit 1; fi
echo "Start compression.sh test on $(date)" | tee -a $dir/compression.log
looping=$1
counter=1
sourcemedia=$mediapath
result=0

##### Functions
copy(){
	rm -r $2 2> /dev/null
	sync
	echo -n "		Copy from $1 to $2" | tee -a $dir/compression.log
	start=$(date +%s)
	cp -rf $1 $2
	sync
	stop=$(date +%s)
	duration=$((($stop-$start)/60+1))
	echo "	$duration minutes" | tee -a $dir/compression.log
}

compare(){
	echo "		Comparing the source file $1 and $2" | tee -a $dir/compression.log
	diff -r $1 $2 >> $dir/check_if_fail/error_compression.log
	if [ $? -ne 0 ]; then
		echo "****** Error during the diff process, $(date) ******" | tee -a $dir/compression.log
		echo "Failed in compression.sh test, $(date)" >> $dir/error.log
		result=1
	else
		echo "		No error found." | tee -a $dir/compression.log
		echo "		Done diff process, $(date)." | tee -a $dir/compression.log	
	fi
}
#####

echo "		Clearing all contents from previous test in the device..."

sdloop=1
while [ $sdloop -le $stloop ]; do
	rm -r $dir/cv$sdloop/tempfile$sdloop 2> /dev/null
	mkdir $dir/cv$sdloop/tempfile$sdloop 2> /dev/null
	sdloop=$(($sdloop+1))
done
sync

while [ $counter -le $looping ]; do 
	sdloop=1
	echo "Loop #$counter : compression" | tee -a $dir/compression.log

	while [ $sdloop -le $stloop ]; do
		spaces=$(df -m | grep $dir/cv$sdloop | awk '{ print($4) }')

		if [ $spaces -ge 4000 ]; then
			size=""
		elif [ $spaces -ge 2000 ]; then
			size="/1GBFiles"
		elif [ $spaces -ge 1000 ]; then
			size="/1GBFiles/512MBFiles"
		elif [ $spaces -ge 512 ]; then
			size="/1GBFiles/512MBFiles/256MBFiles"
		else
			echo "cv$sdloop device run out of space, $(date)" | tee -a $dir/error.log
			exit 1
		fi

		copy $sourcemedia/$size $dir/cv$sdloop/tempfile$sdloop/src$sdloop
		
		echo -n "		Compress $dir/cv$sdloop/tempfile$sdloop/src$sdloop" | tee -a $dir/compression.log
		start=$(date +%s)
		tar -czf $dir/cv$sdloop/tempfile$sdloop/src$sdloop.tar.gz $dir/cv$sdloop/tempfile$sdloop 2> /dev/null
		wait
		stop=$(date +%s)
		duration=$((($stop-$start)/60+1))
		echo "	$duration minutes" | tee -a $dir/compression.log
		
		rm -r $dir/cv$sdloop/tempfile$sdloop/src$sdloop 2> /dev/null
		sync
		
		echo -n "		Uncompress $dir/cv$sdloop/tempfile$sdloop/src$sdloop.tar.gz" | tee -a $dir/compression.log
		start=$(date +%s)
		tar -xzf $dir/cv$sdloop/tempfile$sdloop/src$sdloop.tar.gz 2> /dev/null
		wait
		stop=$(date +%s)
		duration=$((($stop-$start)/60+1))
		echo "	$duration minutes" | tee -a $dir/compression.log

		compare $sourcemedia$size $dir/cv$sdloop/tempfile$sdloop/src$sdloop
		
		sdloop=$(($sdloop+1))
	done
	counter=$(($counter+1))
done

if [ ! -s $dir/check_if_fail/error_compression.log ]; then
	rm -r $dir/check_if_fail/error_compression.log 2> /dev/null
fi

echo "Completed compression.sh, $(date)" | tee -a $dir/compression.log

if [ $result -eq 0 ]; then
	echo -n "	[PASS] - " | tee -a $dir/logfile.log
else		
	echo -n "	[FAIL] - " | tee -a $dir/logfile.log
fi
# Token Ring: End ~
