#! /system/bin/sh
dir=`dirname $0`
PASS_FILE=$dir/pass.log
FAIL_FILE=$dir/fail.log
ALL_OUTPUT_FILE=$dir/detailed_output.log
LOG_FILE=$dir/vp8_encoding.log
REMOVE_TEMP_FILES=true
EXIT_ON_FAILURE=true
SAVE_DETAILED_OUTPUT=false

# Remove any leftover log files
rm -r $dir/*.log 2> /dev/null

if [ $# -lt 1 ]
then
	echo "Usage: $0 <duration in minute(s)>" | tee -a $FAIL_FILE
	exit 1	
fi
if [ ! -d /data/vp8_encoding/vp8_files ]; then mkdir /data/vp8_encoding/vp8_files; fi
test_time=$1
echo "`date` VP8 Encoding Test" | tee -a $LOG_FILE $ALL_OUTPUT_FILE
echo "Test Parameters:" | tee -a $LOG_FILE $ALL_OUTPUT_FILE
echo "  Run Time: $test_time seconds" | tee -a $LOG_FILE $ALL_OUTPUT_FILE
echo "  Log File: $LOG_FILE" | tee -a $LOG_FILE $ALL_OUTPUT_FILE
echo "  Detailed Log File: $ALL_OUTPUT_FILE" | tee -a $LOG_FILE $ALL_OUTPUT_FILE
echo "" | tee -a $LOG_FILE $ALL_OUTPUT_FILE

function fail_test() {
    mv $LOG_FILE $FAIL_FILE
    if [ "$REMOVE_TEMP_FILES" = true ]
    then
        exit 1
    fi
}

function encode_verify_and_cleanup {
    # Parameters: Width Height Bitrate
    output_file_postfix=`date +%s`
    output_file=vp8_encoded_output_file_$output_file_postfix.vp8
    stderr_output_file=vp8_encoding_stderr_$output_file_postfix.log
    # New Command
    echo "Executing ./new_va_encode -t VP8 -w $1 -h $2 --bitrate $3 --initialqp 28 -framecount 1 -rcMode CBR --stridealign 64 -slices 2 -o $output_file 1>>$ALL_OUTPUT_FILE 2>$stderr_output_file" | tee -a $LOG_FILE
    # The new_va_encode binary comes from the Android souce code repository
    ./new_va_encode -t VP8 -w $1 -h $2 --bitrate $3 --initialqp 28 -framecount 1 -rcMode CBR --stridealign 64 -slices 2 -o $output_file 1>>$ALL_OUTPUT_FILE 2>$stderr_output_file
    return_code=$?
    if [ $return_code -ne 0 ]; then
        echo "ERROR: Return code received ($return_code) is non-zero, indicating error occurred. Failing test." | tee -a $LOG_FILE $ALL_OUTPUT_FILE
        fail_test
        return 1
    fi
    if [ -s $stderr_output_file ]; then
        echo "ERROR: Output received on STDERR and saved to $stderr_output_file. Failing test." | tee -a $LOG_FILE $ALL_OUTPUT_FILE
        fail_test
        return 1
    fi
    if [ ! -f $output_file ]; then
        echo "ERROR: Output file ($output_file) not found. Failing test." | tee -a $LOG_FILE $ALL_OUTPUT_FILE
        fail_test
        return 1
    fi
    if [ ! -s $output_file ]; then
        echo "ERROR: Output file ($output_file) size is 0 bytes. Failing test." | tee -a $LOG_FILE $ALL_OUTPUT_FILE
        fail_test
        return 1
    fi
    # Delete our temporary files
    if [ "$REMOVE_TEMP_FILES" = true ]
    then
        rm $output_file
        rm $stderr_output_file
    fi
}

loop_count=1
end_time=$((`date +%s`+($test_time*60)))
echo "Start time is `date`" | tee -a $LOG_FILE $ALL_OUTPUT_FILE
while [ `date +%s` -lt $end_time ]
do
    echo "`date` Loop $loop_count. $(($end_time-`date +%s`)) seconds remaining." | tee -a $LOG_FILE $ALL_OUTPUT_FILE
    # Parameters: Width Height Bitrate
    encode_verify_and_cleanup 176 144 100000
    encode_verify_and_cleanup 320 240 400000
    encode_verify_and_cleanup 640 480 1000000
    encode_verify_and_cleanup 1920 1080 5000000
    loop_count=$((loop_count+1))
done

# Delete our detailed output
if [ "$SAVE_DETAILED_OUTPUT" != true ]
then
    rm $ALL_OUTPUT_FILE
fi

echo "Test Passed! Moving log file from $LOG_FILE to $PASS_FILE" | tee -a $LOG_FILE $ALL_OUTPUT_FILE
mv $LOG_FILE $PASS_FILE
