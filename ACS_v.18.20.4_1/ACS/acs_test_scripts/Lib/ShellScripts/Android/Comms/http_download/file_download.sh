#!/system/bin/sh

# This script is a simple test to repeatedly download a file using wget as a
#  a test of network connectivity. This script was designed to be executed
#  in the Android OS.
# Author(s): Jongyoon.Choi@intel.com, Carl.G.Sapp@intel.com

EXE_DIR=`dirname $0`
LOG_FILE=file_download.log

# Set our default arguments
test_time=5*60 # Default test duration 5 minutes
download_url=http://216.165.129.134/directory/mavibot/dist/1.0.0-M3/mavibot-1.0.0-M3-bin.tar.gz
sleep_time_seconds=0 # Don't sleep by default
expected_file_checksum=bc4ad0d96df02a07d9d3fe785409e8be
allowed_retries=20

# Parse our arguments
args=( "$@" )
arg_index=0
while [ $arg_index -lt $# ]
do
    arg=${args[$arg_index]}
    #echo "Argument Index $arg_index = $arg"
    if [ $arg == "-h" ] || [ $arg == "--help" ]
    then
        echo "Usage: file_download.sh [OPTION]..."
        echo "Continuously download files over the network."
        echo -e "  -p, --http_proxy=HTTP_PROXY\t set the http_proxy environment variable to this value before starting download"
        echo -e "  -s, --https_proxy=HTTPS_PROXY\t set the https_proxy environment variable to this value before starting download"
        echo -e "  -d, --duration_minutes=MINUTES\t amount of time to run test"
        echo -e "  -i, --sleep_seconds=SECONDS\t sleep this many seconds between iterations"
        echo -e "  -u, --url=URL\t url to download file from"
        echo -e "  -c, --file_checksum=CHECKSUM_HEX_LOWERCASE\t MD5 checksum of the file that will be downloaded"
        echo -e "  -a, --allowed_retries=ALLOWED_RETRIES\t number of allowed retries in case of checksum error"
        echo -e "  -h, --help\t display this help and exit"
        exit
    fi
    if [ $arg == "-p" ] || [[ $arg == --http_proxy=* ]]
    then
        if [ $arg == "-p" ]; then
            arg_index=$(($arg_index+1))
            arg2=${args[$arg_index]}
        else
            arg2=`echo $arg | cut -d \= -f 2`
        fi
        export http_proxy=$arg2
    fi
    if [ $arg == "-s" ] || [[ $arg == --https_proxy=* ]]
    then
        if [ $arg == "-s" ]; then
            arg_index=$(($arg_index+1))
            arg2=${args[$arg_index]}
        else
            arg2=`echo $arg | cut -d \= -f 2`
        fi
        export https_proxy=$arg2
    fi
    if [ $arg == "-d" ] || [[ $arg == --duration_minutes=* ]]
    then
        if [ $arg == "-d" ]; then
            arg_index=$(($arg_index+1))
            arg2=${args[$arg_index]}
        else
            arg2=`echo $arg | cut -d \= -f 2`
        fi
        test_time=$(($arg2*60))
    fi
    if [ $arg == "-i" ] || [[ $arg == --sleep_seconds=* ]]
    then
        if [ $arg == "-i" ]; then
            arg_index=$(($arg_index+1))
            arg2=${args[$arg_index]}
        else
            arg2=`echo $arg | cut -d \= -f 2`
        fi
        sleep_time_seconds=$arg2
    fi
    if [ $arg == "-u" ] || [[ $arg == --url=* ]]
    then
        if [ $arg == "-u" ]; then
            arg_index=$(($arg_index+1))
            arg2=${args[$arg_index]}
        else
            arg2=`echo $arg | cut -d \= -f 2`
        fi
        download_url=$arg2
    fi
    if [ $arg == "-c" ] || [[ $arg == --file_checksum=* ]]
    then
        if [ $arg == "-c" ]; then
            arg_index=$(($arg_index+1))
            arg2=${args[$arg_index]}
        else
            arg2=`echo $arg | cut -d \= -f 2`
        fi
        expected_file_checksum=$arg2
    fi
    if [ $arg == "-a" ] || [[ $arg == --allowed_retries=* ]]
    then
        if [ $arg == "-a" ]; then
            arg_index=$(($arg_index+1))
            arg2=${args[$arg_index]}
        else
            arg2=`echo $arg | cut -d \= -f 2`
        fi
        allowed_retries=$arg2
    fi
    arg_index=$(($arg_index+1))
done
# Finished parsing arguments

# Get our file name out of the URL
filename=`echo $download_url | awk -F/ '{print $(NF)}'`
# Move to our EXE directory
cd $EXE_DIR
# Remove old log files
if [ -f pass.log ]; then rm pass.log; fi;
if [ -f fail.log ]; then rm fail.log; fi;
if [ -f $LOG_FILE ]; then rm $LOG_FILE; fi;

echo "Start Time: $(date)" >> $LOG_FILE
echo "Test Parameters:" >> $LOG_FILE
echo "    Download URL: $download_url" >> $LOG_FILE
echo "    File Name: $filename" >> $LOG_FILE
echo "    Expected File Checksum: $expected_file_checksum" >> $LOG_FILE
echo "    Test Duration: $test_time seconds" >> $LOG_FILE
echo "    Sleep Time Between Tests: $sleep_time_seconds seconds" >> $LOG_FILE
echo "    EXE Directory: $EXE_DIR" >> $LOG_FILE
echo "    Log File: $LOG_FILE" >> $LOG_FILE

loop=1
total_download_time=0
avg_download_time=0
max_download_time=0
min_download_time=0
num_retries=0
start_time=`date +%s`
while [ $((`date +%s`-$start_time)) -lt $test_time ]
do
    echo " " >> $LOG_FILE
    echo "   Loop: $loop - $(date)" >> $LOG_FILE
    echo "   Downloading $download_url ... " >> $LOG_FILE

    dl_start_time=$(date +%s)
    wget $download_url 2>>$LOG_FILE
    duration=$((`date +%s`-$dl_start_time))
    total_download_time=$(($total_download_time+$duration))
    echo "Download Time: $duration seconds" >> $LOG_FILE
    if [ $duration -lt $min_download_time ]
    then
        min_download_time=$(($duration))
    fi

    if [ $duration -gt $max_download_time ]
    then
        max_download_time=$(($duration))
    fi

    file_checksum=$(md5sum $filename | awk '{ print substr( $0, 0, 32 ) }')

    if [[ -z "$file_checksum" ]]
    then
        num_retries=$(($num_retries+1))
        echo "FAILED, checksum is null" >> $LOG_FILE
        echo "Retries = $num_retries" >> $LOG_FILE
        # Let's sleep a few seconds before retrying since it failed.
        sleep 5
    fi

    echo "   Reference Checksum: $expected_file_checksum" >> $LOG_FILE
    echo "   Calculated Checksum: $file_checksum" >> $LOG_FILE

    if [ $file_checksum == $expected_file_checksum ] && [ -n "$file_checksum" ];
    then
        echo "PASSED" >> $LOG_FILE
    else
        num_retries=$(($num_retries+1))
        echo "FAILED, checksum did not match" >> $LOG_FILE
        echo "Retries = $num_retries" >> $LOG_FILE
        # Let's sleep a few seconds before retrying since it failed.
        sleep 5
    fi

    rm -r $filename
    sync

    loop=$(($loop+1))
    # Sleep
    sleep $sleep_time_seconds
done
avg_download_time=$(($total_download_time/$loop))
echo "Ran $loop loops." >> $LOG_FILE
echo "Average download time was $avg_download_time seconds." >> $LOG_FILE
echo "Maximum download time was $max_download_time seconds." >> $LOG_FILE
echo "Minimum download time was $min_download_time seconds." >> $LOG_FILE
echo "End Time: $(date)" >> $LOG_FILE
if [ $num_retries -gt $allowed_retries ]
then
    cp $LOG_FILE fail.log
else
    cp $LOG_FILE pass.log
fi