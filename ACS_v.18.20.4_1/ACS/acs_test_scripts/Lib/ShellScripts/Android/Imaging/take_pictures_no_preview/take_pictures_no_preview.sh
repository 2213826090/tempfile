#!/system/bin/sh

################################################################################################################
#description :
#    Repeatedly take pictures without preview for the specified amount of time($1).
#    This will take NUMBER_OF_FRAMES($3) images at one time, and wait for DELAY_BETWEEN_SHOT($2) until next shot.
#    Note that captured image size depends on WIDTH($4) and HEIGHT($5).
#    Because of this reason, EXPECTED_IMAGE_SIZE($6) is required for verification of operation.
#parameters/usage :
#    $1 : How long the test will run, in minutes
#    $2 : Delay bettwen taking shots in seconds
#    $3 : Number of images to capture at onc
#    $4 : Width of the image
#    $5 : Height of the image
#    $6 : Expected file size of the image. This is required for verification purpose.
#         If EXPECTED_IMAGE_SIZE > 0, the size of each captured image will be compared with EXPECTED_IMAGE_SIZE,
#         and the test will fail in the case of a mismatch. If EXPECTED_IMAGE_SIZE = 0 then
#         it simply makes sure that the expected number of files are created on each iteration.
#author : Jongyoon Choi
#date : 5 September 2014
#organization : INTEL PEG-SVE-DSV
################################################################################################################

dir=`dirname $0`
log_file=$dir/logfile.log
fail_log_file=$dir/fail.log
pass_log_file=$dir/pass.log
rm -r $dir/*.log $dir/*.raw 2> /dev/null

if [ $# -lt 6 ]
then
    echo "ERROR: $0 wrong usage, please provide 6 inputs" > $log_file
    echo "ERROR: $0 #duration #delay #of_frames #width #height #expected_size" > $log_file
    mv $dir/logfile.txt $dir/fail.log
    exit 1
fi

duration_minutes=$1
delay_between_shot=$2
number_of_frames=$3
image_width=$4
image_height=$5
expected_img_size=$6

duration_seconds=`expr $duration_minutes \* 60`
base_time=`date +%s`
current_time=`date +%s`
end_time=`expr $base_time + $duration_seconds`

echo "$0: Current time in Seconds since beginning of the Time is $base_time" >> $log_file
echo "$0: Test will complete when world moves past $end_time" >> $log_file

P="-o $dir/testimage_@.raw --device /dev/video0 --input 0 --parm type=1,capturemode=CI_MODE_STILL_CAPTURE --cvf_parm=0,0,0 --fmt type=1,width=$image_width,height=$image_height,pixelformat=SGRBG10 --reqbufs count=2,memory=USERPTR --exposure=5400,0,0,256 --capture=$number_of_frames"

mount -o remount -w /dev/block/mmcblk0p6 /system

$dir/v4l2n --enuminput

echo "$0: /system/v4l2n $P" >> $log_file

shot_count=0

while [ $current_time -le $end_time ]
do
    $dir/v4l2n $P

    image_count=0
    for image_file in `ls -a $dir/*.raw`
    do
        if [ "$expected_img_size" -gt 0 ]; then
            captured_image_size=`stat -c %s $dir/$image_file`
            if [ "$expected_img_size" -ne "$captured_image_size" ]; then
                echo "File size of $image_file is $captured_image_size, it does not match the expected size $expected_img_size" >> $log_file
                mv $log_file $fail_log_file
                exit 0
            fi
         fi
        image_count=$((image_count+1))
    done

    shot_count=$((shot_count+1))
    if [ "$image_count" -ne "$number_of_frames" ]; then
        echo "Shot $shot_count: $number_of_frames images should be captured, but only $image_count images were taken" >> $log_file
        mv $log_file $fail_log_file
        exit 0
    else
        echo "Shot $shot_count: $number_of_frames images were taken"  >> $log_file
    fi

    rm $dir/*.raw 2> /dev/null

    sleep $delay_between_shot
    current_time=`date +%s`
done

mv $log_file $pass_log_file

exit 0