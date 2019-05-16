#!/system/bin/sh
dir=$(pwd)
FAIL(){
    echo "Test failed!\n"| tee -a  $dir/logfile.txt
    mv $dir/logfile.txt $dir/fail.log
    exit 1
}

PASS(){
    echo "Test completed Successfully!\n"| tee -a  $dir/logfile.txt
    mv $dir/logfile.txt $dir/pass.log
    exit 1
}

VerifySensorResult(){
    if [ -z "$2" ]; then
        echo "$1 sensor data could not be read" | tee -a  $dir/logfile.txt
        success=false
        return 1
    fi
    return 0
}

if [ $# -lt 4 ]
then
    echo "sensorhub_client usage: ./sensorhub_client <param_file> <VERIFY?> <runtime_in_minutes> <sleep_time_between_each_sample>" | tee -a  $dir/logfile.txt
    echo "  ACCEL, accelerometer;        GYRO, gyroscope;                    COMPS, compass;
            BARO, barometer;             ALS_P, ALS;                         PS_P, Proximity;
            TERMC, terminal context;     LPE_P, LPE;                         PHYAC, physical activity;
            GSSPT, gesture spotting;     GSFLK, gesture flick;               RVECT, rotation vector;
            GRAVI, gravity;              LACCL, linear acceleration;         ORIEN, orientation;
            9DOF, 9dof;                  PEDOM, pedometer;                   MAGHD, magnetic heading;
            SHAKI, shaking;              MOVDT, move detect;                 STAP, stap;
            PTZ, pan tilt zoom;          LTVTL, lift vertical;               DVPOS, device position;
            SCOUN, step counter;         SDET, step detector;                SIGMT, significant motion;
            6AGRV, game_rotation vector; 6AMRV, geomagnetic_rotation vector; 6DOFG, 6dofag;
            6DOFM, 6dofam;               LIFLK, lift look;                   DTWGS, dtwgs;
            GSPX, gesture hmm;           GSETH, gesture eartouch;            BIST, BIST;" | tee -a  $dir/logfile.txt
	FAIL
fi

params=$1
VERIFY=$2
duration_minutes=$3
sleep_between_iterations=$4
ALS_P=0
PS_P=0
BARO=0
ACCEL=0
GYRO=0
COMPS=0
fail=0

if [ $VERIFY -eq 1 ]; then
    if test -e $params ; then
        while read line 
        do
        sensor=$(echo -e "$line" | cut -d "=" -f 1)
        echo "Sensor is $sensor" | tee -a  $dir/logfile.txt
        case $sensor in
            ALS_P)
                ALS_P=1
                read line
                als_low=$(echo -e "$line" | cut -d "=" -f 2)
                echo "ALS_LOW is $als_low" | tee -a  $dir/logfile.txt
                read line
                als_high=$(echo -e "$line" | cut -d "=" -f 2)
                echo "ALS_High is $als_high" | tee -a  $dir/logfile.txt
            ;;
            PS_P)
                PS_P=1
                read line
                ps_low=$(echo -e "$line" | cut -d "=" -f 2)
                echo "PS_LOW is $ps_low" | tee -a  $dir/logfile.txt
                read line
                ps_high=$(echo -e "$line" | cut -d "=" -f 2)
                echo "PS_HIGH is $ps_high" | tee -a  $dir/logfile.txt
            ;;
            BARO)
                BARO=1
                read line
                baro_low=$(echo -e "$line" | cut -d "=" -f 2)
                echo "BARO_LOW is $baro_low" | tee -a  $dir/logfile.txt
                read line
                baro_high=$(echo -e "$line" | cut -d "=" -f 2)
                echo "BARO_High is $baro_high"    | tee -a  $dir/logfile.txt
            ;;
            GYRO)
                GYRO=1
                read line
                gyro_x_low=$(echo -e "$line" | cut -d "=" -f 2)
                echo "GYRO_LOW is $gyro_x_low" | tee -a  $dir/logfile.txt
                read line
                gyro_x_high=$(echo -e "$line" | cut -d "=" -f 2)
                echo "GYRO_High is $gyro_x_high"    | tee -a  $dir/logfile.txt 
                read line
                gyro_y_low=$(echo -e "$line" | cut -d "=" -f 2)
                echo "GYRO_LOW is $gyro_y_low" | tee -a  $dir/logfile.txt
                read line
                gyro_y_high=$(echo -e "$line" | cut -d "=" -f 2)
                echo "GYRO_High is $gyro_y_high"   | tee -a  $dir/logfile.txt  
                read line
                gyro_z_low=$(echo -e "$line" | cut -d "=" -f 2)
                echo "GYRO_LOW is $gyro_z_low" | tee -a  $dir/logfile.txt
                read line
                gyro_z_high=$(echo -e "$line" | cut -d "=" -f 2)
                echo "GYRO_High is $gyro_z_high"   | tee -a  $dir/logfile.txt  
            ;;
            ACCEL)
                ACCEL=1
                read line
                accel_x_low=$(echo -e "$line" | cut -d "=" -f 2)
                echo "ACCEL_LOW is $accel_x_low" | tee -a  $dir/logfile.txt
                read line
                accel_x_high=$(echo -e "$line" | cut -d "=" -f 2)
                echo "ACCEL_High is $accel_x_high"   | tee -a  $dir/logfile.txt  
                read line
                accel_y_low=$(echo -e "$line" | cut -d "=" -f 2)
                echo "ACCEL_LOW is $accel_y_low" | tee -a  $dir/logfile.txt
                read line
                accel_y_high=$(echo -e "$line" | cut -d "=" -f 2)
                echo "ACCEL_High is $accel_y_high"     | tee -a  $dir/logfile.txt
                read line
                accel_z_low=$(echo -e "$line" | cut -d "=" -f 2)
                echo "ACCEL_LOW is $accel_z_low" | tee -a  $dir/logfile.txt
                read line
                accel_z_high=$(echo -e "$line" | cut -d "=" -f 2)
                echo "ACCEL_High is $accel_z_high" | tee -a  $dir/logfile.txt  
            ;;
            COMPS)
                COMPS=1
                read line
                comps_x_low=$(echo -e "$line" | cut -d "=" -f 2)
                echo "COMPS_LOW is $comps_x_low" | tee -a  $dir/logfile.txt
                read line
                comps_x_high=$(echo -e "$line" | cut -d "=" -f 2)
                echo "COMPS_High is $comps_x_high"   | tee -a  $dir/logfile.txt  
                read line
                comps_y_low=$(echo -e "$line" | cut -d "=" -f 2)
                echo "COMPS_LOW is $comps_y_low" | tee -a  $dir/logfile.txt
                read line
                comps_y_high=$(echo -e "$line" | cut -d "=" -f 2)
                echo "COMPS_High is $comps_y_high"  | tee -a  $dir/logfile.txt   
                read line
                comps_z_low=$(echo -e "$line" | cut -d "=" -f 2)
                echo "COMPS_LOW is $comps_z_low" | tee -a  $dir/logfile.txt
                read line
                comps_z_high=$(echo -e "$line" | cut -d "=" -f 2)
                echo "COMPS_High is $comps_z_high"  | tee -a  $dir/logfile.txt
            ;;
            *)
            echo "script does not recognise the sensor type" | tee -a  $dir/logfile.txt
            FAIL
            ;;
        esac
        done < $params
    else
        echo "$params not found." | tee -a  $dir/logfile.txt
        FAIL
    fi
fi

duration_seconds=`expr $duration_minutes \* 60`
base_time=`date +%s`
current_time=`date +%s`
end_time=`expr $base_time + $duration_seconds`
echo "Verify is $VERIFY, Duration Minutes $duration_minutes, Duration Seconds $duration_seconds" | tee -a  $dir/logfile.txt
echo "base_time $base_time, current_time $current_time end_time $end_time" | tee -a  $dir/logfile.txt
iteration=0
success=true
if [ $VERIFY -eq 1 ]; then
    echo "Will verify every iteration" | tee -a  $dir/logfile.txt
    while [ $current_time -le $end_time ]
    do
        if [ $ALS_P -eq 1 ]; then
        result=$(./sensorhub_client -c0 -tALS_P -r1 -d0)
        als_data=$(echo $result | cut -d ' ' -f 21)
        if VerifySensorResult "ALS_P" $als_data; then
            if [ $als_data -lt $als_low -o $als_data -gt $als_high ]; then
                echo "ALS data reported is wrong.! Expected data is between $als_low and $als_high, value prvided by sensorhub is $als_data" | tee -a  $dir/logfile.txt
                FAIL
            fi
        fi
        sleep $sleep_between_iterations
        fi
        if [ $PS_P -eq 1 ]; then
        result=$(./sensorhub_client -c0 -tPS_P -r1 -d0)
        ps_data=$(echo $result | cut -d ' ' -f 19)
        if VerifySensorResult "PS_P" $ps_data; then
            if [ $ps_data -lt $ps_low -o $ps_data -gt $ps_high ]; then
                echo "PS data reported is wrong.! Expected data is between $ps_low and $ps_high, value prvided by sensorhub is $ps_data" | tee -a  $dir/logfile.txt
                FAIL
            fi
        fi
        sleep $sleep_between_iterations
        fi
        if [ $BARO -eq 1 ]; then
        result=$(./sensorhub_client -c0 -tBARO -r1 -d0)
        baro_data=$(echo $result | cut -d ' ' -f 21)
        if VerifySensorResult "BARO_PS" $baro_data; then
            if [ $baro_data -lt $baro_low -o $baro_data -gt $baro_high ]; then
                echo "BARO data reported is wrong.! Expected data is between $baro_low and $baro_high, value prvided by sensorhub is $baro_data" | tee -a  $dir/logfile.txt
                FAIL
            fi
        fi
        sleep $sleep_between_iterations
        fi
        if [ $GYRO -eq 1 ]; then
        result=$(./sensorhub_client -c0 -tGYRO -r1 -d0)
        gyro_x_data=$(echo $result | cut -d " " -f 21 | cut -d "," -f 1)
        gyro_y_data=$(echo $result | cut -d " " -f 22 | cut -d "," -f 1)
        gyro_z_data=$(echo $result | cut -d " " -f 23 | cut -d "," -f 1)
        if VerifySensorResult "GYRO_X" $gyro_x_data && VerifySensorResult "GYRO_Y" $gyro_y_data && VerifySensorResult "GYRO_Z" $gyro_z_data; then
            if [ $gyro_x_data -lt $gyro_x_low -o $gyro_x_data -gt $gyro_x_high ]; then
            if [ $gyro_y_data -lt $gyro_y_low -o $gyro_y_data -gt $gyro_y_high ]; then
            if [ $gyro_z_data -lt $gyro_z_low -o $gyro_z_data -gt $gyro_z_high ]; then
                echo "Gyroscope Sensor data reported is a wrong.!" | tee -a  $dir/logfile.txt
                echo "Expected data is between $gyro_x_low and $gyro_x_high, value prvided by sensorhub is $gyro_x_data " | tee -a  $dir/logfile.txt
                echo "Expected data is between $gyro_y_low and $gyro_x_high, value prvided by sensorhub is $gyro_y_data " | tee -a  $dir/logfile.txt
                echo "Expected data is between $gyro_y_low and $gyro_x_high, value prvided by sensorhub is $gyro_z_data " | tee -a  $dir/logfile.txt
                FAIL
            fi
            fi
            fi
        fi
        sleep $sleep_between_iterations
        fi
        if [ $COMPS -eq 1 ]; then
        result=$(./sensorhub_client -c0 -tCOMPS -r1 -d0)
        comps_x_data=$(echo $result | cut -d " " -f 21 | cut -d "," -f 1)
        comps_y_data=$(echo $result | cut -d " " -f 22 | cut -d "," -f 1)
        comps_z_data=$(echo $result | cut -d " " -f 23 | cut -d "," -f 1)
        if VerifySensorResult "COMPS_X" $comps_x_data && VerifySensorResult "COMPS_Y" $comps_y_data && VerifySensorResult "COMPS_Z" $comps_z_data; then
            if [ $comps_x_data -lt $comps_x_low -o $comps_x_data -gt $comps_x_high ]; then
            if [ $comps_y_data -lt $comps_y_low -o $comps_y_data -gt $comps_y_high ]; then
            if [ $comps_z_data -lt $comps_z_low -o $comps_z_data -gt $comps_z_high ]; then
                echo "COMPS Sensor data reported is a wrong.!" | tee -a  $dir/logfile.txt
                echo "Expected data is between $comps_x_low and $comps_x_high, value prvided by sensorhub is $comps_x_data " | tee -a  $dir/logfile.txt
                echo "Expected data is between $comps_y_low and $comps_y_high, value prvided by sensorhub is $comps_y_data " | tee -a  $dir/logfile.txt
                echo "Expected data is between $comps_z_low and $comps_z_high, value prvided by sensorhub is $comps_z_data " | tee -a  $dir/logfile.txt
                FAIL
            fi
            fi
            fi
        fi
        sleep $sleep_between_iterations
        fi
        if [ $ACCEL -eq 1 ]; then
        result=$(./sensorhub_client -c0 -tACCEL -r1 -d0)
        accel_x_data=$(echo $result | cut -d " " -f 21 | cut -d "," -f 1)
        accel_y_data=$(echo $result | cut -d " " -f 22 | cut -d "," -f 1)
        accel_z_data=$(echo $result | cut -d " " -f 23 | cut -d "," -f 1)
        if VerifySensorResult "ACCEL_X" $accel_x_data && VerifySensorResult "ACCEL_Y" $accel_y_data && VerifySensorResult "ACCEL_Z" $accel_z_data; then
            if [ $accel_x_data -lt $accel_x_low -o $accel_x_data -gt $accel_x_high ]; then
            if [ $accel_y_data -lt $accel_y_low -o $accel_y_data -gt $accel_y_high ]; then
            if [ $accel_z_data -lt $accel_z_low -o $accel_z_data -gt $accel_z_high ]; then
                echo "Accelerometer Sensor data reported is a wrong.!" | tee -a  $dir/logfile.txt
                echo "Expected data is between $accel_x_low and $accel_x_high, value prvided by sensorhub is $accel_x_data " | tee -a  $dir/logfile.txt
                echo "Expected data is between $accel_y_low and $accel_y_high, value prvided by sensorhub is $accel_y_data " | tee -a  $dir/logfile.txt
                echo "Expected data is between $accel_z_low and $accel_z_high, value prvided by sensorhub is $accel_z_data " | tee -a  $dir/logfile.txt
                FAIL
            fi
            fi
            fi          
        fi
        sleep $sleep_between_iterations
        fi
        if [ "$success" = true ]; then
            echo "no errors reported in iteration: $iteration" | tee -a  $dir/logfile.txt
            sleep $sleep_between_iterations
            iteration=$(($iteration+1))
            current_time=`date +%s`
        else
            FAIL
        fi
    done
else
    echo "Will skip verification" | tee -a  $dir/logfile.txt
    while [ $current_time -le $end_time ]
    do
        if [ $ALS_P -eq 1 ]; then
        result=$(./sensorhub_client -c0 -tALS_P -r1 -d0)

        sleep $sleep_between_iterations
        fi
        if [ $PS_P -eq 1 ]; then
        result=$(./sensorhub_client -c0 -tPS_P -r1 -d0)
        
        sleep $sleep_between_iterations
        fi
        if [ $BARO -eq 1 ]; then
        result=$(./sensorhub_client -c0 -tBARO -r1 -d0)
        
        sleep $sleep_between_iterations
        fi
        if [ $GYRO -eq 1 ]; then
        result=$(./sensorhub_client -c0 -tGYRO -r1 -d0)
        
        sleep $sleep_between_iterations
        fi
        if [ $COMPS -eq 1 ]; then
        result=$(./sensorhub_client -c0 -tCOMPS -r1 -d0)
        
        sleep $sleep_between_iterations
        fi
        if [ $ACCEL -eq 1 ]; then
        result=$(./sensorhub_client -c0 -tACCEL -r1 -d0)

        sleep $sleep_between_iterations
        fi
        echo "no errors reported in iteration: $iteration" | tee -a  $dir/logfile.txt
        sleep $sleep_between_iterations
        iteration=$(($iteration+1))
        current_time=`date +%s`
    done
fi

if [ "$success" = true ]; then
    echo "\nDone with the test!\n" | tee -a  $dir/logfile.txt
    PASS
else
    echo "\nTest failed!\n" | tee -a  $dir/logfile.txt
    FAIL
fi

