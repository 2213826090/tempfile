#pshtest_multi_cfg6.sh
#Rev2.0
#Owner: Soo, Swee Kiong
#Date: 12/10/2012
#Change: 1st script.
#Date: 12/19/2012
#Change: Updated the PSH sysfs path to dynamic.
#Date: 12/28/2012
#Change: Fixed some coding issue.
#Date: 1/2/2013
#Change: Updated the major pause time between 2 IPC commands
#Date: 1/22/2013
#Change: Create result.log to store the test result.
#Date: 2/25/2013
#Change: Updated with new resp-cat, reset and get status commands.
#Date: 7/15/2014
#Change: (vbpeters) Copied pshtest_multi_cfg6.sh to read_sensors_loop.sh so that filename is more
#   generic and independent of platform and configuration.  This allows greater reuse of automation
#   software that invokes this script.  This script currently works on the Moorefield (Anniedale) platform.
#   Renamed log files accordingly.
#!/system/bin/sh

# Test Variables
DATA_LOG_FILE=read_sensors_loop_data.log
RESULT_LOG_FILE=read_sensors_loop_results.log
EXE_FILE=`pwd`/resp_cat

# Catch Ctrl-C
trap bashtrap INT
bashtrap()
{
        echo "CTRL+C detected !! Exit Program"
        exit
}

# Remove our log files if they already exist
if [ -f $DATA_LOG_FILE ]
then
    rm $DATA_LOG_FILE
fi
if [ -f $RESULT_LOG_FILE ]
then
    rm $RESULT_LOG_FILE
fi

#List of sensors                            
sensor1=1       #Accelerometer              
sensor2=2       #Gyroscope                           
sensor3=3       #Magnetometer                        
sensor4=4       #Barometer                            
sensor5=5       #Ambient Light Sensor                
sensor6=6       #Proximity Sensor                    
sensor7=102     #Terminal                                            
sensor8=105     #9DOF                                                
sensor9=108     #Gravity                                             
sensor10=109    #Orientation 

#sensor1=1       #Accelerometer              
#sensor2=2       #Gyroscope                           
#sensor3=3       #Magnetometer                        
#sensor4=4       #Brometer                            
#sensor5=5       #Ambient Light Sensor                
#sensor6=6       #Proximity Sensor                    
#sensor7=102     #Terminal 
#sensor8=105     #9DOF                                               
#sensor9=108     #Gravity  
#sensor10=109    #Orientation

#Sensor label
ssr1=Accelerometer
ssr2=Gyroscope
ssr3=9DOF
ssr4=Magnetometer
ssr5=Barometer
ssr6=Proximity_Sensor
ssr7=Terminal
ssr8=9DOF
ssr9=Gravity
ssr10=Orientation

#Input for test duration
a=$(($1*60*60))

#Config for CMD_GET_SINGLE 
#a=20 #Streaming duration in sec                     
b=100 ## of iterations 

# Defining the PSH sysfs path
i=0
for i in $(seq 0 10);
do
        if [ -f /sys/class/hwmon/hwmon$i/device/control ];
        then
                export CONTROL=/sys/class/hwmon/hwmon$i/device/control
                break
        fi
done

echo "PSH sysfs path: $CONTROL" | tee -a $DATA_LOG_FILE $RESULT_LOG_FILE

# Configuration before test starts
pkill sensorhubd
#cd /sys/class/hwmon/hwmon2/device
echo 0 4 1 > $CONTROL
sleep 2
echo 0 4 5 > $CONTROL
sleep 2
echo 0 4 6 > $CONTROL
sleep 2
echo 0 4 102 > $CONTROL
sleep 2
echo 0 4 105 > $CONTROL
sleep 2
echo 0 4 106 > $CONTROL
sleep 2

#IPC CMD_RESET
echo 0 0 0 > $CONTROL

current_time=`date +%s`
start_time=$current_time
end_time=`expr $start_time + $a`
timestamp=`date +%Y%m%d-%H%M%s`

#Test Config
echo "PSH IPC testing for sensor = "$ssr1,$ssr2,$ssr3,$ssr4,$ssr5 | tee -a $DATA_LOG_FILE $RESULT_LOG_FILE
echo "Test duration = $1 hour(s)" | tee -a $DATA_LOG_FILE $RESULT_LOG_FILE
echo "Start time = $timestamp" | tee -a $DATA_LOG_FILE $RESULT_LOG_FILE

current_time=`date +%s`
start_time=$current_time
end_time=`expr $start_time + $a`
timestamp=`date +%Y%m%d-%H%M%s`

while [[ $current_time -lt $end_time ]]
do

    ##IPC CMD_GET_STATUS
    # Start resp_cat printout                                            
#   cd /sys/class/hwmon/hwmon2/device
#   sleep 2

    $EXE_FILE 5 >> $DATA_LOG_FILE &
    pid=$!
    sleep 2

#   cd /sys/class/hwmon/hwmon2/device

    echo "Get status starts" >> $DATA_LOG_FILE

    current_time=`date +%s`                                              
    start_time=$current_time                                             
    timestamp=`date +%Y%m%d-%H%M%s`                                      
    echo "Get status start time: $timestamp" >> $DATA_LOG_FILE

    echo 0 11 0 0xff 0xff 0xff 0xff > $CONTROL

    wait $pid

    echo "Get status stops" >> $DATA_LOG_FILE

    current_time=`date +%s`
    start_time=$current_time
    timestamp=`date +%Y%m%d-%H%M%s`  

    echo "Get status stop time: $timestamp" >> $DATA_LOG_FILE

    #Pause time
    n=$RANDOM
    echo "Pause time =  $((n%=10))" >> $DATA_LOG_FILE
        echo "Pause time =  $((n%=10))"
    sleep $((n%=10))

    
    ##IPC CMD_GET_SINGLE
    # Start resp_cat printout
#   cd /sys/class/hwmon/hwmon2/device                         
#   sleep 2 

    $EXE_FILE 70 >> $DATA_LOG_FILE &
    pid=$!
    sleep 2

#   cd /sys/class/hwmon/hwmon2/device
#   sleep 2

    x=0
    echo "Get single starts" >> $DATA_LOG_FILE
    current_time=`date +%s`                     
    start_time=$current_time                    
    timestamp=`date +%Y%m%d-%H%M%s`             
    echo "Get single start time: $timestamp" >> $DATA_LOG_FILE

    while [[ $x -lt $b ]]
    do
        echo $x >> $DATA_LOG_FILE
        echo 0 2 $sensor1 > $CONTROL
        echo 0 2 $sensor2 > $CONTROL 
            echo 0 2 $sensor3 > $CONTROL  
            echo 0 2 $sensor4 > $CONTROL
            echo 0 2 $sensor5 > $CONTROL    
        sleep 5
        x=$(($x+1))
        sleep 2
    done
    wait $pid

    echo "Get single stops" >> $DATA_LOG_FILE
    current_time=`date +%s`                     
    start_time=$current_time                    
    timestamp=`date +%Y%m%d-%H%M%s`             
    echo "Get single stop time: $timestamp" >> $DATA_LOG_FILE

        #Pause time
        n=$RANDOM
        echo "Pause time =  $((n%=300+60))" >> $DATA_LOG_FILE
        echo "Pause time =  $((n%=300+60))"
        sleep $((n%=300+60))


    ##IPC CMD_CFG_STREAM & CMD_STOP_STREAM
    # Start resp_cat printout                                 
#   cd /sys/class/hwmon/hwmon2/device                         
#   sleep 2                                       

    $EXE_FILE 1230 >> $DATA_LOG_FILE &
    pid=$!                                                                  
    sleep 2                                                   

#   cd /sys/class/hwmon/hwmon2/device   
#   sleep 2

    echo "Cfg stream starts" >> $DATA_LOG_FILE

    current_time=`date +%s` 
    start_time=$current_time 
    timestamp=`date +%Y%m%d-%H%M%s` 

    echo "Cfg stream start time: $timestamp" >> $DATA_LOG_FILE

    echo 0 3 $sensor1 10 0 0 0 > $CONTROL
    echo 0 3 $sensor2 10 0 0 0 > $CONTROL 
    echo 0 3 $sensor3 10 0 0 0 > $CONTROL 
    echo 0 3 $sensor4 10 0 0 0 > $CONTROL 
    echo 0 3 $sensor5 10 0 0 0 > $CONTROL

    sleep 1200

    echo 0 4 $sensor1 > $CONTROL
    echo 0 4 $sensor2 > $CONTROL 
    echo 0 4 $sensor3 > $CONTROL 
    echo 0 4 $sensor4 > $CONTROL 
    echo 0 4 $sensor5 > $CONTROL

    sleep 30

    wait $pid

    echo "Cfg stream stops" >> $DATA_LOG_FILE

    current_time=`date +%s`
    start_time=$current_time
    timestamp=`date +%Y%m%d-%H%M%s` 

    echo "Cfg stream stop time: $timestamp" >> $DATA_LOG_FILE

        #Pause time
        n=$RANDOM
        echo "Pause time =  $((n%=600+60))" >> $DATA_LOG_FILE
        echo "Pause time =  $((n%=600+60))"
        sleep $((n%=600+60))
    
done

echo "Stop Time = $timestamp" | tee -a $DATA_LOG_FILE $RESULT_LOG_FILE
echo "Test Completed" | tee -a $DATA_LOG_FILE $RESULT_LOG_FILE
