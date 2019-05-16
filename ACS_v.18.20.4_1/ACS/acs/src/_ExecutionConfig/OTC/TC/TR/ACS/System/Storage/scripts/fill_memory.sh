#! /bin/bash -x

fill_memory() {
        if [ -z $1 ]
        then
                echo "No parameters are given to the function. Coding error from script side. Please contact the author for solutions. FAIL"
                exit 1
        fi

        free_value=$(adb shell df | grep $1 | awk '{print $4}' | tr -d "G")
        if [ -z $free_value ]
        then
                echo "Parameter inputted not found. FAIL"
                exit 1
        fi

        adjusted_free=$( echo "$free_value - 0.2 " | bc -l )
        if [ -z $adjusted_free ]
        then
                echo "var:adjusted_free: null or NAN value for this var. Maybe the memory is already full?"
        else
                real_free=$(echo $adjusted_free | awk -F'.' '{print $1}')
                decimal_free=$(echo $adjusted_free | awk -F'.' '{print $2}')

                if [ $real_free -ne 0 ]
                then
                        count1=$( echo "$real_free * 1024000" | bc -l )
                        adb shell toolbox dd if=/dev/urandom of=$1/testGB_$real_free count=$count1 bs=1024
                fi

                if [ $decimal_free -ne 0 ]
                then
                        count2=$( echo "$decimal_free * 102400" | bc -l )
                        adb shell toolbox dd if=/dev/urandom of=$1/testMB_$decimal_free count=$count2 bs=1024
                fi
        fi

        sleep 40
        #interface verification step
        #verify_interface=$(python fill_memory.py)
        #if [ "$verify_interface" == "PASS" ]
        #then
        #        echo "PASS"
        #else
        #        echo $verify_interface
        #fi
}

fill_memory_1MB() {
        if [ -z $1 ]
        then
                echo "No parameters are given to the function. Coding error from script side. Please contact the author for solutions. FAIL"
                exit 1
        fi

        free_value=$(adb shell df | grep $1 | awk '{print $4}' | tr -d "G")
        if [ -z $free_value ]
        then
                echo "Parameter inputted not found. FAIL"
                exit 1
        fi

        countnr=$(echo " $free_value * 1000 " | bc -l | awk -F'.' '{print $1}')
        if [ $countnr -eq 0]
        then
                echo "Variable calculus failed for countnr. FAIL"
                exit 1
        fi

        i=1
        while [ $i -le $countnr ]
        do
                adb shell dd if=/dev/urandom of=test1MB_$i count=1024 bs=1024
                i=$(($i + 1))
        done

        sleep 40
        #interface verification step
        #verify_interface=$(python fill_memory.py)
        #if [ "$verify_interface" == "PASS" ]
        #then
        #        echo "PASS"
        #else
        #        echo $verify_interface
        #fi
}

if [ "$#" -ne 3 ]
then
        echo "Usage: bash fill_memory.sh <test_type> <mountpoint> <clear-memory?-expected yes or no>. FAIL"
        exit 1
else
        if [ $1 -eq 1 ]
        then
                fill_memory $2
        fi
        if [ $1 -eq 2 ]
        then
                fill_memory_1MB $2
        fi

        #teardown and cleanup by choice
        #adb shell am force-stop com.android.settings
        if [ "$3" == "yes" ]
        then
                adb shell rm $2/test*
        fi
fi

echo "PASS"
