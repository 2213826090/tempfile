#/bin/bash

if [ -z "$1" ] ; then
    echo "Usage: ./check_install_unistall .apkfile packagename"
    exit 0
fi

if [ -z "$2" ] ; then
    echo "Usage: ./check_install_unistall .apkfile packagename"
    exit 0
fi


function remove_trail {
    WORD=$1
    WSIZE=`echo "${#WORD}-1" | bc`
    WORD=`echo $WORD | cut -c 1-$WSIZE`
    echo $WORD
}


INSTALL=`adb install -r "$1" 2> /dev/null | grep Success`

INSTALL=$(remove_trail $INSTALL)

if [ "${INSTALL}" == "Success" ] ; then
    echo "Pass" > /dev/null
else
    echo "install Fail"
    exit 0
fi



UNINSTALL=`adb uninstall $2 | grep Success`

UNINSTALL=$(remove_trail $UNINSTALL)


if [ "${UNINSTALL}" == "Success" ] ; then
    echo "Pass"
else
    echo "uninstall Fail"
    exit 0
fi






