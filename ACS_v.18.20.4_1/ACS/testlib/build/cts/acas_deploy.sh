#! /bin/bash
PREREQ_INSTALL=false
ACAS_RUNNER=false
CTS_X86=false
CTS_ARM=false
GTS=false
CTS_MEDIA=false
HELP=false

#PASSWD=""

CTS_VERSION=cts-6.0_r5
CTS_MEDIA_VERSION=cts-media-1.2
GTS_VERSION=gts-3.0_r3


for i in "$@"; do
    case $i in
        -h|--help)
        HELP=true
        shift
        ;;
        -i|--install)
        PREREQ_INSTALL=true
        shift
        ;;
        -r|--acas-runner)
        ACAS_RUNNER=true
        shift
        ;;
        -x|--cts-x86)
        CTS_X86=true
        shift
        ;;
        -a|--cts-arm)
        CTS_ARM=true
        shift
        ;;
        -g|--gts)
        GTS=true
        shift
        ;;
        -m|--cts-media)
        CTS_MEDIA=true
        shift
        ;;
        -p=*|--password=*)
        PASSWD="${i#*=}"
        shift
        ;;
        *)
        usage "Unknown option"
        ;;
    esac
done

function usage {
    echo $1
    echo "USAGE: $0 -p=somepass [options]"
    echo "  Example: $0 -p=somepass -i -r -x -a -g -m"
    echo "  -p, --password      Paswword"
    echo "  -i, --install       Install prerequisites"
    echo "  -r, --acas-runner    Instal ACAS runner"
    echo "  -x, --cts-x86       Copy latest CTS x86 suite"
    echo "  -a, --cts-arm       Copy latest CTS arm suite"
    echo "  -g, --gts           Copy latest GTS suite"
    echo "  -m, --cts-media     Copy latest CTS media"
    exit 0
}

if [ ! $PASSWD ]; then
    usage "Please provide password!!!"
fi

if [ $HELP = true ]; then
    usage
fi

function apt_get_install {
 if [ $(dpkg-query -W -f='${Status}' $1 2>/dev/null | grep -c "ok installed") -eq 0 ];
  then
   echo $PASSWD | sudo -S apt-get install -y $1;
  else
   echo "$1 already installed"
 fi
}

echo ""

if [ $PREREQ_INSTALL = true ]; then
    echo "<-- Installing dependecies..."
    echo "Adding i386 dependecies..."
    echo $PASSWD | sudo -S dpkg --add-architecture i386
    echo "Running apt-get update if needed...."
    echo $PASSWD | sudo -S apt-get update
    echo "Installing adb tools...."
    apt_get_install android-tools-adb
    echo "Installing linux dependent libraries...."
    apt_get_install lib32z1
    apt_get_install libc6-i386
    apt_get_install lib32stdc++6
    apt_get_install fping
    apt_get_install gdebi
    apt_get_install p7zip-full
    echo "Install python-pip if needed...."
    apt_get_install python-dev
    apt_get_install python-pip
    echo "Install python libs...."
    echo $PASSWD | sudo -S pip install --proxy http://proxy-ir.intel.com:911 python-dateutil
    echo "Install screen if needed...."
    apt_get_install screen
    echo "Install sshpass if needed...."
    apt_get_install sshpass
    echo "--> Dependencies installed"
    echo ""

    #echo "<-- Add e-mail sender host to known hosts..."
    #ssh-keygen -f "~/.ssh/known_hosts" -R 10.237.112.149
    #ssh-keyscan -H 10.237.112.149 >> ~/.ssh/known_hosts
    #echo "--> Done"
    #echo ""

    echo "<-- Getting Android dependecies..."
    wget -N http://awesome.rb.intel.com/DATA/CTS/aapt
    chmod +x aapt
    echo $PASSWD | sudo -S mv aapt /usr/bin/aapt
    wget -N http://awesome.rb.intel.com/DATA/CTS/adb
    chmod +x adb
    echo $PASSWD | sudo -S mv adb /usr/bin/adb
    wget -N http://awesome.rb.intel.com/DATA/CTS/fastboot
    chmod +x fastboot
    echo $PASSWD | sudo -S mv fastboot /usr/bin/fastboot
    echo "--> Done"
    echo ""

    echo "<-- Installing PhoneFlashTool..."
    if [ $(dpkg-query -W -f='${Status}' phoneflashtool 2>/dev/null | grep -c "ok installed") -eq 0 ];
      then
       wget -N -k http://awesome.rb.intel.com/DATA/CTS/ACAS/platformflashtool_5.5.1.0_internal_linux_x86_64.deb 2> /dev/null
       echo $PASSWD | sudo -S dpkg -i platformflashtool_5.5.1.0_internal_linux_x86_64.deb
      else
       echo "phoneflashtool already installed"
    fi

    echo $PASSWD | sudo -S ln -sf /usr/bin/adb /usr/bin/adb.pft
    echo $PASSWD | sudo -S ln -sf /usr/bin/fastboot /usr/bin/fastboot.pft
    echo $PASSWD | sudo -S ln -sf platformflashtool /usr/bin/phoneflashtool
    echo "--> Done"
    echo ""

    # For flasing using update + flash userdata over user signed to enable USB debugging
    echo "<-- Getting resources folder..."
    mkdir -p ~/acas
    cd ~/acas
    wget -N -k http://awesome.rb.intel.com/DATA/CTS/ACAS/resources.zip 2> /dev/null
    unzip -o resources.zip > /dev/null
    rm resources.zip
    wget -N -k http://awesome.rb.intel.com/DATA/CTS/ACAS/DownloadTool.zip 2> /dev/null
    unzip -o DownloadTool.zip > /dev/null
    rm DownloadTool.zip
    echo "--> Done"
    echo ""
    cd
    echo "<-- Getting adb keys..."
    wget -N http://awesome.rb.intel.com/DATA/CTS/adbkey.key 2> /dev/null
    chmod +x adbkey.key
    wget -N http://awesome.rb.intel.com/DATA/CTS/adbkey.pub 2> /dev/null
    chmod +x adbkey.pub
    wget -N http://awesome.rb.intel.com/DATA/CTS/ACAS/make_ext4fs 2> /dev/null
    chmod +x make_ext4fs
    if [ ! -d "$HOME/.android" ]; then
        cd $HOME
        mkdir .android
    fi
    mv adbkey.key $HOME/.android/adbkey
    mv adbkey.pub $HOME/.android/adbkey.pub
    adb kill-server
    if [ ! -d "$HOME/bin" ]; then
        cd $HOME
        mkdir bin
    fi
    mv make_ext4fs $HOME/bin/make_ext4fs
    echo "--> Done"
    echo ""

    echo "<-- Adding current user to dialout group - needed for HW relays control..."
    echo $PASSWD | sudo -S usermod -a -G dialout `whoami`
    echo "--> Done"
    echo ""

fi


if [ $ACAS_RUNNER = true ]; then
    echo "<-- Getting the ACAS runner..."
    mkdir -p ~/acas
    cd ~/acas/
    wget -N http://awesome.rb.intel.com/DATA/CTS/ACAS/run_suite 2> /dev/null
    chmod +x run_suite
    wget -N http://awesome.rb.intel.com/DATA/CTS/ACAS/settings_cts_x86.py 2> /dev/null
    wget -N http://awesome.rb.intel.com/DATA/CTS/ACAS/settings_cts_arm.py 2> /dev/null
    wget -N http://awesome.rb.intel.com/DATA/CTS/ACAS/settings_gts.py 2> /dev/null
    wget -N http://awesome.rb.intel.com/DATA/CTS/ACAS/settings_aft.py 2> /dev/null
    cd
    echo "--> ACAS runner installed! It can be found at the following path: ~/acas/"
    echo ""
fi

if [ $CTS_ARM = true ]; then
    echo "<-- Getting CTS arm suite at $HOME/acas/compliance/cts/$CTS_VERSION/arm..."
    mkdir -p ~/acas/compliance/cts/$CTS_VERSION/arm
    rm -rf ~/acas/compliance/cts/$CTS_VERSION/arm/*
    cd ~/acas/compliance/cts/$CTS_VERSION/arm/
    wget -N -k http://awesome.rb.intel.com/DATA/CTS/ACAS/android-$CTS_VERSION-linux_x86-arm.zip 2> /dev/null
    unzip -o android-$CTS_VERSION-linux_x86-arm.zip > /dev/null
    rm android-$CTS_VERSION-linux_x86-arm.zip
    chmod +x ~/acas/compliance
    cd -
    echo "--> CTS arm suite deployed! It can be found at the following path: ~/acas/compliance/CTS/$CTS_VERSION/arm"
    echo ""
fi

if [ $CTS_X86 = true ]; then
    echo "<-- Getting CTS arm suite at $HOME/acas/compliance/cts/$CTS_VERSION/x86..."
    mkdir -p ~/acas/compliance/cts/$CTS_VERSION/x86
    rm -rf ~/acas/compliance/cts/$CTS_VERSION/x86/*
    cd ~/acas/compliance/cts/$CTS_VERSION/x86/
    wget -N -k http://awesome.rb.intel.com/DATA/CTS/ACAS/android-$CTS_VERSION-linux_x86-x86.zip 2> /dev/null
    unzip -o android-$CTS_VERSION-linux_x86-x86.zip > /dev/null
    rm android-$CTS_VERSION-linux_x86-x86.zip
    chmod +x ~/acas/compliance
    cd -
    echo "--> CTS X86 suite deployed! It can be found at the following path: ~/acas/compliance/CTS/$CTS_VERSION/x86"
    echo ""
fi

if [ $GTS = true ]; then
    echo "<-- Getting GTS suite at $HOME/compliance/GTS/$GTS_VERSION..."
    mkdir -p ~/acas/compliance/gts/$GTS_VERSION/
    rm -rf ~/acas/compliance/gts/$GTS_VERSION/*
    cd ~/acas/compliance/gts/$GTS_VERSION/
    wget -N -k http://awesome.rb.intel.com/DATA/CTS/ACAS/android-$GTS_VERSION.zip 2> /dev/null
    unzip -o android-$GTS_VERSION.zip > /dev/null
    rm android-$GTS_VERSION.zip
    chmod +x ~/acas/compliance
    cd -
    echo "--> GTS suite deployed! It can be found at the following path: ~/acas/compliance/CTS/$GTS_VERSION/"
    echo ""
fi

if [ $CTS_MEDIA = true ]; then
    echo "<-- Getting CTS media files at $HOME/acas/compliance/CTS/..."
    mkdir -p ~/acas/compliance/ctsMedia/
    rm -rf ~/acas/compliance/ctsMedia/android-$CTS_MEDIA_VERSION
    cd ~/acas/compliance/ctsMedia/
    wget -N http://awesome.rb.intel.com/DATA/CTS/ACAS/android-$CTS_MEDIA_VERSION.zip 2> /dev/null
    unzip -o android-$CTS_MEDIA_VERSION.zip > /dev/null
    rm android-$CTS_MEDIA_VERSION.zip
    chmod +x ~/acas/compliance
    cd -
    echo "--> CTS media deployed! It can be found at the following path: ~/acas/compliance/CTS/android-$CTS_MEDIA_VERSION"
    echo ""
fi

echo "Done!"
