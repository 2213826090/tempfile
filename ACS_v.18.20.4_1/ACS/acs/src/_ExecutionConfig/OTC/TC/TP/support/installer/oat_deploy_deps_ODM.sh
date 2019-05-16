#!/bin/bash

#set -x

INSTALLER="dpkg"
PIP_INSTALLER="pip"
PKG_MGR="apt-get install"

function show_usage(){
    echo "Usage: $0 command [parameters]"
    echo "commands:"
    echo "      sudo $0 --install      - Install OAT-l tools on host, '--install' can be omitted." 
    echo "      sudo $0 --remove/-r     - Clean up OAT-l tools installed on host."
    exit 0
}

OS_Ver=`cat /etc/issue | awk '{print $2}' | awk -F '.' '{print $1$2}'`
if uname -a | grep i386
then
        ARCH=i386
else
        ARCH=amd64
fi

function uninstall_pip_pkg(){
    ${PKG_MGR} python-pip
    if [ $? -eq 0 ]; then
        QUE_PKG=`pip freeze | grep ^$1 > /dev/null 2>&1`
        if [ $? -eq 0 ]; then
            echo "Package $1 is installed, removing it..."
            pip uninstall -y $1
            if [ $? -ne 0 ]; then
                echo "WARNING: failed to uninstall $1"
                echo "         please uninstall it manually by command: "
                echo "             sudo pip uninstall $1 "
                exit 255
            fi
        fi
    else
        echo "WARNING: OAT-l dependency is not completed because of pip installed failed."
        echo "Please make sure pip installed properly by command: "
        echo "          sudo apt-get install python-pip"
        exit 255 
    fi
}

function install_pip_pkg(){
    echo "install $1, please wait..."
    pip install $1
    if [ $? -ne 0 ]; then
        echo "WARNING: failed to install $1"
        echo "         please install it manually by command: "
        echo "             sudo -E pip install $1 "
        echo "$3"
        echo "Installed package: $2"
        echo "For cleaning up all installed packages:"
        echo "    ./oat_deploy_deps.sh -r"
        exit 255
    fi

    QUE_PKG=`pip freeze | grep $1 > /dev/null 2>&1`
    if [ $? -eq 0 ]; then
        echo "$1 is installed successfully"
    else
        echo "WARNING: install $1 failed, please install $1 manually"
        echo "Installed package: $2"
        echo "For cleaning up all installed packages:"
        echo "    ./oat_deploy_deps.sh -r"
        exit 255
    fi
}

function upgrade_pip_pkg(){
    echo "install --upgrade $1, please wait..."
    pip install --upgrade $1
    if [ $? -ne 0 ]; then
        echo "WARNING: failed to install --upgrade $1"
        echo "         please install it manually by command: "
        echo "             sudo -E pip install --upgrade $1 "
        echo "$3"
        echo "Installed package: $2"
        echo "For cleaning up all installed packages:"
        echo "    ./oat_deploy_deps.sh -r"
        exit 255
    fi

    UP_QUE_PKG=`pip freeze | grep $1 > /dev/null 2>&1`
    if [ $? -eq 0 ]; then
        echo "$1 is installed successfully"
    else
        echo "WARNING: install $1 failed, please install $1 manually"
        echo "Installed package: $2"
        echo "For cleaning up all installed packages:"
        echo "    ./oat_deploy_deps.sh -r"
        exit 255
    fi
}


function uninstall_deb_pkg(){
    ${INSTALLER} -l | grep $1 > /dev/null 2>&1
    if [ $? -eq 0 ]; then
        echo "you have installed $1 on your machine, now uninstall it"
        ${INSTALLER} -r $1 > /dev/null 2>&1
        if [ $? -ne 0 ];then
            echo "WARNING:fail to uninstall $1. please uninstall it manually using software center or following command:"
            echo "    sudo dpkg -r $1"
            exit 255
        fi
    fi
}

function install_internal_debian(){
    apt-get install $1
    if [ $? -ne 0 ];then
        echo "WARNING:fail to install oat-l. please install it manually following below steps:"
        echo "Installed package: $2"
        echo "For cleaning up all installed packages:"
        echo "    ./oat_deploy_deps.sh -r"
        exit 255
    fi
}

function install_testcases_deps(){
    echo "Installing oat-l dependence ..."
    install_internal_debian "beanstalkd"
    install_internal_debian "python-yaml"
}

function uninstall_old_pillow_pkg(){
    echo "Uninstall pillow dependence ..."
    uninstall_pip_pkg "pillow"
	uninstall_pip_pkg "pil-compat"
	sudo rm -rf /usr/lib/python2.7/dist-packages/PIL
	sudo rm -rf /usr/lib/python2.7/dist-packages/Pillow*
	sudo rm -rf /usr/bin/pil*
	sudo rm -rf /usr/lib/python2.7/dist-packages/pil_compat*
}

function install_pillow(){

    echo "Installing pillow dependence ..."
    install_internal_debian "libjpeg-dev"
    install_internal_debian "zlib1g-dev"
    install_internal_debian "libpng12-dev"

    echo "intall Pillow"
    pip install pil-compat --upgrade
    if [ $? -ne 0 ];then
        echo "WARNING:fail to intall Pillow. please install it manually following below steps:"
        echo "    sudo -E pip install pil-compat --upgrade"
        echo "    When you met the following problem:"
        echo "    >>> import PIL"
        echo "    >>> import Image"
        echo "    Traceback (most recent call last):"
        echo "    File "<stdin>", line 1, in <module>"
        echo "    ImportError: No module named Image"
        echo "    You can use "sudo pip install pil-compat" to solve it."
        echo "Installed package: $1"
        echo "For cleaning up all installed packages:"
        echo "    ./oat_deploy_deps.sh -r"
        cd ..
        exit 255
    fi

    echo "intall Pillow library to /usr/lib/python2.7/dist-packages by force"
    sudo rm -rf /usr/lib/python2.7/dist-packages/PIL
    sudo rm -rf /usr/lib/python2.7/dist-packages/Pillow*

    sudo mv -f /usr/lib/python2.7/site-packages/PIL /usr/lib/python2.7/dist-packages
    sudo mv -f /usr/lib/python2.7/site-packages/Pillow* /usr/lib/python2.7/dist-packages

}


function install_easy_install_pkg(){
    easy_install $1
    if [ $? -ne 0 ];then
        echo "WARNING:fail to install oat-l. please install it manually following below steps:"
        echo "sudo easy_install $1"
        echo "For cleaning up all installed packages:"
        echo "    ./oat_deploy_deps.sh -r"
        exit 255
    fi
}

function install_QRcodepackage(){
    if [[ "z"$OS_Ver == "z1204" ]];then
        install_easy_install_pkg "pypng"
        install_easy_install_pkg "pyqrcode"
        wget https://launchpad.net/qr-tools/trunk/1.2/+download/python-qrtools_1.2_all.deb
        gdebi python-qrtools_1.2_all.deb
        if [ $? -ne 0 ];then
            echo "WARNING:fail to install QRcodepackage. please install it manually following below steps:"
            echo "    Downloading the deb ball from \"https://launchpad.net/qr-tools/trunk/1.2/+download/python-qrtools_1.2_all.deb"
            echo "    sudo gdebi python-qrtools_1.2_all.deb"
            echo "Installed package: $1"
            echo "For cleaning up all installed packages:"
            echo "    ./oat_deploy_deps.sh -r"
            exit 255
        fi
    else
        install_easy_install_pkg "pypng"
        install_easy_install_pkg "pyqrcode"
        install_internal_debian "python-qrtools"
    fi    
}

function update_repo(){
    echo "Update repo ..."
    apt-get update
}

function disk_space_check(){
    echo "Disk usage:"
    df -H | grep -vE '^Filesystem|tmpfs|cdrom' | awk '{ print $5 " " $1 }' | while read output;
    do
        echo $output
        usep=$(echo $output | awk '{ print $1}' | cut -d'%' -f1  )
        partition=$(echo $output | awk '{ print $2 }' )
        if [ $usep -ge 90 ]; then
            echo "Running out of space \"$partition ($usep%)\" on $(hostname) as on $(date)" |
            echo "Please clean up the log folder."
            exit 255
        fi
    done
}

function confirm () {
    # call with a prompt string or use a default
    read -r -p "$1" response
    case $response in
        [yY][eE][sS]|[yY]) 
            true
            ;;
        *)
            false
            ;;
    esac
}

function clean_temp(){
    rm -rf "/usr/lib/python2.7/dist-packages/uiautomator"
}

function install_host(){
    echo "Deploying host"
    echo "Upinstalling requests"
    uninstall_pip_pkg "requests"
#    echo "Uninstalling nose"
#    uninstall_pip_pkg "nose"
    echo "Uninstalling old pillow pkg"
    uninstall_old_pillow_pkg
    echo "Uninstalling uiautomator"
    uninstall_deb_pkg "uiautomator"

    echo "Cleaning temporaries"
    clean_temp

    if [ -f /usr/local/man ];then
       rm /usr/local/man
    fi
    
    disk_space_check
    echo "Installing requests"
    install_pip_pkg "requests" "None" ""
    echo "Installing paramiko"
    install_pip_pkg "paramiko" "None" ""
    echo "Installing fileDownload"
    install_pip_pkg "fileDownloader.py" "None" ""
    echo "upgrade pyserial"
    upgrade_pip_pkg "pyserial" "None" ""
    echo "Installing uiautomator"
    install_internal_debian "uiautomator" "'requests'"
#    echo "Installing nose"
#    install_pip_pkg "nose" "'requests', 'uiautomator'" "If the symlink '/usr/local/man' is exists and broken, please remove it, and re-execute this script"
    echo "Installing oat-l dependence"
    install_testcases_deps "'beanstalkd', 'python-yaml'"
    echo "Installing pillow"
    install_pillow "'libjpeg-dev', 'zlib1g-dev', 'libpng12-dev', 'pillow'"
    echo "Installing QRcodepackage"
    install_QRcodepackage "'pypng','pyqrcode','python-qrtools'"
    echo "Installing exif tools"
    install_internal_debian "libimage-exiftool-perl"
    echo "Installing python-opencv"
    install_internal_debian "python-opencv"
    echo "Installing python-numpy"
    install_internal_debian "python-numpy"
    echo "Installing python-scipy"
    install_internal_debian "python-scipy"
    if [[uname -a | grep i386]];then
        echo "32bit system"
    else
        echo "64bit system need install lib32"
        install_internal_debian "lib32z1"
        install_internal_debian "lib32ncurses5"
        install_internal_debian "lib32bz2-1.0"
        install_internal_debian "libstdc++6:i386"
    fi

    disk_space_check
}

if [ x$1 != x ]; then
    if [ $1 = "-r" -o $1 = "--remove" ]; then
        echo "----------------------------------------------" 
#        uninstall_pip_pkg "nose"
        uninstall_pip_pkg "uiautomator"
        clean_temp
        echo "Cleaning existed OAT-l tools/lib on host successfully"
        echo "----------------------------------------------"
        exit 0
    elif [ $1 = "--install" ]; then
        update_repo
        install_host
    else
        show_usage
    fi
else
    update_repo
    install_host
fi
