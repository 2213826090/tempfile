#!/bin/bash
#$1:username  $2:passwd  $3:build NO.
last_week=2
interaction=1
current_dir=`pwd`

((this_week=10#`date +%V` + 1))
week_day=`date +%w`
if [ $this_week -gt $last_week ]; then
    sed -i "4 s/=.*$/=1/" $0
    sed -i "3 s/=.*$/=$this_week/" $0
    interaction=1
else
    ((new_interaction=$interaction + 1))
    sed -i "4 s/=.*$/=$new_interaction/" $0
    interaction=$new_interaction
fi

year=`date +%y`
((week=10#`date +%V` + 1))
mkdir -p /tmp/acs/
rm -rf /tmp/acs/*

wget --http-user=$1 --http-password=$2 https://mcg-depot.intel.com/artifactory/cactus-absp-tl/acs-engineering/$3/acs_core.zip -P /tmp/acs/ --no-check-certificate
wget --http-user=$1 --http-password=$2 https://mcg-depot.intel.com/artifactory/cactus-absp-tl/acs-engineering/$3/campaigns.zip -P /tmp/acs/ --no-check-certificate
rm -rf /tmp/acs/index*
for i in `ls /tmp/acs/`;do
echo $i;
unzip /tmp/acs/$i -d /tmp/acs/
done

rm -rf /tmp/acs/ACS/
mkdir -p /tmp/acs/ACS/acs/
cp -r /tmp/acs/acs_fwk/src/ /tmp/acs/ACS/acs/
cp -r /tmp/acs/acs_test_scripts/ /tmp/acs/ACS/
cp -r /tmp/acs/testlib/ /tmp/acs/ACS/
mkdir -p /tmp/acs/ACS/acs_test_suites/
cp -r /tmp/acs/BOOT /tmp/acs/ACS/acs_test_suites/
cp -r /tmp/acs/_Catalogs /tmp/acs/ACS/acs_test_suites/
cp -r /tmp/acs/conf /tmp/acs/ACS/acs_test_suites/
cp -r /tmp/acs/_Configs /tmp/acs/ACS/acs_test_suites/
cp -r /tmp/acs/ExtraLibs /tmp/acs/ACS/acs_test_suites/
cp -r /tmp/acs/__init__.py /tmp/acs/ACS/acs_test_suites/
cp -r /tmp/acs/OTC/ /tmp/acs/ACS/acs/src/_ExecutionConfig/
cp -r /tmp/acs/SCRIPT_HELPERS /tmp/acs/ACS/acs_test_suites/
mkdir -p /tmp/acs/ACS/acs_test_suites/OTC
mv /tmp/acs/ACS/acs/src/_ExecutionConfig/OTC/utils /tmp/acs/ACS/acs_test_suites/OTC/
cd /tmp/acs/
cp -r /tmp/acs/ACS/acs/src/appt /tmp/acs/ACS/
chmod -R 777 ACS
zip -r ACS_v.$year.$this_week.${week_day}_$interaction.zip ./ACS/
cp ACS_v.$year.$this_week.${week_day}_$interaction.zip ${current_dir}
