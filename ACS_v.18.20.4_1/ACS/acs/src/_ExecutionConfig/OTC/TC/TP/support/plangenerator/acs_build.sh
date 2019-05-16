#!/bin/bash
set -e
echo "********************calculate the package version********************"
rm -rf acs_pkg_version.txt
wget https://mcg-depot.intel.com/artifactory/acs_test_artifacts/OTC_Android_Auto_Test_Suite/pre-release/acs_pkg_version.txt --no-check-certificate
last_week=`cat acs_pkg_version.txt | grep 'last_week' | cut -d '=' -f 2`
interaction=`cat acs_pkg_version.txt | grep 'interaction' | cut -d '=' -f 2`

year=`date +%y`
((this_week=10#`date +%V`))
week_day=`date +%w`
if [ $this_week -gt $last_week ]; then
    echo "111111111111111111111"
    sed -i "/interaction/ s/=.*$/=1/" ./acs_pkg_version.txt
    sed -i "/last_week/ s/=.*$/=$this_week/" ./acs_pkg_version.txt
    interaction=1
else
    ((new_interaction=$interaction + 1))
    echo "++++++++++++++++++++++++++++++++++++++++++++++++"
    echo "new_interaction: $new_interaction"
    sed -i "/interaction/ s/=.*$/=$new_interaction/" ./acs_pkg_version.txt
    cat ./acs_pkg_version.txt
    interaction=$new_interaction
fi
curl -i -u cliu14x:AP4S4yyA4WdbszMSAh4QNBrv1QK -T acs_pkg_version.txt "https://mcg-depot.intel.com/artifactory/acs_test_artifacts/OTC_Android_Auto_Test_Suite/pre-release/" -k

year=`date +%y`
((week=10#`date +%V`))
echo "the package version is $year.$this_week.${week_day}_$interaction"
echo
echo "********************create work_dir********************"
time_stamp=`date "+%Y-%m-%d_%H-%M-%S_%N"`
work_dir="/tmp/acs/$time_stamp"
mkdir -p $work_dir
echo "the work_dir is $work_dir"
echo
echo "********************download acs_core.zip and campaigns.zip********************"
cd $work_dir
rm -rf *
wget --http-user='cliu14x' --http-password='bus@2017' https://mcg-depot.intel.com/artifactory/cactus-absp-sh/acs-engineering/$1/acs_core.zip -P . --no-check-certificate
wget --http-user='cliu14x' --http-password='bus@2017' https://mcg-depot.intel.com/artifactory/cactus-absp-sh/acs-engineering/$1/campaigns.zip -P . --no-check-certificate
echo
echo "********************unzip files********************"
rm -rf index*
for i in `ls .`;do
    echo $i;
    unzip $i -d . >/dev/null 2>&1 || echo "unzip failed"
done
echo
echo "********************copy files to ACS folder to make ACS package********************"
rm -rf ACS/
mkdir -p ACS/acs/
cp -r acs_fwk/src/ ACS/acs/
cp -r acs_test_scripts/ ACS/
mkdir -p ACS/acs_test_suites/
cp -r BOOT ACS/acs_test_suites/
cp -r _Catalogs ACS/acs_test_suites/
cp -r conf ACS/acs_test_suites/
cp -r _Configs ACS/acs_test_suites/
cp -r ExtraLibs ACS/acs_test_suites/
cp -r __init__.py ACS/acs_test_suites/
cp -r ST ACS/acs_test_suites/
#cp -r /tmp/acs/OTC /tmp/acs/ACS/acs_test_suites/
cp -r OTC/ ACS/acs/src/_ExecutionConfig/
cp -r SCRIPT_HELPERS ACS/acs_test_suites/
mv testlib ACS/
mkdir -p ACS/acs_test_suites/OTC
mv ACS/acs/src/_ExecutionConfig/OTC/utils ACS/acs_test_suites/OTC/
cp OTC/TC/TP/support/plangenerator/CatalogLauncher.py ACS/acs/src/
cp -r ACS/acs/src/aapt ACS/
chmod -R 777 ACS/
echo
echo "********************create ACS package********************"
zip -r ACS_v.$year.$this_week.${week_day}_$interaction.zip ./ACS/ >/dev/null 2>&1 || echo "zip failed"
echo "ACS_v.$year.$this_week.${week_day}_$interaction.zip zip completed"
echo "the package name is $work_dir/ACS_v.$year.$this_week.${week_day}_$interaction.zip"
echo 
echo "********************upload ACS package to Artifactory********************"
curl -i -u cliu14x:AP4S4yyA4WdbszMSAh4QNBrv1QK -T ACS_v.$year.$this_week.${week_day}_$interaction.zip "https://mcg-depot.intel.com/artifactory/acs_test_artifacts/OTC_Android_Auto_Test_Suite/pre-release/" -k
echo :
echo "$work_dir":"ACS_v.$year.$this_week.${week_day}_$interaction.zip"
