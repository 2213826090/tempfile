#! /bin/bash

if [ $# -ne 1 ]; then
        echo "Usage: $0 <Merge pretest, formaltest plan and posttestplan>"
        echo "Usage: ./this script + DIR of Campaign"
        exit 1
fi

Campaign_dir=$1
posttestplan=Post_Test.xml

function merge()
{
	out=$1
	out=${out%xml}
	out=${out}merge.xml
	line=$(grep -n '<TestCases>' $1 | awk -F : '{print $1}')
	tailline=$(grep -n '<TestCases>' $3 | awk -F : '{print $1}')
	head -$line $1 > $out
	grep '<TestCase Id=' $2 >> $out
	grep '<TestCase Id=' $1 >> $out
	let "tailline=$tailline+1"
	tail -n +$tailline $3 >> $out
}

cd $Campaign_dir
for pretestplan in $(ls *pretest.xml)
do
	domain=${pretestplan%_pretest.xml}
	for formalplan in $(ls $domain.* | grep -v 'pretest' | grep -v 'merge')
	do
                merge $formalplan $pretestplan $posttestplan
	done
done
