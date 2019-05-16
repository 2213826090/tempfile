while read line
do
    grep_list=$grep_list"|"$line
done < $PWD"/words.txt"
grep_list=${grep_list:1}

grep --color=always -EIRin "$grep_list" $1
grep --color=always -EIRin "[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Za-z]{2,6}\b" $1
