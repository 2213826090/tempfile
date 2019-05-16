This app is used by OTC QA teams in order to generate TCs and Campaigns based
on other existing TCs.

Requierments:
- java 1.7 oracle version
- ACS/acs_test_suites/OTC/TC and ACS/acs_test_suites/OTC/CAMPAIGN folders copied to ACS/acs/src/_ExecutionConfig

TCInterface run :

1.  From ACS/acs_test_suites/OTC/utils/TCInterface run
    setup.sh

2.  From ACS/acs/src/_ExecutionConfig run
    run.sh

3.  Select TCs from the right list and click the add button to insert them into the left list.
    Click idividual TC from the left to see the parameters that can be edited, then click the save button to make changes persistent.
    Reorder the TC's using drag and drop.
    Click the generate button when finished configurating the TC paramater and order.

4.  From ACS/acs/src folder run
    python ACS.py -c Campaign.xml -d ECS-Android-LLP
