-- BXT M  req.requirement_id, req.e_feature_id_broxton_p_ivi_m will lead multiline of same TC, just create the first one
select distinct test.test_id, test.domain, test.name test_name, test.tag ,test.subject,test.priority test_priority, test.automation_fw,
req.requirement_id, req.e_feature_id_broxton_p_ivi_m, jira.status jira_status, jira.priority jira_priority
from hpalm_requirements req
join hpalm_requirement_coverages cover on req.requirement_id=cover.requirement_id
join hpalm_tests test on cover.test_id=test.test_id
join jira_issues jira on req.e_feature_id_broxton_p_ivi_m=jira.issue_key 
WHERE         jira.android_version != ''
              and jira.status != 'Rejected'
              and test.status='Committed'
              and test.execution_type = 'Auto'
              and test.tc_type not in ('Reliability','Iterative','Long Lasting','Low Resource') 
              and (test.tag not like '%Depend_%' and test.tag not like '%Multi-device%')
              and jira.priority in ('P1-Stopper')
              and test.automation_fw not in ('Davinci','CTS','Other') 
              
              and (test.applicable_android_release = '' or test.applicable_android_release like '%M%' collate Chinese_PRC_CS_AI)
              and (test.soc_dependency like '%Common%' or test.soc_dependency like '%Broxton%')
              and test.not_applicable_for_platforms not like '%Broxton%'
              and test.subject like '%Subject\SSG\System Functional Tests\Graphics_System%'