--Bxtp M
select distinct test.domain, test.priority ,test.test_id, test.name, test.automation_fw, test.subject
from syncenter_new.db_owner.hpalm_requirements req
join syncenter_new.db_owner.hpalm_requirement_coverages cover on req.requirement_id=cover.requirement_id
join syncenter_new.db_owner.hpalm_tests test on cover.test_id=test.test_id
join syncenter_new.db_owner.jira_issues jira on req.e_feature_id_broxton_p_ivi_m=jira.issue_key 
WHERE  test.status='Committed'
              and jira.android_version != ''
              and (test.applicable_android_release = '' or applicable_android_release like '%M%' collate Chinese_PRC_CS_AI)
              and test.not_applicable_for_platforms not like '%Broxton%'
              and (test.soc_dependency like '%Common%' or test.soc_dependency like '%Broxton%')
              and test.subject like '%Subject\SSG\System Functional Tests%'
              and jira.priority in ('P1-Stopper')
              and jira.status != 'Rejected'

              and test.execution_type = 'Auto'
              and test.tc_type not in ('Reliability','Iterative','Long Lasting','Low Resource') 