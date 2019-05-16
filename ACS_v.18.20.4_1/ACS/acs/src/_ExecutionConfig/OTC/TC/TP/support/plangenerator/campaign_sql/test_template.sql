select distinct test.test_id, test.domain, test.name test_name, test.subject, test.tag ,test.priority test_priority, test.automation_fw, jira.priority jira_priority
from hpalm_requirements req
join hpalm_requirement_coverages cover on req.requirement_id=cover.requirement_id
join hpalm_tests test on cover.test_id=test.test_id
join jira_issues jira on req.{PLATFORM}=jira.issue_key
WHERE         jira.android_version != ''
              and jira.status != 'Rejected'
              and test.status='Committed'
              and test.execution_type = 'Auto'
              and test.domain = '{DOMAIN}'
              and test.tc_type not in ('Reliability','Iterative','Long Lasting','Low Resource')
              and (test.tag not like '%Depend_%' and test.tag not like '%Multi-device%')
              and jira.priority in ('{REQ_PRIORITY}')
              and test.automation_fw not in ('Davinci','CTS','Other')

              and (test.applicable_android_release = '' or test.applicable_android_release like '%{ANDROID_VERSION}%')
              and (test.soc_dependency like '%Common%' or test.soc_dependency like '%{SOC_DEPENDENCY}%')
              and test.not_applicable_for_platforms not like '%{APPLICABLE_FOR_PLATFORMS}%'
              and test.subject like '%Subject\SSG\System Functional Tests%'
              and jira.label like '%{LABLES}%'
              and test.tag like '%{TAG}%'
              order by test.subject