watcher_list = {
    'mock_watcher':{
        'groups': ['mock'],
        'selector': {'textContains':'Process system isn\'t responding.'},
        'action': 'click',
        'action_view': {'text': 'OK'},
        },
    'process_system': {
        'groups': ['wifi', 'cts'],
        'selector': {'textContains':'Process system isn\'t responding.'},
        'action': 'click',
        'action_view': {'text': 'OK'},
        },
    'quick_settings': {
        'groups': ['wifi', 'cts'],
        'selector': {'resourceId': 'com.android.systemui:id/quick_settings_container'},
        'action': 'press',
        'action_view': 'back',
        },
    'app_has_stopped': {
        'groups': ['wifi', 'cts'],
        'selector': {'textContains':'has stopped.'},
        'action': 'click',
        'action_view': {'text': 'OK'},
        },
    'M_SD_Card': {
        'groups': ['wifi', 'cts'],
        'selector': {'textContains':'This SD Card'},
        'action': 'click',
        'action_view': {'text': 'Ok'},
        },
    'startup_wizard_SIM': {
        'groups': ['wifi', 'cts'],
        'selector': {'textContains':'Select SIM card'},
        'action': 'press',
        'action_view': 'back',
        },
    'p2p_invitation': {
        'groups': ['wifi'],
        'selector': {'text':'Invitation to connect'},
        'action': 'click',
        'action_view': {'text': 'Decline'},
        }
}


handler_list = {
    'mock_watcher':{
        'groups': ['mock'],
        'selector': {'textContains':'Process system isn\'t responding.'},
        'action': 'click',
        'action_view': {'text': 'OK'},
        },
    'process_system': {
        'groups': ['wifi', 'cts'],
        'selector': {'resourceId': 'android:id/button2', 'text': 'Wait'},
        'action': 'click',
        'action_view': {'text': 'OK'},
        },
    'quick_settings': {
        'groups': ['wifi', 'cts', 'aft'],
        'selector': {'resourceId': 'com.android.systemui:id/quick_settings_container'},
        'action': 'press',
        'action_view': 'back',
        },
    'app_has_stopped': {
        'groups': ['wifi', 'cts', 'aft'],
        'selector': {'textContains':'has stopped.'},
        'action': 'click',
        'action_view': {'text': 'OK'},
        },
    'M_SD_Card': {
        'groups': ['wifi', 'cts', 'aft'],
        'selector': {'textContains':'This SD Card'},
        'action': 'click',
        'action_view': {'text': 'Ok'},
        },
    'startup_wizard_SIM': {
        'groups': ['wifi', 'cts', 'aft'],
        'selector': {'textContains':'Select SIM card'},
        'action': 'press',
        'action_view': 'back',
        },
    'p2p_invitation': {
        'groups': ['wifi'],
        'selector': {'text':'Invitation to connect'},
        'action': 'click',
        'action_view': {'text': 'Decline'},
        },
    'google_check': {
        'groups': ['wifi', 'cts'],
        'selector': {'textContains':'Allow Google to regularly check device activity'},
        'action': 'click',
        'action_view': {'resourceId':'com.android.vending:id/positive_button'},
        #'action_view': {'text':'ACCEPT'},
        },
    'select_sim_data': {
        'groups': ['wifi', 'cts'],
        'selector': {'textContains':'Select a SIM for data'},
        'action': 'click',
        'action_view': {'resourceId':'com.android.settings:id/title', 'index': 0},
        },
    'modem_unrecoverable': {
        'groups': ['wifi'],
        'selector': {'textContains':'modem unrecoverable'},
        'action': 'click',
        'action_view': {'text': 'OK'},
        },
    'goole_app_stopped': {
        'groups': ['wifi'],
        'selector': {'textContains':'Google App has stopped'},
        'action': 'click',
        'action_view': {'text': 'Close'},
        }
}
