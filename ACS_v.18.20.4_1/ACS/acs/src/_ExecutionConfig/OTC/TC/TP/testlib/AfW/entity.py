# -*- coding: utf-8 -*-


class MDM_Entity():
    """
    entity for intel afw mdm apps
    """
    # main page
    create_and_delete_profile = "com.intel.afw.mdm:id/btnCreOrDelPro"
    device_provision = "com.intel.afw.mdm:id/btnDevProvision"
    app_provision = "com.intel.afw.mdm:id/btnAppProvision"
    intents_sharing = "com.intel.afw.mdm:id/btnIntentAndDataShr"
    app_config_policy = "com.intel.afw.mdm:id/btnAppConfAndPolicy"
    device_management_policy = "com.intel.afw.mdm:id/btnDevAndProfMgt"
    vpn_management = "com.intel.afw.mdm:id/btnVpnAndCert"
    third_party = "com.intel.afw.mdm:id/btnThirdParty"
    legacy_policy = "com.intel.afw.mdm:id/btnLegacyDP"
    # device provision
    set_profile_name = "com.intel.afw.mdm:id/btnSetProfileName"
    check_profile_app = "com.intel.afw.mdm:id/btnIsProfileOwnerApp"
    check_device_app = "com.intel.afw.mdm:id/btnIsDeviceOwnerApp"
    profile_input_box = "com.intel.afw.mdm:id/content_edit"
    create_user = "com.intel.afw.mdm:id/btnCreateUser"
    create_and_init_user = "com.intel.afw.mdm:id/btnCreateAndInitializeUser"
    switch_user = "com.intel.afw.mdm:id/btnSwitchUser"
    remove_user = "com.intel.afw.mdm:id/btnRemoveUser"
    clear_device_owner_app = "com.intel.afw.mdm:id/btnClearDeviceOwnerApp"
    user_name_input = "com.intel.afw.mdm:id/content_edit"
    # app provision
    hide_apps = "com.intel.afw.mdm:id/btnHideApplications"
    list_hidden_apps = "com.intel.afw.mdm:id/btnHasHiddenApplications"
    enable_system_apps = "com.intel.afw.mdm:id/btnEnableSystemApps"
    block_uninstall_apps = "com.intel.afw.mdm:id/btnBlockUninstallApps"
    blocked_uninstall_apps = "com.intel.afw.mdm:id/btnHasBlockedUninstallApps"
    select_all = "Select All"
    list_contents = "android:id/list"
    # intents data sharing
    toggle_caller_id = "com.intel.afw.mdm:id/btnSetCrossProfileCallerIdDisabled"
    add_x_intent_filter = "com.intel.afw.mdm:id/btnAddCrossProfileIntentFilter"
    clear_x_intent_filter = "com.intel.afw.mdm:id/btnClearCrossProfileIntentFilters"
    add_persistent_preferred = "com.intel.afw.mdm:id/btnAddPersistentPreferredActivity"
    clear_persistent_preferred = "com.intel.afw.mdm:id/btnClearPackagePersistentPreferredActivities"
    send_intent_to_handle = "com.intel.afw.mdm:id/btnSendIntent"
    send_intent_to_speech = "com.intel.afw.mdm:id/btnSendRecognizeSpeechIntent"
    add_x_widget = "com.intel.afw.mdm:id/btnAddCrossProfileWidgetProvider"
    remove_x_widget = "com.intel.afw.mdm:id/btnRemoveCrossProfileWidgetProvider"
    get_x_widget = "com.intel.afw.mdm:id/btnGetCrossProfileWidgetProviders"
    send_intent_to_audio_effect = "com.intel.afw.mdm:id/btnDisplayAudioEffectControlPanel"
    send_intent_to_account_sync = "com.intel.afw.mdm:id/btnAccountSyncSettings"
    widget_pages = "com.google.android.googlequicksearchbox:id/apps_customize_pane_content"
    # App config and policy
    set_restriction_provider = "com.intel.afw.mdm:id/btnSetRestrictionProvider"
    check_restriction_provider = "com.intel.afw.mdm:id/btnChkRestrictionProvider"
    install_restriction_schema = "Install apprestrictionschema"
    set_apprestriction_schema = "Set Restrictions Policy For apprestrictionschema"
    set_chrome_restriction = "Set Application Restriction"
    clear_chrome_restriction = "Clear Application Restriction"
    get_chrome_restriction = "com.intel.afw.mdm:id/btnGetApplicationRestriction"
    set_clear_auto_time = "com.intel.afw.mdm:id/btnSetAutoTimeRequired"
    set_auto_time = "Set Auto Time Required"
    clear_auto_time = "Clear Auto Time Required"
    # device/profile management policy
    master_volume_mute_state = "com.intel.afw.mdm:id/btnSetOrGetMasterVol"
    screen_capture_state = "com.intel.afw.mdm:id/btnSetOrGetScreenCap"
    set_user_restrictions = "com.intel.afw.mdm:id/btnSetUserRestrictions"
    disable_account_management = "com.intel.afw.mdm:id/btnDisableAccManagement"
    pre_populated_email = "com.intel.afw.mdm:id/btnAddPrePopulatedEmailAddress"
    disable_screen_capture = "Disable Screen Capture"
    enable_screen_capture = "Enable Screen Capture"
    account_type_input = "android.widget.EditText"
    set_lock_task = "com.intel.afw.mdm:id/btnSetLockTask"
    lock_task_mode = "com.intel.afw.mdm:id/btnSetLockTaskState"
    global_setting = "com.intel.afw.mdm:id/btnSetSettings"
    # vpn/certificate management
    install_ca_cert = "com.intel.afw.mdm:id/btnInstallCaCert"
    uninstall_ca_cert = "com.intel.afw.mdm:id/btnUninstallCaCert"
    check_ca_cert = "com.intel.afw.mdm:id/btnHasCaCertInstalled"
    get_installed_ca_cert = "com.intel.afw.mdm:id/btnGetInstalledCaCerts"
    uninstall_all_user_ca_cert = "com.intel.afw.mdm:id/btnUninstallAllUserCaCerts"
    set_global_proxy = "com.intel.afw.mdm:id/btnSetRecommendedGlobalProxy"
    clear_global_proxy = "com.intel.afw.mdm:id/btnClearRecommendedGlobalProxy"
    proxy_edit = "com.intel.afw.mdm:id/proxy_edit"
    proxy_port_edit = "com.intel.afw.mdm:id/port_edit"
    proxy_exclude_list = "com.intel.afw.mdm:id/exclList_edit"
    # 3rd party app
    set_input_method = "com.intel.afw.mdm:id/btnSetPermittedInputMethod"
    get_input_method = "com.intel.afw.mdm:id/btnGetPermittedInputMethod"
    set_accessibility_service = "com.intel.afw.mdm:id/btnSetPermittedAccessibilityServices"
    get_accessibility_service = "com.intel.afw.mdm:id/btnGetPermittedAccessibilityServices"
    # legacy device policy config
    set_camera_state = "com.intel.afw.mdm:id/btnSetCameraState"
    set_keyguard_disable_feature = "com.intel.afw.mdm:id/btnSetKeyguardFeatures"
    set_password_expiration_time = "com.intel.afw.mdm:id/btnSetPasswordExpiration"
    get_password_info = "com.intel.afw.mdm:id/btnGetPasswordInfo"
    lock_now = "com.intel.afw.mdm:id/btnLockNow"
    wipe_include_sd = "com.intel.afw.mdm:id/btnWipeDataIncludeSDCard"
    wipe_exclude_sd = "com.intel.afw.mdm:id/btnWipeDataExcludeSDCard"
    wipe_with_frp = "com.intel.afw.mdm:id/btnWipeDataWithFRP"

    # lock ui
    lock_clock_view = "com.android.systemui:id/clock_view"
    lock_screen_scroll_view = "com.android.systemui:id/scroll_view"
    lock_pin_pad = "com.android.systemui:id/key_enter"
    lock_pin_entry = "com.android.systemui:id/pinEntry"

    # app launcher
    app_launcher = "Apps"
    apps_page_indicator = "com.google.android.googlequicksearchbox:id/apps_customize_page_indicator"
    apps_pane_content = "com.google.android.googlequicksearchbox:id/apps_customize_pane_content"

    # recent page
    dismiss_task = "com.android.systemui:id/dismiss_task"

    # not provision
    setup_managed_profile = "com.intel.afw.mdm:id/btnSetupProfile"
    install_a_trust_agent = "Install A Trust Agent Sample"
    grant_trust = "com.android.trustagent.testsample:id/enable_trust"
    revoke_trust = "com.android.trustagent.testsample:id/revoke_trust"

    # user restrictions
    disallow_modify_account = "Disallow Modify Accounts "
    disallow_share_location = "disallow_share_location "
    disallow_apps_control = "disallow apps control "
    disallow_install_apps = "disallow install apps "
    disallow_unknown_source = "disallow install unknown sources "
    disallow_config_vpn = "disallow config vpn "
    disallow_remove_users = "disallow remove user "
    disallow_factory_reset = "disallow factory reset "
    disallow_config_credentials = "disallow config credentials "
    disallow_add_user = "disallow add user "
    ensure_verify_apps = "ensure verify apps "
    disallow_config_tethering = "disallow config tethering "
    disallow_config_wifi = "disallow config wifi "
    disallow_config_bluetooth = "disallow config bluetooth "
    disallow_adjust_volume = "disallow adjust volume "
    disallow_cross_copy_paste = "disallow cross profile copy paste "

    # global settings
    global_adb_enabled = "adb enabled "
    global_auto_time = "auto time "
    global_auto_time_zone = "auto time zone "
    global_bluetooth = "bluetooth "
    global_data_roaming = "data roaming "
    global_development_settings = "development settings enabled "
    global_usb_mass_storage = "usb mass storage enabled "
    global_wifi = "wifi "
    global_skip_first_hint = "skip first use hints "
    global_install_non_market_apps = "install non market apps "
    global_spinner_network = "com.intel.afw.mdm:id/SpinnerNetworkPre"
    global_spinner_wifi = "com.intel.afw.mdm:id/SpinnerWifiSP"
    global_spinner_input = "com.intel.afw.mdm:id/SpinnerDefIM"
    global_spinner_location = "com.intel.afw.mdm:id/SpinnerLocMode"

    # activities for some apps:
    intel_mdm = "com.intel.afw.mdm/com.intel.afw.mdm.MainWindow"
    camera = "com.google.android.GoogleCamera/com.android.camera.CameraLauncher"
    chrome = "com.android.chrome/com.google.android.apps.chrome.Main"
    app_schema = "com.example.android.apprestrictionschema/com.example.android.apprestrictionschema.MainActivity"
    settings = "com.android.settings/com.android.settings.Settings"
    downloads = "com.android.providers.downloads.ui/com.android.providers.downloads.ui.DownloadList"


class Settings():
    screen_lock = "Screen lock"
    lock_swipe = "Swipe"
    lock_pin = "PIN"
    password_entry = "com.android.settings:id/password_entry"
    pin_encryption = "Encryption"
    unknown_source = "Unknown sources"
    switch_widget = "android:id/switchWidget"


class Remote():
    qq_ime = {
        "sub_path": "AfW/Apks/",
        "name": "QQ_IME.apk",
        "pkg_name": "package:com.tencent.qqpinyin"
    }

    sample_mdm = {
        "sub_path": "AfW/Apks/",
        "name": "Sample_MDM.apk",
        "name_m": "Sample_MDM_M.apk",
        "pkg_name": "package:com.intel.afw.mdm"
    }

    test_back = {
        "sub_path": "AfW/Apks/",
        "name": "TestBack2.apk",
        "pkg_name": "package:foo.bar.testback"
    }

    restriction_schema = {
        "sub_path": "AfW/Apks/",
        "name": "appRestrictionsSchema.apk",
        "pkg_name": "package:com.example.android.apprestrictionschema"
    }

    mdm_check = {
        "sub_path": "AfW/Apks/",
        "name": "AFW_Test.apk",
        "pkg_name": "package:com.intel.afw.mdmforcheck"
    }

    app_launcher = {
        "sub_path": "AfW/Apks/",
        "name": "Launcher2.apk",
        "pkg_name": "package:com.android.launcher"
    }

    cert_file = {
        "sub_path": "AfW/Files/",
        "name": "key_alpha.cer"
    }

    cert_3rd_m = {
        "sub_path": "AfW/Files/",
        "name": "Cert_Verification_M.apk",
        "pkg_name": "package:com.intel.afw.certsInstallation"
    }

