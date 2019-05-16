"""

:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related
to the source code ("Material") are owned by Intel Corporation or its
suppliers or licensors. Title to the Material remains with Intel Corporation
or its suppliers and licensors. The Material contains trade secrets and
proprietary and confidential information of Intel or its suppliers and
licensors.

The Material is protected by worldwide copyright and trade secret laws and
treaty provisions. No part of the Material may be used, copied, reproduced,
modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual
property right is granted to or conferred upon you by disclosure or delivery
of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

:organization: INTEL SVE DSV
:summary: This file implements the Moorefield platform DeviceChecks module
:since: 12/14/2014
:author: srdubbak
"""

from acs_test_scripts.Device.Module.Common.DeviceChecksModule.IDeviceChecksModule import IDeviceChecksModule
from ErrorHandling.DeviceException import DeviceException
from Device.Module.DeviceModuleBase import DeviceModuleBase


class MoorefieldDeviceChecksModule(IDeviceChecksModule, DeviceModuleBase):

    def __init__(self):
        super(MoorefieldDeviceChecksModule, self).__init__()

    def init(self):
        """
        Initialize nw module

        :rtype: UtilitiesFWK.Utilities.Global
        :return: Init status
        """
        verdict = Global.SUCCESS
        self._networking_properties = self.configuration
        return verdict

    def get_mmio_reg_address(self, reg_name=None):
        """
        List of Display Registers and its values

        reg_name type: str
        reg_name param value : display register name
        """
        MOFD_MMIO_REG_DICT = {
            'ablm_hist_ctl': 'c0061260' ,
            'aimg_enhance_bin': 'c0061264' ,
            'aud_buf_a_addr': 'c0069040' ,
            'aud_buf_a_length': 'c0069044' ,
            'aud_buf_b_addr': 'c0069048' ,
            'aud_buf_b_length': 'c006904c' ,
            'aud_buf_c_addr': 'c0069050' ,
            'aud_buf_c_length': 'c0069054' ,
            'aud_buf_ch_swp': 'c0069024' ,
            'aud_buf_config': 'c0069020' ,
            'aud_buf_d_addr': 'c0069058' ,
            'aud_buf_d_length': 'c006905c' ,
            'aud_ch_status_0': 'c0069008' ,
            'aud_ch_status_1': 'c006900c' ,
            'aud_cntl_st': 'c0069060' ,
            'aud_config': 'c0069000' ,
            'aud_hdmi_cts': 'c0069010' ,
            'aud_hdmi_status': 'c0069068' ,
            'aud_hdmiw_infofr': 'c0069114' ,
            'aud_n_enable': 'c0069014' ,
            'aud_sample_rate': 'c0069018' ,
            'chicken_bit': 'c0070400' ,
            'cimg_enhance_bin': 'c0062264' ,
            'crcctrlcolora_blue': 'c0060058' ,
            'crcctrlcolora_green': 'c0060054' ,
            'crcctrlcolora_red': 'c0060050' ,
            'crcctrlcolora_residual': 'c006005c' ,
            'crcctrlcolorb_blue': 'c0061058' ,
            'crcctrlcolorb_green': 'c0061054' ,
            'crcctrlcolorb_red': 'c0061050' ,
            'crcctrlcolorb_residual': 'c006105c' ,
            'crcctrlcolorc_blue': 'c0062058' ,
            'crcctrlcolorc_green': 'c0062054' ,
            'crcctrlcolorc_red': 'c0062050' ,
            'crcctrlcolorc_residual': 'c006205c' ,
            'crcrescolora_blue': 'c0060068' ,
            'crcrescolora_green': 'c0060064' ,
            'crcrescolora_red': 'c0060060' ,
            'crcrescolora_residual': 'c006006c' ,
            'crcrescolorb_blue': 'c0061068' ,
            'crcrescolorb_green': 'c0061064' ,
            'crcrescolorb_red': 'c0061060' ,
            'crcrescolorb_residual': 'c006106c' ,
            'crcrescolorc_blue': 'c0062068' ,
            'crcrescolorc_green': 'c0062064' ,
            'crcrescolorc_red': 'c0062060' ,
            'crcrescolorc_residual': 'c006206c' ,
            'curabase': 'c0070084' ,
            'curacntr': 'c0070080' ,
            'curapalet0': 'c0070090' ,
            'curapalet1': 'c0070094' ,
            'curapalet2': 'c0070098' ,
            'curapalet3': 'c007009c' ,
            'curapos': 'c0070088' ,
            'curb_palet0': 'c00700d0' ,
            'curb_palet1': 'c00700d4' ,
            'curb_palet2': 'c00700d8' ,
            'curb_palet3': 'c00700dc' ,
            'curbbase': 'c00700c4' ,
            'curbcntr': 'c00700c0' ,
            'curbpos': 'c00700c8' ,
            'curc_palet0': 'c00700f0' ,
            'curc_palet1': 'c00700f4' ,
            'curc_palet2': 'c00700f8' ,
            'curc_palet3': 'c00700fc' ,
            'curcbase': 'c00700e4' ,
            'curccntr': 'c00700e0' ,
            'curcpos': 'c00700e8' ,
            'dclrkm': 'c0030154' ,
            'dclrkv': 'c0030150' ,
            'ddl1': 'c0070060' ,
            'ddl2': 'c0070064' ,
            'ddl3': 'c0070068' ,
            'ddl4': 'c007006c' ,
            'dovasta': 'c0030008' ,
            'dovcsta': 'c0038008' ,
            'dpalette_a': 'c000a000' ,
            'dpalette_b': 'c000a800' ,
            'dpalette_c': 'c000ac00' ,
            'dpll_ctrl': 'c000f018' ,
            'dspacntr': 'c0070180' ,
            'dspacontalpha': 'c00701a8' ,
            'dspakeymaxval': 'c00701a0' ,
            'dspakeyminval': 'c0070194' ,
            'dspakeymsk': 'c0070198' ,
            'dspalinoff': 'c0070184' ,
            'dspapos': 'c007018c' ,
            'dsparb': 'c0070030' ,
            'dsparb2': 'c007002c' ,
            'dspasize': 'c0070190' ,
            'dspastride': 'c0070188' ,
            'dspasurf': 'c007019c' ,
            'dspatileoff': 'c00701a4' ,
            'dspbcntr': 'c0071180' ,
            'dspbcontalpha': 'c00711a8' ,
            'dspbkeymaxval': 'c00711a0' ,
            'dspbkeyminval': 'c0071194' ,
            'dspbkeymsk': 'c0071198' ,
            'dspblinoff': 'c0071184' ,
            'dspbpos': 'c007118c' ,
            'dspbsize': 'c0071190' ,
            'dspbstride': 'c0071188' ,
            'dspbsurf': 'c007119c' ,
            'dspbtileoff': 'c00711a4' ,
            'dspccntr': 'c0072180' ,
            'dspccontalpha': 'c00721a8' ,
            'dspckeymaxval': 'c00721a0' ,
            'dspckeyminval': 'c0072194' ,
            'dspckeymsk': 'c0072198' ,
            'dspclinoff': 'c0072184' ,
            'dspclk_gate_d': 'c0070500' ,
            'dspcpos': 'c007218c' ,
            'dspcsize': 'c0072190' ,
            'dspcstride': 'c0072188' ,
            'dspcsurf': 'c007219c' ,
            'dspctileoff': 'c00721a4' ,
            'dspdcntr': 'c0073180' ,
            'dspdcontalpha': 'c00731a8' ,
            'dspdkeymaxval': 'c00731a0' ,
            'dspdkeymsk': 'c0073198' ,
            'dspdlinoff': 'c0073184' ,
            'dspdpos': 'c007318c' ,
            'dspdsize': 'c0073190' ,
            'dspdstride': 'c0073188' ,
            'dspdsurf': 'c007319c' ,
            'dspdtileoff': 'c00731a4' ,
            'dspecntr': 'c0074180' ,
            'dspecontalpha': 'c00741a8' ,
            'dspekeymaxval': 'c00741a0' ,
            'dspekeyminval': 'c0074194' ,
            'dspekeymsk': 'c0074198' ,
            'dspelinoff': 'c0074184' ,
            'dspepos': 'c007418c' ,
            'dspesize': 'c0074190' ,
            'dspestride': 'c0074188' ,
            'dspesurf': 'c007419c' ,
            'dspetileoff': 'c00741a4' ,
            'dspfcntr': 'c0075180' ,
            'dspfcontalpha': 'c00751a8' ,
            'dspfkeymaxval': 'c00751a0' ,
            'dspfkeyminval': 'c0075194' ,
            'dspfkeymsk': 'c0075198' ,
            'dspflinoff': 'c0075184' ,
            'dspfpos': 'c007518c' ,
            'dspfsize': 'c0075190' ,
            'dspfstride': 'c0075188' ,
            'dspfsurf': 'c007519c' ,
            'dspftileoff': 'c00751a4' ,
            'dspsrctrl': 'c007005c' ,
            'dwi_npos': 'c003012c' ,
            'dwinsz': 'c0030130' ,
            'fbdcchicken': 'c0070508' ,
            'fw1': 'c0070034' ,
            'fw2': 'c0070038' ,
            'fw3': 'c007003c' ,
            'fw4': 'c0070050' ,
            'fw5': 'c0070054' ,
            'fw6': 'c0070058' ,
            'fw7': 'c0070070' ,
            'gci_control': 'c000650c' ,
            'hdcp_akey_hi': 'c0061424' ,
            'hdcp_akey_lo': 'c006141c' ,
            'hdcp_akey_med': 'c0061420' ,
            'hdcp_aksv_hi': 'c0061450' ,
            'hdcp_aksv_lo': 'c0061454' ,
            'hdcp_an_hi': 'c0061414' ,
            'hdcp_an_lo': 'c0061410' ,
            'hdcp_bksv_hi': 'c006140c' ,
            'hdcp_bksv_lo': 'c0061408' ,
            'hdcp_config': 'c0061400' ,
            'hdcp_dbg_stat': 'c006144c' ,
            'hdcp_init': 'c0061404' ,
            'hdcp_rep': 'c0061444' ,
            'hdcp_ri': 'c0061418' ,
            'hdcp_sha1_in': 'c0061440' ,
            'hdcp_status': 'c0061448' ,
            'hdcp_v_0': 'c006142c' ,
            'hdcp_v_1': 'c0061430' ,
            'hdcp_v_2': 'c0061434' ,
            'hdcp_v_3': 'c0061438' ,
            'hdcp_v_4': 'c006143c' ,
            'hdmi_misr_config': 'c006117c' ,
            'hdmi_misr_signature': 'c0061180' ,
            'hdmib': 'c0061140' ,
            'hdmiphyclkctl': 'c0061124' ,
            'hdmiphydatactl': 'c0061120' ,
            'hdmiphymbiasctl_1': 'c0061128' ,
            'hdmiphymiscctl': 'c0061134' ,
            'hdmiphyobsctl': 'c0061130' ,
            'hdmiphytimctl': 'c006112c' ,
            'hist_threshold_gd': 'c0061268' ,
            'hist_threshold_gd2': 'c0062268' ,
            'horz_ph': 'c0030124' ,
            'init_phs': 'c0030128' ,
            'mipa_rd_data_return0': 'c000b118' ,
            'mipa_rd_data_return1': 'c000b11c' ,
            'mipa_rd_data_return2': 'c000b120' ,
            'mipa_rd_data_return3': 'c000b124' ,
            'mipa_rd_data_return4': 'c000b128' ,
            'mipa_rd_data_return5': 'c000b12c' ,
            'mipa_rd_data_return6': 'c000b130' ,
            'mipa_rd_data_return7': 'c000b134' ,
            'mipia_autopwg': 'c007000c' ,
            'mipia_clk_lane_switching_time_cnt': 'c000b088' ,
            'mipia_cmd_add': 'c000b110' ,
            'mipia_cmd_len': 'c000b114' ,
            'mipia_ctrl': 'c000b104' ,
            'mipia_data_add': 'c000b108' ,
            'mipia_data_len': 'c000b10c' ,
            'mipia_dbi_bw_ctrl_reg': 'c000b084' ,
            'mipia_dbi_resolution_reg': 'c000b024' ,
            'mipia_dbi_typec_ctrl': 'c000b100' ,
            'mipia_device_ready_reg': 'c000b000' ,
            'mipia_device_reset_timer': 'c000b01c' ,
            'mipia_dphy_param_reg': 'c000b080' ,
            'mipia_dpi_ctrl_reg': 'c000b048' ,
            'mipia_dpi_data_register': 'c000b04c' ,
            'mipia_dpi_resolution_reg': 'c000b020' ,
            'mipia_dsi_func_prg__reg': 'c000b00c' ,
            'mipia_eot_disable_register': 'c000b05c' ,
            'mipia_gen_fifo_stat_register': 'c000b074' ,
            'mipia_high_low_switch_count': 'c000b044' ,
            'mipia_horiz_active_area_count': 'c000b034' ,
            'mipia_horiz_back_porch_count': 'c000b02c' ,
            'mipia_horiz_front_porch_count': 'c000b030' ,
            'mipia_horiz_sync_padding_count': 'c000b028' ,
            'mipia_hs_gen_ctrl_register': 'c000b070' ,
            'mipia_hs_gen_data_register': 'c000b068' ,
            'mipia_hs_ls_dbi_enable_reg': 'c000b078' ,
            'mipia_hs_tx_timeout_reg': 'c000b010' ,
            'mipia_init_count_register': 'c000b050' ,
            'mipia_intr_en_reg': 'c000b008' ,
            'mipia_intr_en_reg_1': 'c000b094' ,
            'mipia_intr_stat_reg': 'c000b004' ,
            'mipia_intr_stat_reg_1': 'c000b090' ,
            'mipia_lp_byteclk_register': 'c000b060' ,
            'mipia_lp_gen_ctrl_register': 'c000b06c' ,
            'mipia_lp_gen_data_register': 'c000b064' ,
            'mipia_lp_rx_timeout_reg': 'c000b014' ,
            'mipia_max_return_pkt_size_register': 'c000b054' ,
            'mipia_port_ctrl': 'c0061190' ,
            'mipia_rd_data_valid': 'c000b138' ,
            'mipia_reserved': 'c000b07c' ,
            'mipia_stp_sta_stall_reg': 'c000b08c' ,
            'mipia_tearing_ctr': 'c0061194' ,
            'mipia_turn_around_timeout_reg': 'c000b018' ,
            'mipia_vert_back_porch_count': 'c000b03c' ,
            'mipia_vert_front_porch_count': 'c000b040' ,
            'mipia_vert_sync_padding_count': 'c000b038' ,
            'mipia_video_mode_format_register': 'c000b058' ,
            'mipic_clk_lane_switching_time_cnt': 'c000b888' ,
            'mipic_cmd_add': 'c000b910' ,
            'mipic_cmd_len': 'c000b914' ,
            'mipic_ctrl': 'c000b904' ,
            'mipic_data_add': 'c000b908' ,
            'mipic_data_len': 'c000b90c' ,
            'mipic_dbi_bw_ctrl_reg': 'c000b884' ,
            'mipic_dbi_resolution_reg': 'c000b824' ,
            'mipic_device_ready_reg': 'c000b800' ,
            'mipic_device_reset_timer': 'c000b81c' ,
            'mipic_dphy_param_reg': 'c000b880' ,
            'mipic_dpi_ctrl_reg': 'c000b848' ,
            'mipic_dpi_data_register': 'c000b84c' ,
            'mipic_dpi_resolution_reg': 'c000b820' ,
            'mipic_dsi_func_prg__reg': 'c000b80c' ,
            'mipic_eot_disable_register': 'c000b85c' ,
            'mipic_gen_fifo_stat_register': 'c000b874' ,
            'mipic_high_low_switch_count': 'c000b844' ,
            'mipic_horiz_active_area_count': 'c000b834' ,
            'mipic_horiz_back_porch_count': 'c000b82c' ,
            'mipic_horiz_front_porch_count': 'c000b830' ,
            'mipic_horiz_sync_padding_count': 'c000b828' ,
            'mipic_hs_gen_ctrl_register': 'c000b870' ,
            'mipic_hs_gen_data_register': 'c000b868' ,
            'mipic_hs_ls_dbi_enable_reg': 'c000b878' ,
            'mipic_hs_tx_timeout_reg': 'c000b810' ,
            'mipic_init_count_register': 'c000b850' ,
            'mipic_intr_en_reg': 'c000b808' ,
            'mipic_intr_en_reg_1': 'c000b894' ,
            'mipic_intr_stat_reg': 'c000b804' ,
            'mipic_intr_stat_reg_1': 'c000b890' ,
            'mipic_lp_byteclk_register': 'c000b860' ,
            'mipic_lp_gen_ctrl_register': 'c000b86c' ,
            'mipic_lp_gen_data_register': 'c000b864' ,
            'mipic_lp_rx_timeout_reg': 'c000b814' ,
            'mipic_max_return_pkt_size_register': 'c000b854' ,
            'mipic_port_ctrl': 'c0062190' ,
            'mipic_rd_data_return0': 'c000b918' ,
            'mipic_rd_data_return1': 'c000b91c' ,
            'mipic_rd_data_return2': 'c000b920' ,
            'mipic_rd_data_return3': 'c000b924' ,
            'mipic_rd_data_return4': 'c000b928' ,
            'mipic_rd_data_return5': 'c000b92c' ,
            'mipic_rd_data_return6': 'c000b930' ,
            'mipic_rd_data_return7': 'c000b934' ,
            'mipic_rd_data_valid': 'c000b938' ,
            'mipic_reserved': 'c000b87c' ,
            'mipic_stp_sta_stall_reg': 'c000b88c' ,
            'mipic_tearing_ctr': 'c0062194' ,
            'mipic_turn_around_timeout_reg': 'c000b818' ,
            'mipic_vert_back_porch_count': 'c000b83c' ,
            'mipic_vert_front_porch_count': 'c000b840' ,
            'mipic_vert_sync_padding_count': 'c000b838' ,
            'mipic_video_mode_format_register': 'c000b858' ,
            'oa_uv_hcoefs': 'c0030700' ,
            'oabuf_0u': 'c0030108' ,
            'oabuf_0v': 'c003010c' ,
            'oabuf_0y': 'c0030100' ,
            'oabuf_1u': 'c0030110' ,
            'oabuf_1v': 'c0030114' ,
            'oabuf_1y': 'c0030104' ,
            'oacomd': 'c0030168' ,
            'oaconfig': 'c0030164' ,
            'oagamc0': 'c0030024' ,
            'oagamc1': 'c0030020' ,
            'oagamc2': 'c003001c' ,
            'oagamc3': 'c0030018' ,
            'oagamc4': 'c0030014' ,
            'oagamc5': 'c0030010' ,
            'oastart_0u': 'c0030178' ,
            'oastart_0v': 'c003017c' ,
            'oastart_0y': 'c0030170' ,
            'oastart_1u': 'c0030180' ,
            'oastart_1v': 'c0030184' ,
            'oastart_1y': 'c0030174' ,
            'oastride': 'c0030118' ,
            'oatest': 'c0030004' ,
            'oatileoff_0u': 'c0030190' ,
            'oatileoff_0v': 'c0030194' ,
            'oatileoff_0y': 'c0030188' ,
            'oatileoff_1u': 'c0030198' ,
            'oatileoff_1v': 'c003019c' ,
            'oatileoff_1y': 'c003018c' ,
            'obuf_0u': 'c0038108' ,
            'ocbuf_0v': 'c003810c' ,
            'ocbuf_0y': 'c0038100' ,
            'ocbuf_1u': 'c0038110' ,
            'ocbuf_1v': 'c0038114' ,
            'ocbuf_1y': 'c0038104' ,
            'occlrc0': 'c0038148' ,
            'occlrc1': 'c003814c' ,
            'occomd': 'c0038168' ,
            'occonfig': 'c0038164' ,
            'ocdclrkm': 'c0038154' ,
            'ocdclrkv': 'c0038150' ,
            'ocdwinpos': 'c003812c' ,
            'ocdwinsz': 'c0038130' ,
            'ocgamc0': 'c0038024' ,
            'ocgamc1': 'c0038020' ,
            'ocgamc2': 'c003801c' ,
            'ocgamc3': 'c0038018' ,
            'ocgamc4': 'c0038014' ,
            'ocgamc5': 'c0038010' ,
            'ochorz_ph': 'c0038124' ,
            'ocinit_phs': 'c0038128' ,
            'oclrc0': 'c0030148' ,
            'oclrc1': 'c003014c' ,
            'ocschrken': 'c0038160' ,
            'ocschrkvh': 'c0038158' ,
            'ocschrkvl': 'c003815c' ,
            'ocsheight': 'c003813c' ,
            'ocstart_0u': 'c0038178' ,
            'ocstart_0v': 'c003817c' ,
            'ocstart_0y': 'c0038170' ,
            'ocstart_1u': 'c0038180' ,
            'ocstart_1v': 'c0038184' ,
            'ocstart_1y': 'c0038174' ,
            'ocstride': 'c0038118' ,
            'ocswidth': 'c0038134' ,
            'ocswidthsw': 'c0038138' ,
            'octest': 'c0038004' ,
            'octileoff_0u': 'c0038190' ,
            'octileoff_0v': 'c0038194' ,
            'octileoff_0y': 'c0038188' ,
            'octileoff_1u': 'c0038198' ,
            'octileoff_1v': 'c003819c' ,
            'octileoff_1y': 'c003818c' ,
            'ocuv_hcoefs': 'c0038700' ,
            'ocuv_vph': 'c0038120' ,
            'ocuvscale': 'c0038144' ,
            'ocyrgb_vph': 'c003811c' ,
            'ova_uvscalev': 'c00301a4' ,
            'ova_yrgbscale': 'c0030140' ,
            'ovaadd': 'c0030000' ,
            'ovc_uvscalev': 'c00381a4' ,
            'ovc_yrgbscale': 'c0038140' ,
            'ovcadd': 'c0038000' ,
            'pfit_control': 'c0061230' ,
            'pfit_pgm_ratios': 'c0061234' ,
            'pipe_cgm_control_a': 'c0067a00' ,
            'pipe_cgm_control_b': 'c0069a00' ,
            'pipe_cgm_control_c': 'c006ba00' ,
            'pipe_cgm_csc_coeffecient01_a': 'c0067900' ,
            'pipe_cgm_csc_coeffecient01_b': 'c0069900' ,
            'pipe_cgm_csc_coeffecient01_c': 'c006b900' ,
            'pipe_cgm_csc_coeffecient23_a': 'c0067904' ,
            'pipe_cgm_csc_coeffecient23_b': 'c0069904' ,
            'pipe_cgm_csc_coeffecient23_c': 'c006b904' ,
            'pipe_cgm_csc_coeffecient45_a': 'c0067908' ,
            'pipe_cgm_csc_coeffecient45_b': 'c0069908' ,
            'pipe_cgm_csc_coeffecient45_c': 'c006b908' ,
            'pipe_cgm_csc_coeffecient67_a': 'c006790c' ,
            'pipe_cgm_csc_coeffecient67_b': 'c006990c' ,
            'pipe_cgm_csc_coeffecient67_c': 'c006b90c' ,
            'pipe_cgm_csc_coeffecient8_a': 'c0067910' ,
            'pipe_cgm_csc_coeffecient8_b': 'c0069910' ,
            'pipe_cgm_csc_coeffecient8_c': 'c006b910' ,
            'pipe_conf_a': 'c0070008' ,
            'pipe_conf_b': 'c0071008' ,
            'pipe_conf_c': 'c0072008' ,
            'pipe_hblank_a': 'c0060004' ,
            'pipe_hblank_b': 'c0061004' ,
            'pipe_hblank_c': 'c0062004' ,
            'pipe_hsync_a': 'c0060008' ,
            'pipe_hsync_b': 'c0061008' ,
            'pipe_hsync_c': 'c0062008' ,
            'pipe_htotal_a': 'c0060000' ,
            'pipe_htotal_b': 'c0061000' ,
            'pipe_htotal_c': 'c0062000' ,
            'pipe_srcsz_a': 'c006001c' ,
            'pipe_srcsz_b': 'c006101c' ,
            'pipe_srcsz_c': 'c006201c' ,
            'pipe_vblank_a': 'c0060010' ,
            'pipe_vblank_b': 'c0061010' ,
            'pipe_vblank_c': 'c0062010' ,
            'pipe_vsync_a': 'c0060014' ,
            'pipe_vsync_b': 'c0061014' ,
            'pipe_vsync_c': 'c0062014' ,
            'pipe_vsyncshift_a': 'c0060028' ,
            'pipe_vsyncshift_b': 'c0061028' ,
            'pipe_vsyncshift_c': 'c0062028' ,
            'pipe_vtotal_a': 'c006000c' ,
            'pipe_vtotal_b': 'c006100c' ,
            'pipe_vtotal_c': 'c006200c' ,
            'pipea_color_coef0': 'c0060070' ,
            'pipea_color_coef11': 'c0060078' ,
            'pipea_color_coef12': 'c006007c' ,
            'pipea_color_coef2': 'c0060074' ,
            'pipea_color_coef21': 'c0060080' ,
            'pipea_color_coef22': 'c0060084' ,
            'pipea_dsl': 'c0070000' ,
            'pipea_repeated_frame_count_threshold': 'c0060090' ,
            'pipea_slc': 'c0070004' ,
            'pipeaframehigh': 'c0070040' ,
            'pipeaframepixel': 'c0070044' ,
            'pipeagcmaxblue': 'c0070018' ,
            'pipeagcmaxgreen': 'c0070014' ,
            'pipeagcmaxred': 'c0070010' ,
            'pipeastat': 'c0070024' ,
            'pipeb_color_coef0': 'c0061070' ,
            'pipeb_color_coef11': 'c0061078' ,
            'pipeb_color_coef12': 'c006107c' ,
            'pipeb_color_coef2': 'c0061074' ,
            'pipeb_color_coef21': 'c0061080' ,
            'pipeb_color_coef22': 'c0061084' ,
            'pipeb_dsl': 'c0071000' ,
            'pipeb_slc': 'c0071004' ,
            'pipebframehigh': 'c0071040' ,
            'pipebframepixel': 'c0071044' ,
            'pipebgcmaxblue': 'c0071018' ,
            'pipebgcmaxgreen': 'c0071014' ,
            'pipebgcmaxred': 'c0071010' ,
            'pipebstat': 'c0071024' ,
            'pipec_color_coef0': 'c0062070' ,
            'pipec_color_coef11': 'c0062078' ,
            'pipec_color_coef12': 'c006207c' ,
            'pipec_color_coef2': 'c0062074' ,
            'pipec_color_coef21': 'c0062080' ,
            'pipec_color_coef22': 'c0062084' ,
            'pipec_dsl': 'c0072000' ,
            'pipec_slc': 'c0072004' ,
            'pipecframehigh': 'c0072040' ,
            'pipecframepixel': 'c0072044' ,
            'pipecgcmaxblue': 'c0072018' ,
            'pipecgcmaxgreen': 'c0072014' ,
            'pipecgcmaxred': 'c0072010' ,
            'pipecstat': 'c0072024' ,
            'ramclk_gate_d': 'c0070504' ,
            'schrken': 'c0030160' ,
            'schrkvh': 'c0030158' ,
            'schrkvl': 'c003015c' ,
            'sheight': 'c003013c' ,
            'spdkeyminval': 'c0073194' ,
            'swidth': 'c0030134' ,
            'swidthsw': 'c0030138' ,
            'uv_vph': 'c0030120' ,
            'uvscale': 'c0030144' ,
            'video_dip_ctl': 'c0061170' ,
            'video_dip_data': 'c0061178' ,
            'yrgb_vph': 'c003011c'
        }

        return MOFD_MMIO_REG_DICT.get(reg_name)

    def read_mmio_reg_32(self, reg_address=None):

        """
        Reads the value in the register address specified.
        :rtype : string
        :return : value in the address
        """
        peeknpoke_path = "/system/bin/peeknpoke"

        cmd = "adb shell {0} r {1} 32".format(peeknpoke_path, reg_address)
        status, output = self.device.run_cmd(cmd, 2)
        temp_string = output.split(" ")
        if temp_string[0] != "Failed":
            temp_string = temp_string[(len(temp_string) - 1)].split("\r")[0]
            reg_val = int(temp_string, 0)
            return reg_val
        else:
            err_msg = cmd + " failed to access MMIO register"
            raise DeviceException(DeviceException.OPERATION_FAILED, err_msg)

    def check_mipi_errors(self):
        """
        Pipe-A and Pipe-C (if it is enabled for example in dual link mode):

            Will check that all bits except for Special Packet Sent interrupt
            (bit30) and Turn Around Ack Timeout (bit 23) are set to 0b0.

        RETURN
            return_str and reg_val:
            return_str will be either "PASS", "INCONCLUSIVE", "FAIL",
                "Panel is command mode" INCONCLUSIVE is return if mipia is off.

            reg_val will be the value read for this register. If return_str is
            "INCONCLUSIVE", then reg_val is meaningless.

        """
        reg_addr_value = self.get_mmio_reg_address(reg_name="mipia_port_ctrl")
        reg_val = self.read_mmio_reg_32(reg_address=reg_addr_value)

        # Check if the panel is in video mode, otherwise this register
        # is meaningless
        _mode_en_bit = reg_val & 0x80000000
        _dual_mode_en_bit = reg_val & 0x04000000
        if (not _mode_en_bit == 0) & (not reg_val == 0xffffffff):
            # Read mipia_intr_stat_reg
            reg_addr_value = self.get_mmio_reg_address(reg_name="mipia_intr_stat_reg")
            reg_val = self.read_mmio_reg_32(reg_address=reg_addr_value)

            # Check the register bits but clear bit 30 and 23 to 0 first
            reg_val &= 0xbf7fffff
            if reg_val == 0:
                return "PASS"
            elif reg_val == 0xbf7fffff:
                return "INCONCLUSIVE"
            else:
                return "FAIL"
                self.logger.error("CheckDisplayRegisters : Mipi Pipe-A errors found. ")
                raise DeviceException(DeviceException.OPERATION_FAILED,"CheckDisplayRegisters : Mipi Pipe-A errors found. ")
            #If dual link is enabled then mipic is also enabled so checking fo rmipic_intr_stat_reg.
            if (not _dual_mode_en_bit == 0):
                # Read mipic_intr_stat_reg
                reg_addr_value = self.get_mmio_reg_address(reg_name="mipic_intr_stat_reg")
                reg_val = self.read_mmio_reg_32(reg_address=reg_addr_value)

                # Check the register bits but clear bit 30 and 23 to 0 first
                reg_val &= 0xbf7fffff
                if reg_val == 0:
                    return "PASS"
                elif reg_val == 0xbf7fffff:
                    return "INCONCLUSIVE"
                else:
                    return "FAIL"
                    self.logger.error("CheckDisplayRegisters : Mipi Pipe-C errors found. ")
                    raise DeviceException(DeviceException.OPERATION_FAILED,"CheckDisplayRegisters : Mipi Pipe-C errors found. ")
        else:
            return "Panel is command mode, irq status register is meaningless"


    def check_hdmi_errors(self):
        """
        Pipe B(HDMI):
            Operates on Pipe B (HDMI)

            Will check bit 31 for buffer underflow.

        RETURN
            return_str and reg_val:
            return_str will be either "PASS", "INCONCLUSIVE", "FAIL",

            reg_val will be the value read for this register. If return_str is
            "INCONCLUSIVE", then reg_val is meaningless.
        """
        #Check whether hdmi is connected.
        reg_addr_value_hdmi = self.get_mmio_reg_address(reg_name="pipe_conf_b")
        hdmi_enable = self.read_mmio_reg_32(reg_address=reg_addr_value_hdmi)
        # Read pipebstat and mask for buffer underflow bit
        reg_addr_value = self.get_mmio_reg_address(reg_name="pipebstat")
        reg_val = self.read_mmio_reg_32(reg_address=reg_addr_value)
        hdmi_enable &= 0x80000000

        if ((not hdmi_enable == 0x80000000) and (not reg_val == 0xffffffff) ):

            buffer_under_flow_bit = reg_val & 0x80000000
            if buffer_under_flow_bit == 0:
                return "PASS"
            else:
                return "FAIL"
        else:
            return "INCONCLUSIVE: HDMI not connected/enabled", 0xffffffff


    def check_pipe_hang(self):
        """
        Pipe A:
            This method checks for a pipe hand by checking the pipea_dsl register
            This only works for video mode panels.
        PipeB:
            This method checks for a pipe hang by checking the pipeb_dsl register

        """
        # Pipe A: Check if the panel is in video mode, otherwise this register is meaningless
        # Pipe B: Check if pipe b (HDMI) is enabled

        reg_addr_value_a = self.get_mmio_reg_address(reg_name="mipia_port_ctrl")
        reg_val_a = self.read_mmio_reg_32(reg_address=reg_addr_value_a)
        reg_addr_value_b = self.get_mmio_reg_address(reg_name="pipe_conf_b")
        reg_val_b = self.read_mmio_reg_32(reg_address=reg_addr_value_b)
        hung_a = False
        hung_b = False
        status = ""
        _mode_en_bit_a = reg_val_a & 0x80000000
        _mode_en_bit_b = reg_val_b & 0x80000000
        if (not _mode_en_bit_a == 0) and (not reg_val_a == 0xffffffff):
            # check the register a few times to make sure it is changing
            # if it is not, then the pipe is hanged
            reg_addr_value = self.get_mmio_reg_address(reg_name="pipea_dsl")
            orig_reg_value = self.read_mmio_reg_32(reg_address=reg_addr_value)
            hung_a = True
            for i in range(1,5):
                if (self.read_mmio_reg_32(reg_address=reg_addr_value) <> orig_reg_value):
                    hung_a = False
                    status ="PipeA is not hung."
                    self.logger.debug("The pipe A status : "+status)
                    break
                else:
                    status = "PipeA is hung."

        if (not _mode_en_bit_b == 0) and (not reg_val_b == 0xffffffff):
            # check the register a few times to make sure it is changing
            # if it is not, then the pipe is hanged
            reg_addr_value = self.get_mmio_reg_address(reg_name="pipeb_dsl")
            orig_reg_value = self.read_mmio_reg_32(reg_address=reg_addr_value)
            hung_b = True
            for i in range(1,5):
                if (self.read_mmio_reg_32(reg_address=reg_addr_value) <> orig_reg_value):
                    hung_b = False
                    status ="PipeB is not hung "
                    break
                else:
                    status = "PipeB is hung "
        else:
            self.logger.debug( "INCONCLUSIVE, Panel is command mode")

        if ((not hung_a) and (not hung_b)) :
            self.logger.debug("The pipe status :"+status)
        else:
            # Log the failure if all numbers are the same
            self.logger.error("The pipe status : "+status)
            raise DeviceException(DeviceException.OPERATION_FAILED,"The pipe status : "+status)

    def enable_tearing_effect_detection(self):
        """

        This method enables the tearing effect detection.
        This only works for command mode panels.

        RETURN
            "PASS" of TE mean tearing effect detection is enabled, "FAIL" is it is not
            "INCONCLUSIVE, Panel is video mode" is this is a video mode panel.

        """

        # Check if the panel is in command mode
        reg_addr_value = self.get_mmio_reg_address(reg_name="mipia_port_ctrl")
        reg_val = self.read_mmio_reg_32(reg_address=reg_addr_value)

        _mode_en_bit = reg_val & 0x80000000

        if (_mode_en_bit == 0) & (not reg_val == 0xffffffff):
            te_enable = ((reg_val >> 2) & 0b11)
            if te_enable == 2:
                return "PASS"
            else:
                return "FAIL"
        else:
            return "INCONCLUSIVE, Panel is video mode or command mode in self refresh mode"