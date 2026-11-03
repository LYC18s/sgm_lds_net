# Operates ONLY on the calling project's private imported BD. Never overwrite
# hardware/vivado_sgm/system/system.bd or production artifacts from this file.
set lds_net_root [file normalize [file join [file dirname [info script]] ..]]
proc rl_disconnect {name} {
    set p [get_bd_pins -quiet $name]
    if {[llength $p]} {set n [get_bd_nets -quiet -of_objects $p];if {[llength $n]} {disconnect_bd_net $n $p}}
}
proc rl_remove_cell {name} {
    set c [get_bd_cells -quiet $name]
    if {![llength $c]} {return}
    foreach p [get_bd_intf_pins -of_objects $c] {
        set n [get_bd_intf_nets -quiet -of_objects $p]
        if {[llength $n]} {delete_bd_objs $n}
    }
    delete_bd_objs $c
}
proc rl_mem_clock {cell clock reset interfaces} {
    connect_bd_net [get_bd_pins $clock] [get_bd_pins $cell/ACLK]
    connect_bd_net [get_bd_pins $reset] [get_bd_pins $cell/ARESETN]
    foreach p $interfaces {
        connect_bd_net [get_bd_pins $clock] [get_bd_pins $cell/${p}_ACLK]
        connect_bd_net [get_bd_pins $reset] [get_bd_pins $cell/${p}_ARESETN]
    }
}
foreach f [glob [file join $lds_net_root hardware rtl lds_net *.v]] {
    if {![llength [get_files -quiet $f]]} {add_files -norecurse $f}
}
update_compile_order -fileset sources_1
if {![llength [get_bd_cells -quiet lds_net_perception_0]]} {
    foreach required {stereo_sgm_0 cam1_axis_bcast triple_rgb_compositor_0 processing_system7_0_axi_periph} {
        if {![llength [get_bd_cells -quiet $required]]} {error "Missing retained production block $required"}
    }
    if {[llength [get_bd_cells -quiet axi_vdma_0]] || [llength [get_bd_cells -quiet cam0_median]] || [llength [get_bd_cells -quiet cam1_median]]} {
        error "LDS-Net starts from the compact four-path SGM production BD; right RGB DDR/medians must already be removed"
    }
    foreach cell {v5_cnn_0 v5_boxes_0 v5_mem_intercon} {rl_remove_cell $cell}
    foreach p {processing_system7_0_axi_periph/M07_ACLK processing_system7_0_axi_periph/M07_ARESETN processing_system7_0/S_AXI_HP3_ACLK} {rl_disconnect $p}
    set_property -dict [list CONFIG.PCW_USE_S_AXI_HP0 1 CONFIG.PCW_USE_S_AXI_HP3 1] [get_bd_cells processing_system7_0]
    create_bd_cell -type ip -vlnv xilinx.com:ip:clk_wiz:6.0 lds_net_clock
    set_property -dict [list CONFIG.PRIM_IN_FREQ 100.000 CONFIG.CLKOUT1_REQUESTED_OUT_FREQ 100.000 CONFIG.USE_RESET false CONFIG.USE_LOCKED true] [get_bd_cells lds_net_clock]
    connect_bd_net [get_bd_pins processing_system7_0/FCLK_CLK0] [get_bd_pins lds_net_clock/clk_in1]
    create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 lds_net_reset
    connect_bd_net [get_bd_pins lds_net_clock/clk_out1] [get_bd_pins lds_net_reset/slowest_sync_clk]
    connect_bd_net [get_bd_pins lds_net_clock/locked] [get_bd_pins lds_net_reset/dcm_locked]
    connect_bd_net [get_bd_pins processing_system7_0/FCLK_RESET0_N] [get_bd_pins lds_net_reset/ext_reset_in]
    connect_bd_net [get_bd_pins tdm_zero1/dout] [get_bd_pins lds_net_reset/mb_debug_sys_rst]
    foreach {name reference} {lds_net_perception_0 lds_net_perception lds_net_display_0 lds_net_display lds_net_video_tap_0 lds_net_video_tap} {
        create_bd_cell -type module -reference $reference $name
    }
    set nn_clock lds_net_clock/clk_out1
    set nn_reset lds_net_reset/peripheral_aresetn
    connect_bd_net [get_bd_pins $nn_clock] [get_bd_pins lds_net_perception_0/aclk] [get_bd_pins lds_net_display_0/nn_clk] [get_bd_pins lds_net_video_tap_0/nn_clk] \
        [get_bd_pins processing_system7_0_axi_periph/M07_ACLK] [get_bd_pins processing_system7_0/S_AXI_HP0_ACLK] [get_bd_pins processing_system7_0/S_AXI_HP3_ACLK]
    connect_bd_net [get_bd_pins $nn_reset] [get_bd_pins lds_net_perception_0/aresetn] [get_bd_pins lds_net_display_0/nn_rst_n] [get_bd_pins lds_net_video_tap_0/nn_rst_n] \
        [get_bd_pins processing_system7_0_axi_periph/M07_ARESETN]
    connect_bd_net [get_bd_pins processing_system7_0/FCLK_CLK1] [get_bd_pins lds_net_display_0/aclk] [get_bd_pins lds_net_video_tap_0/aclk]
    connect_bd_net [get_bd_pins rst_processing_system7_0_150M/peripheral_aresetn] [get_bd_pins lds_net_display_0/aresetn] [get_bd_pins lds_net_video_tap_0/aresetn]
    connect_bd_intf_net [get_bd_intf_pins processing_system7_0_axi_periph/M07_AXI] [get_bd_intf_pins lds_net_perception_0/S_AXI]
    connect_bd_intf_net [get_bd_intf_pins processing_system7_0_axi_periph/M08_AXI] [get_bd_intf_pins lds_net_display_0/S_AXI]
    foreach {cell count} {lds_net_hp3 1 lds_net_hp0 3} {
        create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:2.1 $cell
        set_property -dict [list CONFIG.NUM_SI $count CONFIG.NUM_MI 1] [get_bd_cells $cell]
    }
    rl_mem_clock lds_net_hp3 $nn_clock $nn_reset {S00 M00}
    rl_mem_clock lds_net_hp0 $nn_clock $nn_reset {S00 S01 S02 M00}
    connect_bd_intf_net [get_bd_intf_pins lds_net_perception_0/M_NET] [get_bd_intf_pins lds_net_hp3/S00_AXI]
    connect_bd_intf_net [get_bd_intf_pins lds_net_hp3/M00_AXI] [get_bd_intf_pins processing_system7_0/S_AXI_HP3]
    connect_bd_intf_net [get_bd_intf_pins lds_net_perception_0/M_CAPTURE] [get_bd_intf_pins lds_net_hp0/S00_AXI]
    connect_bd_intf_net [get_bd_intf_pins lds_net_perception_0/M_MASK] [get_bd_intf_pins lds_net_hp0/S01_AXI]
    connect_bd_intf_net [get_bd_intf_pins lds_net_display_0/M_AXI] [get_bd_intf_pins lds_net_hp0/S02_AXI]
    connect_bd_intf_net [get_bd_intf_pins lds_net_hp0/M00_AXI] [get_bd_intf_pins processing_system7_0/S_AXI_HP0]
    # Logical left is physical camera1. Tap primary is a wire-through and can
    # never stall SGM/video because the inference-only FIFO is full.
    set camera_sink [get_bd_intf_pins cam1_axis_bcast/S_AXIS]
    disconnect_bd_intf_net [get_bd_intf_nets -of_objects $camera_sink] $camera_sink
    connect_bd_intf_net [get_bd_intf_pins alinx_ov5640_1/m_axis_video] [get_bd_intf_pins lds_net_video_tap_0/S_AXIS]
    connect_bd_intf_net [get_bd_intf_pins lds_net_video_tap_0/M_AXIS] $camera_sink
    create_bd_cell -type ip -vlnv xilinx.com:ip:axis_data_fifo:2.0 lds_net_input_cdc
    set_property -dict [list CONFIG.TDATA_NUM_BYTES 3 CONFIG.HAS_TKEEP 1 CONFIG.TUSER_WIDTH 1 CONFIG.HAS_TLAST 1 CONFIG.IS_ACLK_ASYNC 1 CONFIG.FIFO_DEPTH 1024 CONFIG.FIFO_MODE 1] [get_bd_cells lds_net_input_cdc]
    connect_bd_net [get_bd_pins processing_system7_0/FCLK_CLK1] [get_bd_pins lds_net_input_cdc/s_axis_aclk]
    connect_bd_net [get_bd_pins $nn_clock] [get_bd_pins lds_net_input_cdc/m_axis_aclk]
    connect_bd_net [get_bd_pins lds_net_video_tap_0/branch_reset_n] [get_bd_pins lds_net_input_cdc/s_axis_aresetn]
    connect_bd_intf_net [get_bd_intf_pins lds_net_video_tap_0/NN_AXIS] [get_bd_intf_pins lds_net_input_cdc/S_AXIS]
    connect_bd_intf_net [get_bd_intf_pins lds_net_input_cdc/M_AXIS] [get_bd_intf_pins lds_net_perception_0/S_VIDEO]
    connect_bd_net [get_bd_pins lds_net_perception_0/tap_enable] [get_bd_pins lds_net_video_tap_0/enable_nn]
    connect_bd_net [get_bd_pins lds_net_video_tap_0/branch_cancel_nn] [get_bd_pins lds_net_perception_0/branch_cancel]
    connect_bd_net [get_bd_pins lds_net_video_tap_0/dropped_frames_nn] [get_bd_pins lds_net_perception_0/raw_drops]
    foreach pin {authorize_valid authorize_ready authorize_slot authorize_frame authorize_base authorize_accept display_event_valid display_event_ready display_event_clear display_event_slot display_event_frame} {
        connect_bd_net [get_bd_pins lds_net_display_0/$pin] [get_bd_pins lds_net_perception_0/$pin]
    }
    connect_bd_intf_net [get_bd_intf_pins triple_rgb_compositor_0/m_axis_video] [get_bd_intf_pins lds_net_display_0/S_AXIS]
    connect_bd_intf_net [get_bd_intf_pins lds_net_display_0/M_AXIS] [get_bd_intf_pins v_axi4s_vid_out_0/video_in]
    assign_bd_address
    foreach {cell offset} {lds_net_perception_0 0x43C40000 lds_net_display_0 0x43C50000} {
        assign_bd_address -force -offset $offset -range 64K -target_address_space [get_bd_addr_spaces processing_system7_0/Data] [get_bd_addr_segs -of_objects [get_bd_intf_pins $cell/S_AXI]]
    }
}
# Independent PS-rendered UI reader. Preserve existing NN/capture/mask routes.
set_property CONFIG.NUM_SI 4 [get_bd_cells lds_net_hp0]
if {![llength [get_bd_intf_pins -quiet lds_net_display_0/M_UI_AXI]]} {
    foreach ip [get_ips -quiet -filter {IPDEF =~ xilinx.com:module_ref:lds_net_display:*}] {update_module_reference $ip}
}
if {![llength [get_bd_nets -quiet -of_objects [get_bd_pins lds_net_hp0/S03_ACLK]]]} {
    connect_bd_net [get_bd_pins lds_net_clock/clk_out1] [get_bd_pins lds_net_hp0/S03_ACLK]
    connect_bd_net [get_bd_pins lds_net_reset/peripheral_aresetn] [get_bd_pins lds_net_hp0/S03_ARESETN]
}
if {![llength [get_bd_intf_nets -quiet -of_objects [get_bd_intf_pins lds_net_display_0/M_UI_AXI]]]} {
    connect_bd_intf_net [get_bd_intf_pins lds_net_display_0/M_UI_AXI] [get_bd_intf_pins lds_net_hp0/S03_AXI]
}
assign_bd_address
# proc_sys_reset inherits ACTIVE_LOW on its auxiliary input. An inactive
# auxiliary reset is therefore ONE, whereas mb_debug_sys_rst remains ZERO.
# Repair existing private BDs as well as newly created ones. Never share the
# debug tie-low with this active-low reset input.
foreach key {C_EXT_RESET_HIGH C_AUX_RESET_HIGH} {
    if {[get_property CONFIG.$key [get_bd_cells lds_net_reset]]!=0} {
        set_property CONFIG.$key 0 [get_bd_cells lds_net_reset]
    }
}
if {![llength [get_bd_cells -quiet lds_net_aux_inactive]]} {
    create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:1.1 lds_net_aux_inactive
    set_property -dict [list CONFIG.CONST_WIDTH 1 CONFIG.CONST_VAL 1] [get_bd_cells lds_net_aux_inactive]
}
rl_disconnect lds_net_reset/aux_reset_in
connect_bd_net [get_bd_pins lds_net_aux_inactive/dout] [get_bd_pins lds_net_reset/aux_reset_in]
if {[llength [get_bd_cells -quiet -filter {NAME =~ v5_*}]]} {error "V5 cells remain in the LDS-Net production graph"}
# Source dependency removal happens only in the private project.
foreach f [get_files -quiet *hardware/rtl/v5/*.v] {remove_files $f}
set_property -dict [list CONFIG.LEFT_Y 300 CONFIG.DISP_Y 300 CONFIG.RIGHT_Y 300] [get_bd_cells triple_rgb_compositor_0]
validate_bd_design
if {[get_property CONFIG.C_EXT_RESET_HIGH [get_bd_cells lds_net_reset]]!=0} {error "NN reset polarity did not propagate from active-low PS reset"}
if {[get_property CONFIG.C_AUX_RESET_HIGH [get_bd_cells lds_net_reset]]!=0 || [get_property CONFIG.CONST_VAL [get_bd_cells lds_net_aux_inactive]]!=1} {error "NN auxiliary reset must be inactive: active-low input tied HIGH"}
save_bd_design
puts "PASS: isolated LDS-Net BD: physical-left tap, autonomous CNN/mask, HP3/HP0, independent NN MMCM; retained SGM/video"
