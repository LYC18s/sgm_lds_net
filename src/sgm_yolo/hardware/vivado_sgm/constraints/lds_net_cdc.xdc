# Handshake controls use two synchronizer stages. Multibit bundles are held
# until acknowledgement; constrain their physical propagation to less than
# the two destination-clock settling cycles. No functional datapath is waived.
set rl_nn_clock [get_clocks -quiet -filter {NAME =~ clk_out1*lds_net_clock*}]
set rl_video_clock [get_clocks -quiet clk_fpga_1]
    set rl_first [get_cells -hier -quiet -regexp {.*lds_net_(display|video_tap)_0/.*(ack_sync0|busy_sync0|req_sync0|ack0|request0|cancel0|enable0)_reg.*}]
    set_false_path -to [get_pins -of_objects $rl_first -filter {REF_PIN_NAME == D}]
    set rl_to_nn [get_cells -hier -quiet -regexp {.*lds_net_display_0/.*(payload|group_hold|row_hold|base_hold|frame_hold)_reg.*}]
    set rl_to_video [get_cells -hier -quiet -regexp {.*lds_net_display_0/.*(response|nn_group|nn_row|nn_frame|nn_error)_reg.*}]
        set_max_delay -datapath_only 10.000 -from $rl_to_nn -to $rl_nn_clock
        set_false_path -hold -from $rl_to_nn -to $rl_nn_clock
        set_max_delay -datapath_only 7.000 -from $rl_to_video -to $rl_video_clock
        set_false_path -hold -from $rl_to_video -to $rl_video_clock
    set rl_gray [get_cells -hier -quiet -filter {NAME =~ *lds_net_video_tap_0*/gray_source_reg*}]
    set rl_gray_first [get_cells -hier -quiet -filter {NAME =~ *lds_net_video_tap_0*/drops0_reg*}]
        set_max_delay -datapath_only 7.000 -from $rl_gray -to $rl_gray_first
        set_false_path -hold -from $rl_gray -to $rl_gray_first
        set_bus_skew 7.000 -from $rl_gray -to $rl_gray_first
# UIF1 control/status RPCs start in the 100 MHz domain, opposite to row fetch.
# Bundles remain stable until the four-phase acknowledgement retires them.
set ui_to_video [get_cells -hier -quiet -regexp {.*lds_net_display_0/.*ui/(commit_rpc|status_rpc)/payload_reg.*}]
set ui_to_memory [get_cells -hier -quiet -regexp {.*lds_net_display_0/.*ui/(commit_rpc|status_rpc)/response_reg.*}]
set_max_delay -datapath_only 7.000 -from $ui_to_video -to $rl_video_clock
set_false_path -hold -from $ui_to_video -to $rl_video_clock
set_max_delay -datapath_only 10.000 -from $ui_to_memory -to $rl_nn_clock
set_false_path -hold -from $ui_to_memory -to $rl_nn_clock
# Synthesis merges payload[48] with the identical UI transaction-kind register.
# It is the same held request bundle, released only after the RPC acknowledgement.
set ui_bus_tags [get_cells -hier -quiet -regexp {.*lds_net_display_0/.*control_bus/ui_writing_reg.*}]
set_max_delay -datapath_only 10.000 -from $ui_bus_tags -to $rl_nn_clock
set_false_path -hold -from $ui_bus_tags -to $rl_nn_clock
