# Export only an actually routed, checked LDS-Net design. Production remains
# untouched here; the Python packager checks evidence before publication.
set root [file normalize [file join [file dirname [info script]] ..]]
set build [file join $root build lds_net_system]
set out [file join $build export]
file mkdir $out
source [file join $root tools vivado_threads.tcl]
open_project [file join $build lds_net_system.xpr]
set run_status [get_property STATUS [get_runs impl_1]]
if {[string match "route_design Complete*" $run_status] || [string match "write_bitstream Complete*" $run_status]} {
    open_run impl_1
} else {
    # Constraint-only corrections can mark a completed run out of date. Reuse
    # its real routed netlist only if every hardware input still predates it.
    set routed [file join $build lds_net_system.runs impl_1 system_wrapper_routed.dcp]
    if {$run_status ne "Out-of-date" || ![file exists $routed]} {error "A completed routed implementation is required: $run_status"}
    set hardware_inputs [concat [glob [file join $root hardware rtl lds_net *.v]] [glob [file join $root hardware rtl sgm *.v]] \
        [glob -nocomplain [file join $root hardware calibration *]] [list [file join $root hardware rtl triple_rgb_compositor.v] \
        [file join $build lds_net_system.srcs sources_1 bd system system.bd]]]
    foreach path $hardware_inputs {
        if {[file isfile $path] && [file mtime $path]>[file mtime $routed]} {error "Hardware changed after routed checkpoint: $path"}
    }
    puts "INFO: reusing completed routed checkpoint after constraint-only correction; all hardware inputs still predate it"
    open_checkpoint $routed
}
read_xdc [file join $root hardware vivado_sgm constraints lds_net_cdc.xdc]
if {[llength [get_cells -quiet system_i/v5_*]]} {error "V5 remains in the implementation"}
foreach name {lds_net_perception_0 lds_net_display_0 lds_net_video_tap_0 stereo_sgm_0} {
    if {![llength [get_cells -quiet system_i/$name]]} {error "Missing required system cell $name"}
}
set nn [get_clocks -quiet -filter {NAME =~ clk_out1*lds_net_clock*}]
set sgm [get_clocks -quiet clk_fpga_3]
if {[llength $nn]!=1 || [llength $sgm]!=1} {error "Missing unique NN/SGM clock"}
foreach group {rl_first rl_to_nn rl_to_video rl_gray rl_gray_first} {
    if {![llength [set $group]]} {error "Missing constrained CDC endpoint group $group"}
}
if {[llength [get_cells -hier -quiet -filter {NAME =~ *lds_net_display_0*/control_bus/ui_bus_rpc*}]]} {
    foreach group {ui_to_video ui_to_memory ui_bus_tags} {
        if {![llength [set $group]]} {error "Missing constrained UI100 CDC bundle $group"}
    }
}
foreach clock [concat $nn $sgm] {
    if {abs([get_property PERIOD $clock]-10.0)>0.002} {error "NN and SGM must be analyzed at the real 100 MHz clocks"}
}
set strict [get_clocks -filter {NAME != I && NAME != axi_dynclk_0_PXL_CLK_O && NAME != system_i/rgb2dvi_0/U0/SerialClk}]
# Use actual routed delays for the final physical repair. This changes cell
# placement/replication, not RTL arithmetic, clocks or timing requirements.
# Skip it entirely when the required setup paths already pass.
set repair 0
foreach option {-from -to} {
    if {[llength [get_timing_paths -quiet $option $strict -delay_type max -slack_lesser_than 0 -max_paths 1]]} {set repair 1}
}
if {$repair} {
    report_timing -from $strict -max_paths 20 -file [file join $out strict_setup_before_repair.rpt]
    report_timing -to $strict -max_paths 20 -file [file join $out strict_setup_to_before_repair.rpt]
    puts "INFO: refining remaining non-HDMI paths with routed physical optimization"
    phys_opt_design -directive AggressiveExplore
    route_design -directive Explore
    # Retain expensive physical work and its exact failing paths if a further
    # targeted correction is needed; never publish this as a passed image.
    write_checkpoint -force [file join $out lds_net_repair_candidate.dcp]
    report_timing -from $strict -max_paths 20 -file [file join $out strict_setup_after_repair.rpt]
    report_timing -to $strict -max_paths 20 -file [file join $out strict_setup_to_after_repair.rpt]
}
foreach option {-from -to} {
    if {[llength [get_timing_paths -quiet $option $strict -delay_type max -slack_lesser_than 0 -max_paths 1]]} {error "Non-HDMI setup violation; no board image exported"}
}
if {[llength [get_timing_paths -quiet -delay_type min -slack_lesser_than 0 -max_paths 1]]} {error "Hold violation; no board image exported"}
set pulse [report_pulse_width -clocks $strict -all_violators -no_header -return_string]
if {[regexp {(^|[[:space:]])-[0-9]+\.[0-9]+} $pulse]} {error "Non-HDMI period/pulse violation; no board image exported"}
set skew [report_bus_skew -return_string]
set f [open [file join $out bus_skew.rpt] w]; puts $f $skew; close $f
if {[regexp {Slack\s+\(VIOLATED\)} $skew]} {error "CDC bus-skew violation; no board image exported"}
report_timing -from $nn -to $nn -max_paths 10 -file [file join $out lds_net_setup.rpt]
report_timing -from $sgm -to $sgm -max_paths 10 -file [file join $out sgm_setup.rpt]
report_timing_summary -file [file join $out timing.rpt]
report_utilization -file [file join $out utilization.rpt]
report_cdc -details -file [file join $out cdc.rpt]
report_drc -file [file join $out drc.rpt]
report_exceptions -coverage -file [file join $out exceptions.rpt]
set nn_worst [get_timing_paths -from $nn -to $nn -max_paths 1]
set sgm_worst [get_timing_paths -from $sgm -to $sgm -max_paths 1]
set ui_control_hz 0
set ui_setup_ns 0
set ui_regs [get_cells -hier -quiet -filter {NAME =~ *lds_net_display_0*/ui/shadow_*_reg* && REF_NAME =~ FD*}]
set ui_bus [get_cells -hier -quiet -filter {NAME =~ *lds_net_display_0*/control_bus/ui_bus_rpc*}]
if {[llength $ui_bus]} {
    if {![llength $ui_regs]} {error "100 MHz UI control registers missing"}
    foreach cell $ui_regs {
        set clock [get_clocks -of_objects [get_pins $cell/C]]
        if {[llength $clock]!=1 || abs([get_property PERIOD $clock]-10.0)>0.002} {error "UI control register not at 100 MHz"}
    }
    set ui_control_hz 100000000
    set ui_cells [get_cells -hier -filter {NAME =~ *lds_net_display_0*/ui/* && (REF_NAME =~ FD* || REF_NAME =~ RAMB*)}]
    report_timing -from $ui_cells -to $ui_cells -max_paths 10 -file [file join $out ui_setup.rpt]
    set ui_setup_ns [get_property SLACK [get_timing_paths -from $ui_cells -to $ui_cells -max_paths 1]]
    if {$ui_setup_ns<0} {error "UI routed setup violation; no board image exported"}
}
set f [open [file join $out timing.json] w]
puts $f "{\"nn_setup_ns\":[get_property SLACK $nn_worst],\"sgm_setup_ns\":[get_property SLACK $sgm_worst],\"ui_setup_ns\":$ui_setup_ns,\"ui_control_hz\":$ui_control_hz,\"non_hdmi_setup_passed\":true,\"all_hold_passed\":true,\"bus_skew_passed\":true,\"timing_closed\":false,\"status\":\"LDS_NET_HDMI_1080P60_TRIAL\",\"board_tested\":false}"
close $f
write_checkpoint -force [file join $out lds_net_export_routed.dcp]
# Preserve the existing runtime HDMI profile and its disclosed timing limits.
# No period, hold, CDC or DRC violation is concealed by the export process.
write_bitstream -force [file join $out sgm_yolo.bit]
set handoffs [get_files -all -quiet */system.hwh]
set handoff [file join $build lds_net_system.gen sources_1 bd system hw_handoff system.hwh]
if {![file exists $handoff]} {
    set handoff [file join $build folded32.gen sources_1 bd system hw_handoff system.hwh]
}
if {![file exists $handoff]} {error "Matching generated HWH missing"}
file copy -force $handoff [file join $out sgm_yolo.hwh]
# The checked bitstream is exported explicitly above, rather than through the
# project's write_bitstream run. Export the optional hardware handoff without
# asking Vivado to fetch a second bitstream from that unrelated run state.
write_hw_platform -fixed -force [file join $out sgm_yolo.xsa]
close_project
puts "PASS: routed LDS-Net bit/HWH exported privately; HDMI timing trial and board testing remain disclosed"
exit
