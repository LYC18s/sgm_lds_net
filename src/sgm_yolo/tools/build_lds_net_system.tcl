# Private complete-system build. Never publishes a bitstream/HWH or modifies
# the production source BD. The final packager must separately verify all gates.
set root [file normalize [file join [file dirname [info script]] ..]]
set build [file join $root build lds_net_system]
set mode bd
if {[llength $argv]} {set mode [lindex $argv 0]}
if {$mode ni {bd synth route}} {error "Expected bd/synth/route"}
source [file join $root tools vivado_threads.tcl]
if {[file exists [file join $build lds_net_system.xpr]]} {
    open_project [file join $build lds_net_system.xpr]
} else {
    create_project -force lds_net_system $build -part xc7z020clg400-2
    set_property target_language Verilog [current_project]
    set_property ip_repo_paths [file join $root hardware vivado_2017 repo] [current_project]
    set_property XPM_LIBRARIES {XPM_CDC XPM_MEMORY XPM_FIFO} [current_project]
    update_ip_catalog
    foreach folder {sgm lds_net} {add_files -norecurse [glob [file join $root hardware rtl $folder *.v]]}
    add_files -norecurse [file join $root hardware rtl triple_rgb_compositor.v]
    import_files -norecurse [file join $root hardware vivado_sgm system system.bd]
    add_files -fileset constrs_1 -norecurse [glob [file join $root hardware vivado_sgm constraints *.xdc]]
    set_property USED_IN_SYNTHESIS false [get_files */runtime_hdmi.xdc]
    set_property PROCESSING_ORDER LATE [get_files */runtime_hdmi.xdc]
}
foreach folder {sgm lds_net} {
    foreach f [glob [file join $root hardware rtl $folder *.v]] {
        if {![llength [get_files -quiet $f]]} {add_files -norecurse $f}
    }
}
foreach f [get_files -quiet *display_area_20261023/display_synth.dcp] {remove_files $f}
foreach f [glob [file join $root hardware vivado_sgm constraints *.xdc]] {
    if {![llength [get_files -quiet $f]]} {add_files -fileset constrs_1 -norecurse $f}
}
set_property PROCESSING_ORDER LATE [get_files */lds_net_cdc.xdc]
set_property USED_IN_SYNTHESIS false [get_files */lds_net_cdc.xdc]
# A failed first integration may leave partially created cells in memory or in
# a project autosave. Reimport the known source graph into THIS private file.
set private_bd [get_property NAME [get_files */system.bd]]
if {![file exists [file join $build bd_complete.txt]]} {
    if {[string first "[file normalize $build]/" [file normalize $private_bd]]!=0} {error "Refusing to replace a BD outside the private build"}
    file copy -force [file join $root hardware vivado_sgm system system.bd] $private_bd
}
open_bd_design [get_files */system.bd]
source [file join $root tools integrate_lds_net_bd.tcl]
# Never trust source timestamps alone: module_ref can silently keep a DCP
# whose interface is unchanged while its register/algorithm body has changed.
if {![info exists ::env(LDS_NET_BUILD_PYTHON)]} {error "Use tools/run_ui100.py system so Python/cache/temp paths remain workspace-local"}
set ooc_python $::env(LDS_NET_BUILD_PYTHON)
set ooc_helper [file join $root tools lds_net_ooc_sources.py]
set ooc_dirty [split [string trim [exec $ooc_python -I -B $ooc_helper plan]]]
foreach ip $ooc_dirty {
    if {$ip eq ""} {continue}
    puts "INFO: refreshing content-unverified module reference $ip"
    update_module_reference [get_ips $ip]
}
validate_bd_design
set marker [open [file join $build bd_complete.txt] w]
puts $marker "LDS-Net BD validation completed; no production image published"
close $marker
update_compile_order -fileset sources_1
write_bd_tcl -force [file join $build lds_net_system.tcl]
if {$mode ne "bd"} {
    generate_target all [get_files */system.bd]
    make_wrapper -files [get_files */system.bd] -top -import
    set_property top system_wrapper [current_fileset]
    update_compile_order -fileset sources_1
    create_ip_run [get_files */system.bd]
    # Replan after interface/default propagation and force each stale leaf run.
    save_bd_design
    set ooc_dirty [split [string trim [exec $ooc_python -I -B $ooc_helper plan]]]
    foreach ip $ooc_dirty {
        if {$ip eq ""} {continue}
        set leaf [get_runs -quiet ${ip}_synth_1]
        if {[llength $leaf]!=1} {error "Missing module-reference synthesis run: $ip"}
        reset_run $leaf
    }
    foreach failed [get_runs -quiet -filter {STATUS =~ *ERROR* || STATUS =~ *Running*}] {reset_run $failed}
    # Retain the validated synthesis strategies; cache invalidation is handled
    # uniformly by the complete dependency hashes above, including the DMA.
    foreach name {system_lds_net_perception_0_0 system_lds_net_display_0_0 system_lds_net_video_tap_0_0} {
        set run [get_runs -quiet ${name}_synth_1]
        if {$name eq "system_lds_net_display_0_0" && [llength $run]} {
            set_property strategy Flow_AreaOptimized_high $run
            set_property STEPS.SYNTH_DESIGN.ARGS.FLATTEN_HIERARCHY full $run
            set_property STEPS.SYNTH_DESIGN.ARGS.CONTROL_SET_OPT_THRESHOLD 16 $run
        }
        if {$name eq "system_lds_net_perception_0_0" && [llength $run]} {
            # Retain the existing validated NN synthesis configuration; do not
            # reapply the older area experiment while rebuilding only display.
            set_property strategy {Vivado Synthesis Defaults} $run
            set_property STEPS.SYNTH_DESIGN.ARGS.FLATTEN_HIERARCHY rebuilt $run
            set_property STEPS.SYNTH_DESIGN.ARGS.CONTROL_SET_OPT_THRESHOLD 16 $run
            set_property STEPS.SYNTH_DESIGN.ARGS.MAX_DSP 138 $run
        }
    }
    reset_run synth_1
    launch_runs synth_1 -jobs 1
    wait_on_run synth_1
    if {[get_property PROGRESS [get_runs synth_1]] ne "100%"} {error "LDS-Net full-system synthesis failed"}
    puts [exec $ooc_python -I -B $ooc_helper record]
    open_run synth_1
    report_utilization -file [file join $build utilization_synth.rpt]
    report_utilization -hierarchical -hierarchical_depth 7 -file [file join $build utilization_hierarchy.rpt]
    report_timing_summary -file [file join $build timing_synth.rpt]
    write_checkpoint -force [file join $build lds_net_synth.dcp]
    if {$mode eq "route"} {
        close_design
        set_property strategy Area_Explore [get_runs impl_1]
        set_property STEPS.PLACE_DESIGN.ARGS.DIRECTIVE Default [get_runs impl_1]
        set_property STEPS.PLACE_DESIGN.TCL.PRE [file join $root tools lds_net_compact_control_sets.tcl] [get_runs impl_1]
        set_property STEPS.ROUTE_DESIGN.ARGS.DIRECTIVE Explore [get_runs impl_1]
        set_property STEPS.PHYS_OPT_DESIGN.IS_ENABLED true [get_runs impl_1]
        set_property STEPS.PHYS_OPT_DESIGN.ARGS.DIRECTIVE AggressiveExplore [get_runs impl_1]
        launch_runs impl_1 -to_step route_design -jobs 1
        wait_on_run impl_1
        if {![string match "route_design Complete*" [get_property STATUS [get_runs impl_1]]]} {error "LDS-Net full-system routing did not complete: [get_property STATUS [get_runs impl_1]]"}
        open_run impl_1
        report_utilization -file [file join $build utilization_routed.rpt]
        report_timing_summary -file [file join $build timing_routed.rpt]
        report_cdc -details -file [file join $build cdc.rpt]
        report_drc -file [file join $build drc.rpt]
        # No write_bitstream here: negative setup/hold, resource overuse, CDC
        # issues and the known HDMI constraints require explicit checked gates.
    }
}
close_project
puts "PASS: LDS-Net private system $mode stage completed; production artifacts unchanged"
exit
