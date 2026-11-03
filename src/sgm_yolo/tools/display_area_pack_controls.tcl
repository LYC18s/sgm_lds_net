# Equivalent physical mapping only; RTL/state/clock/reset semantics unchanged.
# Equivalent carry remapping described by AMD UG904:
# https://docs.amd.com/r/en-US/ug904-vivado-implementation/Carry-Optimization
# Release short (at most 8-bit) fixed macros for flexible LUT packing.
# Longer arithmetic retains dedicated fast carry; no clock/reset changes.
set_property CARRY_REMAP 2 [get_cells -hier -filter {REF_NAME == CARRY4 && NAME !~ *lds_net_display_0*/ui/*}]
# The new DDR UI reader has timing-sensitive short burst/reservation adders.
# Keep its dedicated carry chains; compact mapping elsewhere stays unchanged.
set ui_carries [get_cells -hier -quiet -filter {REF_NAME == CARRY4 && NAME =~ *lds_net_display_0*/ui/*}]
if {[llength $ui_carries]} {set_property CARRY_REMAP 0 $ui_carries}
# DMA reservation/occupancy updates remain timing-sensitive even after AR
# staging. Preserve their native carry instead of spreading the short counter
# arithmetic through LUT chains. Other compact arithmetic keeps area mapping.
set dma_counter_carries [get_cells -hier -quiet -filter {REF_NAME == CARRY4 && (NAME =~ */reader/reserved* || NAME =~ */reader/used*)}]
if {[llength $dma_counter_carries]} {set_property CARRY_REMAP 0 $dma_counter_carries}
opt_design -carry_remap -srl_remap_modes {{max_depth_srl_to_ffs 4}}
proc driver_ref {net} {
    set drivers [get_cells -quiet -of_objects [get_pins -quiet -of_objects $net -filter {DIRECTION == OUT}]]
    if {[llength $drivers]!=1} {return unknown}
    return [get_property REF_NAME $drivers]
}
set groups [dict create]
foreach cell [get_cells -hier -filter {REF_NAME == FDRE}] {
    if {[string match *lds_net_display_0*/ui/* [get_property NAME $cell]]} {continue}
    set clock [get_property NAME [get_nets -of_objects [get_pins $cell/C]]]
    set enable [get_property NAME [get_nets -of_objects [get_pins $cell/CE]]]
    set reset [get_property NAME [get_nets -of_objects [get_pins $cell/R]]]
    dict lappend groups "$clock|$enable|$reset" [get_property NAME $cell]
}
set changed 0
set changed_groups 0
dict for {key cells} $groups {
    if {[llength $cells]>16} {continue}
    set first [lindex $cells 0]
    set ce [get_nets -of_objects [get_pins $first/CE]]
    set reset [get_nets -of_objects [get_pins $first/R]]
    set extra [expr {([driver_ref $ce] eq "VCC" ? 0 : 2)+([driver_ref $reset] eq "GND" ? 0 : 1)}]
    if {$extra==0} {continue}
    set protected 0
    foreach value [concat [get_property ASYNC_REG [get_cells $cells]] [get_property DONT_TOUCH [get_cells $cells]]] {
        if {$value in {1 TRUE true}} {set protected 1}
    }
    if {$protected} {continue}
    set absorbable 1
    foreach name $cells {
        set cell [get_cells $name]
        if {[get_property ASYNC_REG $cell] in {1 TRUE true} || [get_property DONT_TOUCH $cell] in {1 TRUE true}} {set absorbable 0;break}
        set dnet [get_nets -of_objects [get_pins $name/D]]
        set ref [driver_ref $dnet]
        if {![regexp {^LUT([1-6])$} $ref unused inputs] || $inputs+$extra>6 ||
            [llength [get_pins -quiet -of_objects $dnet -filter {DIRECTION == IN}]]!=1} {set absorbable 0;break}
    }
    # A group of <=3 FFs wastes almost an entire slice. Even a new feedback
    # LUT per FF is bounded below one slice, and lets those FFs join a common
    # clock-only control set. Larger groups require absorption in existing LUTs.
    if {$absorbable || [llength $cells]<=3} {
        set_property CONTROL_SET_REMAP ALL [get_cells $cells]
        incr changed [llength $cells]
        incr changed_groups
    }
}
puts "INFO: equivalent compact mapping: $changed FFs in $changed_groups complete control groups"
opt_design -property_opt_only
opt_design -control_set_merge -merge_equivalent_drivers
report_control_sets -file [file join $out compact_control_sets.rpt]
report_utilization -file [file join $out compact_control_utilization.rpt]
