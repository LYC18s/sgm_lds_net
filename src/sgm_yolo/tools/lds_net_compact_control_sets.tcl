# Reuse the exact equivalent mapping used by full-system resource verification.
# This pre-placement hook never publishes a bitstream or modifies RTL.
set root [file normalize [file join [file dirname [info script]] ..]]
set out [file join $root build control_sets]
file mkdir $out
report_control_sets -file [file join $out control_sets_before.rpt]
source [file join $root tools display_area_pack_controls.tcl]
report_control_sets -file [file join $out control_sets_after.rpt]
report_utilization -file [file join $out compact_opt_utilization.rpt]
