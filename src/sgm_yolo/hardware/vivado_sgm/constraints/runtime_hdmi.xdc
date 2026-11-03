# User-tested runtime 1080p60 profile programmed by dual_ov5640/clock.py:
# 100 MHz reference, feedback 52, input divider 7, output divider 1.
# Serial = 742.857143 MHz; BUFR /5 pixel = 148.571429 MHz.
# This exceeds the serializer's minimum clock-period specification on -2.
# Keep the actual clocks in the timing report; do not hide these violations.
# Apply late and only during implementation, after the vendor clock XDCs.
# Keep vendor clock names so their existing CDC constraints still apply.
set hdmi_mmcm [get_cells -hier -filter {NAME =~ *axi_dynclk_0* && REF_NAME =~ MMCME2*}]
# Match the initial primitive configuration as well: this lets Vivado derive
# feedback clocks and MMCM jitter from the same dividers the PS programs.
set_property CLKFBOUT_MULT_F 52.0 $hdmi_mmcm
set_property DIVCLK_DIVIDE 7 $hdmi_mmcm
set_property CLKOUT0_DIVIDE_F 1.0 $hdmi_mmcm
set hdmi_input [get_pins -of_objects $hdmi_mmcm -filter {REF_PIN_NAME == CLKIN1}]
set hdmi_output [get_pins -of_objects $hdmi_mmcm -filter {REF_PIN_NAME == CLKOUT0}]
create_generated_clock -name I -source $hdmi_input -multiply_by 52 -divide_by 7 $hdmi_output
# BUFR divide-by-five is high for two source cycles, low for three.
create_generated_clock -name axi_dynclk_0_PXL_CLK_O -source $hdmi_output -edges {1 5 11} [get_pins system_i/axi_dynclk_0/U0/BUFR_inst/O]
create_generated_clock -name system_i/rgb2dvi_0/U0/SerialClk -source $hdmi_output -divide_by 1 [get_pins system_i/rgb2dvi_0/U0/SerialClk]
