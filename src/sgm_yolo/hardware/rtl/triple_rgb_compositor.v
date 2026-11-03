`timescale 1ns / 1ps

// =============================================================================
// triple_rgb_compositor
// -----------------------------------------------------------------------------
// 1920x1080 AXI4-Stream RGB compositor for this project.
//
// Full-size 640x480 panels on a 1920x1080 canvas. Coordinates are parameters
// so the original RGB placement can be retained without changing raster size.
// Defaults: left (0,0), right (640,0), disparity (1280,0).
//
// The three input streams come from MM2S VDMAs.  The disparity stream already
// contains 0..255 visualization data in DDR. A fixed palette maps the upper
// six disparity bits using a nonlinear Jet LUT (far blue, near red).
// Output SOF/TLAST are regenerated from the 1920x1080
// canvas counters; input SOF/TLAST are intentionally not forwarded.
// =============================================================================
module triple_rgb_compositor #(
    parameter LEFT_X=0, LEFT_Y=0,
    parameter RIGHT_X=640, RIGHT_Y=0,
    parameter DISP_X=640, DISP_Y=0,
    parameter RIGHT_ENABLE=0,
    parameter SWAP_RGB_INPUTS=0,
    parameter COLOR_DISPARITY=1
) (
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME aclk, ASSOCIATED_BUSIF s0_axis_video:s1_axis_video:s2_axis_video:m_axis_video, ASSOCIATED_RESET aresetn" *)
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 aclk CLK" *)
    input  wire        aclk,

    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME aresetn, POLARITY ACTIVE_LOW" *)
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 aresetn RST" *)
    input  wire        aresetn,

    // -------------------------------------------------------------------------
    // Input 0: left camera, RGB888
    // -------------------------------------------------------------------------
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME s0_axis_video, TDATA_NUM_BYTES 3, HAS_TKEEP 1, HAS_TLAST 1, HAS_TREADY 1, HAS_TUSER 1, TUSER_WIDTH 1" *)
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s0_axis_video TDATA" *)
    input  wire [23:0] s0_axis_video_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s0_axis_video TKEEP" *)
    input  wire [2:0]  s0_axis_video_tkeep,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s0_axis_video TLAST" *)
    input  wire        s0_axis_video_tlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s0_axis_video TUSER" *)
    input  wire        s0_axis_video_tuser,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s0_axis_video TVALID" *)
    input  wire        s0_axis_video_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s0_axis_video TREADY" *)
    output wire        s0_axis_video_tready,

    // -------------------------------------------------------------------------
    // Input 1: right camera, RGB888
    // -------------------------------------------------------------------------
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME s1_axis_video, TDATA_NUM_BYTES 3, HAS_TKEEP 1, HAS_TLAST 1, HAS_TREADY 1, HAS_TUSER 1, TUSER_WIDTH 1" *)
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s1_axis_video TDATA" *)
    input  wire [23:0] s1_axis_video_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s1_axis_video TKEEP" *)
    input  wire [2:0]  s1_axis_video_tkeep,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s1_axis_video TLAST" *)
    input  wire        s1_axis_video_tlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s1_axis_video TUSER" *)
    input  wire        s1_axis_video_tuser,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s1_axis_video TVALID" *)
    input  wire        s1_axis_video_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s1_axis_video TREADY" *)
    output wire        s1_axis_video_tready,

    // -------------------------------------------------------------------------
    // Input 2: disparity, 8-bit gray from DDR MM2S VDMA
    // -------------------------------------------------------------------------
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME s2_axis_video, TDATA_NUM_BYTES 1, HAS_TKEEP 1, HAS_TLAST 1, HAS_TREADY 1, HAS_TUSER 1, TUSER_WIDTH 1" *)
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s2_axis_video TDATA" *)
    input  wire [7:0]  s2_axis_video_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s2_axis_video TKEEP" *)
    input  wire [0:0]  s2_axis_video_tkeep,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s2_axis_video TLAST" *)
    input  wire        s2_axis_video_tlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s2_axis_video TUSER" *)
    input  wire        s2_axis_video_tuser,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s2_axis_video TVALID" *)
    input  wire        s2_axis_video_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s2_axis_video TREADY" *)
    output wire        s2_axis_video_tready,

    // -------------------------------------------------------------------------
    // Output: full 1920x1080 RGB888 stream
    // -------------------------------------------------------------------------
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME m_axis_video, TDATA_NUM_BYTES 3, HAS_TKEEP 1, HAS_TLAST 1, HAS_TREADY 1, HAS_TUSER 1, TUSER_WIDTH 1" *)
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m_axis_video TDATA" *)
    output wire [23:0] m_axis_video_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m_axis_video TKEEP" *)
    output wire [2:0]  m_axis_video_tkeep,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m_axis_video TLAST" *)
    output wire        m_axis_video_tlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m_axis_video TUSER" *)
    output wire        m_axis_video_tuser,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m_axis_video TVALID" *)
    output wire        m_axis_video_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m_axis_video TREADY" *)
    input  wire        m_axis_video_tready
);

    localparam integer OUTPUT_WIDTH  = 1920;
    localparam integer OUTPUT_HEIGHT = 1080;
    localparam integer PANEL_WIDTH   = 640;
    localparam integer PANEL_HEIGHT  = 480;


    reg [10:0] x_pos;
    reg [10:0] y_pos;

    wire in_left =
        (x_pos >= LEFT_X) && (x_pos < LEFT_X + PANEL_WIDTH) &&
        (y_pos >= LEFT_Y) && (y_pos < LEFT_Y + PANEL_HEIGHT);

    wire in_right = RIGHT_ENABLE &&
        (x_pos >= RIGHT_X) && (x_pos < RIGHT_X + PANEL_WIDTH) &&
        (y_pos >= RIGHT_Y) && (y_pos < RIGHT_Y + PANEL_HEIGHT);

    wire in_disp =
        (x_pos >= DISP_X) && (x_pos < DISP_X + PANEL_WIDTH) &&
        (y_pos >= DISP_Y) && (y_pos < DISP_Y + PANEL_HEIGHT);

    // Fixed range avoids introducing flicker through per-frame normalization.
    // HDMI rgb2dvi consumes {red, green, blue}. There is no invalid-match mask.
    wire [5:0] disparity = s2_axis_video_tdata[7:2];
    // Metric color anchors: 1m orange, 2.5m yellow, 5m green, 10m cyan; interpolated RGB.
    // HDMI byte order is {R,G,B}; fixed scaling avoids color flicker.
    function [23:0] jet_rgb;
        input [5:0] d;
        begin
            case (d)
                6'd0: jet_rgb=24'h000080;
                6'd1: jet_rgb=24'h0001ff;
                6'd2: jet_rgb=24'h00c0ff;
                6'd3: jet_rgb=24'h00fffe;
                6'd4: jet_rgb=24'h00ff7f;
                6'd5: jet_rgb=24'h00ff33;
                6'd6: jet_rgb=24'h01ff00;
                6'd7: jet_rgb=24'h4aff00;
                6'd8: jet_rgb=24'h80ff00;
                6'd9: jet_rgb=24'habff00;
                6'd10: jet_rgb=24'hccff00;
                6'd11: jet_rgb=24'he8ff00;
                6'd12: jet_rgb=24'hffff00;
                6'd13: jet_rgb=24'hffee00;
                6'd14: jet_rgb=24'hffe000;
                6'd15: jet_rgb=24'hffd400;
                6'd16: jet_rgb=24'hffca00;
                6'd17: jet_rgb=24'hffc100;
                6'd18: jet_rgb=24'hffb800;
                6'd19: jet_rgb=24'hffb100;
                6'd20: jet_rgb=24'hffaa00;
                6'd21: jet_rgb=24'hffa400;
                6'd22: jet_rgb=24'hff9f00;
                6'd23: jet_rgb=24'hff9a00;
                6'd24: jet_rgb=24'hff9500;
                6'd25: jet_rgb=24'hff9100;
                6'd26: jet_rgb=24'hff8d00;
                6'd27: jet_rgb=24'hff8900;
                6'd28: jet_rgb=24'hff8600;
                6'd29: jet_rgb=24'hff8300;
                6'd30: jet_rgb=24'hff8000;
                6'd31: jet_rgb=24'hff7700;
                6'd32: jet_rgb=24'hff7000;
                6'd33: jet_rgb=24'hff6800;
                6'd34: jet_rgb=24'hff6200;
                6'd35: jet_rgb=24'hff5b00;
                6'd36: jet_rgb=24'hff5500;
                6'd37: jet_rgb=24'hff4f00;
                6'd38: jet_rgb=24'hff4a00;
                6'd39: jet_rgb=24'hff4500;
                6'd40: jet_rgb=24'hff4000;
                6'd41: jet_rgb=24'hff3b00;
                6'd42: jet_rgb=24'hff3700;
                6'd43: jet_rgb=24'hff3200;
                6'd44: jet_rgb=24'hff2e00;
                6'd45: jet_rgb=24'hff2a00;
                6'd46: jet_rgb=24'hff2700;
                6'd47: jet_rgb=24'hff2300;
                6'd48: jet_rgb=24'hff2000;
                6'd49: jet_rgb=24'hff1c00;
                6'd50: jet_rgb=24'hff1900;
                6'd51: jet_rgb=24'hff1600;
                6'd52: jet_rgb=24'hff1300;
                6'd53: jet_rgb=24'hff1100;
                6'd54: jet_rgb=24'hff0e00;
                6'd55: jet_rgb=24'hff0b00;
                6'd56: jet_rgb=24'hff0900;
                6'd57: jet_rgb=24'hff0700;
                6'd58: jet_rgb=24'hff0400;
                6'd59: jet_rgb=24'hff0200;
                6'd60: jet_rgb=24'hff0000;
                6'd61: jet_rgb=24'hff0000;
                6'd62: jet_rgb=24'hff0000;
                6'd63: jet_rgb=24'hff0000;




            endcase
        end
    endfunction
    wire [23:0] disp_rgb = COLOR_DISPARITY ? jet_rgb(disparity) :
                         {s2_axis_video_tdata, s2_axis_video_tdata, s2_axis_video_tdata};

    // The left panel must use the same logical left/reference camera as SGM.
    wire [23:0] left_rgb = SWAP_RGB_INPUTS ? s1_axis_video_tdata : s0_axis_video_tdata;
    wire [23:0] right_rgb = SWAP_RGB_INPUTS ? s0_axis_video_tdata : s1_axis_video_tdata;
    wire left_valid = SWAP_RGB_INPUTS ? s1_axis_video_tvalid : s0_axis_video_tvalid;
    wire right_valid = SWAP_RGB_INPUTS ? s0_axis_video_tvalid : s1_axis_video_tvalid;

    assign m_axis_video_tdata =
        in_left  ? left_rgb :
        in_right ? right_rgb :
        in_disp  ? disp_rgb             :
                   24'h000000;

    assign m_axis_video_tkeep = 3'b111;

    // In the active panel area, output valid follows the selected input VDMA.
    // In the black background, pixels are synthesized locally and are always
    // valid while reset is released.
    assign m_axis_video_tvalid =
        aresetn &&
        (in_left  ? left_valid :
         in_right ? right_valid :
         in_disp  ? s2_axis_video_tvalid :
                    1'b1);

    // Generate output frame/line markers from the 1920x1080 canvas itself.
    assign m_axis_video_tuser = (x_pos == 0) && (y_pos == 0);
    assign m_axis_video_tlast = (x_pos == OUTPUT_WIDTH - 1);

    // Only the currently displayed panel is allowed to advance its input VDMA.
    assign s0_axis_video_tready =
        aresetn && (SWAP_RGB_INPUTS ? in_right : in_left) && m_axis_video_tready;

    assign s1_axis_video_tready =
        aresetn && (SWAP_RGB_INPUTS ? in_left : in_right) && m_axis_video_tready;

    assign s2_axis_video_tready =
        aresetn && in_disp && m_axis_video_tready;

    // Canvas coordinates advance only on a successful output AXIS transfer.
    always @(posedge aclk) begin
        if (!aresetn) begin
            x_pos <= 11'd0;
            y_pos <= 11'd0;
        end
        else if (m_axis_video_tvalid && m_axis_video_tready) begin
            if (x_pos == OUTPUT_WIDTH - 1) begin
                x_pos <= 11'd0;
                if (y_pos == OUTPUT_HEIGHT - 1)
                    y_pos <= 11'd0;
                else
                    y_pos <= y_pos + 11'd1;
            end
            else begin
                x_pos <= x_pos + 11'd1;
            end
        end
    end

    // Input sideband values are not forwarded.  They are intentionally consumed
    // only so synthesis/lint can see that the ports are deliberate.
    wire unused_sidebands = &{
        1'b0,
        s0_axis_video_tkeep, s0_axis_video_tlast, s0_axis_video_tuser,
        s1_axis_video_tkeep, s1_axis_video_tlast, s1_axis_video_tuser,
        s2_axis_video_tkeep, s2_axis_video_tlast, s2_axis_video_tuser
    };

endmodule
