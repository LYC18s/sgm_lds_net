`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// Registered AXI4-Stream video 1-to-2 broadcaster.
//
// This is deliberately simple and robust.  One input beat is buffered and is
// held until BOTH outputs have accepted it.  Maximum service rate is one beat
// every two aclk cycles.  At this project's 142.857 MHz AXIS clock that is
// still >71 Mpixel/s, comfortably above 640x480@60 (18.432 Mpixel/s).
//////////////////////////////////////////////////////////////////////////////////
module axis_video_broadcast2 #(parameter DROP_STALLED_SGM=0) (
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME aclk, ASSOCIATED_BUSIF S_AXIS:M0_AXIS:M1_AXIS, ASSOCIATED_RESET aresetn, FREQ_HZ 142857132" *)
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 aclk CLK" *)
    input  wire        aclk,
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME aresetn, POLARITY ACTIVE_LOW" *)
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 aresetn RST" *)
    input  wire        aresetn,

    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME S_AXIS, TDATA_NUM_BYTES 3, HAS_TKEEP 1, HAS_TLAST 1, HAS_TREADY 1, HAS_TUSER 1, TUSER_WIDTH 1" *)
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TDATA" *)
    input  wire [23:0] s_axis_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TVALID" *)
    input  wire        s_axis_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TREADY" *)
    output wire        s_axis_tready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TUSER" *)
    input  wire        s_axis_tuser,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TLAST" *)
    input  wire        s_axis_tlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TKEEP" *)
    input  wire [2:0]  s_axis_tkeep,

    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME M0_AXIS, TDATA_NUM_BYTES 3, HAS_TKEEP 1, HAS_TLAST 1, HAS_TREADY 1, HAS_TUSER 1, TUSER_WIDTH 1" *)
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M0_AXIS TDATA" *)
    output wire [23:0] m0_axis_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M0_AXIS TVALID" *)
    output wire        m0_axis_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M0_AXIS TREADY" *)
    input  wire        m0_axis_tready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M0_AXIS TUSER" *)
    output wire        m0_axis_tuser,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M0_AXIS TLAST" *)
    output wire        m0_axis_tlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M0_AXIS TKEEP" *)
    output wire [2:0]  m0_axis_tkeep,

    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME M1_AXIS, TDATA_NUM_BYTES 3, HAS_TKEEP 1, HAS_TLAST 1, HAS_TREADY 1, HAS_TUSER 1, TUSER_WIDTH 1" *)
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M1_AXIS TDATA" *)
    output wire [23:0] m1_axis_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M1_AXIS TVALID" *)
    output wire        m1_axis_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M1_AXIS TREADY" *)
    input  wire        m1_axis_tready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M1_AXIS TUSER" *)
    output wire        m1_axis_tuser,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M1_AXIS TLAST" *)
    output wire        m1_axis_tlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M1_AXIS TKEEP" *)
    output wire [2:0]  m1_axis_tkeep
);

    generate if (DROP_STALLED_SGM) begin : live_camera
        // The sensor cannot pause DVP. M0 capture therefore never waits for
        // SGM. On an M1 stall, discard the rest of that input frame and seek
        // the next SOF; a pending M1 beat remains stable until accepted.
        reg [28:0] capture_data, sgm_data;
        reg capture_valid, sgm_valid, seek_sof;
        assign s_axis_tready=aresetn && (!capture_valid || m0_axis_tready);
        assign {m0_axis_tdata,m0_axis_tkeep,m0_axis_tlast,m0_axis_tuser}=capture_data;
        assign {m1_axis_tdata,m1_axis_tkeep,m1_axis_tlast,m1_axis_tuser}=sgm_data;
        assign m0_axis_tvalid=capture_valid && aresetn;
        assign m1_axis_tvalid=sgm_valid && aresetn;
        wire input_fire=s_axis_tvalid && s_axis_tready;
        wire sgm_room=!sgm_valid || m1_axis_tready;
        always @(posedge aclk or negedge aresetn) begin
            if(!aresetn) begin
                capture_data<=0; sgm_data<=0;
                capture_valid<=0; sgm_valid<=0; seek_sof<=1;
            end else begin
                if(m0_axis_tready) capture_valid<=0;
                if(m1_axis_tready) sgm_valid<=0;
                if(input_fire) begin
                    capture_data<={s_axis_tdata,s_axis_tkeep,s_axis_tlast,s_axis_tuser};
                    capture_valid<=1;
                    if ((!seek_sof || s_axis_tuser) && sgm_room) begin
                        sgm_data<={s_axis_tdata,s_axis_tkeep,s_axis_tlast,s_axis_tuser};
                        sgm_valid<=1; seek_sof<=0;
                    end else if(!sgm_room) seek_sof<=1;
                end
            end
        end
    end else begin : lossless
    reg [23:0] data_q;
    reg        user_q;
    reg        last_q;
    reg [2:0]  keep_q;
    reg        hold_valid;
    reg        pending0;
    reg        pending1;

    assign s_axis_tready = aresetn && !hold_valid;

    assign m0_axis_tdata  = data_q;
    assign m0_axis_tuser  = user_q;
    assign m0_axis_tlast  = last_q;
    assign m0_axis_tkeep  = keep_q;
    assign m0_axis_tvalid = hold_valid && pending0;

    assign m1_axis_tdata  = data_q;
    assign m1_axis_tuser  = user_q;
    assign m1_axis_tlast  = last_q;
    assign m1_axis_tkeep  = keep_q;
    assign m1_axis_tvalid = hold_valid && pending1;

    wire take0 = m0_axis_tvalid && m0_axis_tready;
    wire take1 = m1_axis_tvalid && m1_axis_tready;

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            data_q     <= 24'd0;
            user_q     <= 1'b0;
            last_q     <= 1'b0;
            keep_q     <= 3'b111;
            hold_valid <= 1'b0;
            pending0   <= 1'b0;
            pending1   <= 1'b0;
        end else begin
            if (!hold_valid) begin
                if (s_axis_tvalid) begin
                    data_q     <= s_axis_tdata;
                    user_q     <= s_axis_tuser;
                    last_q     <= s_axis_tlast;
                    keep_q     <= s_axis_tkeep;
                    hold_valid <= 1'b1;
                    pending0   <= 1'b1;
                    pending1   <= 1'b1;
                end
            end else begin
                if (take0)
                    pending0 <= 1'b0;
                if (take1)
                    pending1 <= 1'b0;

                // Both consumers have accepted this buffered beat.
                if ((take0 || !pending0) && (take1 || !pending1)) begin
                    hold_valid <= 1'b0;
                    pending0   <= 1'b0;
                    pending1   <= 1'b0;
                end
            end
        end
    end

    end endgenerate
endmodule
