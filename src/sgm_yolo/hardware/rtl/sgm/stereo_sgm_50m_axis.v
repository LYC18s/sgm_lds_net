`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// Dual-camera AXI4-Stream -> Gray@axis_clk -> Async FIFO -> 50MHz SGM
// -> small result FIFO -> transactional DDR -> AXI4-Stream display
//
// v2 数据路径：
//
//  Camera AXIS @142.857MHz
//          |
//       RGB2Gray
//          |
//   gray+TLAST+TUSER (10bit)
//          |
//   Async FIFO 16384
//          |
//      50MHz domain
//          |
//      stereo sync
//          |
//  Census -> Hamming -> SGM -> WTA -> FIFO258 -> Median
//          |
//   packed disparity FIFO (DDR output stores complete frames)
//          |
//  AXIS @142.857MHz
//
// 左右各有一个 16384x10 输入异步 FIFO。
// 原摄像头 IP 内部的 DVP->AXIS FIFO 保持不变；这里是第二级 CDC，作用不同。
//////////////////////////////////////////////////////////////////////////////////
module stereo_sgm_50m_axis #(
    parameter IMG_WIDTH  = 640,
    parameter IMG_HEIGHT = 480,
    parameter MAX_DISP   = 64,
    // Physical camera 0 is on the right in the user's fixed camera rig.
    // Keep the two CDC FIFOs physical; exchange grayscale at the paired read.
    parameter SWAP_INPUTS = 0,
    // Measured fixed-rig VGA maps; generic test cores may bypass rectification.
    parameter RECTIFY_INPUTS = 0,
    // Transactional output DMA commits complete frames in DDR, not input DDR.
    parameter STREAM_OUTPUT = 0,
    parameter CORE_LANES = 16,
    parameter CORE_PIXEL_II = 8,
    parameter CORE_PATHS = 4,
    parameter MEDIAN_ENABLE = 2,
    parameter INPUT_DEPTH = 8192
)(
    // -------------------------------------------------------------------------
    // Camera / VDMA AXIS domain (~142.857 MHz)
    // -------------------------------------------------------------------------
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME axis_clk, ASSOCIATED_BUSIF S_LEFT_AXIS:S_RIGHT_AXIS:M_DISP_AXIS, ASSOCIATED_RESET axis_aresetn, FREQ_HZ 142857132" *)
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 axis_clk CLK" *)
    input  wire        axis_clk,
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME axis_aresetn, POLARITY ACTIVE_LOW" *)
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 axis_aresetn RST" *)
    input  wire        axis_aresetn,

    // -------------------------------------------------------------------------
    // SGM domain (50 MHz)
    // -------------------------------------------------------------------------
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME sgm_clk, ASSOCIATED_RESET sgm_aresetn, FREQ_HZ 100000000" *)
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 sgm_clk CLK" *)
    input  wire        sgm_clk,
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME sgm_aresetn, POLARITY ACTIVE_LOW" *)
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 sgm_aresetn RST" *)
    input  wire        sgm_aresetn,

    // Left RGB888 AXI4-Stream
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME S_LEFT_AXIS, TDATA_NUM_BYTES 3, HAS_TKEEP 1, HAS_TLAST 1, HAS_TREADY 1, HAS_TUSER 1, TUSER_WIDTH 1" *)
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_LEFT_AXIS TDATA" *)
    input  wire [23:0] s_left_axis_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_LEFT_AXIS TVALID" *)
    input  wire        s_left_axis_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_LEFT_AXIS TREADY" *)
    output wire        s_left_axis_tready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_LEFT_AXIS TUSER" *)
    input  wire        s_left_axis_tuser,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_LEFT_AXIS TLAST" *)
    input  wire        s_left_axis_tlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_LEFT_AXIS TKEEP" *)
    input  wire [2:0]  s_left_axis_tkeep,

    // Right RGB888 AXI4-Stream
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME S_RIGHT_AXIS, TDATA_NUM_BYTES 3, HAS_TKEEP 1, HAS_TLAST 1, HAS_TREADY 1, HAS_TUSER 1, TUSER_WIDTH 1" *)
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_RIGHT_AXIS TDATA" *)
    input  wire [23:0] s_right_axis_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_RIGHT_AXIS TVALID" *)
    input  wire        s_right_axis_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_RIGHT_AXIS TREADY" *)
    output wire        s_right_axis_tready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_RIGHT_AXIS TUSER" *)
    input  wire        s_right_axis_tuser,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_RIGHT_AXIS TLAST" *)
    input  wire        s_right_axis_tlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_RIGHT_AXIS TKEEP" *)
    input  wire [2:0]  s_right_axis_tkeep,

    // 8-bit disparity AXI4-Stream, returned to axis_clk domain
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME M_DISP_AXIS, TDATA_NUM_BYTES 1, HAS_TKEEP 1, HAS_TLAST 1, HAS_TREADY 1, HAS_TUSER 1, TUSER_WIDTH 1" *)
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_DISP_AXIS TDATA" *)
    output wire [7:0]  m_disp_axis_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_DISP_AXIS TVALID" *)
    output wire        m_disp_axis_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_DISP_AXIS TREADY" *)
    input  wire        m_disp_axis_tready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_DISP_AXIS TUSER" *)
    output wire        m_disp_axis_tuser,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_DISP_AXIS TLAST" *)
    output wire        m_disp_axis_tlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_DISP_AXIS TKEEP" *)
    output wire [0:0]  m_disp_axis_tkeep,

    output wire [7:0]  debug_status
);

    initial if(!((CORE_LANES==16 && CORE_PIXEL_II==8) || (CORE_LANES==32 && CORE_PIXEL_II==4)))
        $error("Unsupported SGM lane/II configuration");

    localparam INPUT_FIFO_DEPTH = INPUT_DEPTH;
    localparam INPUT_COUNT_W = $clog2(INPUT_FIFO_DEPTH)+1;

    // XPM asynchronous FIFOs require rst to be synchronous to wr_clk.
    // Either domain can request a reset; release it separately in each
    // FIFO's write domain instead of feeding a foreign reset into its FSM.
    wire fifo_rst_async = (~axis_aresetn) | (~sgm_aresetn);
    wire fifo_axis_rst, fifo_sgm_rst;
    xpm_cdc_async_rst #(.DEST_SYNC_FF(4), .RST_ACTIVE_HIGH(1))
        u_fifo_axis_reset (.src_arst(fifo_rst_async), .dest_clk(axis_clk),
                           .dest_arst(fifo_axis_rst));
    xpm_cdc_async_rst #(.DEST_SYNC_FF(4), .RST_ACTIVE_HIGH(1))
        u_fifo_sgm_reset (.src_arst(fifo_rst_async), .dest_clk(sgm_clk),
                          .dest_arst(fifo_sgm_rst));

    // =========================================================================
    // RGB -> Gray in the fast AXIS domain
    // =========================================================================
    wire left_fifo_full, left_fifo_prog_full, left_fifo_empty;
    wire left_fifo_wr_busy, left_fifo_rd_busy, left_fifo_overflow;
    wire right_fifo_full, right_fifo_prog_full, right_fifo_empty;
    wire right_fifo_wr_busy, right_fifo_rd_busy, right_fifo_overflow;

    // 留出至少 16 个 FIFO 位置给 RGB2Gray 已经在途的流水数据。
    assign s_left_axis_tready  = axis_aresetn && !left_fifo_prog_full  && !left_fifo_wr_busy;
    assign s_right_axis_tready = axis_aresetn && !right_fifo_prog_full && !right_fifo_wr_busy;

    wire left_axis_fire  = s_left_axis_tvalid  && s_left_axis_tready;
    wire right_axis_fire = s_right_axis_tvalid && s_right_axis_tready;

    wire [7:0] left_gray;
    wire [7:0] right_gray;
    wire left_gray_valid;
    wire right_gray_valid;

    takk_rgb2gray u_gray_left (
        .clk(axis_clk),
        .rst_n(axis_aresetn),
        .data_in_r(s_left_axis_tdata[7:0]),
        .data_in_g(s_left_axis_tdata[15:8]),
        .data_in_b(s_left_axis_tdata[23:16]),
        .data_in_valid(left_axis_fire),
        .data_out(left_gray),
        .data_out_valid(left_gray_valid)
    );

    takk_rgb2gray u_gray_right (
        .clk(axis_clk),
        .rst_n(axis_aresetn),
        .data_in_r(s_right_axis_tdata[7:0]),
        .data_in_g(s_right_axis_tdata[15:8]),
        .data_in_b(s_right_axis_tdata[23:16]),
        .data_in_valid(right_axis_fire),
        .data_out(right_gray),
        .data_out_valid(right_gray_valid)
    );

    // RGB2Gray 是两级寄存结构。用两级 sideband 管线对齐 TUSER/TLAST。
    reg [1:0] left_sof_pipe;
    reg [1:0] left_last_pipe;
    reg [1:0] right_sof_pipe;
    reg [1:0] right_last_pipe;

    always @(posedge axis_clk or negedge axis_aresetn) begin
        if (!axis_aresetn) begin
            left_sof_pipe   <= 2'b00;
            left_last_pipe  <= 2'b00;
            right_sof_pipe  <= 2'b00;
            right_last_pipe <= 2'b00;
        end else begin
            left_sof_pipe[0]   <= left_axis_fire  ? s_left_axis_tuser : 1'b0;
            left_sof_pipe[1]   <= left_sof_pipe[0];
            left_last_pipe[0]  <= left_axis_fire  ? s_left_axis_tlast : 1'b0;
            left_last_pipe[1]  <= left_last_pipe[0];

            right_sof_pipe[0]  <= right_axis_fire ? s_right_axis_tuser : 1'b0;
            right_sof_pipe[1]  <= right_sof_pipe[0];
            right_last_pipe[0] <= right_axis_fire ? s_right_axis_tlast : 1'b0;
            right_last_pipe[1] <= right_last_pipe[0];
        end
    end

    // =========================================================================
    // Left gray async FIFO: 16384 x 10 = {gray[7:0], TLAST, TUSER}
    // =========================================================================
    wire [9:0] left_fifo_dout;
    reg left_fifo_rd_en;
    wire left_fifo_wr_en = left_gray_valid && !left_fifo_full && !left_fifo_wr_busy;

    xpm_fifo_async #(
        .FIFO_MEMORY_TYPE("block"),
        .ECC_MODE("no_ecc"),
        .RELATED_CLOCKS(0),
        .FIFO_WRITE_DEPTH(INPUT_FIFO_DEPTH),
        .WRITE_DATA_WIDTH(10),
        .WR_DATA_COUNT_WIDTH(INPUT_COUNT_W),
        .PROG_FULL_THRESH(INPUT_FIFO_DEPTH-16),
        .FULL_RESET_VALUE(0),
        .USE_ADV_FEATURES("0707"),
        .READ_MODE("fwft"),
        .FIFO_READ_LATENCY(0),
        .READ_DATA_WIDTH(10),
        .RD_DATA_COUNT_WIDTH(INPUT_COUNT_W),
        .PROG_EMPTY_THRESH(8),
        .DOUT_RESET_VALUE("0"),
        .CDC_SYNC_STAGES(2),
        .WAKEUP_TIME(0)
    ) u_left_gray_fifo (
        .rst(fifo_axis_rst),
        .wr_clk(axis_clk),
        .wr_en(left_fifo_wr_en),
        .din({left_gray, left_last_pipe[1], left_sof_pipe[1]}),
        .full(left_fifo_full),
        .overflow(left_fifo_overflow),
        .prog_full(left_fifo_prog_full),
        .wr_data_count(), .almost_full(), .wr_ack(), .wr_rst_busy(left_fifo_wr_busy),
        .rd_clk(sgm_clk),
        .rd_en(left_fifo_rd_en),
        .dout(left_fifo_dout),
        .empty(left_fifo_empty),
        .underflow(), .rd_rst_busy(left_fifo_rd_busy), .prog_empty(),
        .rd_data_count(), .almost_empty(), .data_valid(),
        .sleep(1'b0), .injectsbiterr(1'b0), .injectdbiterr(1'b0),
        .sbiterr(), .dbiterr()
    );

    // =========================================================================
    // Right gray async FIFO: 16384 x 10
    // =========================================================================
    wire [9:0] right_fifo_dout;
    reg right_fifo_rd_en;
    wire right_fifo_wr_en = right_gray_valid && !right_fifo_full && !right_fifo_wr_busy;

    xpm_fifo_async #(
        .FIFO_MEMORY_TYPE("block"),
        .ECC_MODE("no_ecc"),
        .RELATED_CLOCKS(0),
        .FIFO_WRITE_DEPTH(INPUT_FIFO_DEPTH),
        .WRITE_DATA_WIDTH(10),
        .WR_DATA_COUNT_WIDTH(INPUT_COUNT_W),
        .PROG_FULL_THRESH(INPUT_FIFO_DEPTH-16),
        .FULL_RESET_VALUE(0),
        .USE_ADV_FEATURES("0707"),
        .READ_MODE("fwft"),
        .FIFO_READ_LATENCY(0),
        .READ_DATA_WIDTH(10),
        .RD_DATA_COUNT_WIDTH(INPUT_COUNT_W),
        .PROG_EMPTY_THRESH(8),
        .DOUT_RESET_VALUE("0"),
        .CDC_SYNC_STAGES(2),
        .WAKEUP_TIME(0)
    ) u_right_gray_fifo (
        .rst(fifo_axis_rst),
        .wr_clk(axis_clk),
        .wr_en(right_fifo_wr_en),
        .din({right_gray, right_last_pipe[1], right_sof_pipe[1]}),
        .full(right_fifo_full),
        .overflow(right_fifo_overflow),
        .prog_full(right_fifo_prog_full),
        .wr_data_count(), .almost_full(), .wr_ack(), .wr_rst_busy(right_fifo_wr_busy),
        .rd_clk(sgm_clk),
        .rd_en(right_fifo_rd_en),
        .dout(right_fifo_dout),
        .empty(right_fifo_empty),
        .underflow(), .rd_rst_busy(right_fifo_rd_busy), .prog_empty(),
        .rd_data_count(), .almost_empty(), .data_valid(),
        .sleep(1'b0), .injectsbiterr(1'b0), .injectdbiterr(1'b0),
        .sbiterr(), .dbiterr()
    );

    wire [7:0] left_head_gray   = SWAP_INPUTS ? right_fifo_dout[9:2] : left_fifo_dout[9:2];
    wire       left_head_last   = left_fifo_dout[1];
    wire       left_head_sof    = left_fifo_dout[0];
    wire [7:0] right_head_gray  = SWAP_INPUTS ? left_fifo_dout[9:2] : right_fifo_dout[9:2];
    wire       right_head_last  = right_fifo_dout[1];
    wire       right_head_sof   = right_fifo_dout[0];

    // =========================================================================
    // Complete results are committed to on-chip BRAM before exposing SOF.
    // Aborting an input frame cannot create a short disparity VDMA frame.
    wire result_available, result_overflow;
    wire run_enable = result_available;

    // =========================================================================
    // Left/right frame synchronizer + frame lifecycle
    // =========================================================================
    localparam ST_SEEK  = 2'd0;
    localparam ST_RESET = 2'd1;
    localparam ST_RUN   = 2'd2;
    localparam ST_DRAIN = 2'd3;

    reg [1:0] sync_state;
    reg marker_error_latched;

    wire heads_ready = !left_fifo_empty && !right_fifo_empty &&
                       !left_fifo_rd_busy && !right_fifo_rd_busy;

    wire pair_marker_match = (left_head_sof  == right_head_sof) &&
                             (left_head_last == right_head_last);

    reg [9:0] input_x;
    reg [9:0] input_y;

    wire expected_sof  = (input_x == 0) && (input_y == 0);
    wire expected_last = (input_x == IMG_WIDTH-1);
    wire expected_marker_ok =
        (left_head_sof  == expected_sof)  &&
        (right_head_sof == expected_sof)  &&
        (left_head_last == expected_last) &&
        (right_head_last== expected_last);

    // Core 在 SEEK/RESET 中保持复位；RUN/DRAIN 中运行。
    wire core_rst_n = sgm_aresetn &&
                      ((sync_state == ST_RUN) || (sync_state == ST_DRAIN));

    wire core_pair_ready;
    wire input_pair_ready, rectified_valid;
    wire [7:0] rectified_left,rectified_right;
    wire [7:0] disparity_u8;
    wire disparity_valid;
    wire [5:0] disparity_raw_unused;
    wire median_fifo_overflow;

    wire pair_fire = (sync_state == ST_RUN) && run_enable && heads_ready &&
                     input_pair_ready && pair_marker_match && expected_marker_ok;

    generate if(RECTIFY_INPUTS) begin: calibrated_input
        sgm_live_rectifier rectifier(.clk(sgm_clk),.rst_n(core_rst_n),
            .left_in(left_head_gray),.right_in(right_head_gray),
            .in_valid(pair_fire),.in_ready(input_pair_ready),
            .left_out(rectified_left),.right_out(rectified_right),
            .out_valid(rectified_valid),.out_ready(core_pair_ready));
    end else begin: unrectified_input
        assign input_pair_ready=core_pair_ready;
        assign rectified_valid=pair_fire;
        assign rectified_left=left_head_gray;
        assign rectified_right=right_head_gray;
    end endgenerate

    // FIFO read enables are exactly the pair acceptance pulse in RUN。
    // SEEK 中仅用于寻找同一帧 SOF。
    always @* begin
        left_fifo_rd_en  = 1'b0;
        right_fifo_rd_en = 1'b0;

        case (sync_state)
            ST_SEEK: begin
                // Drain partial frames independently. Holding a valid SOF
                // does not require the opposite FIFO to contain a pixel yet.
                left_fifo_rd_en = !left_fifo_empty && !left_fifo_rd_busy && !left_head_sof;
                right_fifo_rd_en = !right_fifo_empty && !right_fifo_rd_busy && !right_head_sof;
            end

            ST_RUN: begin
                if (pair_fire) begin
                    left_fifo_rd_en  = 1'b1;
                    right_fifo_rd_en = 1'b1;
                end
            end

            default: begin
                // RESET / DRAIN 均不读取新的相机像素。
            end
        endcase
    end

    // =========================================================================
    // Complete SGM core
    // =========================================================================
    sgm_stereo_core_50m #(
        .IMG_WIDTH(IMG_WIDTH),
        .IMG_HEIGHT(IMG_HEIGHT),
        .MAX_DISP(MAX_DISP),
        .OUTPUT_STALL(STREAM_OUTPUT),
        .PIXEL_II(CORE_PIXEL_II),.PATHS(CORE_PATHS),.MEDIAN_ENABLE(MEDIAN_ENABLE)
    ) u_sgm_core (
        .clk(sgm_clk),
        .rst_n(core_rst_n),
        .left_gray(rectified_left),
        .right_gray(rectified_right),
        .pair_valid(rectified_valid),
        .output_ready(result_available),
        .pair_ready(core_pair_ready),
        .disparity_u8(disparity_u8),
        .disparity_valid(disparity_valid),
        .disparity_raw(disparity_raw_unused),
        .median_fifo_overflow(median_fifo_overflow)
    );

    // =========================================================================
    // Final disparity raster count
    // =========================================================================
    reg [9:0] disp_x;
    reg [9:0] disp_y;

    wire final_disp_pixel = disparity_valid &&
                            (disp_x == IMG_WIDTH-1) &&
                            (disp_y == IMG_HEIGHT-1);

    always @(posedge sgm_clk or negedge core_rst_n) begin
        if (!core_rst_n) begin
            disp_x <= 10'd0;
            disp_y <= 10'd0;
        end else if (disparity_valid) begin
            if (disp_x == IMG_WIDTH-1) begin
                disp_x <= 10'd0;
                if (disp_y == IMG_HEIGHT-1)
                    disp_y <= 10'd0;
                else
                    disp_y <= disp_y + 1'b1;
            end else begin
                disp_x <= disp_x + 1'b1;
            end
        end
    end

    wire disp_sof  = disparity_valid && (disp_x == 0) && (disp_y == 0);
    wire disp_last = disparity_valid && (disp_x == IMG_WIDTH-1);

    // =========================================================================
    // Sync state machine
    // =========================================================================
    always @(posedge sgm_clk or negedge sgm_aresetn) begin
        if (!sgm_aresetn) begin
            sync_state <= ST_SEEK;
            input_x <= 10'd0;
            input_y <= 10'd0;
            marker_error_latched <= 1'b0;
        end else begin
            case (sync_state)
                ST_SEEK: begin
                    input_x <= 10'd0;
                    input_y <= 10'd0;
                    if (run_enable && heads_ready && left_head_sof && right_head_sof)
                        sync_state <= ST_RESET;
                end

                ST_RESET: begin
                    // Core 已在本状态保持 reset；SOF 仍停在 FIFO 头。
                    input_x <= 10'd0;
                    input_y <= 10'd0;
                    sync_state <= ST_RUN;
                end

                ST_RUN: begin
                    if (heads_ready && core_pair_ready && run_enable &&
                        (!pair_marker_match || !expected_marker_ok)) begin
                        // 不消费错误 beat，回 SEEK 后重新寻找下一对 SOF。
                        marker_error_latched <= 1'b1;
                        sync_state <= ST_SEEK;
                    end else if (pair_fire) begin
                        if ((input_x == IMG_WIDTH-1) && (input_y == IMG_HEIGHT-1)) begin
                            // 最后一个真实像素已经进入 Census。
                            // 后面等待 Census 右/下补零、SGM/WTA/Median 全部排空。
                            input_x <= 10'd0;
                            input_y <= 10'd0;
                            sync_state <= ST_DRAIN;
                        end else if (input_x == IMG_WIDTH-1) begin
                            input_x <= 10'd0;
                            input_y <= input_y + 1'b1;
                        end else begin
                            input_x <= input_x + 1'b1;
                        end
                    end
                end

                ST_DRAIN: begin
                    // 以 Median 最终第 307200 个有效输出作为一帧真正完成。
                    if (final_disp_pixel)
                        sync_state <= ST_SEEK;
                end

                default: sync_state <= ST_SEEK;
            endcase
        end
    end

    // =========================================================================
    // Atomic complete-frame output, with AXIS backpressure and CDC handshake.
    generate if(!STREAM_OUTPUT) begin: atomic_bram_output
    sgm_result_frame #(.W(IMG_WIDTH), .H(IMG_HEIGHT), .PACKED_DISPARITY(RECTIFY_INPUTS)) u_result_frame (
        .write_clk(sgm_clk), .write_reset(fifo_sgm_rst),
        .capture_reset(!core_rst_n), .pixel(disparity_u8),
        .pixel_valid(disparity_valid), .available(result_available),
        .overflow(result_overflow), .read_clk(axis_clk), .read_reset(fifo_axis_rst),
        .data(m_disp_axis_tdata), .valid(m_disp_axis_tvalid),
        .user(m_disp_axis_tuser), .last(m_disp_axis_tlast), .ready(m_disp_axis_tready)
    );
    end else begin: streamed_ddr_output
        // Census/median may still emit padded pixels after input throttling.
        // Reserve three rows, the 258-pixel interstage FIFO and pipeline slack.
        localparam DEPTH=4096, RESERVE=3*IMG_WIDTH+258+64;
        wire full,empty,prog_full,wr_busy,rd_busy;
        wire [7:0] word;
        wire [5:0] packed_disparity;
        assign result_available=!prog_full && !wr_busy && !fifo_sgm_rst;
        assign m_disp_axis_tvalid=!empty && !rd_busy && axis_aresetn;
        assign {m_disp_axis_tuser,m_disp_axis_tlast,packed_disparity}=word;
        assign m_disp_axis_tdata={packed_disparity,packed_disparity[5:4]};
        xpm_fifo_async #(.FIFO_MEMORY_TYPE("block"),.FIFO_WRITE_DEPTH(DEPTH),
            .WRITE_DATA_WIDTH(8),.READ_DATA_WIDTH(8),.READ_MODE("fwft"),
            .FIFO_READ_LATENCY(0),.CDC_SYNC_STAGES(3),
            .PROG_FULL_THRESH(DEPTH-RESERVE),.PROG_EMPTY_THRESH(10),
            .WR_DATA_COUNT_WIDTH(13),.RD_DATA_COUNT_WIDTH(13),
            .USE_ADV_FEATURES("0707"),.DOUT_RESET_VALUE("0")) results(
            .rst(fifo_sgm_rst),.wr_clk(sgm_clk),.rd_clk(axis_clk),
            .din({disp_sof,disp_last,disparity_u8[7:2]}),
            .wr_en(disparity_valid && !wr_busy),
            .rd_en(m_disp_axis_tvalid && m_disp_axis_tready),.dout(word),
            .full(full),.empty(empty),.prog_full(prog_full),.prog_empty(),
            .wr_rst_busy(wr_busy),.rd_rst_busy(rd_busy),.overflow(result_overflow),
            .underflow(),.wr_ack(),.data_valid(),.wr_data_count(),.rd_data_count(),
            .sleep(1'b0),.injectsbiterr(1'b0),.injectdbiterr(1'b0),.sbiterr(),.dbiterr(),
            .almost_full(),.almost_empty());
    end endgenerate
    assign m_disp_axis_tkeep=1'b1;
    // [7] invalid input frame or median overflow; [6] result overwrite;
    // [5:4] input overflow; [3] seeking/draining; [2] committed frame busy.
    assign debug_status = {
        (marker_error_latched | median_fifo_overflow), result_overflow,
        right_fifo_overflow, left_fifo_overflow, (sync_state != ST_RUN),
        !result_available, right_fifo_full, left_fifo_full
    };

    // TKEEP currently only used by the original AXIS branch/VDMA.
    wire _unused_keep = &{1'b0, s_left_axis_tkeep, s_right_axis_tkeep};

endmodule
