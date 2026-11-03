`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// 640x480 / 64 disparity / 2-path SGM core, 50 MHz domain.
//
// v2 变化：
//   1. RGB2Gray 已移动到 142.857 MHz AXIS 域，本模块直接接收 8bit gray。
//   2. Census 自己按需取真实像素，并在行尾/帧尾内部补 0。
//   3. pair_ready=1 时上游才允许同时弹出左右灰度 FIFO。
//   4. WTA 与 Median 之间加入 258 深度同步 FIFO。
//////////////////////////////////////////////////////////////////////////////////
module sgm_stereo_core_50m #(
    parameter IMG_WIDTH  = 640,
    parameter IMG_HEIGHT = 480,
    parameter MAX_DISP   = 64,
    parameter OUTPUT_STALL = 0,
    parameter PIXEL_II = 4,
    parameter PATHS = 4,
    parameter MEDIAN_ENABLE = 1
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire [7:0]  left_gray,
    input  wire [7:0]  right_gray,
    input  wire        pair_valid,
    input  wire        output_ready,
    output wire        pair_ready,

    output wire [7:0]  disparity_u8,
    output wire        disparity_valid,
    output wire [5:0]  disparity_raw,
    output wire        median_fifo_overflow
);

    // Fixed 50 MHz; two groups share 32 arithmetic lanes. Pause virtual
    // Census padding as well as real pixels when result credits run low.
    reg [$clog2(PIXEL_II)-1:0] scan_phase;
    always @(posedge clk or negedge rst_n)
        if(!rst_n) scan_phase<=0;
        else scan_phase<=scan_phase+1'b1;
    wire scan_enable=scan_phase==0 && (!OUTPUT_STALL || output_ready);
    initial if(PIXEL_II!=4 && PIXEL_II!=8) $error("Core requires PIXEL_II=4 or 8");

    // -------------------------------------------------------------------------
    // 5x5 Census
    // -------------------------------------------------------------------------
    wire [24:0] census_l;
    wire [24:0] census_r;
    wire census_l_valid;
    wire census_r_valid;
    wire census_l_ready;
    wire census_r_ready;
    wire [7:0] census_l_center_gray;
    wire [7:0] census_r_center_gray_unused;

    // 两个 Census 从同一个 reset、同一个 pair_valid 启动，状态应始终一致。
    assign pair_ready = census_l_ready && census_r_ready && (!OUTPUT_STALL || output_ready);
    wire census_input_valid = pair_valid && pair_ready;

    sgm_packed_census #(
        .IMG_WIDTH(IMG_WIDTH),
        .IMG_HEIGHT(IMG_HEIGHT),
        .ADDR_WIDTH(10),.USE_SCAN_ENABLE(1)
    ) u_census_l (
        .scan_enable(scan_enable),
        .clk(clk),
        .rst_n(rst_n),
        .data_in(left_gray),
        .data_in_valid(census_input_valid),
        .data_in_ready(census_l_ready),
        .data_out(census_l),
        .data_out_valid(census_l_valid),
        .center_gray_out(census_l_center_gray)
    );

    sgm_packed_census #(
        .IMG_WIDTH(IMG_WIDTH),
        .IMG_HEIGHT(IMG_HEIGHT),
        .ADDR_WIDTH(10),.USE_SCAN_ENABLE(1)
    ) u_census_r (
        .scan_enable(scan_enable),
        .clk(clk),
        .rst_n(rst_n),
        .data_in(right_gray),
        .data_in_valid(census_input_valid),
        .data_in_ready(census_r_ready),
        .data_out(census_r),
        .data_out_valid(census_r_valid),
        .center_gray_out(census_r_center_gray_unused)
    );

    // -------------------------------------------------------------------------
    // 64 disparity Hamming
    // -------------------------------------------------------------------------
    wire [MAX_DISP*6-1:0] hamming_cost;
    wire hamming_valid;
    wire [7:0] hamming_gray;

    sgm_folded32_hamming #(
        .IMG_WIDTH(IMG_WIDTH),.LANES(PIXEL_II==8?8:32)
    ) u_hamming (
        .clk(clk),
        .rst_n(rst_n),
        .data_in_L(census_l),
        .data_in_R(census_r),
        .data_in_gray(census_l_center_gray),
        .data_in_L_valid(census_l_valid),
        .data_in_R_valid(census_r_valid),
        .data_out(hamming_cost),
        .data_out_valid(hamming_valid),
        .data_out_gray(hamming_gray)
    );

    // -------------------------------------------------------------------------
    // 2-path SGM
    // -------------------------------------------------------------------------
    // 63+400=463; 511 remains an illegal disparity sentinel.
    localparam PATH_W = 9;
    localparam SUM_W  = 11;

    wire [MAX_DISP*SUM_W-1:0]  sgm_cost;
    wire                        sgm_valid;
    wire [MAX_DISP*PATH_W-1:0] sgm_h_unused;
    wire [MAX_DISP*PATH_W-1:0] sgm_v_unused;

    generate if(PIXEL_II==8) begin: folded16
    sgm_four_tdm16_aggr #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT),.PATHS(PATHS)) u_sgm (
        .clk(clk),.rst_n(rst_n),.data_in_cost(hamming_cost),
        .data_in_gray(hamming_gray),.data_in_valid(hamming_valid),
        .data_out(sgm_cost),.data_out_valid(sgm_valid)
    );
    end else begin: folded32
    sgm_four_parallel32_aggr #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT)) u_sgm (
        .clk(clk),.rst_n(rst_n),.data_in_cost(hamming_cost),
        .data_in_gray(hamming_gray),.data_in_valid(hamming_valid),
        .data_out(sgm_cost),.data_out_valid(sgm_valid)
    );
    end endgenerate

    // -------------------------------------------------------------------------
    // WTA -> 258-entry FIFO -> Median
    // -------------------------------------------------------------------------
    wire [5:0] raw_disp;
    wire raw_disp_valid;
    wire [5:0] filtered_disp;
    wire filtered_disp_valid;

    generate if(MEDIAN_ENABLE==1) begin: filtered
    takk_wta_median_top #(
        .IMG_WIDTH(IMG_WIDTH),
        .IMG_HEIGHT(IMG_HEIGHT),
        .MAX_DISP(MAX_DISP),
        .COST_W(SUM_W),
        .DISP_W(6),
        .MEDIAN_FIFO_DEPTH(258)
    ) u_wta_median (
        .clk(clk),
        .rst_n(rst_n),
        .data_in_cost(sgm_cost),
        .data_in_valid(sgm_valid),
        .raw_disp(raw_disp),
        .raw_disp_valid(raw_disp_valid),
        .filtered_disp(filtered_disp),
        .filtered_disp_valid(filtered_disp_valid),
        .median_fifo_overflow(median_fifo_overflow)
    );

    end else begin: compact_filter
        if(PIXEL_II==8) begin: folded_wta
        sgm_wta8 #(.MAX_DISP(MAX_DISP),.COST_W(SUM_W),.DISP_W(6)) wta(
            .clk(clk),.rst_n(rst_n),.data_in_cost(sgm_cost),.data_in_valid(sgm_valid),
            .data_out_disp(raw_disp),.data_out_min_cost(),.data_out_valid(raw_disp_valid));
        end else begin: parallel_wta
        takk_wta #(.MAX_DISP(MAX_DISP),.COST_W(SUM_W),.DISP_W(6)) wta(
            .clk(clk),.rst_n(rst_n),.data_in_cost(sgm_cost),.data_in_valid(sgm_valid),
            .data_out_disp(raw_disp),.data_out_min_cost(),.data_out_valid(raw_disp_valid));
        end
        if(MEDIAN_ENABLE==2) begin: horizontal
            sgm_median3_horizontal #(.IMG_WIDTH(IMG_WIDTH)) median(
                .clk(clk),.rst_n(rst_n),.din(raw_disp),.valid(raw_disp_valid),
                .dout(filtered_disp),.out_valid(filtered_disp_valid));
        end else begin: bypass
            assign filtered_disp=raw_disp;assign filtered_disp_valid=raw_disp_valid;
        end
        assign median_fifo_overflow=1'b0;
    end endgenerate
    // 0..63 -> 0..255 的无乘法映射。
    assign disparity_u8    = {filtered_disp, filtered_disp[5:4]};
    assign disparity_valid = filtered_disp_valid;
    assign disparity_raw   = filtered_disp;

endmodule
