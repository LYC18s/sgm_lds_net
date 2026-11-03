`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// 模块名称：takk_median3x3
//
// 3x3 disparity 中值滤波，完整 IMG_WIDTH x IMG_HEIGHT 输出，零填充边缘。
//
// 与 Census 一样采用“按需读取”：
//   - reset 后第一个真实 disparity 立即进入窗口；
//   - 左/上边界由清零窗口/顶部历史屏蔽自然形成；
//   - 每行末尾内部自动补 1 个右侧 0，此时 data_in_ready=0；
//   - 帧末内部自动补 1 行底部 0，此时 data_in_ready=0；
//   - 最终仍输出完整 W x H 个像素。
//////////////////////////////////////////////////////////////////////////////////

module takk_median3x3 #(
    parameter IMG_WIDTH  = 640,
    parameter IMG_HEIGHT = 480,
    parameter DISP_W     = 6
)(
    input  wire                  clk,
    input  wire                  rst_n,

    input  wire [DISP_W-1:0]     data_in_disp,
    input  wire                  data_in_valid,
    output wire                  data_in_ready,

    output reg  [DISP_W-1:0]     data_out_disp,
    output reg                   data_out_valid
);

    // 虚拟扫描：x=0..W（W 为右侧补 0），y=0..H（H 为底部补 0）。
    reg [10:0] vx;
    reg [10:0] vy;

    wire need_real_pixel = (vx < IMG_WIDTH) && (vy < IMG_HEIGHT);
    assign data_in_ready = rst_n && need_real_pixel;

    wire step = need_real_pixel ? data_in_valid : 1'b1;
    wire [DISP_W-1:0] current_disp = need_real_pixel ? data_in_disp : {DISP_W{1'b0}};

    //=========================================================================
    // 两行历史
    //=========================================================================
    (* ram_style = "distributed" *) reg [DISP_W-1:0] line1 [0:IMG_WIDTH-1];
    (* ram_style = "distributed" *) reg [DISP_W-1:0] line2 [0:IMG_WIDTH-1];

    integer init_i;
    initial begin
        for (init_i=0; init_i<IMG_WIDTH; init_i=init_i+1) begin
            line1[init_i] = {DISP_W{1'b0}};
            line2[init_i] = {DISP_W{1'b0}};
        end
    end

    wire [9:0] ram_addr = (vx < IMG_WIDTH) ? vx[9:0] : 10'd0;
    wire [DISP_W-1:0] hist1_raw = line1[ram_addr];
    wire [DISP_W-1:0] hist2_raw = line2[ram_addr];

    // 顶部屏蔽旧帧历史；右补 0 列不访问真实 RAM。
    wire [DISP_W-1:0] hist1 = ((vx < IMG_WIDTH) && (vy >= 1)) ? hist1_raw : {DISP_W{1'b0}};
    wire [DISP_W-1:0] hist2 = ((vx < IMG_WIDTH) && (vy >= 2)) ? hist2_raw : {DISP_W{1'b0}};

    // History is overwritten before vy permits it to affect the window.
    // Keep RAM writes outside the asynchronous-reset process: otherwise
    // Vivado implements these arrays as thousands of flip-flops and muxes.
    always @(posedge clk) begin
        if (rst_n && step && vx < IMG_WIDTH) begin
            line2[ram_addr] <= hist1_raw;
            line1[ram_addr] <= current_disp;
        end
    end

    //=========================================================================
    // 3x3 窗口
    //=========================================================================
    reg [DISP_W-1:0] window [0:2][0:2];
    integer wr;
    integer wc;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (wr=0; wr<3; wr=wr+1)
                for (wc=0; wc<3; wc=wc+1)
                    window[wr][wc] <= {DISP_W{1'b0}};
        end else if (step) begin
            if (vx == 0) begin
                // 新一行：左边界历史清 0。
                for (wr=0; wr<3; wr=wr+1) begin
                    window[wr][2] <= {DISP_W{1'b0}};
                    window[wr][1] <= {DISP_W{1'b0}};
                end
            end else begin
                for (wr=0; wr<3; wr=wr+1) begin
                    window[wr][2] <= window[wr][1];
                    window[wr][1] <= window[wr][0];
                end
            end

            window[0][0] <= current_disp;
            window[1][0] <= hist1;
            window[2][0] <= hist2;

        end
    end

    //=========================================================================
    // 虚拟坐标推进
    //=========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vx <= 0;
            vy <= 0;
        end else if (step) begin
            if (vx == IMG_WIDTH) begin
                vx <= 0;
                if (vy == IMG_HEIGHT)
                    vy <= 0;
                else
                    vy <= vy + 1'b1;
            end else begin
                vx <= vx + 1'b1;
            end
        end
    end

    // 当当前虚拟位置 >=1 时，窗口中心落在真实图像。
    wire output_region_now = step &&
                             (vx >= 1) && (vx <= IMG_WIDTH) &&
                             (vy >= 1) && (vy <= IMG_HEIGHT);

    reg window_region_d1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            window_region_d1 <= 1'b0;
        else
            window_region_d1 <= output_region_now;
    end

    //=========================================================================
    // Stage1：每行 3 个数排序
    //=========================================================================
    wire [DISP_W-1:0] row0_lo_w, row0_mid_w, row0_hi_w;
    wire [DISP_W-1:0] row1_lo_w, row1_mid_w, row1_hi_w;
    wire [DISP_W-1:0] row2_lo_w, row2_mid_w, row2_hi_w;

    sort3_comb #(.W(DISP_W)) u_sort_row0 (
        .a(window[0][0]), .b(window[0][1]), .c(window[0][2]),
        .lo(row0_lo_w), .mid(row0_mid_w), .hi(row0_hi_w)
    );
    sort3_comb #(.W(DISP_W)) u_sort_row1 (
        .a(window[1][0]), .b(window[1][1]), .c(window[1][2]),
        .lo(row1_lo_w), .mid(row1_mid_w), .hi(row1_hi_w)
    );
    sort3_comb #(.W(DISP_W)) u_sort_row2 (
        .a(window[2][0]), .b(window[2][1]), .c(window[2][2]),
        .lo(row2_lo_w), .mid(row2_mid_w), .hi(row2_hi_w)
    );

    reg [DISP_W-1:0] row0_lo_s1, row0_mid_s1, row0_hi_s1;
    reg [DISP_W-1:0] row1_lo_s1, row1_mid_s1, row1_hi_s1;
    reg [DISP_W-1:0] row2_lo_s1, row2_mid_s1, row2_hi_s1;
    reg valid_s1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_s1 <= 1'b0;
            row0_lo_s1 <= 0; row0_mid_s1 <= 0; row0_hi_s1 <= 0;
            row1_lo_s1 <= 0; row1_mid_s1 <= 0; row1_hi_s1 <= 0;
            row2_lo_s1 <= 0; row2_mid_s1 <= 0; row2_hi_s1 <= 0;
        end else begin
            valid_s1 <= window_region_d1;
            if (window_region_d1) begin
                row0_lo_s1 <= row0_lo_w; row0_mid_s1 <= row0_mid_w; row0_hi_s1 <= row0_hi_w;
                row1_lo_s1 <= row1_lo_w; row1_mid_s1 <= row1_mid_w; row1_hi_s1 <= row1_hi_w;
                row2_lo_s1 <= row2_lo_w; row2_mid_s1 <= row2_mid_w; row2_hi_s1 <= row2_hi_w;
            end
        end
    end

    //=========================================================================
    // Stage2：A=max(row_min), B=median(row_mid), C=min(row_max)
    //=========================================================================
    wire [DISP_W-1:0] mins_lo_w, mins_mid_w, mins_hi_w;
    wire [DISP_W-1:0] mids_lo_w, mids_mid_w, mids_hi_w;
    wire [DISP_W-1:0] maxs_lo_w, maxs_mid_w, maxs_hi_w;

    sort3_comb #(.W(DISP_W)) u_sort_row_mins (
        .a(row0_lo_s1), .b(row1_lo_s1), .c(row2_lo_s1),
        .lo(mins_lo_w), .mid(mins_mid_w), .hi(mins_hi_w)
    );
    sort3_comb #(.W(DISP_W)) u_sort_row_mids (
        .a(row0_mid_s1), .b(row1_mid_s1), .c(row2_mid_s1),
        .lo(mids_lo_w), .mid(mids_mid_w), .hi(mids_hi_w)
    );
    sort3_comb #(.W(DISP_W)) u_sort_row_maxs (
        .a(row0_hi_s1), .b(row1_hi_s1), .c(row2_hi_s1),
        .lo(maxs_lo_w), .mid(maxs_mid_w), .hi(maxs_hi_w)
    );

    reg [DISP_W-1:0] A_s2, B_s2, C_s2;
    reg valid_s2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_s2 <= 1'b0;
            A_s2 <= 0; B_s2 <= 0; C_s2 <= 0;
        end else begin
            valid_s2 <= valid_s1;
            if (valid_s1) begin
                A_s2 <= mins_hi_w;
                B_s2 <= mids_mid_w;
                C_s2 <= maxs_lo_w;
            end
        end
    end

    //=========================================================================
    // Stage3：median(A,B,C)
    //=========================================================================
    wire [DISP_W-1:0] final_lo_w, final_mid_w, final_hi_w;
    sort3_comb #(.W(DISP_W)) u_sort_final (
        .a(A_s2), .b(B_s2), .c(C_s2),
        .lo(final_lo_w), .mid(final_mid_w), .hi(final_hi_w)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            data_out_disp  <= {DISP_W{1'b0}};
            data_out_valid <= 1'b0;
        end else begin
            data_out_valid <= valid_s2;
            if (valid_s2)
                data_out_disp <= final_mid_w;
        end
    end

endmodule
