`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// 模块名称：takk_wta_median_top
//
// SGM cost -> WTA -> 258 深度同步 FIFO -> 3x3 Median
//
// FIFO 的目的不是跨时钟，而是解耦两种窗口节奏：
//   - Census/SGM/WTA 每行有自己的 bubble；
//   - Median 在每行末尾要内部补 1 个 0，并在帧末补 1 行 0。
//
// Median 通过 data_in_ready 按需从 FIFO 读取。
//////////////////////////////////////////////////////////////////////////////////

module takk_wta_median_top #(
    parameter IMG_WIDTH  = 640,
    parameter IMG_HEIGHT = 480,
    parameter MAX_DISP   = 64,
    parameter COST_W     = 11,
    parameter DISP_W     = 6,
    parameter MEDIAN_FIFO_DEPTH = 258
)(
    input  wire                        clk,
    input  wire                        rst_n,

    input  wire [MAX_DISP*COST_W-1:0] data_in_cost,
    input  wire                        data_in_valid,

    output wire [DISP_W-1:0]          raw_disp,
    output wire                       raw_disp_valid,

    output wire [DISP_W-1:0]          filtered_disp,
    output wire                       filtered_disp_valid,

    // 调试：正常工作时应始终为 0。
    output wire                       median_fifo_overflow
);

    wire [COST_W-1:0] min_cost_debug;

    takk_wta #(
        .MAX_DISP(MAX_DISP),
        .COST_W  (COST_W),
        .DISP_W  (DISP_W)
    ) u_wta (
        .clk              (clk),
        .rst_n            (rst_n),
        .data_in_cost     (data_in_cost),
        .data_in_valid    (data_in_valid),
        .data_out_disp    (raw_disp),
        .data_out_min_cost(min_cost_debug),
        .data_out_valid   (raw_disp_valid)
    );

    //==========================================================================
    // WTA -> Median 同步 FIFO，严格按用户指定使用 258 深度。
    //==========================================================================
    wire [DISP_W-1:0] fifo_dout;
    wire fifo_full;
    wire fifo_empty;
    wire fifo_wr_en = raw_disp_valid && !fifo_full;

    reg overflow_latched;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            overflow_latched <= 1'b0;
        else if (raw_disp_valid && fifo_full)
            overflow_latched <= 1'b1;
    end
    assign median_fifo_overflow = overflow_latched;

    wire median_in_ready;
    wire fifo_rd_en = median_in_ready && !fifo_empty;

    sgm_sync_fifo #(
        .DATA_W(DISP_W),
        .DEPTH (MEDIAN_FIFO_DEPTH)
    ) u_wta_median_fifo (
        .clk  (clk),
        .rst_n(rst_n),
        .din  (raw_disp),
        .wr_en(fifo_wr_en),
        .full (fifo_full),
        .dout (fifo_dout),
        .rd_en(fifo_rd_en),
        .empty(fifo_empty)
    );

    //==========================================================================
    // Median：只在自己需要真实 disparity 时从 FIFO 取数。
    // 右边界/底边界补 0 时 median_in_ready=0，FIFO 自动保持。
    //==========================================================================
    takk_median3x3 #(
        .IMG_WIDTH (IMG_WIDTH),
        .IMG_HEIGHT(IMG_HEIGHT),
        .DISP_W    (DISP_W)
    ) u_median (
        .clk           (clk),
        .rst_n         (rst_n),
        .data_in_disp  (fifo_dout),
        .data_in_valid (fifo_rd_en),
        .data_in_ready (median_in_ready),
        .data_out_disp (filtered_disp),
        .data_out_valid(filtered_disp_valid)
    );

endmodule

//////////////////////////////////////////////////////////////////////////////////
// 小型同步 FIFO。
//
// 之所以不用 xpm_fifo_sync，是因为这里明确要求 DEPTH=258（非 2 的幂），
// 自己写环形指针最直接，且 258x6 只有约 1.5 kbit。
// dout 为 FWFT 风格：非空时直接指向当前读指针的数据。
//////////////////////////////////////////////////////////////////////////////////
module sgm_sync_fifo #(
    parameter DATA_W = 6,
    parameter DEPTH  = 258,
    parameter PTR_W  = 9,
    parameter CNT_W  = 9
)(
    input  wire              clk,
    input  wire              rst_n,
    input  wire [DATA_W-1:0] din,
    input  wire              wr_en,
    output wire              full,
    output wire [DATA_W-1:0] dout,
    input  wire              rd_en,
    output wire              empty
);

    (* ram_style = "distributed" *) reg [DATA_W-1:0] mem [0:DEPTH-1];
    reg [PTR_W-1:0] wr_ptr;
    reg [PTR_W-1:0] rd_ptr;
    reg [CNT_W-1:0] count;

    assign empty = (count == 0);
    assign full  = (count == DEPTH);
    assign dout  = mem[rd_ptr];

    wire do_write = wr_en && !full;
    wire do_read  = rd_en && !empty;

    // Only pointers/count need reset; unread locations have no validity.
    always @(posedge clk)
        if (rst_n && do_write) mem[wr_ptr] <= din;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr <= 0;
            rd_ptr <= 0;
            count  <= 0;
        end else begin
            if (do_write) begin
                if (wr_ptr == DEPTH-1)
                    wr_ptr <= 0;
                else
                    wr_ptr <= wr_ptr + 1'b1;
            end

            if (do_read) begin
                if (rd_ptr == DEPTH-1)
                    rd_ptr <= 0;
                else
                    rd_ptr <= rd_ptr + 1'b1;
            end

            case ({do_write, do_read})
                2'b10: count <= count + 1'b1;
                2'b01: count <= count - 1'b1;
                default: count <= count;
            endcase
        end
    end

endmodule
