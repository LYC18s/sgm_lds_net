`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// 模块名称：sgm_line_ram
//
// 功能：
//   垂直 SGM 路径使用的一行状态缓存。
//
// 端口A：同步读
// 端口B：同步写
//
// 一个时钟周期中可以：
//   - 读取“当前输入列 x”的上一行状态
//   - 同时写回“上一拍已经算完的列 x-1”的当前行状态
//
// 这种访问方式适合后续映射到 FPGA Block RAM。
//
// 注意：
//   RAM本身不需要reset清零。
//   因为垂直路径在 y=0 时明确执行 L=C，完全忽略旧RAM内容。
//   即使整机reset后RAM中存在旧值，也不会污染新帧第一行。
//////////////////////////////////////////////////////////////////////////////////

module sgm_line_ram #(
    parameter DATA_W = 640,
    parameter DEPTH  = 640,
    parameter ADDR_W = 10
)(
    input  wire                  clk,

    // 同步读端口
    input  wire                  rd_en,
    input  wire [ADDR_W-1:0]     rd_addr,
    output reg  [DATA_W-1:0]     rd_data,

    // 同步写端口
    input  wire                  wr_en,
    input  wire [ADDR_W-1:0]     wr_addr,
    input  wire [DATA_W-1:0]     wr_data
);

    (* ram_style = "block" *) reg [DATA_W-1:0] mem [0:DEPTH-1];

    always @(posedge clk) begin

        if (rd_en)
            rd_data <= mem[rd_addr];

        if (wr_en)
            mem[wr_addr] <= wr_data;

    end

endmodule
